#include "TransformResultDialog.h"
#include <QClipboard>
#include <QApplication>
#include <QHeaderView>
#include <QMessageBox>
#include <QGridLayout>
#include <QTabWidget>

TransformResultDialog::TransformResultDialog(const std::vector<std::string>& groups,
                                             const std::vector<std::pair<std::string, std::string>>& pairs,
                                             QWidget* parent)
    : QDialog(parent), allGroups(groups), availablePairs(pairs)
{
    setWindowTitle(QStringLiteral("转换矩阵计算结果"));
    setMinimumSize(600, 700);
    resize(650, 750);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QGroupBox* selectGroup = new QGroupBox(QStringLiteral("选择分组"), this);
    QGridLayout* selectLayout = new QGridLayout;
    
    selectLayout->addWidget(new QLabel(QStringLiteral("源分组:"), this), 0, 0);
    sourceGroupCombo = new QComboBox(this);
    selectLayout->addWidget(sourceGroupCombo, 0, 1);
    
    selectLayout->addWidget(new QLabel(QStringLiteral("目标分组:"), this), 0, 2);
    targetGroupCombo = new QComboBox(this);
    selectLayout->addWidget(targetGroupCombo, 0, 3);
    
    selectGroup->setLayout(selectLayout);
    mainLayout->addWidget(selectGroup);

    statusLabel = new QLabel(this);
    mainLayout->addWidget(statusLabel);

    QHBoxLayout* actionLayout = new QHBoxLayout;
    recomputeBtn = new QPushButton(QStringLiteral("计算"), this);
    actionLayout->addStretch();
    actionLayout->addWidget(recomputeBtn);
    mainLayout->addLayout(actionLayout);

    QTabWidget* tabWidget = new QTabWidget(this);

    QWidget* matrixTab = new QWidget(tabWidget);
    QVBoxLayout* matrixLayout = new QVBoxLayout(matrixTab);
    
    QGroupBox* abGroup = new QGroupBox(QStringLiteral("转换矩阵 (源→目标)"), matrixTab);
    QVBoxLayout* abLayout = new QVBoxLayout;
    matrixABTable = new QTableWidget(4, 4, matrixTab);
    matrixABTable->horizontalHeader()->setVisible(false);
    matrixABTable->verticalHeader()->setVisible(false);
    abLayout->addWidget(matrixABTable);
    copyABBtn = new QPushButton(QStringLiteral("复制矩阵"), matrixTab);
    abLayout->addWidget(copyABBtn);
    abGroup->setLayout(abLayout);

    QGroupBox* baGroup = new QGroupBox(QStringLiteral("转换矩阵 (目标→源)"), matrixTab);
    QVBoxLayout* baLayout = new QVBoxLayout;
    matrixBATable = new QTableWidget(4, 4, matrixTab);
    matrixBATable->horizontalHeader()->setVisible(false);
    matrixBATable->verticalHeader()->setVisible(false);
    baLayout->addWidget(matrixBATable);
    copyBABtn = new QPushButton(QStringLiteral("复制矩阵"), matrixTab);
    baLayout->addWidget(copyBABtn);
    baGroup->setLayout(baLayout);

    matrixLayout->addWidget(abGroup);
    matrixLayout->addWidget(baGroup);
    tabWidget->addTab(matrixTab, QStringLiteral("矩阵结果"));

    QWidget* transformTab = new QWidget(tabWidget);
    QVBoxLayout* transformLayout = new QVBoxLayout(transformTab);
    
    QGroupBox* singleGroup = new QGroupBox(QStringLiteral("单点转换"), transformTab);
    QVBoxLayout* singleLayout = new QVBoxLayout;
    
    QHBoxLayout* inputLayout = new QHBoxLayout;
    inputLayout->addWidget(new QLabel(QStringLiteral("输入点坐标:"), transformTab));
    inputLayout->addWidget(new QLabel(QStringLiteral("X:"), transformTab));
    inputX = new QLineEdit(transformTab);
    inputX->setPlaceholderText(QStringLiteral("0.0"));
    inputX->setMaximumWidth(80);
    inputLayout->addWidget(inputX);
    inputLayout->addWidget(new QLabel(QStringLiteral("Y:"), transformTab));
    inputY = new QLineEdit(transformTab);
    inputY->setPlaceholderText(QStringLiteral("0.0"));
    inputY->setMaximumWidth(80);
    inputLayout->addWidget(inputY);
    inputLayout->addWidget(new QLabel(QStringLiteral("Z:"), transformTab));
    inputZ = new QLineEdit(transformTab);
    inputZ->setPlaceholderText(QStringLiteral("0.0"));
    inputZ->setMaximumWidth(80);
    inputLayout->addWidget(inputZ);
    inputLayout->addStretch();
    singleLayout->addLayout(inputLayout);
    
    transformBtn = new QPushButton(QStringLiteral("转换"), transformTab);
    singleLayout->addWidget(transformBtn);
    
    QHBoxLayout* resultLayout = new QHBoxLayout;
    resultLayout->addWidget(new QLabel(QStringLiteral("转换结果:"), transformTab));
    resultLabel = new QLabel(QStringLiteral("(等待输入...)"), transformTab);
    resultLabel->setStyleSheet(QStringLiteral("font-weight: bold; color: blue;"));
    resultLayout->addWidget(resultLabel);
    resultLayout->addStretch();
    copyResultBtn = new QPushButton(QStringLiteral("复制结果"), transformTab);
    copyResultBtn->setEnabled(false);
    resultLayout->addWidget(copyResultBtn);
    singleLayout->addLayout(resultLayout);
    
    singleGroup->setLayout(singleLayout);
    transformLayout->addWidget(singleGroup);

    QGroupBox* batchGroup = new QGroupBox(QStringLiteral("批量转换"), transformTab);
    QVBoxLayout* batchLayout = new QVBoxLayout;
    
    QHBoxLayout* batchSelectLayout = new QHBoxLayout;
    batchSelectLayout->addWidget(new QLabel(QStringLiteral("选择要转换的分组:"), transformTab));
    batchGroupCombo = new QComboBox(transformTab);
    batchSelectLayout->addWidget(batchGroupCombo);
    batchSelectLayout->addStretch();
    batchLayout->addLayout(batchSelectLayout);
    
    QHBoxLayout* matrixSelectLayout = new QHBoxLayout;
    matrixSelectLayout->addWidget(new QLabel(QStringLiteral("选择转换矩阵:"), transformTab));
    useABRadio = new QRadioButton(QStringLiteral("源→目标 (A→B)"), transformTab);
    useABRadio->setChecked(true);
    useBARadio = new QRadioButton(QStringLiteral("目标→源 (B→A)"), transformTab);
    matrixSelectLayout->addWidget(useABRadio);
    matrixSelectLayout->addWidget(useBARadio);
    matrixSelectLayout->addStretch();
    batchLayout->addLayout(matrixSelectLayout);
    
    batchTransformBtn = new QPushButton(QStringLiteral("执行批量转换"), transformTab);
    batchLayout->addWidget(batchTransformBtn);
    
    batchResultTable = new QTableWidget(0, 7, transformTab);
    batchResultTable->setHorizontalHeaderLabels(QStringList() 
        << QStringLiteral("序号") 
        << QStringLiteral("原始X") << QStringLiteral("原始Y") << QStringLiteral("原始Z")
        << QStringLiteral("转换X") << QStringLiteral("转换Y") << QStringLiteral("转换Z"));
    batchResultTable->horizontalHeader()->setStretchLastSection(true);
    batchResultTable->setMaximumHeight(200);
    batchLayout->addWidget(batchResultTable);
    
    copyBatchBtn = new QPushButton(QStringLiteral("复制所有结果"), transformTab);
    copyBatchBtn->setEnabled(false);
    batchLayout->addWidget(copyBatchBtn);
    
    batchGroup->setLayout(batchLayout);
    transformLayout->addWidget(batchGroup);

    tabWidget->addTab(transformTab, QStringLiteral("坐标转换"));

    mainLayout->addWidget(tabWidget);

    QHBoxLayout* btnLayout = new QHBoxLayout;
    closeBtn = new QPushButton(QStringLiteral("关闭"), this);
    btnLayout->addStretch();
    btnLayout->addWidget(closeBtn);
    mainLayout->addLayout(btnLayout);

    sourceGroupCombo->clear();
    targetGroupCombo->clear();
    for (const auto& g : allGroups) {
        sourceGroupCombo->addItem(QString::fromStdString(g));
        targetGroupCombo->addItem(QString::fromStdString(g));
    }

    for (const auto& g : allGroups) {
        batchGroupCombo->addItem(QString::fromStdString(g));
    }

    connect(sourceGroupCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &TransformResultDialog::onGroupSelectionChanged);
    connect(targetGroupCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &TransformResultDialog::onGroupSelectionChanged);
    connect(copyABBtn, &QPushButton::clicked, this, &TransformResultDialog::onCopyAB);
    connect(copyBABtn, &QPushButton::clicked, this, &TransformResultDialog::onCopyBA);
    connect(recomputeBtn, &QPushButton::clicked, this, &TransformResultDialog::onRecompute);
    connect(closeBtn, &QPushButton::clicked, this, &QDialog::accept);
    connect(transformBtn, &QPushButton::clicked, this, &TransformResultDialog::onTransformPoint);
    connect(copyResultBtn, &QPushButton::clicked, this, &TransformResultDialog::onCopyResult);
    connect(batchTransformBtn, &QPushButton::clicked, this, &TransformResultDialog::onBatchTransform);
    connect(copyBatchBtn, &QPushButton::clicked, this, &TransformResultDialog::onCopyBatchResults);
}

TransformResultDialog::~TransformResultDialog() {}

std::string TransformResultDialog::getSelectedSourceGroup() const {
    return sourceGroupCombo->currentText().toStdString();
}

std::string TransformResultDialog::getSelectedTargetGroup() const {
    return targetGroupCombo->currentText().toStdString();
}

void TransformResultDialog::onGroupSelectionChanged() {
    QString source = sourceGroupCombo->currentText();
    QString target = targetGroupCombo->currentText();
    
    if (source.isEmpty() || target.isEmpty()) {
        statusLabel->setText(QStringLiteral("请选择源分组和目标分组"));
        return;
    }
    
    if (source == target) {
        statusLabel->setText(QStringLiteral("源分组和目标分组不能相同"));
        return;
    }
    
    bool pairExists = false;
    for (const auto& p : availablePairs) {
        if (p.first == source.toStdString() && p.second == target.toStdString()) {
            pairExists = true;
            break;
        }
    }
    
    if (!pairExists) {
        statusLabel->setText(QStringLiteral("错误: %1 和 %2 之间没有配对点").arg(source).arg(target));
    } else {
        statusLabel->setText(QStringLiteral("提示: 点击「计算」按钮计算 %1 → %2 的转换矩阵").arg(source).arg(target));
    }
}

void TransformResultDialog::updateMatrices(const Eigen::Matrix4d& matAB, const Eigen::Matrix4d& matBA) {
    currentMatAB = matAB;
    currentMatBA = matBA;
    displayMatrix(matrixABTable, matAB);
    displayMatrix(matrixBATable, matBA);
}

void TransformResultDialog::displayMatrix(QTableWidget* table, const Eigen::Matrix4d& mat) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            QTableWidgetItem* item = new QTableWidgetItem(QString::number(mat(i,j), 'f', 8));
            item->setTextAlignment(Qt::AlignCenter);
            table->setItem(i, j, item);
        }
    }
    for (int i = 0; i < 4; ++i) {
        table->setColumnWidth(i, 130);
        table->setRowHeight(i, 30);
    }
}

void TransformResultDialog::copyMatrixToClipboard(const Eigen::Matrix4d& mat) {
    QString text;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            text += QString::number(mat(i,j), 'f', 8);
            if (j < 3) text += "\t";
        }
        text += "\n";
    }
    QApplication::clipboard()->setText(text);
}

void TransformResultDialog::onCopyAB() {
    copyMatrixToClipboard(currentMatAB);
}

void TransformResultDialog::onCopyBA() {
    copyMatrixToClipboard(currentMatBA);
}

void TransformResultDialog::onRecompute() {
    QString source = sourceGroupCombo->currentText();
    QString target = targetGroupCombo->currentText();
    
    if (source.isEmpty() || target.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("警告"), QStringLiteral("请选择源分组和目标分组！"));
        return;
    }
    
    if (source == target) {
        QMessageBox::warning(this, QStringLiteral("警告"), QStringLiteral("源分组和目标分组不能相同！"));
        return;
    }
    
    emit recomputeRequested(source.toStdString(), target.toStdString());
}

Eigen::Vector3d TransformResultDialog::transformPoint(const Eigen::Vector3d& point, const Eigen::Matrix4d& mat) {
    Eigen::Vector4d homogenous(point.x(), point.y(), point.z(), 1.0);
    Eigen::Vector4d transformed = mat * homogenous;
    return Eigen::Vector3d(transformed.x(), transformed.y(), transformed.z());
}

void TransformResultDialog::onTransformPoint() {
    bool okX, okY, okZ;
    double x = inputX->text().toDouble(&okX);
    double y = inputY->text().toDouble(&okY);
    double z = inputZ->text().toDouble(&okZ);
    
    if (!okX || !okY || !okZ) {
        QMessageBox::warning(this, QStringLiteral("警告"), QStringLiteral("请输入有效的坐标值！"));
        return;
    }
    
    Eigen::Vector3d inputPoint(x, y, z);
    Eigen::Matrix4d matToUse = useABRadio->isChecked() ? currentMatAB : currentMatBA;
    Eigen::Vector3d result = transformPoint(inputPoint, matToUse);
    
    QString resultText = QStringLiteral("(%1, %2, %3)")
        .arg(result.x(), 0, 'f', 6)
        .arg(result.y(), 0, 'f', 6)
        .arg(result.z(), 0, 'f', 6);
    
    resultLabel->setText(resultText);
    copyResultBtn->setEnabled(true);
}

void TransformResultDialog::onCopyResult() {
    QString text = resultLabel->text();
    QApplication::clipboard()->setText(text);
}

void TransformResultDialog::onBatchTransform() {
    QString selectedGroup = batchGroupCombo->currentText();
    if (selectedGroup.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("警告"), QStringLiteral("请选择要转换的分组！"));
        return;
    }
    
    emit batchTransformRequested(selectedGroup.toStdString(), 
                                  targetGroupCombo->currentText().toStdString(),
                                  useABRadio->isChecked());
}

void TransformResultDialog::setBatchTransformResults(const std::vector<Point3D>& originalPoints, 
                                                      const std::vector<Point3D>& transformedPoints) {
    batchOriginalPoints = originalPoints;
    batchTransformedPoints = transformedPoints;
    
    batchResultTable->setRowCount(static_cast<int>(originalPoints.size()));
    
    for (size_t i = 0; i < originalPoints.size(); ++i) {
        batchResultTable->setItem(static_cast<int>(i), 0, 
            new QTableWidgetItem(QString::number(i + 1)));
        
        batchResultTable->setItem(static_cast<int>(i), 1, 
            new QTableWidgetItem(QString::number(originalPoints[i].x, 'f', 6)));
        batchResultTable->setItem(static_cast<int>(i), 2, 
            new QTableWidgetItem(QString::number(originalPoints[i].y, 'f', 6)));
        batchResultTable->setItem(static_cast<int>(i), 3, 
            new QTableWidgetItem(QString::number(originalPoints[i].z, 'f', 6)));
        
        if (i < transformedPoints.size()) {
            batchResultTable->setItem(static_cast<int>(i), 4, 
                new QTableWidgetItem(QString::number(transformedPoints[i].x, 'f', 6)));
            batchResultTable->setItem(static_cast<int>(i), 5, 
                new QTableWidgetItem(QString::number(transformedPoints[i].y, 'f', 6)));
            batchResultTable->setItem(static_cast<int>(i), 6, 
                new QTableWidgetItem(QString::number(transformedPoints[i].z, 'f', 6)));
        }
    }
    
    batchResultTable->resizeColumnsToContents();
    copyBatchBtn->setEnabled(true);
}

void TransformResultDialog::onCopyBatchResults() {
    QString text;
    text += QStringLiteral("序号\t原始X\t原始Y\t原始Z\t转换X\t转换Y\t转换Z\n");
    
    for (size_t i = 0; i < batchOriginalPoints.size(); ++i) {
        text += QString::number(i + 1) + "\t";
        text += QString::number(batchOriginalPoints[i].x, 'f', 6) + "\t";
        text += QString::number(batchOriginalPoints[i].y, 'f', 6) + "\t";
        text += QString::number(batchOriginalPoints[i].z, 'f', 6) + "\t";
        
        if (i < batchTransformedPoints.size()) {
            text += QString::number(batchTransformedPoints[i].x, 'f', 6) + "\t";
            text += QString::number(batchTransformedPoints[i].y, 'f', 6) + "\t";
            text += QString::number(batchTransformedPoints[i].z, 'f', 6);
        }
        text += "\n";
    }
    
    QApplication::clipboard()->setText(text);
}
