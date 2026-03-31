#include "TransformResultDialog.h"
#include <QClipboard>
#include <QApplication>
#include <QHeaderView>
#include <QMessageBox>

TransformResultDialog::TransformResultDialog(const std::vector<std::string>& groups,
                                             const std::vector<std::pair<std::string, std::string>>& pairs,
                                             QWidget* parent)
    : QDialog(parent), allGroups(groups), availablePairs(pairs)
{
    setWindowTitle(QStringLiteral("转换矩阵计算结果"));
    setMinimumSize(600, 750);
    resize(650, 800);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QGroupBox* selectGroup = new QGroupBox(QStringLiteral("选择分组"), this);
    QHBoxLayout* selectLayout = new QHBoxLayout;
    
    selectLayout->addWidget(new QLabel(QStringLiteral("源分组:"), this));
    sourceGroupCombo = new QComboBox(this);
    selectLayout->addWidget(sourceGroupCombo);
    
    selectLayout->addWidget(new QLabel(QStringLiteral("目标分组:"), this));
    targetGroupCombo = new QComboBox(this);
    selectLayout->addWidget(targetGroupCombo);
    
    selectGroup->setLayout(selectLayout);
    mainLayout->addWidget(selectGroup);

    statusLabel = new QLabel(this);
    mainLayout->addWidget(statusLabel);

    QString title1 = QStringLiteral("转换矩阵");
    QGroupBox* abGroup = new QGroupBox(title1, this);
    QVBoxLayout* abLayout = new QVBoxLayout;
    matrixABTable = new QTableWidget(4, 4, this);
    matrixABTable->horizontalHeader()->setVisible(false);
    matrixABTable->verticalHeader()->setVisible(false);
    abLayout->addWidget(matrixABTable);
    copyABBtn = new QPushButton(QStringLiteral("复制矩阵"), this);
    abLayout->addWidget(copyABBtn);
    abGroup->setLayout(abLayout);

    QGroupBox* baGroup = new QGroupBox(title1, this);
    QVBoxLayout* baLayout = new QVBoxLayout;
    matrixBATable = new QTableWidget(4, 4, this);
    matrixBATable->horizontalHeader()->setVisible(false);
    matrixBATable->verticalHeader()->setVisible(false);
    baLayout->addWidget(matrixBATable);
    copyBABtn = new QPushButton(QStringLiteral("复制矩阵"), this);
    baLayout->addWidget(copyBABtn);
    baGroup->setLayout(baLayout);

    mainLayout->addWidget(abGroup);
    mainLayout->addWidget(baGroup);

    QHBoxLayout* btnLayout = new QHBoxLayout;
    recomputeBtn = new QPushButton(QStringLiteral("计算"), this);
    closeBtn = new QPushButton(QStringLiteral("关闭"), this);
    btnLayout->addStretch();
    btnLayout->addWidget(recomputeBtn);
    btnLayout->addWidget(closeBtn);
    mainLayout->addLayout(btnLayout);

    updateGroupComboBoxes();

    connect(sourceGroupCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &TransformResultDialog::onGroupSelectionChanged);
    connect(targetGroupCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &TransformResultDialog::onGroupSelectionChanged);
    connect(copyABBtn, &QPushButton::clicked, this, &TransformResultDialog::onCopyAB);
    connect(copyBABtn, &QPushButton::clicked, this, &TransformResultDialog::onCopyBA);
    connect(recomputeBtn, &QPushButton::clicked, this, &TransformResultDialog::onRecompute);
    connect(closeBtn, &QPushButton::clicked, this, &QDialog::accept);
}

TransformResultDialog::~TransformResultDialog() {}

void TransformResultDialog::updateGroupComboBoxes() {
    sourceGroupCombo->blockSignals(true);
    targetGroupCombo->blockSignals(true);
    
    sourceGroupCombo->clear();
    targetGroupCombo->clear();
    
    for (const auto& g : allGroups) {
        sourceGroupCombo->addItem(QString::fromStdString(g));
        targetGroupCombo->addItem(QString::fromStdString(g));
    }
    
    sourceGroupCombo->blockSignals(false);
    targetGroupCombo->blockSignals(false);
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

std::string TransformResultDialog::getSelectedSourceGroup() const {
    return sourceGroupCombo->currentText().toStdString();
}

std::string TransformResultDialog::getSelectedTargetGroup() const {
    return targetGroupCombo->currentText().toStdString();
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
