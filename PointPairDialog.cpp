#include "PointPairDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QMessageBox>
#include <QInputDialog>

PointPairDialog::PointPairDialog(const std::vector<std::string>& groups,
                                 const std::vector<Point3D>& points,
                                 QWidget* parent)
    : QDialog(parent), groups(groups), allPoints(points)
{
    setWindowTitle(QStringLiteral("点配对设置"));
    
    setMinimumSize(700, 500);
    resize(800, 550);

    QGroupBox* sourceGroup = new QGroupBox(QStringLiteral("源点选择"), this);
    QGroupBox* targetGroup = new QGroupBox(QStringLiteral("目标点选择"), this);
    QGroupBox* pairGroup = new QGroupBox(QStringLiteral("已配对"), this);

    sourceGroupCombo = new QComboBox(this);
    targetGroupCombo = new QComboBox(this);
    sourceList = new QListWidget(this);
    targetList = new QListWidget(this);
    pairList = new QListWidget(this);

    sourceList->setSelectionMode(QListWidget::SingleSelection);
    targetList->setSelectionMode(QListWidget::SingleSelection);

    addPairBtn = new QPushButton(QStringLiteral("添加配对→"), this);
    removePairBtn = new QPushButton(QStringLiteral("← 删除配对"), this);
    okBtn = new QPushButton(QStringLiteral("确定"), this);
    cancelBtn = new QPushButton(QStringLiteral("取消"), this);

    for (const auto& g : groups) {
        sourceGroupCombo->addItem(QString::fromStdString(g));
        targetGroupCombo->addItem(QString::fromStdString(g));
    }

    QHBoxLayout* sourceLayout = new QHBoxLayout;
    sourceLayout->addWidget(new QLabel(QStringLiteral("分组:"), this));
    sourceLayout->addWidget(sourceGroupCombo);
    sourceLayout->addWidget(sourceList);

    QVBoxLayout* sourceGroupLayout = new QVBoxLayout;
    sourceGroupLayout->addLayout(sourceLayout);
    sourceGroup->setLayout(sourceGroupLayout);

    QHBoxLayout* targetLayout = new QHBoxLayout;
    targetLayout->addWidget(new QLabel(QStringLiteral("分组:"), this));
    targetLayout->addWidget(targetGroupCombo);
    targetLayout->addWidget(targetList);

    QVBoxLayout* targetGroupLayout = new QVBoxLayout;
    targetGroupLayout->addLayout(targetLayout);
    targetGroup->setLayout(targetGroupLayout);

    QHBoxLayout* pairBtnLayout = new QHBoxLayout;
    pairBtnLayout->addWidget(addPairBtn);
    pairBtnLayout->addWidget(removePairBtn);

    QVBoxLayout* pairGroupLayout = new QVBoxLayout;
    pairGroupLayout->addWidget(pairList);
    pairGroupLayout->addLayout(pairBtnLayout);
    pairGroup->setLayout(pairGroupLayout);

    QHBoxLayout* bottomLayout = new QHBoxLayout;
    bottomLayout->addStretch();
    bottomLayout->addWidget(okBtn);
    bottomLayout->addWidget(cancelBtn);

    QHBoxLayout* middleLayout = new QHBoxLayout;
    middleLayout->addWidget(sourceGroup);
    middleLayout->addWidget(targetGroup);
    middleLayout->addWidget(pairGroup);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addLayout(middleLayout);
    mainLayout->addLayout(bottomLayout);
    setLayout(mainLayout);

    connect(sourceGroupCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &PointPairDialog::onSourceGroupChanged);
    connect(targetGroupCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &PointPairDialog::onTargetGroupChanged);
    connect(addPairBtn, &QPushButton::clicked, this, &PointPairDialog::onAddPair);
    connect(removePairBtn, &QPushButton::clicked, this, &PointPairDialog::onRemovePair);
    connect(okBtn, &QPushButton::clicked, this, &QDialog::accept);
    connect(cancelBtn, &QPushButton::clicked, this, &QDialog::reject);

    updateSourceList();
    updateTargetList();
}

PointPairDialog::~PointPairDialog() {}

void PointPairDialog::onSourceGroupChanged(int index) {
    Q_UNUSED(index);
    updateSourceList();
}

void PointPairDialog::onTargetGroupChanged(int index) {
    Q_UNUSED(index);
    updateTargetList();
}

void PointPairDialog::updateSourceList() {
    sourceList->clear();
    QString currentGroup = sourceGroupCombo->currentText();

    for (const auto& p : allPoints) {
        if (QString::fromStdString(p.groupName) == currentGroup) {
            QString itemText = QString("[%1] (%2, %3, %4)")
                .arg(QString::fromStdString(p.id))
                .arg(p.x, 0, 'f', 3)
                .arg(p.y, 0, 'f', 3)
                .arg(p.z, 0, 'f', 3);
            QListWidgetItem* item = new QListWidgetItem(itemText, sourceList);
            item->setData(Qt::UserRole, QString::fromStdString(p.id));
        }
    }
}

void PointPairDialog::updateTargetList() {
    targetList->clear();
    QString currentGroup = targetGroupCombo->currentText();

    for (const auto& p : allPoints) {
        if (QString::fromStdString(p.groupName) == currentGroup) {
            QString itemText = QStringLiteral("[%1] (%2, %3, %4)")
                .arg(QString::fromStdString(p.id))
                .arg(p.x, 0, 'f', 3)
                .arg(p.y, 0, 'f', 3)
                .arg(p.z, 0, 'f', 3);
            QListWidgetItem* item = new QListWidgetItem(itemText, targetList);
            item->setData(Qt::UserRole, QString::fromStdString(p.id));
        }
    }
}

void PointPairDialog::onAddPair() {
    QListWidgetItem* sourceItem = sourceList->currentItem();
    QListWidgetItem* targetItem = targetList->currentItem();

    if (!sourceItem || !targetItem) {
        QMessageBox::warning(this, QStringLiteral("警告"), QStringLiteral("请先在左侧和右侧各选择一个点！"));
        return;
    }

    QString sourceId = sourceItem->data(Qt::UserRole).toString();
    QString targetId = targetItem->data(Qt::UserRole).toString();
    QString sourceGroup = sourceGroupCombo->currentText();
    QString targetGroup = targetGroupCombo->currentText();

    PointPair pair;
    pair.sourceGroup = sourceGroup.toStdString();
    pair.targetGroup = targetGroup.toStdString();
    pair.sourceId = sourceId.toStdString();
    pair.targetId = targetId.toStdString();

    for (const auto& p : allPoints) {
        if (QString::fromStdString(p.groupName) == sourceGroup &&
            QString::fromStdString(p.id) == sourceId) {
            pair.sourcePoint = p;
            break;
        }
    }

    for (const auto& p : allPoints) {
        if (QString::fromStdString(p.groupName) == targetGroup &&
            QString::fromStdString(p.id) == targetId) {
            pair.targetPoint = p;
            break;
        }
    }

    pairs.push_back(pair);

    QString pairText = QString("%1[%2] ↔ %3[%4]")
        .arg(sourceGroup).arg(sourceId)
        .arg(targetGroup).arg(targetId);
    new QListWidgetItem(pairText, pairList);

    sourceList->clearSelection();
    targetList->clearSelection();
}

void PointPairDialog::onRemovePair() {
    int currentRow = pairList->currentRow();
    if (currentRow >= 0) {
        pairList->takeItem(currentRow);
        pairs.erase(pairs.begin() + currentRow);
    }
}
