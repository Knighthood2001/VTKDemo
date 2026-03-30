#include "PointPairDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QMessageBox>

PointPairDialog::PointPairDialog(const std::vector<std::string>& groups,
                                 const std::vector<Point3D>& points,
                                 QWidget* parent)
    : QDialog(parent), groups(groups), allPoints(points)
{
    setWindowTitle(QStringLiteral("点配对设置"));
    setMinimumSize(900, 600);
    resize(950, 650);

    QGroupBox* sourceGroup = new QGroupBox(QStringLiteral("源点"), this);
    QGroupBox* targetGroup = new QGroupBox(QStringLiteral("目标点"), this);
    QGroupBox* pairGroup = new QGroupBox(QStringLiteral("已配对列表"), this);

    sourceGroupCombo = new QComboBox(this);
    targetGroupCombo = new QComboBox(this);
    sourceList = new QListWidget(this);
    targetList = new QListWidget(this);
    pairList = new QListWidget(this);

    sourceList->setSelectionMode(QListWidget::SingleSelection);
    targetList->setSelectionMode(QListWidget::SingleSelection);

    sourceList->setMinimumHeight(200);
    targetList->setMinimumHeight(200);
    pairList->setMinimumHeight(200);

    addPairBtn = new QPushButton(QStringLiteral("添加配对"), this);
    addPairBtn->setMinimumWidth(100);
    removePairBtn = new QPushButton(QStringLiteral("删除选中"), this);
    removePairBtn->setMinimumWidth(100);
    okBtn = new QPushButton(QStringLiteral("确定"), this);
    okBtn->setMinimumWidth(80);
    cancelBtn = new QPushButton(QStringLiteral("取消"), this);
    cancelBtn->setMinimumWidth(80);

    for (const auto& g : groups) {
        sourceGroupCombo->addItem(QString::fromStdString(g));
        targetGroupCombo->addItem(QString::fromStdString(g));
    }

    QVBoxLayout* sourceLayout = new QVBoxLayout;
    sourceLayout->addWidget(new QLabel(QStringLiteral("选择分组:"), this));
    sourceLayout->addWidget(sourceGroupCombo);
    sourceLayout->addWidget(sourceList, 1);
    sourceGroup->setLayout(sourceLayout);

    QVBoxLayout* targetLayout = new QVBoxLayout;
    targetLayout->addWidget(new QLabel(QStringLiteral("选择分组:"), this));
    targetLayout->addWidget(targetGroupCombo);
    targetLayout->addWidget(targetList, 1);
    targetGroup->setLayout(targetLayout);

    QVBoxLayout* pairLayout = new QVBoxLayout;
    pairLayout->addWidget(pairList, 1);
    QHBoxLayout* pairBtnLayout = new QHBoxLayout;
    pairBtnLayout->addWidget(addPairBtn);
    pairBtnLayout->addWidget(removePairBtn);
    pairLayout->addLayout(pairBtnLayout);
    pairGroup->setLayout(pairLayout);

    QHBoxLayout* middleLayout = new QHBoxLayout;
    middleLayout->addWidget(sourceGroup, 1);
    middleLayout->addWidget(targetGroup, 1);
    middleLayout->addWidget(pairGroup, 1);

    QHBoxLayout* bottomLayout = new QHBoxLayout;
    bottomLayout->addStretch();
    bottomLayout->addWidget(okBtn);
    bottomLayout->addWidget(cancelBtn);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addLayout(middleLayout);
    mainLayout->addSpacing(15);
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
            QString itemText = QStringLiteral("[%1] (%2, %3, %4)")
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
        QMessageBox::warning(this, QStringLiteral("警告"),
            QStringLiteral("请先在左侧和右侧各选择一个点！"));
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

    QString pairText = QStringLiteral("%1[%2] <-> %3[%4]")
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
