#include "GroupNameDialog.h"
#include <QMessageBox>

GroupNameDialog::GroupNameDialog(const QString& suggestedName, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QStringLiteral("设置分组名称"));
    setMinimumSize(350, 150);
    resize(400, 180);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QLabel* infoLabel = new QLabel(QStringLiteral("请输入新分组的名称:"), this);
    mainLayout->addWidget(infoLabel);

    QFormLayout* formLayout = new QFormLayout;
    nameEdit = new QLineEdit(suggestedName, this);
    formLayout->addRow(QStringLiteral("分组名称:"), nameEdit);
    mainLayout->addLayout(formLayout);

    QHBoxLayout* btnLayout = new QHBoxLayout;
    confirmBtn = new QPushButton(QStringLiteral("确认"), this);
    cancelBtn = new QPushButton(QStringLiteral("取消"), this);
    btnLayout->addStretch();
    btnLayout->addWidget(confirmBtn);
    btnLayout->addWidget(cancelBtn);
    mainLayout->addLayout(btnLayout);

    connect(confirmBtn, &QPushButton::clicked, this, &GroupNameDialog::onConfirm);
    connect(cancelBtn, &QPushButton::clicked, this, &GroupNameDialog::onCancel);
}

GroupNameDialog::~GroupNameDialog() {}

QString GroupNameDialog::getGroupName() const {
    return nameEdit->text().trimmed();
}

void GroupNameDialog::onConfirm() {
    if (nameEdit->text().trimmed().isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("警告"), QStringLiteral("分组名称不能为空！"));
        return;
    }
    accept();
}

void GroupNameDialog::onCancel() {
    reject();
}
