#pragma once

#include <QDialog>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>

class GroupNameDialog : public QDialog {
    Q_OBJECT

public:
    explicit GroupNameDialog(const QString& suggestedName, QWidget* parent = nullptr);
    ~GroupNameDialog();

    QString getGroupName() const;

private slots:
    void onConfirm();
    void onCancel();

private:
    QLineEdit* nameEdit;
    QPushButton* confirmBtn;
    QPushButton* cancelBtn;
};
