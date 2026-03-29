#pragma once

#include <QDialog>
#include <QWidget>
#include <QLineEdit>
#include <QTextEdit>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QColorDialog>
#include <vector>

class PointCloudInputDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PointCloudInputDialog(QWidget* parent = nullptr);
    ~PointCloudInputDialog();

    struct PointData {
        std::vector<std::vector<float>> points;
        int r, g, b;
        QString groupName;
    };

    PointData getPointData() const;

private slots:
    void onColorButtonClicked();
    void onOkClicked();
    void onCancelClicked();

private:
    void setupUI();

    QLineEdit* groupNameEdit;
    QTextEdit* pointsTextEdit;
    QPushButton* colorButton;
    QLabel* colorPreview;
    
    int currentR, currentG, currentB;
};
