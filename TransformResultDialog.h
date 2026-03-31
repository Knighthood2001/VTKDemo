#pragma once

#include <QDialog>
#include <QTableWidget>
#include <QPushButton>
#include <QLabel>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <Eigen/Dense>

class TransformResultDialog : public QDialog {
    Q_OBJECT

public:
    explicit TransformResultDialog(const Eigen::Matrix4d& matAB, 
                                 const Eigen::Matrix4d& matBA,
                                 const std::string& group1,
                                 const std::string& group2,
                                 QWidget* parent = nullptr);
    ~TransformResultDialog();

private slots:
    void onCopyAB();
    void onCopyBA();
    void onRecompute();

private:
    void updateMatrices(const Eigen::Matrix4d& matAB, const Eigen::Matrix4d& matBA);
    void copyMatrixToClipboard(const Eigen::Matrix4d& mat);
    void displayMatrix(QTableWidget* table, const Eigen::Matrix4d& mat);

    QTableWidget* matrixABTable;
    QTableWidget* matrixBATable;
    QPushButton* copyABBtn;
    QPushButton* copyBABtn;
    QPushButton* recomputeBtn;
    QPushButton* closeBtn;

    Eigen::Matrix4d currentMatAB;
    Eigen::Matrix4d currentMatBA;
};
