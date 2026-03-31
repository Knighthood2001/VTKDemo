#pragma once

#include <QDialog>
#include <QTableWidget>
#include <QPushButton>
#include <QLabel>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <Eigen/Dense>

class TransformResultDialog : public QDialog {
    Q_OBJECT

public:
    explicit TransformResultDialog(const std::vector<std::string>& groups,
                                 const std::vector<std::pair<std::string, std::string>>& availablePairs,
                                 QWidget* parent = nullptr);
    ~TransformResultDialog();

    std::string getSelectedSourceGroup() const;
    std::string getSelectedTargetGroup() const;

signals:
    void recomputeRequested(const std::string& sourceGroup, const std::string& targetGroup);

private slots:
    void onCopyAB();
    void onCopyBA();
    void onRecompute();
    void onGroupSelectionChanged();

private:
    void copyMatrixToClipboard(const Eigen::Matrix4d& mat);
    void displayMatrix(QTableWidget* table, const Eigen::Matrix4d& mat);
    void updateGroupComboBoxes();

public:
    void updateMatrices(const Eigen::Matrix4d& matAB, const Eigen::Matrix4d& matBA);

    QTableWidget* matrixABTable;
    QTableWidget* matrixBATable;
    QPushButton* copyABBtn;
    QPushButton* copyBABtn;
    QPushButton* recomputeBtn;
    QPushButton* closeBtn;
    QComboBox* sourceGroupCombo;
    QComboBox* targetGroupCombo;
    QLabel* statusLabel;

    Eigen::Matrix4d currentMatAB;
    Eigen::Matrix4d currentMatBA;

    std::vector<std::string> allGroups;
    std::vector<std::pair<std::string, std::string>> availablePairs;
};
