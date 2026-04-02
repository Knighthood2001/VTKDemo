#pragma once

#include <QDialog>
#include <QTableWidget>
#include <QPushButton>
#include <QLabel>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QLineEdit>
#include <QRadioButton>
#include <Eigen/Dense>
#include "PointPairDialog.h"

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
    void batchTransformRequested(const std::string& sourceGroup, const std::string& targetGroup, bool useABMatrix);

public slots:
    void setBatchTransformResults(const std::vector<Point3D>& originalPoints, const std::vector<Point3D>& transformedPoints);

private slots:
    void onCopyAB();
    void onCopyBA();
    void onRecompute();
    void onGroupSelectionChanged();
    void onTransformPoint();
    void onCopyResult();
    void onBatchTransform();
    void onCopyBatchResults();

private:
    void copyMatrixToClipboard(const Eigen::Matrix4d& mat);
    void displayMatrix(QTableWidget* table, const Eigen::Matrix4d& mat);
    Eigen::Vector3d transformPoint(const Eigen::Vector3d& point, const Eigen::Matrix4d& mat);

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

    QLineEdit* inputX;
    QLineEdit* inputY;
    QLineEdit* inputZ;
    QPushButton* transformBtn;
    QLabel* resultLabel;
    QPushButton* copyResultBtn;

    QComboBox* batchGroupCombo;
    QRadioButton* useABRadio;
    QRadioButton* useBARadio;
    QPushButton* batchTransformBtn;
    QTableWidget* batchResultTable;
    QPushButton* copyBatchBtn;

    Eigen::Matrix4d currentMatAB;
    Eigen::Matrix4d currentMatBA;

    std::vector<std::string> allGroups;
    std::vector<std::pair<std::string, std::string>> availablePairs;
    std::vector<Point3D> batchOriginalPoints;
    std::vector<Point3D> batchTransformedPoints;
};
