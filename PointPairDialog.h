#pragma once

#include <QDialog>
#include <QComboBox>
#include <QListWidget>
#include <QPushButton>
#include <QLabel>
#include <vector>
#include <string>

struct Point3D {
    float x, y, z;
    std::string id;
    std::string groupName;
};

struct PointPair {
    std::string sourceGroup;
    std::string targetGroup;
    std::string sourceId;
    std::string targetId;
    Point3D sourcePoint;
    Point3D targetPoint;
};

class PointPairDialog : public QDialog {
    Q_OBJECT

public:
    explicit PointPairDialog(const std::vector<std::string>& groups,
                             const std::vector<Point3D>& points,
                             QWidget* parent = nullptr);
    ~PointPairDialog();

    std::vector<PointPair> getPairs() const { return pairs; }

private slots:
    void onSourceGroupChanged(int index);
    void onTargetGroupChanged(int index);
    void onAddPair();
    void onRemovePair();

private:
    void updateSourceList();
    void updateTargetList();

    QComboBox* sourceGroupCombo;
    QComboBox* targetGroupCombo;
    QListWidget* sourceList;
    QListWidget* targetList;
    QListWidget* pairList;
    QPushButton* addPairBtn;
    QPushButton* removePairBtn;
    QPushButton* okBtn;
    QPushButton* cancelBtn;

    std::vector<std::string> groups;
    std::vector<Point3D> allPoints;
    std::vector<PointPair> pairs;

    QString selectedSourceId;
    QString selectedTargetId;
};
