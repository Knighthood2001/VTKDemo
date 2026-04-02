#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_VTK930.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>
#include <QVTKRenderWidget.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkSmartPointer.h>
#include <map>
#include <string>
#include <vector>
#include <QTimer>
#include <vtkCamera.h>
#include <vtkRenderer.h>

class PointCloudInputDialog;
#include "PointPairDialog.h"
#include "TransformResultDialog.h"

struct PointData {
    std::string id;
    float x, y, z;
    std::string groupName;
    int colorR, colorG, colorB;
};

struct TransformationMatrix {
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    Eigen::Matrix4d toMatrix() const {
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        m.block<3,3>(0,0) = rotation;
        m.block<3,1>(0,3) = translation;
        return m;
    }
};

class VTK930 : public QMainWindow
{
    Q_OBJECT

public:
    VTK930(QWidget *parent = nullptr);
    ~VTK930();

private:
    Ui::VTK930 ui;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view;
    std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> groupClouds;
    std::vector<PointData> allPoints;
    std::vector<PointPair> pointPairs;
    vtkSmartPointer<vtkOrientationMarkerWidget> markerWidget_;
    void initialVtkWidget();
    void addPointsToVisualizer(const std::vector<Point3D>& points, int r, int g, int b, const std::string& groupName);
    void updatePointTable();
    void updateGroupList();
    void updateFilterCombo();
    void updatePairList();
    TransformationMatrix computeTransformAB();
    TransformationMatrix computeTransformBA();
    void computeTransformBetween(const std::string& sourceGroup, const std::string& targetGroup, 
                                 TransformationMatrix& matAB, TransformationMatrix& matBA);
    std::vector<std::pair<std::string, std::string>> getAvailablePairs() const;
    void onViewTransform();
    void onRecomputeTransform(const std::string& sourceGroup, const std::string& targetGroup);
    void onBatchTransform(const std::string& groupName, const std::string& targetGroup, bool useABMatrix);

private slots:
    void onOpen();
    void onLoad();
    void onAddGroup();
    void onDeleteGroup();
    void onAddPoint();
    void onDeletePoint();
    void onClearPoints();
    void onFilterChanged(int index);
    void initOrientationMarker();
    void onAddPair();
    void onDeletePair();
};
