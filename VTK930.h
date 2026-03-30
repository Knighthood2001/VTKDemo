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

struct Point3D {
    float x, y, z;
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
    vtkSmartPointer<vtkOrientationMarkerWidget> markerWidget_;
    void initialVtkWidget();
    void addPointsToVisualizer(const std::vector<Point3D>& points, int r, int g, int b, const std::string& groupName);

private slots:
    void onOpen();
    void onLoad();
    void initOrientationMarker(); // 延迟初始化方向轴
};
