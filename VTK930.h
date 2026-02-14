#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_VTK930.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>
#include <QVTKRenderWidget.h>
#include <QVTKOpenGLNativeWidget.h>
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

    void initialVtkWidget();

private slots:
    void onOpen();

};

