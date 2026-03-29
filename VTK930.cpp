#include "VTK930.h"
#include <QFileDialog>
#include <QDebug>
#include <QMessageBox>
#include "PointCloudInputDialog.h"

VTK930::VTK930(QWidget* parent) : QMainWindow(parent) {
    ui.setupUi(this);
    initialVtkWidget();

    connect(ui.actionopen, SIGNAL(triggered()), this, SLOT(onOpen()));
    connect(ui.actionload, SIGNAL(triggered()), this, SLOT(onLoad()));
}

VTK930::~VTK930() {}

void VTK930::onOpen() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".",
        "Open PCD files(*.pcd)");
    if (fileName == "") return;
    pcl::io::loadPCDFile(fileName.toStdString(), *cloud);
    view->addPointCloud(cloud, "cloud");
    view->resetCamera();
    view->spin();
    ui.openGLWidget->update();
}

void VTK930::onLoad() {
    PointCloudInputDialog dialog(this);
    if (dialog.exec() == QDialog::Accepted) {
        PointCloudInputDialog::PointData data = dialog.getPointData();
        
        std::vector<Point3D> points;
        for (const auto& p : data.points) {
            points.push_back({p[0], p[1], p[2]});
        }
        
        if (!points.empty()) {
            addPointsToVisualizer(points, data.r, data.g, data.b, data.groupName.toStdString());
            view->resetCamera();
            view->spin();
            ui.openGLWidget->update();
            
            QMessageBox::information(this, "Success",
                QString("Loaded %1 points into group '%2'").arg(points.size()).arg(data.groupName));
        }
    }
}

void VTK930::addPointsToVisualizer(const std::vector<Point3D>& points, int r, int g, int b, const std::string& groupName) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    if (groupClouds.find(groupName) == groupClouds.end()) {
        cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        groupClouds[groupName] = cloud;
    } else {
        cloud = groupClouds[groupName];
    }

    for (const auto& p : points) {
        pcl::PointXYZRGB point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.r = r;
        point.g = g;
        point.b = b;
        cloud->points.push_back(point);
    }

    view->removePointCloud(groupName);
    view->addPointCloud(cloud, groupName);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
        (double)r/255.0, (double)g/255.0, (double)b/255.0, groupName);
}

void VTK930::initialVtkWidget() {
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow =
        vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    view.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow,
        "viewer", false));
    view->setBackgroundColor(0.2, 0.2, 0.2);

    view->setupInteractor(ui.openGLWidget->interactor(),
        ui.openGLWidget->renderWindow());
    ui.openGLWidget->setRenderWindow(view->getRenderWindow());
}
