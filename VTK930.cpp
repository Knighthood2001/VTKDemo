#include "VTK930.h"
#include <QFileDialog>
#include <QDebug>
#include <QMessageBox>
#include "PointCloudInputDialog.h"
// 补充必要头文件（如果没加）
#include <vtkAnnotatedCubeActor.h>
#include <vtkTextProperty.h>

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

    view->removePointCloud("cloud");
    view->addPointCloud(cloud, "cloud");
    view->resetCamera();
    // ✅ 删掉：view->spin();  致命错误！
    ui.openGLWidget->update();
}

void VTK930::onLoad() {
    PointCloudInputDialog dialog(this);
    if (dialog.exec() == QDialog::Accepted) {
        PointCloudInputDialog::PointData data = dialog.getPointData();

        std::vector<Point3D> points;
        for (const auto& p : data.points) {
            points.push_back({ p[0], p[1], p[2] });
        }

        if (!points.empty()) {
            addPointsToVisualizer(points, data.r, data.g, data.b, data.groupName.toStdString());
            view->resetCamera();
            // ✅ 删掉：view->spin();  致命错误！
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
    }
    else {
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
        (double)r / 255.0, (double)g / 255.0, (double)b / 255.0, groupName);
}

//void VTK930::initialVtkWidget() {
//    // 1. 初始化VTK渲染核心
//    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
//    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow =
//        vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
//    renderWindow->AddRenderer(renderer);
//
//    // 2. 绑定PCL可视化器
//    view.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow,
//        "viewer", false));
//    view->setBackgroundColor(0.2, 0.2, 0.2);
//
//    // 3. 绑定Qt OpenGL Widget（固定顺序）
//    view->setupInteractor(ui.openGLWidget->interactor(), ui.openGLWidget->renderWindow());
//    ui.openGLWidget->setRenderWindow(view->getRenderWindow());
//
//    // ==============================================
//    // 【核心修复】VTK 9.3 兼容：主视图坐标轴（中心显示）
//    // ==============================================
//    vtkSmartPointer<vtkAxesActor> axesActor = vtkSmartPointer<vtkAxesActor>::New();
//    axesActor->SetTotalLength(1.0, 1.0, 1.0);  // 坐标轴长度
//    axesActor->SetCylinderRadius(0.02);        // 轴粗细
//    axesActor->SetConeRadius(0.1);             // 箭头大小
//    renderer->AddActor(axesActor);             // 直接添加到渲染器
//
//    // 直接用 vtkAxesActor 做方向标记，兼容所有VTK9+版本，无API错误
//    vtkSmartPointer<vtkAxesActor> orientationAxes = vtkSmartPointer<vtkAxesActor>::New();
//    orientationAxes->SetTotalLength(1, 1, 1);
//
//    // VTK9.3 专用方向标组件
//    vtkSmartPointer<vtkOrientationMarkerWidget> orientationWidget =
//        vtkSmartPointer<vtkOrientationMarkerWidget>::New();
//    orientationWidget->SetOrientationMarker(orientationAxes);
//    orientationWidget->SetInteractor(ui.openGLWidget->interactor());
//    orientationWidget->SetViewport(0.0, 0.0, 0.2, 0.2); // 左下角位置
//    orientationWidget->SetEnabled(true);
//    orientationWidget->InteractiveOn();
//
//    // 4. 渲染刷新
//    view->resetCamera();
//    ui.openGLWidget->update();
//}
void VTK930::initialVtkWidget()
{
    // 1. 基础渲染初始化（你的原有代码不变）
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    view.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    view->setBackgroundColor(0.2, 0.2, 0.2);

    view->setupInteractor(ui.openGLWidget->interactor(), ui.openGLWidget->renderWindow());
    ui.openGLWidget->setRenderWindow(view->getRenderWindow());

    // 🔥 关键：延迟初始化（解决Qt+VTK坐标系消失问题）
    QTimer::singleShot(50, this, &VTK930::initOrientationMarker);

    view->resetCamera();
    ui.openGLWidget->update();
}

// 🔥 专门实现：左下角固定坐标轴 + 仅旋转 + 不平移不缩放
void VTK930::initOrientationMarker()
{
    auto axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetTotalLength(1.0, 1.0, 1.0);
    axes->SetCylinderRadius(0.02);
    axes->SetAxisLabels(0);

    markerWidget_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();

    markerWidget_->SetOrientationMarker(axes);

    // 🔥 关键1：绑定 renderer
    auto renderer = view->getRenderWindow()->GetRenderers()->GetFirstRenderer();
    markerWidget_->SetDefaultRenderer(renderer);

    // 🔥 关键2：正确 interactor
    markerWidget_->SetInteractor(ui.openGLWidget->renderWindow()->GetInteractor());

    // 🔥 关键3：viewport
    markerWidget_->SetViewport(0.0, 0.0, 0.2, 0.2);

    markerWidget_->SetEnabled(true);
    markerWidget_->InteractiveOn();

    ui.openGLWidget->renderWindow()->Render();
}