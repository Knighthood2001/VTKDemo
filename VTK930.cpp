#include "VTK930.h"
#include <QFileDialog>
#include <QDebug>

VTK930::VTK930(QWidget* parent) : QMainWindow(parent) {
    ui.setupUi(this);

    initialVtkWidget();

    connect(ui.actionopen, SIGNAL(triggered()), this, SLOT(onOpen()));
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

void VTK930::initialVtkWidget() {
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow =
        vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    view.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow,
        "viewer", false));
    // 参数为 R, G, B (取值范围 0.0 ~ 1.0)，浅灰色
    view->setBackgroundColor(0.2, 0.2, 0.2);
    // 如果需要设置渐变背景（上半部分+下半部分），可以用：
    // view->setBackgroundColor(0.1, 0.1, 0.1, 0.2, 0.2, 0.2);
    view->setupInteractor(ui.openGLWidget->interactor(),
        ui.openGLWidget->renderWindow());
    ui.openGLWidget->setRenderWindow(view->getRenderWindow());
}
