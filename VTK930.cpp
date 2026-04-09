#include "VTK930.h"
#include <QFileDialog>
#include <QDebug>
#include <QMessageBox>
#include <QInputDialog>
#include <QSplitter>
#include <QLayout>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QTextStream>
#include <QScrollBar>
#include <limits>
#include <algorithm>
#include "PointCloudInputDialog.h"
#include "PointPairDialog.h"
#include "TransformResultDialog.h"
#include "GroupNameDialog.h"
#include <vtkAnnotatedCubeActor.h>
#include <vtkTextProperty.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <Eigen/SVD>

VTK930::VTK930(QWidget* parent) : QMainWindow(parent), pointSize(2), currentProjectFile("") {
    ui.setupUi(this);
    initialVtkWidget();

    QSplitter* leftSplitter = new QSplitter(Qt::Vertical);
    leftSplitter->addWidget(ui.groupBox);
    leftSplitter->addWidget(ui.pointListBox);
    leftSplitter->addWidget(ui.pairListBox);
    leftSplitter->setHandleWidth(3);
    leftSplitter->setStretchFactor(0, 1);
    leftSplitter->setStretchFactor(1, 2);
    leftSplitter->setStretchFactor(2, 1);

    QLayout* oldLayout = ui.leftPanel->layout();
    if (oldLayout) {
        QLayoutItem* item;
        while ((item = oldLayout->takeAt(0)) != nullptr) {
            if (item->widget()) {
                item->widget()->setParent(nullptr);
            }
            delete item;
        }
        delete oldLayout;
    }
    ui.leftPanel->setLayout(new QVBoxLayout());
    ui.leftPanel->layout()->setContentsMargins(0, 0, 0, 0);
    ui.leftPanel->layout()->addWidget(leftSplitter);

    connect(ui.actionopen, SIGNAL(triggered()), this, SLOT(onOpen()));
    connect(ui.actionload, SIGNAL(triggered()), this, SLOT(onLoad()));
    connect(ui.actionPointSize, SIGNAL(triggered()), this, SLOT(onSetPointSize()));
    connect(ui.actionNewProject, SIGNAL(triggered()), this, SLOT(onNewProject()));
    connect(ui.actionOpenProject, SIGNAL(triggered()), this, SLOT(onOpenProject()));
    connect(ui.actionSaveProject, SIGNAL(triggered()), this, SLOT(onSaveProject()));
    connect(ui.actionSaveAsProject, SIGNAL(triggered()), this, SLOT(onSaveAsProject()));
    connect(ui.addGroupBtn, &QPushButton::clicked, this, &VTK930::onAddGroup);
    connect(ui.deleteGroupBtn, &QPushButton::clicked, this, &VTK930::onDeleteGroup);
    connect(ui.addPointBtn, &QPushButton::clicked, this, &VTK930::onAddPoint);
    connect(ui.deletePointBtn, &QPushButton::clicked, this, &VTK930::onDeletePoint);
    connect(ui.clearPointsBtn, &QPushButton::clicked, this, &VTK930::onClearPoints);
    connect(ui.filterCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &VTK930::onFilterChanged);
    connect(ui.addPairBtn, &QPushButton::clicked, this, &VTK930::onAddPair);
    connect(ui.deletePairBtn, &QPushButton::clicked, this, &VTK930::onDeletePair);
    connect(ui.viewTransformBtn, &QPushButton::clicked, this, &VTK930::onViewTransform);
    connect(ui.clearLogBtn, &QPushButton::clicked, this, &VTK930::onClearLog);
    connect(ui.exportLogBtn, &QPushButton::clicked, this, &VTK930::onExportLog);

    ui.pointTable->setColumnWidth(0, 40);
    ui.pointTable->setColumnWidth(1, 60);
    ui.pointTable->setColumnWidth(2, 60);
    ui.pointTable->setColumnWidth(3, 60);
    ui.pointTable->setColumnWidth(4, 80);

    ui.pairTable->setColumnWidth(0, 100);
    ui.pairTable->setColumnWidth(1, 130);
    ui.pairTable->setColumnWidth(2, 100);
    ui.pairTable->setColumnWidth(3, 130);

    addLog("系统启动完成", "成功");
}

VTK930::~VTK930() {}

void VTK930::onOpen() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    QString fileName = QFileDialog::getOpenFileName(this,
        QStringLiteral("打开点云"), ".",
        QStringLiteral("PCD文件(*.pcd)"));
    if (fileName == "") return;
    pcl::io::loadPCDFile(fileName.toStdString(), *cloud);

    if (cloud->empty()) {
        QMessageBox::warning(this, QStringLiteral("警告"),
            QStringLiteral("点云文件为空！"));
        return;
    }

    view->removePointCloud("cloud");
    view->addPointCloud(cloud, "cloud");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        pointSize, "cloud");
    
    // 计算点云边界框并设置相机
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    double centerX = (minPt.x + maxPt.x) / 2.0;
    double centerY = (minPt.y + maxPt.y) / 2.0;
    double centerZ = (minPt.z + maxPt.z) / 2.0;
    double size = std::max({maxPt.x - minPt.x, maxPt.y - minPt.y, maxPt.z - minPt.z});
    
    view->setCameraPosition(centerX, centerY, centerZ + size * 2, centerX, centerY, centerZ, 0, 1, 0);
    view->resetCamera();
    ui.openGLWidget->update();
    
    addLog(QString("已加载点云文件: %1, 共 %2 个点").arg(fileName).arg(cloud->size()), "成功");
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
            
            int startId = allPoints.size() + 1;
            for (size_t i = 0; i < points.size(); ++i) {
                PointData pd;
                pd.id = std::to_string(startId + i);
                pd.x = static_cast<float>(points[i].x);
                pd.y = static_cast<float>(points[i].y);
                pd.z = static_cast<float>(points[i].z);
                pd.groupName = data.groupName.toStdString();
                pd.colorR = data.r;
                pd.colorG = data.g;
                pd.colorB = data.b;
                allPoints.push_back(pd);
            }
            
            updatePointTable();
            view->resetCamera();
            ui.openGLWidget->update();

            //QMessageBox::information(this, QStringLiteral("成功"),
            //    QStringLiteral("已加载 %1 个点到分组 '%2'").arg(points.size()).arg(data.groupName));
            addLog(QString("已加载 %1 个点到分组 '%2'").arg(points.size()).arg(data.groupName), "成功");
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
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        pointSize, groupName);

    // 计算所有点云的边界框并设置相机
    double minX = std::numeric_limits<double>::max(), minY = std::numeric_limits<double>::max(), minZ = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest(), maxY = std::numeric_limits<double>::lowest(), maxZ = std::numeric_limits<double>::lowest();
    bool hasPoints = false;
    
    for (const auto& pair : groupClouds) {
        if (!pair.second->empty()) {
            hasPoints = true;
            for (const auto& pt : pair.second->points) {
                minX = std::min(minX, (double)pt.x);
                minY = std::min(minY, (double)pt.y);
                minZ = std::min(minZ, (double)pt.z);
                maxX = std::max(maxX, (double)pt.x);
                maxY = std::max(maxY, (double)pt.y);
                maxZ = std::max(maxZ, (double)pt.z);
            }
        }
    }
    
    if (hasPoints) {
        double centerX = (minX + maxX) / 2.0;
        double centerY = (minY + maxY) / 2.0;
        double centerZ = (minZ + maxZ) / 2.0;
        double size = std::max({maxX - minX, maxY - minY, maxZ - minZ});
        
        view->setCameraPosition(centerX, centerY, centerZ + size * 2, centerX, centerY, centerZ, 0, 1, 0);
    }
    
    view->resetCamera();
    ui.openGLWidget->update();

    updatePointTable();
    updateGroupList();
    updateFilterCombo();
}

void VTK930::updatePointTable() {
    QString filter = ui.filterCombo->currentText();
    std::vector<PointData> filteredPoints;

    if (filter == QStringLiteral("全部")) {
        filteredPoints = allPoints;
    } else {
        for (const auto& p : allPoints) {
            if (QString::fromStdString(p.groupName) == filter) {
                filteredPoints.push_back(p);
            }
        }
    }

    ui.pointTable->setRowCount(filteredPoints.size());
    for (size_t i = 0; i < filteredPoints.size(); ++i) {
        const auto& p = filteredPoints[i];
        ui.pointTable->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(p.id)));
        ui.pointTable->setItem(i, 1, new QTableWidgetItem(QString::number(p.x, 'f', 3)));
        ui.pointTable->setItem(i, 2, new QTableWidgetItem(QString::number(p.y, 'f', 3)));
        ui.pointTable->setItem(i, 3, new QTableWidgetItem(QString::number(p.z, 'f', 3)));
        ui.pointTable->setItem(i, 4, new QTableWidgetItem(QString::fromStdString(p.groupName)));
    }
}

void VTK930::updateGroupList() {
    ui.groupList->clear();
    for (const auto& pair : groupClouds) {
        ui.groupList->addItem(QString::fromStdString(pair.first));
    }
}

void VTK930::updateFilterCombo() {
    QString current = ui.filterCombo->currentText();
    ui.filterCombo->clear();
    ui.filterCombo->addItem(QStringLiteral("全部"));
    for (const auto& pair : groupClouds) {
        ui.filterCombo->addItem(QString::fromStdString(pair.first));
    }

    int index = ui.filterCombo->findText(current);
    if (index >= 0) {
        ui.filterCombo->setCurrentIndex(index);
    }
}

void VTK930::onAddGroup() {
    bool ok;
    QString name = QInputDialog::getText(this, QStringLiteral("添加分组"),
        QStringLiteral("请输入分组名称:"),
        QLineEdit::Normal, QStringLiteral("新分组"), &ok);
    if (!ok || name.isEmpty()) return;

    if (groupClouds.find(name.toStdString()) != groupClouds.end()) {
        QMessageBox::warning(this, QStringLiteral("警告"),
            QStringLiteral("该分组已存在！"));
        return;
    }

    groupClouds[name.toStdString()] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    updateGroupList();
    updateFilterCombo();
}

void VTK930::onDeleteGroup() {
    int row = ui.groupList->currentRow();
    if (row < 0) {
        QMessageBox::warning(this, QStringLiteral("警告"),
            QStringLiteral("请选择一个分组"));
        return;
    }

    QString groupName = ui.groupList->item(row)->text();

    QMessageBox::StandardButton reply = QMessageBox::question(this,
        QStringLiteral("确认"),
        QStringLiteral("确认要删除分组 \"%1\" 和其所有点吗？").arg(groupName),
        QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::No) return;

    std::string groupNameStr = groupName.toStdString();
    groupClouds.erase(groupNameStr);

    allPoints.erase(
        std::remove_if(allPoints.begin(), allPoints.end(),
            [&groupNameStr](const PointData& p) { return p.groupName == groupNameStr; }),
        allPoints.end());

    view->removePointCloud(groupNameStr);

    updateGroupList();
    updateFilterCombo();
    updatePointTable();
    ui.openGLWidget->update();
}

void VTK930::onAddPoint() {
    onLoad();
}

void VTK930::onDeletePoint() {
    int row = ui.pointTable->currentRow();
    if (row < 0) {
        QMessageBox::warning(this, QStringLiteral("警告"),
            QStringLiteral("请选择一个点"));
        return;
    }

    int id = ui.pointTable->item(row, 0)->text().toInt();

    for (auto it = allPoints.begin(); it != allPoints.end(); ++it) {
        if (it->id == std::to_string(id)) {
            std::string groupName = it->groupName;

            auto& cloud = groupClouds[groupName];
            cloud->points.erase(cloud->points.begin() + (it - allPoints.begin()));

            allPoints.erase(it);
            break;
        }
    }

    view->removePointCloud(groupClouds.begin()->first);
    for (const auto& pair : groupClouds) {
        if (!pair.second->points.empty()) {
            view->addPointCloud(pair.second, pair.first);
            view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                (double)pair.second->points[0].r / 255.0,
                (double)pair.second->points[0].g / 255.0,
                (double)pair.second->points[0].b / 255.0, pair.first);
        }
    }

    updatePointTable();
    ui.openGLWidget->update();
}

void VTK930::onClearPoints() {
    QMessageBox::StandardButton reply = QMessageBox::question(this,
        QStringLiteral("确认"),
        QStringLiteral("确认要清空所有点吗？"),
        QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::No) return;

    allPoints.clear();
    for (auto& pair : groupClouds) {
        pair.second->points.clear();
        view->removePointCloud(pair.first);
    }
    groupClouds.clear();

    updateGroupList();
    updateFilterCombo();
    updatePointTable();
    ui.openGLWidget->update();
}

void VTK930::onFilterChanged(int index) {
    Q_UNUSED(index);
    updatePointTable();
}

void VTK930::initialVtkWidget()
{
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    view.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    view->setBackgroundColor(0.2, 0.2, 0.2);

    view->setupInteractor(ui.openGLWidget->interactor(), ui.openGLWidget->renderWindow());
    ui.openGLWidget->setRenderWindow(view->getRenderWindow());
    
    // 初始化点选器
    setupPointPicking();

    QTimer::singleShot(50, this, &VTK930::initOrientationMarker);

    view->resetCamera();
    ui.openGLWidget->update();
}

void VTK930::initOrientationMarker()
{
    auto axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetTotalLength(1.0, 1.0, 1.0);
    axes->SetCylinderRadius(0.02);
    axes->SetAxisLabels(0);

    markerWidget_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();

    markerWidget_->SetOrientationMarker(axes);

    auto renderer = view->getRenderWindow()->GetRenderers()->GetFirstRenderer();
    markerWidget_->SetDefaultRenderer(renderer);

    markerWidget_->SetInteractor(ui.openGLWidget->renderWindow()->GetInteractor());

    markerWidget_->SetViewport(0.0, 0.0, 0.2, 0.2);

    markerWidget_->SetEnabled(true);
    markerWidget_->InteractiveOff();

    ui.openGLWidget->renderWindow()->Render();
}

void VTK930::onAddPair() {
    if (groupClouds.size() < 2) {
        QMessageBox::warning(this, QStringLiteral("警告"),
            QStringLiteral("需要至少两个分组才能进行配对！"));
        return;
    }

    std::vector<std::string> groups;
    for (const auto& pair : groupClouds) {
        groups.push_back(pair.first);
    }

    std::vector<Point3D> points;
    for (const auto& p : allPoints) {
        Point3D pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        pt.id = p.id;
        pt.groupName = p.groupName;
        points.push_back(pt);
    }

    PointPairDialog dialog(groups, points, pointPairs, this);
    if (dialog.exec() == QDialog::Accepted) {
        pointPairs = dialog.getPairs();
        updatePairList();
    }
}

void VTK930::onDeletePair() {
    int row = ui.pairTable->currentRow();
    if (row < 0) {
        QMessageBox::warning(this, QStringLiteral("警告"),
            QStringLiteral("请选择要删除的配对"));
        return;
    }

    QMessageBox::StandardButton reply = QMessageBox::question(this,
        QStringLiteral("确认"),
        QStringLiteral("确认要删除选中的配对吗？"),
        QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::No) return;

    pointPairs.erase(pointPairs.begin() + row);
    updatePairList();
}

void VTK930::updatePairList() {
    ui.pairTable->setRowCount(pointPairs.size());

    for (size_t i = 0; i < pointPairs.size(); ++i) {
        const auto& pair = pointPairs[i];
        QString sourcePoint = QString("%1#%2")
            .arg(QString::fromStdString(pair.sourceGroup))
            .arg(QString::fromStdString(pair.sourceId));
        QString targetPoint = QString("%1#%2")
            .arg(QString::fromStdString(pair.targetGroup))
            .arg(QString::fromStdString(pair.targetId));

        ui.pairTable->setItem(i, 0, new QTableWidgetItem(sourcePoint));
        ui.pairTable->setItem(i, 1, new QTableWidgetItem(
            QString("(%1, %2, %3)")
                .arg(pair.sourcePoint.x, 0, 'f', 3)
                .arg(pair.sourcePoint.y, 0, 'f', 3)
                .arg(pair.sourcePoint.z, 0, 'f', 3)));
        ui.pairTable->setItem(i, 2, new QTableWidgetItem(targetPoint));
        ui.pairTable->setItem(i, 3, new QTableWidgetItem(
            QString("(%1, %2, %3)")
                .arg(pair.targetPoint.x, 0, 'f', 3)
                .arg(pair.targetPoint.y, 0, 'f', 3)
                .arg(pair.targetPoint.z, 0, 'f', 3)));
    }
}

void VTK930::onViewTransform() {
    std::vector<std::string> allGroupsInPairs;
    std::set<std::string> groupSet;
    
    for (const auto& pt : allPoints) {
        groupSet.insert(pt.groupName);
    }
    
    for (const auto& pair : pointPairs) {
        groupSet.insert(pair.sourceGroup);
        groupSet.insert(pair.targetGroup);
    }
    
    for (const auto& g : groupSet) {
        allGroupsInPairs.push_back(g);
    }

    std::vector<std::pair<std::string, std::string>> availablePairs = getAvailablePairs();

    TransformResultDialog dialog(allGroupsInPairs, availablePairs, this);

    connect(&dialog, &TransformResultDialog::recomputeRequested,
            this, &VTK930::onRecomputeTransform);
    connect(&dialog, &TransformResultDialog::batchTransformRequested,
            [this](const std::string& groupName, const std::string& targetGroup, bool useABMatrix, 
                   int colorR, int colorG, int colorB) {
                this->onBatchTransform(groupName, targetGroup, useABMatrix, colorR, colorG, colorB);
            });

    dialog.exec();
}

void VTK930::onRecomputeTransform(const std::string& sourceGroup, const std::string& targetGroup) {
    TransformationMatrix matAB, matBA;
    computeTransformBetween(sourceGroup, targetGroup, matAB, matBA);

    TransformResultDialog* dialog = qobject_cast<TransformResultDialog*>(sender());
    if (dialog) {
        QString title1 = QString("%1 → %2 转换矩阵")
            .arg(QString::fromStdString(sourceGroup))
            .arg(QString::fromStdString(targetGroup));
        QString title2 = QString("%1 → %2 转换矩阵")
            .arg(QString::fromStdString(targetGroup))
            .arg(QString::fromStdString(sourceGroup));

        auto abGroup = dialog->findChild<QGroupBox*>();
        if (abGroup) abGroup->setTitle(title1);

        dialog->updateMatrices(matAB.toMatrix(), matBA.toMatrix());
    }
}

void VTK930::onBatchTransform(const std::string& groupName, const std::string& targetGroup, bool useABMatrix,
                              int colorR, int colorG, int colorB) {
    addLog(QString("开始批量转换: %1 → %2").arg(QString::fromStdString(groupName)).arg(QString::fromStdString(targetGroup)), "信息");

    TransformationMatrix matAB, matBA;
    computeTransformBetween(groupName, targetGroup, matAB, matBA);

    Eigen::Matrix4d matToUse = useABMatrix ? matAB.toMatrix() : matBA.toMatrix();

    std::vector<Point3D> originalPoints;
    std::vector<Point3D> transformedPoints;

    for (const auto& pt : allPoints) {
        if (pt.groupName == groupName) {
            Point3D origPt;
            origPt.x = pt.x;
            origPt.y = pt.y;
            origPt.z = pt.z;
            originalPoints.push_back(origPt);

            Eigen::Vector4d homogenous(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4d transformed = matToUse * homogenous;
            
            Point3D transPt;
            transPt.x = transformed.x();
            transPt.y = transformed.y();
            transPt.z = transformed.z();
            transformedPoints.push_back(transPt);
        }
    }

    if (originalPoints.empty()) {
        QMessageBox::warning(this, QStringLiteral("警告"),
            QStringLiteral("分组 %1 中没有点！").arg(QString::fromStdString(groupName)));
        addLog(QString("分组 %1 中没有点").arg(QString::fromStdString(groupName)), "警告");
        return;
    }

    addLog(QString("批量转换完成: 共转换 %1 个点").arg(originalPoints.size()), "成功");

    QString suggestedName = QString("%1_to_%2")
        .arg(QString::fromStdString(groupName))
        .arg(QString::fromStdString(targetGroup));

    GroupNameDialog nameDialog(suggestedName, this);
    if (nameDialog.exec() == QDialog::Accepted) {
        QString newGroupName = nameDialog.getGroupName();
        std::string newGroupStr = newGroupName.toStdString();

        addLog(QString("创建新分组: %1").arg(newGroupName), "信息");

        bool exists = false;
        for (const auto& pt : allPoints) {
            if (pt.groupName == newGroupStr) {
                exists = true;
                break;
            }
        }

        if (exists) {
            addLog(QString("分组 %1 已存在，准备覆盖").arg(newGroupName), "警告");
            QMessageBox::StandardButton reply = QMessageBox::question(this,
                QStringLiteral("确认"),
                QStringLiteral("分组 %1 已存在，是否覆盖？").arg(newGroupName),
                QMessageBox::Yes | QMessageBox::No);
            if (reply == QMessageBox::No) {
                addLog("用户取消覆盖操作", "信息");
                return;
            }
            
            if (groupClouds.find(newGroupStr) != groupClouds.end()) {
                groupClouds[newGroupStr]->points.clear();
                view->removePointCloud(newGroupStr);
            }
            
            std::vector<PointData> newPoints;
            for (const auto& pt : allPoints) {
                if (pt.groupName != newGroupStr) {
                    newPoints.push_back(pt);
                }
            }
            allPoints = newPoints;
            addLog(QString("已删除分组 %1 的旧数据").arg(newGroupName), "信息");
        }

        int idCounter = 0;
        for (size_t i = 0; i < originalPoints.size(); ++i) {
            PointData newPt;
            newPt.id = "pt_" + std::to_string(++idCounter);
            newPt.x = static_cast<float>(transformedPoints[i].x);
            newPt.y = static_cast<float>(transformedPoints[i].y);
            newPt.z = static_cast<float>(transformedPoints[i].z);
            newPt.groupName = newGroupStr;
            
            newPt.colorR = colorR;
            newPt.colorG = colorG;
            newPt.colorB = colorB;
            allPoints.push_back(newPt);
        }

        updateGroupList();
        updatePointTable();

        addLog(QString("已添加 %1 个点到分组 %2").arg(originalPoints.size()).arg(newGroupName), "成功");

        std::vector<Point3D> ptsForVisual;
        for (size_t i = 0; i < originalPoints.size(); ++i) {
            Point3D p;
            p.x = transformedPoints[i].x;
            p.y = transformedPoints[i].y;
            p.z = transformedPoints[i].z;
            ptsForVisual.push_back(p);
        }

        addPointsToVisualizer(ptsForVisual, 
            colorR, 
            colorG, 
            colorB, 
            newGroupStr);

        for (const auto& pair : groupClouds) {
            if (!pair.second->empty() && pair.first != newGroupStr) {
                view->updatePointCloud<pcl::PointXYZRGB>(pair.second, pair.first);
            }
        }

        addLog(QString("批量转换操作完成").arg(newGroupName), "成功");
    }
}

std::vector<std::pair<std::string, std::string>> VTK930::getAvailablePairs() const {
    std::vector<std::pair<std::string, std::string>> pairs;
    std::set<std::pair<std::string, std::string>> uniquePairs;
    
    for (const auto& pair : pointPairs) {
        uniquePairs.insert({pair.sourceGroup, pair.targetGroup});
    }
    
    for (const auto& p : uniquePairs) {
        pairs.push_back(p);
    }
    return pairs;
}

void VTK930::computeTransformBetween(const std::string& sourceGroup, const std::string& targetGroup,
                                      TransformationMatrix& matAB, TransformationMatrix& matBA) {
    matAB = TransformationMatrix();
    matBA = TransformationMatrix();
    matAB.rotation = Eigen::Matrix3d::Identity();
    matAB.translation = Eigen::Vector3d::Zero();
    matBA.rotation = Eigen::Matrix3d::Identity();
    matBA.translation = Eigen::Vector3d::Zero();

    std::vector<PointPair> filteredPairs;
    for (const auto& pair : pointPairs) {
        if (pair.sourceGroup == sourceGroup && pair.targetGroup == targetGroup) {
            filteredPairs.push_back(pair);
        }
    }

    if (filteredPairs.size() < 3) {
        QMessageBox::warning(this, QStringLiteral("警告"),
            QStringLiteral("错误: %1 和 %2 之间只有 %3 对配对点，至少需要3对！")
                .arg(QString::fromStdString(sourceGroup))
                .arg(QString::fromStdString(targetGroup))
                .arg(filteredPairs.size()));
        return;
    }

    std::vector<Eigen::Vector3d> sourcePoints, targetPoints;
    for (const auto& pair : filteredPairs) {
        sourcePoints.push_back(Eigen::Vector3d(pair.sourcePoint.x, pair.sourcePoint.y, pair.sourcePoint.z));
        targetPoints.push_back(Eigen::Vector3d(pair.targetPoint.x, pair.targetPoint.y, pair.targetPoint.z));
    }

    Eigen::Vector3d sourceCentroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d targetCentroid = Eigen::Vector3d::Zero();
    for (const auto& p : sourcePoints) sourceCentroid += p;
    sourceCentroid /= sourcePoints.size();
    for (const auto& p : targetPoints) targetCentroid += p;
    targetCentroid /= targetPoints.size();

    Eigen::MatrixXd sourceCentered(sourcePoints.size(), 3);
    Eigen::MatrixXd targetCentered(targetPoints.size(), 3);
    for (size_t i = 0; i < sourcePoints.size(); ++i) {
        sourceCentered.row(i) = sourcePoints[i] - sourceCentroid;
        targetCentered.row(i) = targetPoints[i] - targetCentroid;
    }

    Eigen::Matrix3d H = sourceCentered.transpose() * targetCentered;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

    if (R.determinant() < 0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        R = V * svd.matrixU().transpose();
    }

    matAB.rotation = R;
    matAB.translation = targetCentroid - R * sourceCentroid;
    matBA.rotation = R.transpose();
    matBA.translation = sourceCentroid - R.transpose() * targetCentroid;
}

TransformationMatrix VTK930::computeTransformAB() {
    TransformationMatrix result;
    result.rotation = Eigen::Matrix3d::Identity();
    result.translation = Eigen::Vector3d::Zero();

    if (pointPairs.size() < 3) {
        return result;
    }

    std::vector<Eigen::Vector3d> sourcePoints, targetPoints;
    for (const auto& pair : pointPairs) {
        sourcePoints.push_back(Eigen::Vector3d(pair.sourcePoint.x, pair.sourcePoint.y, pair.sourcePoint.z));
        targetPoints.push_back(Eigen::Vector3d(pair.targetPoint.x, pair.targetPoint.y, pair.targetPoint.z));
    }

    Eigen::Vector3d sourceCentroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d targetCentroid = Eigen::Vector3d::Zero();
    for (const auto& p : sourcePoints) sourceCentroid += p;
    sourceCentroid /= sourcePoints.size();
    for (const auto& p : targetPoints) targetCentroid += p;
    targetCentroid /= targetPoints.size();

    Eigen::MatrixXd sourceCentered(sourcePoints.size(), 3);
    Eigen::MatrixXd targetCentered(targetPoints.size(), 3);
    for (size_t i = 0; i < sourcePoints.size(); ++i) {
        sourceCentered.row(i) = sourcePoints[i] - sourceCentroid;
        targetCentered.row(i) = targetPoints[i] - targetCentroid;
    }

    Eigen::Matrix3d H = sourceCentered.transpose() * targetCentered;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

    if (R.determinant() < 0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        R = V * svd.matrixU().transpose();
    }

    result.rotation = R;
    result.translation = targetCentroid - R * sourceCentroid;

    return result;
}

TransformationMatrix VTK930::computeTransformBA() {
    TransformationMatrix result;
    result.rotation = Eigen::Matrix3d::Identity();
    result.translation = Eigen::Vector3d::Zero();

    if (pointPairs.size() < 3) {
        return result;
    }

    std::vector<Eigen::Vector3d> sourcePoints, targetPoints;
    for (const auto& pair : pointPairs) {
        sourcePoints.push_back(Eigen::Vector3d(pair.sourcePoint.x, pair.sourcePoint.y, pair.sourcePoint.z));
        targetPoints.push_back(Eigen::Vector3d(pair.targetPoint.x, pair.targetPoint.y, pair.targetPoint.z));
    }

    Eigen::Vector3d sourceCentroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d targetCentroid = Eigen::Vector3d::Zero();
    for (const auto& p : sourcePoints) sourceCentroid += p;
    sourceCentroid /= sourcePoints.size();
    for (const auto& p : targetPoints) targetCentroid += p;
    targetCentroid /= targetPoints.size();

    Eigen::MatrixXd sourceCentered(sourcePoints.size(), 3);
    Eigen::MatrixXd targetCentered(targetPoints.size(), 3);
    for (size_t i = 0; i < sourcePoints.size(); ++i) {
        sourceCentered.row(i) = sourcePoints[i] - sourceCentroid;
        targetCentered.row(i) = targetPoints[i] - targetCentroid;
    }

    Eigen::Matrix3d H = targetCentered.transpose() * sourceCentered;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

    if (R.determinant() < 0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        R = V * svd.matrixU().transpose();
    }

    result.rotation = R;
    result.translation = sourceCentroid - R * targetCentroid;

    return result;
}

void VTK930::addLog(const QString& message, const QString& type) {
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString color;
    
    if (type == "错误") {
        color = "red";
    } else if (type == "警告") {
        color = "orange";
    } else if (type == "成功") {
        color = "green";
    } else {
        color = "black";
    }
    
    QString logEntry = QString("<span style='color:%1;'>[%2] <b>%3:</b> %4</span>")
        .arg(color)
        .arg(timestamp)
        .arg(type)
        .arg(message);
    
    ui.logTextEdit->append(logEntry);
    
    QScrollBar* scrollBar = ui.logTextEdit->verticalScrollBar();
    scrollBar->setValue(scrollBar->maximum());
}

void VTK930::onClearLog() {
    ui.logTextEdit->clear();
    // addLog("日志已清空", "信息");
}

void VTK930::onExportLog() {
    QString fileName = QFileDialog::getSaveFileName(this,
        QStringLiteral("导出日志"),
        "",
        QStringLiteral("文本文件 (*.txt);;所有文件 (*.*)"));
    
    if (fileName.isEmpty()) return;
    
    QFile file(fileName);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);
        out.setCodec("UTF-8");
        out << ui.logTextEdit->toPlainText();
        file.close();
        addLog(QString("日志已导出到: %1").arg(fileName), "成功");
    } else {
        addLog(QString("无法导出日志到: %1").arg(fileName), "错误");
    }
}

void VTK930::onSetPointSize() {
    bool ok;
    int newSize = QInputDialog::getInt(this, 
        QStringLiteral("设置点云点大小"), 
        QStringLiteral("请输入点大小 (1-20):"), 
        pointSize, 1, 20, 1, &ok);
    
    if (ok) {
        pointSize = newSize;
        
        // 更新所有分组的点云渲染属性
        for (const auto& pair : groupClouds) {
            view->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
                pointSize, 
                pair.first);
        }
        
        // 如果有全局cloud，也更新
        if (view->contains("cloud")) {
            view->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
                pointSize, 
                "cloud");
        }
        
        ui.openGLWidget->update();
        
        ui.openGLWidget->renderWindow()->Render();
        
        addLog(QString("点云点大小已设置为: %1").arg(pointSize), "成功");
    }
}

void VTK930::setupPointPicking() {
    if (!view || !ui.openGLWidget->interactor())
        return;

    pointPicker = vtkPointPicker::New();

    vtkRenderWindowInteractor* interactor = ui.openGLWidget->interactor();
    interactor->SetPicker(pointPicker);

    vtkSmartPointer<vtkCallbackCommand> callback = vtkSmartPointer<vtkCallbackCommand>::New();
    callback->SetCallback(VTK930::onPointPicked);
    callback->SetClientData(this);
    
    interactor->AddObserver(vtkCommand::LeftButtonPressEvent, callback);
    
    addLog("点选功能已启用：左键点击选择点", "信息");
}

void VTK930::onPointPicked(vtkObject* caller, unsigned long eventId, void* clientData, void* callData) {
    VTK930* self = static_cast<VTK930*>(clientData);
    if (!self || !self->pointPicker)
        return;

    vtkRenderWindowInteractor* interactor = static_cast<vtkRenderWindowInteractor*>(caller);
    if (!interactor)
        return;

    int* pos = interactor->GetEventPosition();
    self->pointPicker->Pick(pos[0], pos[1], 0, self->view->getRenderWindow()->GetRenderers()->GetFirstRenderer());

    if (self->pointPicker->GetPointId() != -1) {
        double* pickedPos = self->pointPicker->GetPickPosition();

        QString info = QString("选中点 ID: %1, 坐标: (%2, %3, %4)")
                           .arg(self->pointPicker->GetPointId())
                           .arg(pickedPos[0], 0, 'f', 3)
                           .arg(pickedPos[1], 0, 'f', 3)
                           .arg(pickedPos[2], 0, 'f', 3);
        
        QMetaObject::invokeMethod(self, [self, info]() {
            self->addLog(info, "信息");
        }, Qt::QueuedConnection);
    }
}

void VTK930::onNewProject() {
    QMessageBox::StandardButton reply = QMessageBox::question(this,
        QStringLiteral("新建项目"),
        QStringLiteral("是否保存当前项目?"),
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    
    if (reply == QMessageBox::Save) {
        onSaveProject();
    } else if (reply == QMessageBox::Cancel) {
        return;
    }
    
    groupClouds.clear();
    allPoints.clear();
    pointPairs.clear();
    currentProjectFile.clear();
    
    if (view) {
        view->removeAllPointClouds();
        ui.openGLWidget->update();
    }
    
    updateGroupList();
    updatePointTable();
    updateFilterCombo();
    updatePairList();
    
    setWindowTitle(QStringLiteral("VTK930 - 新建项目"));
    addLog("已创建新项目", "信息");
}

void VTK930::onOpenProject() {
    QString fileName = QFileDialog::getOpenFileName(this,
        QStringLiteral("打开项目"),
        "",
        QStringLiteral("VTK930项目 (*.vtkproj);;所有文件 (*.*)"));
    
    if (fileName.isEmpty()) return;
    
    if (loadProject(fileName)) {
        currentProjectFile = fileName;
        setWindowTitle(QStringLiteral("VTK930 - %1").arg(QFileInfo(fileName).baseName()));
        addLog(QString("已打开项目: %1").arg(fileName), "成功");
    } else {
        QMessageBox::warning(this, 
            QStringLiteral("错误"), 
            QStringLiteral("无法打开项目文件"));
    }
}

void VTK930::onSaveProject() {
    if (currentProjectFile.isEmpty()) {
        onSaveAsProject();
        return;
    }
    
    if (saveProject(currentProjectFile)) {
        addLog(QString("项目已保存: %1").arg(currentProjectFile), "成功");
    } else {
        QMessageBox::warning(this, 
            QStringLiteral("错误"), 
            QStringLiteral("保存项目失败"));
    }
}

void VTK930::onSaveAsProject() {
    QString fileName = QFileDialog::getSaveFileName(this,
        QStringLiteral("保存项目"),
        "",
        QStringLiteral("VTK930项目 (*.vtkproj)"));
    
    if (fileName.isEmpty()) return;
    
    if (!fileName.endsWith(".vtkproj", Qt::CaseInsensitive)) {
        fileName += ".vtkproj";
    }
    
    if (saveProject(fileName)) {
        currentProjectFile = fileName;
        setWindowTitle(QStringLiteral("VTK930 - %1").arg(QFileInfo(fileName).baseName()));
        addLog(QString("项目已保存: %1").arg(fileName), "成功");
    } else {
        QMessageBox::warning(this, 
            QStringLiteral("错误"), 
            QStringLiteral("保存项目失败"));
    }
}

bool VTK930::saveProject(const QString& fileName) {
    QJsonObject root;
    root["version"] = "1.0";
    root["projectName"] = QFileInfo(fileName).baseName();
    
    QJsonObject settings;
    settings["pointSize"] = pointSize;
    QJsonArray bgColor;
    bgColor.append(0.2);
    bgColor.append(0.2);
    bgColor.append(0.2);
    settings["backgroundColor"] = bgColor;
    root["settings"] = settings;
    
    QJsonArray groupsArray;
    for (const auto& pair : groupClouds) {
        QJsonObject groupObj;
        groupObj["name"] = QString::fromStdString(pair.first);
        
        QJsonArray colorArray;
        if (!pair.second->empty()) {
            const auto& firstPoint = pair.second->points[0];
            colorArray.append((int)firstPoint.r);
            colorArray.append((int)firstPoint.g);
            colorArray.append((int)firstPoint.b);
        } else {
            colorArray.append(255);
            colorArray.append(255);
            colorArray.append(255);
        }
        groupObj["color"] = colorArray;
        
        QJsonArray pointsArray;
        for (const auto& pt : pair.second->points) {
            QJsonObject pointObj;
            pointObj["x"] = pt.x;
            pointObj["y"] = pt.y;
            pointObj["z"] = pt.z;
            pointsArray.append(pointObj);
        }
        groupObj["points"] = pointsArray;
        
        groupsArray.append(groupObj);
    }
    root["groups"] = groupsArray;
    
    QJsonArray pairsArray;
    for (const auto& pair : pointPairs) {
        QJsonObject pairObj;
        pairObj["sourceGroup"] = QString::fromStdString(pair.sourceGroup);
        pairObj["targetGroup"] = QString::fromStdString(pair.targetGroup);
        pairObj["sourceId"] = QString::fromStdString(pair.sourceId);
        pairObj["targetId"] = QString::fromStdString(pair.targetId);
        
        QJsonObject sourcePointObj;
        sourcePointObj["x"] = pair.sourcePoint.x;
        sourcePointObj["y"] = pair.sourcePoint.y;
        sourcePointObj["z"] = pair.sourcePoint.z;
        pairObj["sourcePoint"] = sourcePointObj;
        
        QJsonObject targetPointObj;
        targetPointObj["x"] = pair.targetPoint.x;
        targetPointObj["y"] = pair.targetPoint.y;
        targetPointObj["z"] = pair.targetPoint.z;
        pairObj["targetPoint"] = targetPointObj;
        
        pairsArray.append(pairObj);
    }
    root["pointPairs"] = pairsArray;
    
    QJsonDocument doc(root);
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly)) {
        return false;
    }
    
    file.write(doc.toJson(QJsonDocument::Indented));
    file.close();
    
    return true;
}

bool VTK930::loadProject(const QString& fileName) {
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) {
        return false;
    }
    
    QByteArray data = file.readAll();
    file.close();
    
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(data, &error);
    if (error.error != QJsonParseError::NoError) {
        qDebug() << "JSON解析错误:" << error.errorString();
        return false;
    }
    
    QJsonObject root = doc.object();
    
    QString version = root["version"].toString();
    if (version != "1.0") {
        QMessageBox::warning(this, 
            QStringLiteral("版本不兼容"), 
            QStringLiteral("不支持的项目文件版本: %1").arg(version));
        return false;
    }
    
    groupClouds.clear();
    allPoints.clear();
    pointPairs.clear();
    
    if (root.contains("settings")) {
        QJsonObject settings = root["settings"].toObject();
        pointSize = settings["pointSize"].toInt(2);
    }
    
    if (root.contains("groups")) {
        QJsonArray groupsArray = root["groups"].toArray();
        
        for (int i = 0; i < groupsArray.size(); ++i) {
            QJsonObject groupObj = groupsArray[i].toObject();
            QString groupName = groupObj["name"].toString();
            
            QJsonArray colorArray = groupObj["color"].toArray();
            int r = colorArray[0].toInt(255);
            int g = colorArray[1].toInt(255);
            int b = colorArray[2].toInt(255);
            
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            
            QJsonArray pointsArray = groupObj["points"].toArray();
            for (int j = 0; j < pointsArray.size(); ++j) {
                QJsonObject ptObj = pointsArray[j].toObject();
                pcl::PointXYZRGB pt;
                pt.x = (float)ptObj["x"].toDouble();
                pt.y = (float)ptObj["y"].toDouble();
                pt.z = (float)ptObj["z"].toDouble();
                pt.r = r;
                pt.g = g;
                pt.b = b;
                cloud->points.push_back(pt);
                
                PointData pd;
                pd.id = std::to_string(j + 1);
                pd.x = pt.x;
                pd.y = pt.y;
                pd.z = pt.z;
                pd.groupName = groupName.toStdString();
                pd.colorR = r;
                pd.colorG = g;
                pd.colorB = b;
                allPoints.push_back(pd);
            }
            
            groupClouds[groupName.toStdString()] = cloud;
            
            if (view) {
                view->removePointCloud(groupName.toStdString());
                view->addPointCloud(cloud, groupName.toStdString());
                view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                    (double)r / 255.0, (double)g / 255.0, (double)b / 255.0, groupName.toStdString());
                view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                    pointSize, groupName.toStdString());
            }
        }
    }
    
    if (root.contains("pointPairs")) {
        QJsonArray pairsArray = root["pointPairs"].toArray();
        
        for (int i = 0; i < pairsArray.size(); ++i) {
            QJsonObject pairObj = pairsArray[i].toObject();
            
            PointPair pair;
            pair.sourceGroup = pairObj["sourceGroup"].toString().toStdString();
            pair.targetGroup = pairObj["targetGroup"].toString().toStdString();
            pair.sourceId = pairObj["sourceId"].toString().toStdString();
            pair.targetId = pairObj["targetId"].toString().toStdString();
            
            QJsonObject sourcePtObj = pairObj["sourcePoint"].toObject();
            pair.sourcePoint.x = (float)sourcePtObj["x"].toDouble();
            pair.sourcePoint.y = (float)sourcePtObj["y"].toDouble();
            pair.sourcePoint.z = (float)sourcePtObj["z"].toDouble();
            pair.sourcePoint.id = pair.sourceId;
            pair.sourcePoint.groupName = pair.sourceGroup;
            
            QJsonObject targetPtObj = pairObj["targetPoint"].toObject();
            pair.targetPoint.x = (float)targetPtObj["x"].toDouble();
            pair.targetPoint.y = (float)targetPtObj["y"].toDouble();
            pair.targetPoint.z = (float)targetPtObj["z"].toDouble();
            pair.targetPoint.id = pair.targetId;
            pair.targetPoint.groupName = pair.targetGroup;
            
            pointPairs.push_back(pair);
        }
    }
    
    if (view) {
        view->resetCamera();
        
        // 计算所有点云的边界框并设置相机
        double minX = std::numeric_limits<double>::max(), minY = std::numeric_limits<double>::max(), minZ = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest(), maxY = std::numeric_limits<double>::lowest(), maxZ = std::numeric_limits<double>::lowest();
        bool hasPoints = false;
        
        for (const auto& pair : groupClouds) {
            if (!pair.second->empty()) {
                hasPoints = true;
                for (const auto& pt : pair.second->points) {
                    minX = std::min(minX, (double)pt.x);
                    minY = std::min(minY, (double)pt.y);
                    minZ = std::min(minZ, (double)pt.z);
                    maxX = std::max(maxX, (double)pt.x);
                    maxY = std::max(maxY, (double)pt.y);
                    maxZ = std::max(maxZ, (double)pt.z);
                }
            }
        }
        
        if (hasPoints) {
            double centerX = (minX + maxX) / 2.0;
            double centerY = (minY + maxY) / 2.0;
            double centerZ = (minZ + maxZ) / 2.0;
            double sizeX = maxX - minX;
            double sizeY = maxY - minY;
            double sizeZ = maxZ - minZ;
            double maxSize = std::max({sizeX, sizeY, sizeZ});
            
            if (maxSize > 0) {
                view->setCameraPosition(centerX, centerY, centerZ + maxSize * 2,
                                        centerX, centerY, centerZ, 0, 1, 0);
            }
        }
        
        ui.openGLWidget->update();
    }
    
    updateGroupList();
    updatePointTable();
    updateFilterCombo();
    updatePairList();
    
    return true;
}
