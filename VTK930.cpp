#include "VTK930.h"
#include <QFileDialog>
#include <QDebug>
#include <QMessageBox>
#include <QInputDialog>
#include <QSplitter>
#include <QLayout>
#include "PointCloudInputDialog.h"
#include "PointPairDialog.h"
#include "TransformResultDialog.h"
#include <vtkAnnotatedCubeActor.h>
#include <vtkTextProperty.h>
#include <Eigen/Dense>
#include <Eigen/SVD>

VTK930::VTK930(QWidget* parent) : QMainWindow(parent) {
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

    ui.pointTable->setColumnWidth(0, 40);
    ui.pointTable->setColumnWidth(1, 60);
    ui.pointTable->setColumnWidth(2, 60);
    ui.pointTable->setColumnWidth(3, 60);
    ui.pointTable->setColumnWidth(4, 80);

    ui.pairTable->setColumnWidth(0, 100);
    ui.pairTable->setColumnWidth(1, 130);
    ui.pairTable->setColumnWidth(2, 100);
    ui.pairTable->setColumnWidth(3, 130);
}

VTK930::~VTK930() {}

void VTK930::onOpen() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    QString fileName = QFileDialog::getOpenFileName(this,
        QString::fromUtf8("\xe6\x89\x93\xe5\xbc\x80\xe7\x82\xb9\xe4\xba\x91"), ".",
        QString::fromUtf8("PCD \xe6\x96\x87\xe4\xbb\xb6(*.pcd)"));
    if (fileName == "") return;
    pcl::io::loadPCDFile(fileName.toStdString(), *cloud);

    view->removePointCloud("cloud");
    view->addPointCloud(cloud, "cloud");
    view->resetCamera();
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
            ui.openGLWidget->update();

            QMessageBox::information(this, QString::fromUtf8("\xe6\x88\x90\xe5\x8a\x9f"),
                QString::fromUtf8("\xe5\xb7\xb2\xe5\x8a\xa0\xe8\xbd\xbd %1 \xe4\xb8\xaa\xe7\x82\xb9\xe5\x88\xb0\xe5\x88\x86\xe7\xbb\x84 '%2'").arg(points.size()).arg(data.groupName));
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

    int startId = allPoints.size();
    for (const auto& p : points) {
        pcl::PointXYZRGB point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.r = r;
        point.g = g;
        point.b = b;
        cloud->points.push_back(point);

        PointData pd;
        pd.id = std::to_string(startId++);
        pd.x = p.x;
        pd.y = p.y;
        pd.z = p.z;
        pd.groupName = groupName;
        pd.colorR = r;
        pd.colorG = g;
        pd.colorB = b;
        allPoints.push_back(pd);
    }

    view->removePointCloud(groupName);
    view->addPointCloud(cloud, groupName);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
        (double)r / 255.0, (double)g / 255.0, (double)b / 255.0, groupName);

    updatePointTable();
    updateGroupList();
    updateFilterCombo();
}

void VTK930::updatePointTable() {
    QString filter = ui.filterCombo->currentText();
    std::vector<PointData> filteredPoints;

    if (filter == QString::fromUtf8("\xe5\x85\xa8\xe9\x83\xa8")) {
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
    ui.filterCombo->addItem(QString::fromUtf8("\xe5\x85\xa8\xe9\x83\xa8"));
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
    QString name = QInputDialog::getText(this, QString::fromUtf8("\xe6\xb7\xbb\xe5\x8a\xa0\xe5\x88\x86\xe7\xbb\x84"),
        QString::fromUtf8("\xe8\xaf\xb7\xe8\xbe\x93\xe5\x85\xa5\xe5\x88\x86\xe7\xbb\x84\xe5\x90\x8d\xe7\xa7\xb0:"),
        QLineEdit::Normal, QString::fromUtf8("\xe6\x96\xb0\xe5\x88\x86\xe7\xbb\x84"), &ok);
    if (!ok || name.isEmpty()) return;

    if (groupClouds.find(name.toStdString()) != groupClouds.end()) {
        QMessageBox::warning(this, QString::fromUtf8("\xe8\xad\xa6\xe5\x91\x8a"),
            QString::fromUtf8("\xe8\xaf\xa5\xe5\x88\x86\xe7\xbb\x84\xe5\xb7\xb2\xe5\xad\x98\xe5\x9c\xa8\xef\xbc\x81"));
        return;
    }

    groupClouds[name.toStdString()] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    updateGroupList();
    updateFilterCombo();
}

void VTK930::onDeleteGroup() {
    int row = ui.groupList->currentRow();
    if (row < 0) {
        QMessageBox::warning(this, QString::fromUtf8("\xe8\xad\xa6\xe5\x91\x8a"),
            QString::fromUtf8("\xe8\xaf\xb7\xe9\x80\x89\xe6\x8b\xa9\xe4\xb8\x80\xe4\xb8\xaa\xe5\x88\x86\xe7\xbb\x84"));
        return;
    }

    QString groupName = ui.groupList->item(row)->text();

    QMessageBox::StandardButton reply = QMessageBox::question(this,
        QString::fromUtf8("\xe7\xa1\xae\xe8\xae\xa4"),
        QString::fromUtf8("\xe7\xa1\xae\xe8\xae\xa4\xe8\xa6\x81\xe5\x88\xa0\xe9\x99\xa4\xe5\x88\x86\xe7\xbb\x84 \"%1\" \xe5\x92\x8c\xe5\x85\xb6\xe6\x89\x80\xe6\x9c\x89\xe7\x82\xb9\xe5\x90\x97\xef\xbc\x9f").arg(groupName),
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
        QMessageBox::warning(this, QString::fromUtf8("\xe8\xad\xa6\xe5\x91\x8a"),
            QString::fromUtf8("\xe8\xaf\xb7\xe9\x80\x89\xe6\x8b\xa9\xe4\xb8\x80\xe4\xb8\xaa\xe7\x82\xb9"));
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
        QString::fromUtf8("\xe7\xa1\xae\xe8\xae\xa4"),
        QString::fromUtf8("\xe7\xa1\xae\xe8\xae\xa4\xe8\xa6\x81\xe6\xb8\x85\xe7\xa9\xba\xe6\x89\x80\xe6\x9c\x89\xe7\x82\xb9\xe5\x90\x97\xef\xbc\x9f"),
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
        QMessageBox::warning(this, QString::fromUtf8("警告"),
            QString::fromUtf8("请选择要删除的配对"));
        return;
    }

    QMessageBox::StandardButton reply = QMessageBox::question(this,
        QString::fromUtf8("确认"),
        QString::fromUtf8("确认要删除选中的配对吗？"),
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
    if (pointPairs.empty()) {
        QMessageBox::warning(this, QString::fromUtf8("警告"),
            QString::fromUtf8("没有任何配对点！请先添加配对点。"));
        return;
    }

    std::vector<std::string> allGroups;
    std::set<std::string> groupSet;
    for (const auto& pair : pointPairs) {
        groupSet.insert(pair.sourceGroup);
        groupSet.insert(pair.targetGroup);
    }
    for (const auto& g : groupSet) {
        allGroups.push_back(g);
    }

    std::vector<std::pair<std::string, std::string>> availablePairs = getAvailablePairs();

    TransformResultDialog dialog(allGroups, availablePairs, this);

    connect(&dialog, &TransformResultDialog::recomputeRequested,
            this, &VTK930::onRecomputeTransform);

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
        QMessageBox::warning(this, QString::fromUtf8("警告"),
            QString::fromUtf8("错误: %1 和 %2 之间只有 %3 对配对点，至少需要3对！")
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
