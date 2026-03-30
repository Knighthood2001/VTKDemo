#include "PointCloudInputDialog.h"
#include <QMessageBox>
#include <sstream>

PointCloudInputDialog::PointCloudInputDialog(QWidget* parent)
    : QDialog(parent), currentR(255), currentG(0), currentB(0)
{
    setWindowTitle(QStringLiteral("加载点"));
    setupUI();
    setMinimumWidth(500);
    setMinimumHeight(450);
    resize(500, 450);
    
    setStyleSheet(R"(
        QDialog {
            background-color: #f5f5f5;
        }
        QGroupBox {
            font-weight: bold;
            border: 2px solid #cccccc;
            border-radius: 6px;
            margin-top: 12px;
            padding-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px;
            color: #333333;
        }
        QLineEdit, QTextEdit {
            border: 2px solid #ddd;
            border-radius: 4px;
            padding: 6px;
            background-color: white;
        }
        QLineEdit:focus, QTextEdit:focus {
            border-color: #4a90d9;
        }
        QPushButton {
            background-color: #4a90d9;
            color: white;
            border: none;
            border-radius: 4px;
            padding: 8px 16px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #357abd;
        }
        QPushButton:pressed {
            background-color: #2d6aa3;
        }
        QLabel {
            color: #333333;
        }
    )");
}

PointCloudInputDialog::~PointCloudInputDialog() {}

void PointCloudInputDialog::setupUI() {
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(20, 20, 20, 20);
    mainLayout->setSpacing(15);

    QGroupBox* groupBox = new QGroupBox("Group Information", this);
    QVBoxLayout* groupLayout = new QVBoxLayout(groupBox);
    
    QLabel* groupLabel = new QLabel(QStringLiteral("组名:"), this);
    groupNameEdit = new QLineEdit(this);
    groupNameEdit->setPlaceholderText(QStringLiteral("输入组名"));
    groupNameEdit->setMinimumHeight(35);
    groupLayout->addWidget(groupLabel);
    groupLayout->addWidget(groupNameEdit);
    mainLayout->addWidget(groupBox);

    QGroupBox* pointsBox = new QGroupBox("Point Coordinates", this);
    QVBoxLayout* pointsLayout = new QVBoxLayout(pointsBox);
    
    QLabel* pointsLabel = new QLabel("Enter points (one point per line, format: X Y Z):", this);
    pointsTextEdit = new QTextEdit(this);
    pointsTextEdit->setPlaceholderText("Example:\n1 2 3\n2 3 4\n3 4 5\n44 55 55");
    pointsTextEdit->setMinimumHeight(150);
    pointsLayout->addWidget(pointsLabel);
    pointsLayout->addWidget(pointsTextEdit);
    mainLayout->addWidget(pointsBox);

    QGroupBox* colorBox = new QGroupBox("Color Settings", this);
    QHBoxLayout* colorLayout = new QHBoxLayout(colorBox);
    colorLayout->setSpacing(15);
    
    colorButton = new QPushButton("Pick Color", this);
    colorButton->setMinimumHeight(40);
    colorButton->setMinimumWidth(150);
    
    colorPreview = new QLabel(this);
    colorPreview->setFixedSize(80, 35);
    colorPreview->setStyleSheet(QString("background-color: rgb(%1,%2,%3); border: 2px solid #aaa; border-radius: 4px;").arg(currentR).arg(currentG).arg(currentB));
    colorPreview->setAlignment(Qt::AlignCenter);
    
    QLabel* previewLabel = new QLabel("Preview:", this);
    
    colorLayout->addWidget(colorButton);
    colorLayout->addStretch();
    colorLayout->addWidget(previewLabel);
    colorLayout->addWidget(colorPreview);
    
    mainLayout->addWidget(colorBox);

    QFrame* separator = new QFrame(this);
    separator->setFrameShape(QFrame::HLine);
    separator->setStyleSheet("border: 1px solid #ddd;");
    mainLayout->addWidget(separator);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->setSpacing(15);
    
    QPushButton* cancelButton = new QPushButton("Cancel", this);
    cancelButton->setStyleSheet(R"(
        QPushButton {
            background-color: #e0e0e0;
            color: #333;
        }
        QPushButton:hover {
            background-color: #d0d0d0;
        }
    )");
    cancelButton->setMinimumHeight(40);
    
    QPushButton* okButton = new QPushButton("Add Points", this);
    okButton->setMinimumHeight(40);
    
    buttonLayout->addStretch();
    buttonLayout->addWidget(cancelButton);
    buttonLayout->addWidget(okButton);
    mainLayout->addLayout(buttonLayout);

    connect(colorButton, &QPushButton::clicked, this, &PointCloudInputDialog::onColorButtonClicked);
    connect(okButton, &QPushButton::clicked, this, &PointCloudInputDialog::onOkClicked);
    connect(cancelButton, &QPushButton::clicked, this, &PointCloudInputDialog::onCancelClicked);
}

void PointCloudInputDialog::onColorButtonClicked() {
    QColor color = QColorDialog::getColor(QColor(currentR, currentG, currentB), this, "Pick a color");
    if (color.isValid()) {
        currentR = color.red();
        currentG = color.green();
        currentB = color.blue();
        colorPreview->setStyleSheet(QString("background-color: rgb(%1,%2,%3); border: 2px solid #aaa; border-radius: 4px;").arg(currentR).arg(currentG).arg(currentB));
    }
}

void PointCloudInputDialog::onOkClicked() {
    if (groupNameEdit->text().trimmed().isEmpty()) {
        QMessageBox::warning(this, "Warning", "Please enter a group name!");
        return;
    }
    if (pointsTextEdit->toPlainText().trimmed().isEmpty()) {
        QMessageBox::warning(this, "Warning", "Please enter at least one point!");
        return;
    }
    accept();
}

void PointCloudInputDialog::onCancelClicked() {
    reject();
}

PointCloudInputDialog::PointData PointCloudInputDialog::getPointData() const {
    PointData data;
    data.r = currentR;
    data.g = currentG;
    data.b = currentB;
    data.groupName = groupNameEdit->text().trimmed();
    
    std::istringstream iss(pointsTextEdit->toPlainText().toStdString());
    std::string line;
    while (std::getline(iss, line)) {
        if (line.empty()) continue;
        
        std::replace(line.begin(), line.end(), ',', ' ');
        std::replace(line.begin(), line.end(), '\t', ' ');
        
        std::istringstream pointIss(line);
        float x, y, z;
        if (pointIss >> x >> y >> z) {
            data.points.push_back({x, y, z});
        }
    }
    
    return data;
}
