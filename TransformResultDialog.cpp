#include "TransformResultDialog.h"
#include <QClipboard>
#include <QApplication>
#include <QHeaderView>

TransformResultDialog::TransformResultDialog(const Eigen::Matrix4d& matAB, 
                                             const Eigen::Matrix4d& matBA,
                                             const std::string& group1,
                                             const std::string& group2,
                                             QWidget* parent)
    : QDialog(parent), currentMatAB(matAB), currentMatBA(matBA)
{
    setWindowTitle(QStringLiteral("转换矩阵计算结果"));
    setMinimumSize(600, 700);
    resize(650, 750);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QString title1 = QString("%1 → %2 转换矩阵")
        .arg(QString::fromStdString(group1))
        .arg(QString::fromStdString(group2));
    QString title2 = QString("%1 → %2 转换矩阵")
        .arg(QString::fromStdString(group2))
        .arg(QString::fromStdString(group1));

    QGroupBox* abGroup = new QGroupBox(title1, this);
    QVBoxLayout* abLayout = new QVBoxLayout;
    matrixABTable = new QTableWidget(4, 4, this);
    matrixABTable->horizontalHeader()->setVisible(false);
    matrixABTable->verticalHeader()->setVisible(false);
    abLayout->addWidget(matrixABTable);
    copyABBtn = new QPushButton(QStringLiteral("复制矩阵"), this);
    abLayout->addWidget(copyABBtn);
    abGroup->setLayout(abLayout);

    QGroupBox* baGroup = new QGroupBox(title2, this);
    QVBoxLayout* baLayout = new QVBoxLayout;
    matrixBATable = new QTableWidget(4, 4, this);
    matrixBATable->horizontalHeader()->setVisible(false);
    matrixBATable->verticalHeader()->setVisible(false);
    baLayout->addWidget(matrixBATable);
    copyBABtn = new QPushButton(QStringLiteral("复制矩阵"), this);
    baLayout->addWidget(copyBABtn);
    baGroup->setLayout(baLayout);

    mainLayout->addWidget(abGroup);
    mainLayout->addWidget(baGroup);

    QHBoxLayout* btnLayout = new QHBoxLayout;
    recomputeBtn = new QPushButton(QStringLiteral("重新计算"), this);
    closeBtn = new QPushButton(QStringLiteral("关闭"), this);
    btnLayout->addStretch();
    btnLayout->addWidget(recomputeBtn);
    btnLayout->addWidget(closeBtn);
    mainLayout->addLayout(btnLayout);

    updateMatrices(matAB, matBA);

    connect(copyABBtn, &QPushButton::clicked, this, &TransformResultDialog::onCopyAB);
    connect(copyBABtn, &QPushButton::clicked, this, &TransformResultDialog::onCopyBA);
    connect(recomputeBtn, &QPushButton::clicked, this, &TransformResultDialog::onRecompute);
    connect(closeBtn, &QPushButton::clicked, this, &QDialog::accept);
}

TransformResultDialog::~TransformResultDialog() {}

void TransformResultDialog::updateMatrices(const Eigen::Matrix4d& matAB, const Eigen::Matrix4d& matBA) {
    currentMatAB = matAB;
    currentMatBA = matBA;
    displayMatrix(matrixABTable, matAB);
    displayMatrix(matrixBATable, matBA);
}

void TransformResultDialog::displayMatrix(QTableWidget* table, const Eigen::Matrix4d& mat) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            QTableWidgetItem* item = new QTableWidgetItem(QString::number(mat(i,j), 'f', 8));
            item->setTextAlignment(Qt::AlignCenter);
            table->setItem(i, j, item);
        }
    }
    for (int i = 0; i < 4; ++i) {
        table->setColumnWidth(i, 130);
        table->setRowHeight(i, 30);
    }
}

void TransformResultDialog::copyMatrixToClipboard(const Eigen::Matrix4d& mat) {
    QString text;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            text += QString::number(mat(i,j), 'f', 8);
            if (j < 3) text += "\t";
        }
        text += "\n";
    }
    QApplication::clipboard()->setText(text);
}

void TransformResultDialog::onCopyAB() {
    copyMatrixToClipboard(currentMatAB);
}

void TransformResultDialog::onCopyBA() {
    copyMatrixToClipboard(currentMatBA);
}

void TransformResultDialog::onRecompute() {
    QDialog::reject();
}
