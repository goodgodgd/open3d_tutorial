#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "open3dexamples.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_rsw_rgb_clicked()
{
    Open3dExamples::ReadShowWrite_RGB("../samples/color1.png", "../results/color1.png");
}

void MainWindow::on_pushButton_rsw_depth_clicked()
{
    Open3dExamples::ReadShowWrite_Depth("../samples/depth1.png", "../results/depth1.png");
}

void MainWindow::on_pushButton_rsw_points_clicked()
{
    Open3dExamples::ReadShowWrite_PointCloud("../samples/color1.png", "../samples/depth1.png",
                                             "../results/pointcloud1.pcd");
}
