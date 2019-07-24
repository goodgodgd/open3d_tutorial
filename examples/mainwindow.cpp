#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "iovis_examples.h"
#include "registration_examples.h"


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
    IoVis_Examples::ReadShowWrite_RGB("../samples/color1.png", "../results/color1.png");
}

void MainWindow::on_pushButton_rsw_depth_clicked()
{
    IoVis_Examples::ReadShowWrite_Depth("../samples/depth1.png", "../results/depth1.png");
}

void MainWindow::on_pushButton_rsw_points_clicked()
{
    IoVis_Examples::ReadShowWrite_PointCloud("../samples/color1.png", "../samples/depth1.png",
                                             "../results/pointcloud1.pcd");
}

void MainWindow::on_pushButton_icp_point_plane_clicked()
{
    RegistrationExamples::IcpPointCloud("../samples/depth1.png", "../samples/depth2.png");
}

void MainWindow::on_pushButton_icp_colored_clicked()
{
    if(!open3d::utility::filesystem::FileExists("../results/ptcloud1.pcd"))
        RegistrationExamples::RgbDepthToPCD("../samples/color1.png", "../samples/depth1.png",
                                            "../results/ptcloud1.pcd");
    if(!open3d::utility::filesystem::FileExists("../results/ptcloud2.pcd"))
        RegistrationExamples::RgbDepthToPCD("../samples/color2.png", "../samples/depth2.png",
                                            "../results/ptcloud2.pcd");
    RegistrationExamples::IcpColoredPointCloud("../results/ptcloud1.pcd", "../results/ptcloud2.pcd");
}

void MainWindow::on_pushButton_vo_colored_clicked()
{
    RegistrationExamples::VisualOdometryRgbDepth("../samples/color1.png", "../samples/depth1.png",
                                                 "../samples/color2.png", "../samples/depth2.png");
}
