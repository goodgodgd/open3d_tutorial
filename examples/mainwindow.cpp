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
    Open3dExamples::ReadShowWrite_RGB("../samples/color1.png", "../results/gray1.png");
}
