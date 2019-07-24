#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_rsw_rgb_clicked();

    void on_pushButton_rsw_depth_clicked();

    void on_pushButton_rsw_points_clicked();

    void on_pushButton_icp_point_plane_clicked();

    void on_pushButton_icp_colored_clicked();

    void on_pushButton_vo_colored_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
