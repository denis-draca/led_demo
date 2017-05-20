#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <led_demo/led.h>
#include <string>
#include <QString>
#include <QIntValidator>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    bool _sel_z1;
    bool _sel_z2;
    bool _sel_z3;
    bool _sel_z4;

    ros::NodeHandle _n;
    ros::Publisher _pub;

private slots:
    void _bu_blink_clicked();
    void _bu_breath_clicked();
    void _bu_set_colour_clicked();
    void _bu_set_zoff_clicked();
    void _bu_set_OFF_clicked();

    void _ch_z1_clicked(bool checked);
    void _ch_z2_clicked(bool checked);
    void _ch_z3_clicked(bool checked);
    void _ch_z4_clicked(bool checked);

};

#endif // MAINWINDOW_H
