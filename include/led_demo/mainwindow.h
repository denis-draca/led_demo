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
#include <QDir>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <QTimer>
#include <sys/wait.h>

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

    bool _sel_table1;
    bool _sel_table2;

    bool _connected;
    bool _connected_2;
    bool _detected;

    ros::NodeHandle _n;
    ros::Publisher _pub_base1;
    ros::Publisher _pub_base2;

    std::string _selected_arduino;
    std::string python_load;

    pid_t pid;
    pid_t pid_2;

    std::vector<std::string> _connected_arduino;
    QTimer *timer;
    QTimer *timer2;

private:


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

    void on__combo1_activated(const QString &arg1);
    void on__bu_connect_clicked();
    void on__bu_disconnect_clicked();
    void on__ch_table1_clicked(bool checked);

    void on__ch_table2_clicked(bool checked);
    void on__bu_breath_rndRGB_clicked();
    void new_thread();
    void safety_thread();
};

#endif // MAINWINDOW_H
