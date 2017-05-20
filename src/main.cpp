#include "led_demo/mainwindow.h"
#include <QApplication>
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wallpusher_listener");

    QApplication app(argc, argv);

    MainWindow main;

    main.show();

    app.exec();

    return 0;
}
