#include "led_demo/mainwindow.h"
#include <QApplication>
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wallpusher_listener");

    try
    {

        QApplication app(argc, argv);
        Q_INIT_RESOURCE(caslogo);

        MainWindow main;

        main.show();

        app.exec();
    }
    catch(const std::exception &e)
    {
        std::cout <<"FAILED TO START: " << e.what() << std::endl;
    }

    return 0;
}
