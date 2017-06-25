#include "led_demo/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    _pub_base1 = _n.advertise<led_demo::led>("/sawyer/base/1", 1);
    _pub_base2 = _n.advertise<led_demo::led>("/sawyer/base/2", 1);

    ui->setupUi(this);
    this->setWindowTitle("LED Demo");

    _sel_z1 = false;
    _sel_z2 = false;
    _sel_z3 = false;
    _sel_z4 = false;

    _sel_table1 = false;
    _sel_table2 = false;

    _connected = false;
    _connected_2 = false;
    _detected = false;

    ui->_ch_z1->setCheckable(true);
    ui->_ch_z2->setCheckable(true);
    ui->_ch_z3->setCheckable(true);
    ui->_ch_z4->setCheckable(true);

    ui->_in_r->setValidator(new QIntValidator(0, 255, this));
    ui->_in_g->setValidator(new QIntValidator(0, 255, this));
    ui->_in_b->setValidator(new QIntValidator(0, 255, this));
    ui->_in_rate->setValidator(new QIntValidator(0, 10000, this));

    connect(ui->_bu_blink, SIGNAL(clicked()), this, SLOT (_bu_blink_clicked()));
    connect(ui->_bu_breath, SIGNAL(clicked()), this, SLOT (_bu_breath_clicked()));
    connect(ui->_bu_set_colour, SIGNAL(clicked()), this, SLOT (_bu_set_colour_clicked()));
    connect(ui->_bu_zone_off, SIGNAL(clicked()), this, SLOT (_bu_set_zoff_clicked()));
    connect(ui->_bu_all_off, SIGNAL(clicked()), this, SLOT (_bu_set_OFF_clicked()));

    connect(ui->_ch_z1, SIGNAL(clicked(bool)), this, SLOT(_ch_z1_clicked(bool)));
    connect(ui->_ch_z2, SIGNAL(clicked(bool)), this, SLOT(_ch_z2_clicked(bool)));
    connect(ui->_ch_z3, SIGNAL(clicked(bool)), this, SLOT(_ch_z3_clicked(bool)));
    connect(ui->_ch_z4, SIGNAL(clicked(bool)), this, SLOT(_ch_z4_clicked(bool)));

    timer = new QTimer(this);
    timer2 = new QTimer(this);

    connect(timer, SIGNAL(timeout()), this, SLOT(new_thread()));
    connect(timer2, SIGNAL(timeout()), this, SLOT(safety_thread()));

    timer->start(100);
    timer2->start();

    _connected_arduino.resize(2);

    std::cout << "checking ros" << std::endl;

    if(!ros::ok())
    {
        throw std::invalid_argument("ros not on");
    }

}

MainWindow::~MainWindow()
{
    if(_connected)
        kill(-pid, SIGKILL);

    if(_connected_2)
        kill(-pid_2, SIGKILL);

    delete ui;
}

void MainWindow::new_thread()
{
    bool new_detect = false;
    bool first = true;

    for(int i = 0; i < 255; i++)
    {
        std::string test = "/sys/class/tty/ttyACM";
        test.append(std::to_string(i));
        if(QDir(test.c_str()).exists())
        {
            std::string temp = "ttyACM";
            temp.append(std::to_string(i));

            bool present = false;

            for(int index = 0; index < ui->_combo1->count(); index++)
            {
                if(ui->_combo1->itemText(index).toUtf8().constData() == temp)
                {
                    present = true;
                }
            }

            if(!present)
            {
               ui->_combo1->addItem(temp.c_str());
            }

            new_detect = true;

            if(first)
            {
                _selected_arduino = temp;
                first = false;
            }
        }
    }

    if(!new_detect)
    {
        ui->_combo1->clear();
    }

    _detected = new_detect;
}

void MainWindow::safety_thread()
{
    int status = 0;
    int status2 = 0;

    pid_t w;
    pid_t w2;

    w = waitpid(pid, &status, WNOHANG);
    w2 = waitpid(pid_2, &status2, WNOHANG);

    if(_connected)
    {
        if(w != 0)
        {
            kill(-pid, SIGKILL);
            ui->_out1->setText("lost connection with table 1");

            _connected = false;

            _connected_arduino.at(0).clear();
        }

    }

    if(_connected_2)
    {
        if(w2 != 0)
        {
            kill(-pid_2, SIGKILL);
            ui->_out1->setText("lost connection with table 2");

            _connected_2 = false;

            _connected_arduino.at(1).clear();
        }
    }
}

void MainWindow::_bu_blink_clicked()
{
    if(!_connected && !_connected_2)
    {
        ui->_out1->setText("No connection has been established");
        return;
    }

    if(!_sel_table1 && !_sel_table2)
    {
        ui->_out1->setText("Select which table you want to set");
        return;
    }


    if(!_connected && _sel_table1)
    {
        ui->_out1->setText("table 1 is not connected. Cancelling Operation");
        return;
    }

    if(!_connected_2 && _sel_table2)
    {
        ui->_out1->setText("table 2 is not connected. Cancelling Operation");
        return;
    }

    std::string r = ui->_in_r->text().toUtf8().constData();
    std::string g = ui->_in_g->text().toUtf8().constData();
    std::string b = ui->_in_b->text().toUtf8().constData();
    std::string rate = ui->_in_rate->text().toUtf8().constData();

    if(r.empty())
    {
        ui->_out1->setText("Please select R value");
        return;
    }
    else if(g.empty())
    {
        ui->_out1->setText("Please select G value");
        return;
    }
    else if(b.empty())
    {
        ui->_out1->setText("Please select B value");
        return;
    }
    else if(rate.empty())
    {
        ui->_out1->setText("Please select rate of change");
        return;
    }

    if(!_sel_z1 && !_sel_z2 && !_sel_z3 && !_sel_z4)
    {
        ui->_out1->setText("no zones have been selected");
        return;
    }

    led_demo::led led_msg;
    std::string display = "Following Zones Will now Blink at given rate: \n";

    if (_sel_z1)
    {
        display.append("Zone 1 \n");
        led_msg.zone.push_back(1);
    }

    if (_sel_z2)
    {
        display.append("Zone 2 \n");
        led_msg.zone.push_back(2);
    }

    if (_sel_z3)
    {
        display.append("Zone 3 \n");
        led_msg.zone.push_back(3);
    }

    if (_sel_z4)
    {
        display.append("Zone 4 \n");
        led_msg.zone.push_back(4);
    }

    display.append("Given rate of change: ");

    display.append(rate);
    display.append("ms \n");

    ui->_out1->setText(display.c_str());

    led_msg.ON = true;
    led_msg.self_control = false;
    led_msg.rgb.push_back(std::stoi(r));
    led_msg.rgb.push_back(std::stoi(g));
    led_msg.rgb.push_back(std::stoi(b));
    led_msg.rgb.push_back(0);
    led_msg.rate = std::stoi(rate);
    led_msg.blink = true;

    if(_sel_table1)
        _pub_base1.publish(led_msg);

    if(_sel_table2)
        _pub_base2.publish(led_msg);
}

void MainWindow::_bu_breath_clicked()
{
    if(!_connected && !_connected_2)
    {
        ui->_out1->setText("No connection has been established");
        return;
    }

    if(!_sel_table1 && !_sel_table2)
    {
        ui->_out1->setText("Select which table you want to set");
        return;
    }


    if(!_connected && _sel_table1)
    {
        ui->_out1->setText("table 1 is not connected. Cancelling Operation");
        return;
    }

    if(!_connected_2 && _sel_table2)
    {
        ui->_out1->setText("table 2 is not connected. Cancelling Operation");
        return;
    }

    std::string r = ui->_in_r->text().toUtf8().constData();
    std::string g = ui->_in_g->text().toUtf8().constData();
    std::string b = ui->_in_b->text().toUtf8().constData();
    std::string rate = ui->_in_rate->text().toUtf8().constData();

    if(r.empty())
    {
        ui->_out1->setText("Please select R value");
        return;
    }
    else if(g.empty())
    {
        ui->_out1->setText("Please select G value");
        return;
    }
    else if(b.empty())
    {
        ui->_out1->setText("Please select B value");
        return;
    }
    else if(rate.empty())
    {
        ui->_out1->setText("Please select rate of change");
        return;
    }

    if(!_sel_z1 && !_sel_z2 && !_sel_z3 && !_sel_z4)
    {
        ui->_out1->setText("no zones have been selected");
        return;
    }

    led_demo::led led_msg;
    std::string display = "Following Zones Will now Pulse at given rate: \n";

    if (_sel_z1)
    {
        display.append("Zone 1 \n");
        led_msg.zone.push_back(1);
    }

    if (_sel_z2)
    {
        display.append("Zone 2 \n");
        led_msg.zone.push_back(2);
    }

    if (_sel_z3)
    {
        display.append("Zone 3 \n");
        led_msg.zone.push_back(3);
    }

    if (_sel_z4)
    {
        display.append("Zone 4 \n");
        led_msg.zone.push_back(4);
    }

    display.append("Given rate of change: ");

    display.append(rate);
    display.append("ms \n");
    ui->_out1->setText(display.c_str());

    led_msg.ON = true;
    led_msg.self_control = false;
    led_msg.rgb.push_back(std::stoi(r));
    led_msg.rgb.push_back(std::stoi(g));
    led_msg.rgb.push_back(std::stoi(b));
    led_msg.rgb.push_back(0);
    led_msg.rate = std::stoi(rate);
    led_msg.blink = false;

    if(_sel_table1)
        _pub_base1.publish(led_msg);

    if(_sel_table2)
        _pub_base2.publish(led_msg);
}

void MainWindow::_bu_set_colour_clicked()
{
    if(!_connected && !_connected_2)
    {
        ui->_out1->setText("No connection has been established");
        return;
    }

    if(!_sel_table1 && !_sel_table2)
    {
        ui->_out1->setText("Select which table you want to set");
        return;
    }


    if(!_connected && _sel_table1)
    {
        ui->_out1->setText("table 1 is not connected. Cancelling Operation");
        return;
    }

    if(!_connected_2 && _sel_table2)
    {
        ui->_out1->setText("table 2 is not connected. Cancelling Operation");
        return;
    }

    std::string r = ui->_in_r->text().toUtf8().constData();
    std::string g = ui->_in_g->text().toUtf8().constData();
    std::string b = ui->_in_b->text().toUtf8().constData();

    if(r.empty())
    {
        ui->_out1->setText("Please select R value");
        return;
    }
    else if(g.empty())
    {
        ui->_out1->setText("Please select G value");
        return;
    }
    else if(b.empty())
    {
        ui->_out1->setText("Please select B value");
        return;
    }

    if(!_sel_z1 && !_sel_z2 && !_sel_z3 && !_sel_z4)
    {
        ui->_out1->setText("no zones have been selected");
        return;
    }

    led_demo::led led_msg;
    std::string display = "Following Zones Will be the given solid colour \n";

    if (_sel_z1)
    {
        display.append("Zone 1 \n");
        led_msg.zone.push_back(1);
    }

    if (_sel_z2)
    {
        display.append("Zone 2 \n");
        led_msg.zone.push_back(2);
    }

    if (_sel_z3)
    {
        display.append("Zone 3 \n");
        led_msg.zone.push_back(3);
    }

    if (_sel_z4)
    {
        display.append("Zone 4 \n");
        led_msg.zone.push_back(4);
    }

    ui->_out1->setText(display.c_str());

    led_msg.ON = true;
    led_msg.self_control = true;
    led_msg.rgb.push_back(std::stoi(r));
    led_msg.rgb.push_back(std::stoi(g));
    led_msg.rgb.push_back(std::stoi(b));
    led_msg.rgb.push_back(0);
    led_msg.blink = false;

    if(_sel_table1)
        _pub_base1.publish(led_msg);

    if(_sel_table2)
        _pub_base2.publish(led_msg);
}

void MainWindow::_bu_set_zoff_clicked()
{
    if(!_connected && !_connected_2)
    {
        ui->_out1->setText("No connection has been established");
        return;
    }

    if(!_sel_table1 && !_sel_table2)
    {
        ui->_out1->setText("Select which table you want to set");
        return;
    }


    if(!_connected && _sel_table1)
    {
        ui->_out1->setText("table 1 is not connected. Cancelling Operation");
        return;
    }

    if(!_connected_2 && _sel_table2)
    {
        ui->_out1->setText("table 2 is not connected. Cancelling Operation");
        return;
    }

    if(!_sel_z1 && !_sel_z2 && !_sel_z3 && !_sel_z4)
    {
        ui->_out1->setText("no zones have been selected");
        return;
    }

    led_demo::led led_msg;
    std::string display = "Following Zones Will be turned off \n";

    if (_sel_z1)
    {
        display.append("Zone 1 \n");
        led_msg.zone.push_back(1);
    }

    if (_sel_z2)
    {
        display.append("Zone 2 \n");
        led_msg.zone.push_back(2);
    }

    if (_sel_z3)
    {
        display.append("Zone 3 \n");
        led_msg.zone.push_back(3);
    }

    if (_sel_z4)
    {
        display.append("Zone 4 \n");
        led_msg.zone.push_back(4);
    }

    ui->_out1->setText(display.c_str());

    led_msg.ON = false;

    led_msg.rgb.push_back(0);
    led_msg.rgb.push_back(0);
    led_msg.rgb.push_back(0);
    led_msg.rgb.push_back(0);

    if(_sel_table1)
        _pub_base1.publish(led_msg);

    if(_sel_table2)
        _pub_base2.publish(led_msg);
}

void MainWindow::_bu_set_OFF_clicked()
{
    if(!_connected && !_connected_2)
    {
        ui->_out1->setText("No connection has been established");
        return;
    }
    ui->_out1->setText("all connected zones will shut off");
    led_demo::led led_msg;
    led_msg.zone.push_back(1);
    led_msg.zone.push_back(2);
    led_msg.zone.push_back(3);
    led_msg.zone.push_back(4);

    led_msg.rgb.push_back(0);
    led_msg.rgb.push_back(0);
    led_msg.rgb.push_back(0);
    led_msg.rgb.push_back(0);

    led_msg.ON = false;

    if(_connected)
        _pub_base1.publish(led_msg);

    if(_connected_2)
        _pub_base2.publish(led_msg);
}

void MainWindow::_ch_z1_clicked(bool checked)
{
    if(checked)
    {
        _sel_z1 = true;
    }

    else
    {
        _sel_z1 = false;
    }

}

void MainWindow::_ch_z2_clicked(bool checked)
{
    if(checked)
        _sel_z2 = true;
    else
        _sel_z2 = false;
}

void MainWindow::_ch_z3_clicked(bool checked)
{
    if(checked)
        _sel_z3 = true;
    else
        _sel_z3 = false;
}

void MainWindow::_ch_z4_clicked(bool checked)
{
    if(checked)
        _sel_z4 = true;
    else
        _sel_z4 = false;
}

void MainWindow::on__combo1_activated(const QString &arg1)
{
    _selected_arduino = arg1.toUtf8().constData();
}

void MainWindow::on__bu_connect_clicked()
{
    if(!_detected)
    {
        ui->_out1->setText("No Detected tables found");
        return;
    }
    for(unsigned int i = 0; i < _connected_arduino.size(); i++)
    {
        if(_connected_arduino.at(i) == _selected_arduino)
        {
            ui->_out1->setText("Already connected. Either disconnect current base to use same port or attach other base to different USB port");
            return;
        }
    }

    python_load.clear();
    python_load = "python /home/denis/catkin_ws/src/rosserial/rosserial_python/nodes/serial_node.py _port:=/dev/";
    python_load = python_load + _selected_arduino;
    python_load.append(" _baud:=115200");

    std::string str = "connecting to arduino on port: \n";
    str = str + _selected_arduino;
    ui->_out1->setText(str.c_str());

    if(!_connected)
    {
        pid = fork();
        if(pid == 0)
        {
            setpgid(getpid(), getpid());
            system(python_load.c_str());
        }

        _connected_arduino.at(0) = _selected_arduino;

        _connected = true;
    }
    else if(!_connected_2)
    {
        pid_2 = fork();
        if(pid_2 == 0)
        {
            setpgid(getpid(), getpid());
            system(python_load.c_str());
        }

        _connected_arduino.at(1) = _selected_arduino;
        _connected_2 = true;
    }

}

void MainWindow::on__bu_disconnect_clicked()
{
    if(!_connected && !_connected_2)
    {
        ui->_out1->setText("Nothing to disconnect");
        return;
    }


    if(_connected)
    {
        kill(-pid, SIGKILL);
    }

    if(_connected_2)
    {
        kill(-pid_2, SIGKILL);
    }
    ui->_out1->setText("Connection Terminated");

    ROS_INFO("CONNECTION HAS BEEN KILLED");
    _connected = false;
    _connected_2 = false;

    _connected_arduino.at(0).clear();
    _connected_arduino.at(1).clear();
}

void MainWindow::on__ch_table1_clicked(bool checked)
{
    if(checked)
        _sel_table1 = true;
    else
        _sel_table1 = false;
}

void MainWindow::on__ch_table2_clicked(bool checked)
{
    if(checked)
        _sel_table2 = true;
    else
        _sel_table2 = false;
}

void MainWindow::on__bu_breath_rndRGB_clicked()
{
    if(!_connected && !_connected_2)
    {
        ui->_out1->setText("No connection has been established");
        return;
    }

    if(!_sel_table1 && !_sel_table2)
    {
        ui->_out1->setText("Select which table you want to set");
        return;
    }


    if(!_connected && _sel_table1)
    {
        ui->_out1->setText("table 1 is not connected. Cancelling Operation");
        return;
    }

    if(!_connected_2 && _sel_table2)
    {
        ui->_out1->setText("table 2 is not connected. Cancelling Operation");
        return;
    }

    std::string r = ui->_in_r->text().toUtf8().constData();
    std::string g = ui->_in_g->text().toUtf8().constData();
    std::string b = ui->_in_b->text().toUtf8().constData();
    std::string rate = ui->_in_rate->text().toUtf8().constData();

    if(r.empty())
    {
        ui->_out1->setText("Please select R value");
        return;
    }
    else if(g.empty())
    {
        ui->_out1->setText("Please select G value");
        return;
    }
    else if(b.empty())
    {
        ui->_out1->setText("Please select B value");
        return;
    }
    else if(rate.empty())
    {
        ui->_out1->setText("Please select rate of change");
        return;
    }

    if(!_sel_z1 && !_sel_z2 && !_sel_z3 && !_sel_z4)
    {
        ui->_out1->setText("no zones have been selected");
        return;
    }

    led_demo::led led_msg;
    std::string display = "Following Zones Will now Pulse at given rate: \n";

    if (_sel_z1)
    {
        display.append("Zone 1 \n");
        led_msg.zone.push_back(1);
    }

    if (_sel_z2)
    {
        display.append("Zone 2 \n");
        led_msg.zone.push_back(2);
    }

    if (_sel_z3)
    {
        display.append("Zone 3 \n");
        led_msg.zone.push_back(3);
    }

    if (_sel_z4)
    {
        display.append("Zone 4 \n");
        led_msg.zone.push_back(4);
    }

    display.append("Given rate of change: ");

    display.append(rate);
    display.append("ms \n");

    ui->_out1->setText(display.c_str());

    led_msg.ON = true;
    led_msg.self_control = false;
    led_msg.rgb.push_back(std::stoi(r));
    led_msg.rgb.push_back(std::stoi(g));
    led_msg.rgb.push_back(std::stoi(b));
    led_msg.rgb.push_back(1);
    led_msg.rate = std::stoi(rate);
    led_msg.blink = false;

    if(_sel_table1)
        _pub_base1.publish(led_msg);

    if(_sel_table2)
        _pub_base2.publish(led_msg);
}
