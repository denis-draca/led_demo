#include "led_demo/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    _pub = _n.advertise<led_demo::led>("/chat_test",1);

    ui->setupUi(this);
    this->setWindowTitle("LED Demo");

    _sel_z1 = false;
    _sel_z2 = false;
    _sel_z3 = false;
    _sel_z4 = false;

    _sel_table1 = false;
    _sel_table2 = false;

    _connected = false;
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
}

MainWindow::~MainWindow()
{
    kill(-pid, SIGKILL);
    delete ui;
}

void MainWindow::_bu_blink_clicked()
{
    if(!_connected)
    {
        ui->_out1->setText("No connection has been established");
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
    led_msg.rate = std::stoi(rate);
    led_msg.blink = true;

    _pub.publish(led_msg);

}

void MainWindow::_bu_breath_clicked()
{
    if(!_connected)
    {
        ui->_out1->setText("No connection has been established");
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
    led_msg.rate = std::stoi(rate);
    led_msg.blink = false;

    _pub.publish(led_msg);
}

void MainWindow::_bu_set_colour_clicked()
{
    if(!_connected)
    {
        ui->_out1->setText("No connection has been established");
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
    led_msg.blink = false;

    _pub.publish(led_msg);
}

void MainWindow::_bu_set_zoff_clicked()
{
    if(!_connected)
    {
        ui->_out1->setText("No connection has been established");
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
    _pub.publish(led_msg);
}

void MainWindow::_bu_set_OFF_clicked()
{
    if(!_connected)
    {
        ui->_out1->setText("No connection has been established");
        return;
    }
    ui->_out1->setText("all zones will shut off");
    led_demo::led led_msg;
    led_msg.zone.push_back(1);
    led_msg.zone.push_back(2);
    led_msg.zone.push_back(3);
    led_msg.zone.push_back(4);
    led_msg.ON = false;

    _pub.publish(led_msg);
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


void MainWindow::on__bu_detect_clicked()
{
    bool first = true;
    ui->_combo1->clear();
    for(int i = 0; i < 255; i++)
    {
        std::string test = "/sys/class/tty/ttyACM";
        test.append(std::to_string(i));
        if(QDir(test.c_str()).exists())
        {
            _detected = true;
            std::string temp = "ttyACM";
            temp.append(std::to_string(i));
            ui->_combo1->addItem(temp.c_str());

            if(first)
            {
                _selected_arduino = temp;
                first = false;
            }
        }
    }

    if(!_detected)
    {
        ui->_out1->setText("No connected arduino's detected");
    }
}

void MainWindow::on__bu_connect_clicked()
{
    if(!_detected)
    {
        ui->_out1->setText("Please detect available arduino's first");
        return;
    }
    if(_connected)
    {
        ui->_out1->setText("Already connected. Please disconect before connecting to a different lighting system");
        return;
    }

    python_load.clear();
    python_load = "python /home/denis/catkin_ws/src/rosserial/rosserial_python/nodes/serial_node.py _port:=/dev/";
    python_load = python_load + _selected_arduino;
    python_load.append(" _baud:=115200");

    std::string str = "connecting to arduino on port: \n";
    str = str + _selected_arduino;
    ui->_out1->setText(str.c_str());

    pid = fork();
    if(pid == 0)
    {
        setpgid(getpid(), getpid());
        system(python_load.c_str());
    }

    _connected = true;
}

void MainWindow::on__bu_disconnect_clicked()
{
    if(!_connected)
    {
        ui->_out1->setText("Nothing to disconnect");
        return;
    }
    ui->_out1->setText("Connection being terminated");
    kill(-pid, SIGKILL);
    ui->_out1->setText("Connection Terminated");

    _connected = false;
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
