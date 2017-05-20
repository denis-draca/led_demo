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
    delete ui;
}

void MainWindow::_bu_blink_clicked()
{
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
