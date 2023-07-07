//
// Created by gero on 6/30/23.
//

#include "MainWindow.h"
#include <iostream>

QWidget *MainWindow::init_chart(QWidget *parent) {
    chart = new QChart();
    chartView = new QChartView(chart, parent);

    setpoints = new QLineSeries(chartView);
    setpoints->setName("Set points");
    pid_inputs = new QLineSeries(chartView);
    pid_inputs->setName("Sys state");
    pid_outputs = new QLineSeries(chartView);
    pid_outputs->setName("PID output");

    chart->addSeries(setpoints);
    chart->addSeries(pid_inputs);
    chart->addSeries(pid_outputs);
    chart->createDefaultAxes();
    chart->setTitle("PID output");
    chart->legend()->setVisible(true);

    chart->axes()[1]->setMin(-0.3);
    chart->axes()[1]->setMax(2.0);
    chart->axes()[0]->setMin(0.0);
    chart->axes()[0]->setMax(x_max);

    chartView->setRenderHint(QPainter::Antialiasing);

    return chartView;
}

QWidget *MainWindow::init_menu(QWidget *parent) {
    auto menu = new QWidget(parent);
    auto layout = new QGridLayout(menu);

    setpoint_slider = new QSlider(menu);
    setpoint_slider->setValue(50);
    slider_label = new QLabel("Setpoint", parent);

    p_input = new QLineEdit(menu);
    i_input = new QLineEdit(menu);
    d_input = new QLineEdit(menu);

    p_input->setText("0.1");
    i_input->setText("0.1");
    d_input->setText("0.0");

    p_label = new QLabel("Kp", menu);
    i_label = new QLabel("Ki", menu);
    d_label = new QLabel("Kd", menu);

    tunings_button = new QPushButton("Set tunings", menu);

    layout->addWidget(setpoint_slider, 0, 0, 3, 1);
    layout->addWidget(slider_label, 3, 0);
    layout->addWidget(p_label, 0,1);
    layout->addWidget(i_label, 1,1);
    layout->addWidget(d_label, 2,1);
    layout->addWidget(p_input, 0,2);
    layout->addWidget(i_input, 1,2);
    layout->addWidget(d_input, 2,2);
    layout->addWidget(tunings_button, 3,2);
    menu->setLayout(layout);

    connect(setpoint_slider, SIGNAL(sliderMoved(int)), this, SLOT(update_setpoint(int)));
    connect(tunings_button, SIGNAL(clicked(bool)), this, SLOT(update_tunings()));

    return menu;
}

void MainWindow::update_setpoint(int val) {
    setpoint = val / 100.0f;
}

MainWindow::MainWindow() {
    auto central_widget = new QWidget(this);
    auto central_layout = new QHBoxLayout(central_widget);

    auto chart_widget = init_chart(central_widget);
    auto menu_widget = init_menu(central_widget);

    QSizePolicy spLeft(QSizePolicy::Preferred, QSizePolicy::Preferred);
    spLeft.setHorizontalStretch(4);
    QSizePolicy spRight(QSizePolicy::Preferred, QSizePolicy::Preferred);
    spRight.setHorizontalStretch(1);

    chart_widget->setSizePolicy(spLeft);
    menu_widget->setSizePolicy(spRight);

    central_layout->addWidget(chart_widget);
    central_layout->addWidget(menu_widget);

    this->setCentralWidget(central_widget);
    this->resize(800, 600);

    pid.set_tunings(0.1, 0.1, 0.000);
    pid.set_windup(0.0, 1.0f);
    pid.set_clamp(0.0, 1.0);

    simtimer = new QTimer();
    connect(simtimer, SIGNAL(timeout()), this, SLOT(update_sim()));
    simtimer->start(dt * 1000.0f);
}

void MainWindow::update_sim() {
    float pid_output = pid.update(setpoint, sys_status, dt);

    setpoints->append(current_time, setpoint);
    pid_inputs->append(current_time, sys_status);
    pid_outputs->append(current_time, pid_output);

//    sys_status += (pid_output) / (1.0f + alpha * dt);
    sys_status = model.update(dt, pid_output);
    current_time += dt;

    if (current_time > x_max) {
        setpoints->clear();
        pid_inputs->clear();
        pid_outputs->clear();
        chart->axes()[0]->setMin(x_max);
        x_max += 3;
        chart->axes()[0]->setMax(x_max);
    }
}

float parse_or_default(const std::string &str, float def) {
    try {
        return std::stof(str);
    } catch (std::exception &e) {
        std::cerr << "Could not parse input: " << str << " !\n Setting default value of: " << def << "\n";
        return def;
    }
}

void MainWindow::update_tunings() {
    auto Kp = parse_or_default(p_input->text().toStdString(), 0.1);
    auto Ki = parse_or_default(i_input->text().toStdString(), 0.1);
    auto Kd = parse_or_default(d_input->text().toStdString(), 0.000);
    pid.set_tunings(Kp, Ki, Kd);
}
