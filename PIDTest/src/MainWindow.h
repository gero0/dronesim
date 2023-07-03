//
// Created by gero on 6/30/23.
//

#ifndef DRONESIMPROJECT_MAINWINDOW_H
#define DRONESIMPROJECT_MAINWINDOW_H


#include <QMainWindow>
#include <QtCharts>
#include <PID.h>
#include "DroneVerticalModel.h"

class MainWindow : public QMainWindow {
Q_OBJECT
public slots:
    void update_sim();
    void update_setpoint(int val);
    void update_tunings();
public:
    MainWindow();
private:
    PID pid;
    DroneVerticalModel model;
    float dt = 0.01;
    float setpoint = 0.5;
    float current_time = 0.0f;
    float sys_status = 0.0f;
    float x_max = 1.0;
    float alpha = 0.02;

    QLineSeries* setpoints;
    QLineSeries* pid_inputs;
    QLineSeries* pid_outputs;
    QChart *chart;
    QChartView* chartView;
    QTimer* simtimer;
    QSlider* setpoint_slider;
    QLabel* slider_label;

    QLineEdit* p_input;
    QLineEdit* i_input;
    QLineEdit* d_input;

    QLabel* p_label;
    QLabel* i_label;
    QLabel* d_label;
    QPushButton* tunings_button;

    QWidget* init_chart(QWidget* parent);
    QWidget* init_menu(QWidget* parent);
};

#endif //DRONESIMPROJECT_MAINWINDOW_H
