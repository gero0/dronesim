//
// Created by gero on 7/7/23.
//

#include "PIDPlot.h"

PIDPlot::PIDPlot(QWidget *parent) {
    this->setParent(parent);
    setpoints = new QLineSeries(this);
    setpoints->setName("Setpoint");
    system_states = new QLineSeries(this);
    system_states->setName("Sys state");
    chart = new QChart();
    chartView = new QChartView(chart, this);

    chart->addSeries(setpoints);
    chart->addSeries(system_states);
    chart->createDefaultAxes();
    chart->setTitle("PID output");
    chart->legend()->setVisible(true);

    layout = new QHBoxLayout(this);
    layout->addWidget(chartView);
    this->setMinimumSize(600, 600);

//    chartView->setRenderHint(QPainter::Antialiasing);
}

void PIDPlot::set_x_range(float min, float max){
    chart->axes()[0]->setRange(min, max);
    x_max = max;
}

void PIDPlot::set_y_range(float min, float max){
    chart->axes()[1]->setRange(min, max);
}

void PIDPlot::clear(){
    setpoints->clear();
    system_states->clear();
}

void PIDPlot::append(float time, float setpoint, float value){
    setpoints->append(time, setpoint);
    system_states->append(time, value);

    if (time > x_max) {
        clear();
        chart->axes()[0]->setMin(x_max);
        x_max += 1.0f;
        chart->axes()[0]->setMax(x_max);
    }
}