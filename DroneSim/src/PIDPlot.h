//
// Created by gero on 7/7/23.
//

#ifndef DRONESIMPROJECT_PIDPLOT_H
#define DRONESIMPROJECT_PIDPLOT_H


#include <QtCharts/QLineSeries>
#include <QtCharts/QChartView>
#include <QBoxLayout>

class PIDPlot : public QWidget {
Q_OBJECT

public:
    PIDPlot(QWidget* parent = nullptr);
    void clear();
    void append(float time, float setpoint, float value);
    void set_x_range(float min, float max);
    void set_y_range(float min, float max);
private:
    QLineSeries *setpoints;
    QLineSeries *system_states;
    QChart *chart;
    QChartView *chartView;
    QHBoxLayout* layout;

    float x_max = 1.0;
};


#endif //DRONESIMPROJECT_PIDPLOT_H
