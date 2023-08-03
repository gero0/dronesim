//
// Created by gero on 7/7/23.
//

#ifndef DRONESIMPROJECT_PLOTWINDOW_H
#define DRONESIMPROJECT_PLOTWINDOW_H

#include <QWidget>
#include <QTabWidget>
#include "PIDPlot.h"

class PlotWindow : public QWidget{
    Q_OBJECT
public:
    PlotWindow(QWidget* parent = nullptr);
    PIDPlot* thrust_plot;
    PIDPlot* pitch_plot;
    PIDPlot* roll_plot;
    PIDPlot* yaw_plot;
    PIDPlot* x_plot;
    PIDPlot* y_plot;
private:
    QTabWidget* tabs;
    QHBoxLayout* layout;
};


#endif //DRONESIMPROJECT_PLOTWINDOW_H
