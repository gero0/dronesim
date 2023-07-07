//
// Created by gero on 7/7/23.
//

#include "PlotWindow.h"

PlotWindow::PlotWindow(QWidget *parent) {
    this->setParent(parent);
    tabs = new QTabWidget(this);

    thrust_plot = new PIDPlot(this);
    pitch_plot = new PIDPlot(this);
    roll_plot = new PIDPlot(this);
    yaw_plot = new PIDPlot(this);

    thrust_plot->set_x_range(0.0f, 3.0f);
    thrust_plot->set_y_range(0.0f, 20.0f);

    pitch_plot->set_x_range(0.0f, 3.0f);
    pitch_plot->set_y_range(-3.14f, 3.14);
    roll_plot->set_x_range(0.0f, 3.0f);
    roll_plot->set_y_range(-3.14f, 3.14);
    yaw_plot->set_x_range(0.0f, 3.0f);
    yaw_plot->set_y_range(-3.14f, 3.14);

    tabs->addTab(thrust_plot, "Thrust");
    tabs->addTab(pitch_plot, "Pitch");
    tabs->addTab(roll_plot, "Roll");
    tabs->addTab(yaw_plot, "Yaw");

    layout = new QHBoxLayout(this);
    layout->addWidget(tabs);
}