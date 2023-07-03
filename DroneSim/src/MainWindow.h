//
// Created by gero on 6/27/23.
//

#ifndef PLOTTING_MAINWINDOW_H
#define PLOTTING_MAINWINDOW_H

#include <QtWidgets>
#include <QtDataVisualization>
#include "DroneModel.h"

class MainWindow : public QMainWindow {
Q_OBJECT

public slots:
    void update_sim_ui();
    void update_sim();

public:
    MainWindow();

private:
    QWidget *central_widget;
    QHBoxLayout *root_layout;
    Q3DScatter *vis_plot;
    QScatter3DSeries vis_series;
    QScatterDataArray vis_data;
    DroneModel drone;

    QWidget* init_vis(QWidget* parent);
    QWidget* init_bars(QWidget* parent);
    QWidget* init_data(QWidget* parent);

    QLabel* position;
    QLabel* rotation;
    QLabel* velocity;
    QLabel* angular;
};


#endif //PLOTTING_MAINWINDOW_H
