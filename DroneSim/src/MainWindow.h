//
// Created by gero on 6/27/23.
//

#ifndef PLOTTING_MAINWINDOW_H
#define PLOTTING_MAINWINDOW_H

#include <QtWidgets>
#include <QtDataVisualization>
#include <iostream>
#include "DroneModel.h"

class MainWindow : public QMainWindow {
Q_OBJECT

public slots:
    void update_sim_ui();
    void update_sim();

public:
    MainWindow();

protected:
    void keyPressEvent(QKeyEvent *event) override;

private:
    QWidget *central_widget;
    QHBoxLayout *root_layout;
    Q3DScatter *vis_plot;
    QScatter3DSeries vis_series;
    QScatterDataArray vis_data;
    DroneModel drone;

    const float dt = 0.01;

    QWidget* init_vis(QWidget* parent);
    QWidget* init_bars(QWidget* parent);
    QWidget* init_data(QWidget* parent);

    QLabel* position;
    QLabel* rotation;
    QLabel* velocity;
    QLabel* angular;
    QLabel* pid_out;
    QLabel* pid_setpoints;

    QProgressBar* bar_fl;
    QProgressBar* bar_fr;
    QProgressBar* bar_bl;
    QProgressBar* bar_br;

    QLabel* label_fl;
    QLabel* label_fr;
    QLabel* label_bl;
    QLabel* label_br;

    void draw_scatter();

    void draw_text();

    void rescale_axes();
};


#endif //PLOTTING_MAINWINDOW_H
