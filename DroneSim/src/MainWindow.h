//
// Created by gero on 6/27/23.
//

#ifndef PLOTTING_MAINWINDOW_H
#define PLOTTING_MAINWINDOW_H

#include <QtWidgets>
#include <QtDataVisualization>
#include <iostream>
#include "DroneModel.h"
#include "PIDWindow.h"
#include "PlotWindow.h"

class MainWindow : public QMainWindow {
Q_OBJECT

public slots:
    void update_sim();
    void open_pid_settings();
    void open_plot_window();

public:
    MainWindow();
    ~MainWindow() override;

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
    float current_time = 0.0f;

    QWidget* init_visualization(QWidget* parent);
    QWidget* init_bars(QWidget* parent);
    QWidget* init_menu_widgets(QWidget* parent);

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

    PIDWindow* pid_window;
    PlotWindow* plot_window;
    QPushButton* open_pid_window_btn;
    QPushButton* open_plot_window_btn;

    void draw_scatter();

    void draw_text();

    void rescale_axes();
};


#endif //PLOTTING_MAINWINDOW_H
