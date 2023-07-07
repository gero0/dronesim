//
// Created by gero on 7/6/23.
//

#include "PIDWindow.h"

PIDWindow::PIDWindow(DroneModel *model, QWidget *parent) : model(model) {
    this->setParent(parent);
    thrust_settings = new PIDSettings(this, "Thrust");
    pitch_settings = new PIDSettings(this, "Pitch");
    roll_settings = new PIDSettings(this, "Roll");
    yaw_settings = new PIDSettings(this, "Yaw");
    layout = new QGridLayout(this);

    layout->addWidget(thrust_settings, 0, 0);
    layout->addWidget(pitch_settings, 1, 0);
    layout->addWidget(roll_settings, 2, 0);
    layout->addWidget(yaw_settings, 3, 0);

    connect(thrust_settings,
            SIGNAL(values_updated(PidTunings)), this, SLOT(update_thrust(PidTunings)));
    connect(pitch_settings,
            SIGNAL(values_updated(PidTunings)), this, SLOT(update_pitch(PidTunings)));
    connect(roll_settings,
            SIGNAL(values_updated(PidTunings)), this, SLOT(update_roll(PidTunings)));
    connect(yaw_settings,
            SIGNAL(values_updated(PidTunings)), this, SLOT(update_yaw(PidTunings)));
}

void PIDWindow::showEvent(QShowEvent *event) {
    thrust_settings->set_values(model->controller.get_thrust_tunings());
    pitch_settings->set_values(model->controller.get_pitch_tunings());
    roll_settings->set_values(model->controller.get_roll_tunings());
    yaw_settings->set_values(model->controller.get_yaw_tunings());
}

void PIDWindow::update_thrust(PidTunings tunings) {
    model->controller.set_thrust_tunings(tunings);
}

void PIDWindow::update_pitch(PidTunings tunings) {
    model->controller.set_pitch_tunings(tunings);
}

void PIDWindow::update_roll(PidTunings tunings) {
    model->controller.set_roll_tunings(tunings);
}

void PIDWindow::update_yaw(PidTunings tunings) {
    model->controller.set_yaw_tunings(tunings);
}
