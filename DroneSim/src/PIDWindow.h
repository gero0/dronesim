//
// Created by gero on 7/6/23.
//

#ifndef DRONESIMPROJECT_PIDWINDOW_H
#define DRONESIMPROJECT_PIDWINDOW_H


#include <QWidget>
#include "PIDSettings.h"
#include "DroneModel.h"

class PIDWindow : public QWidget {
    Q_OBJECT
public:
    explicit PIDWindow(DroneModel* model, QWidget* parent = nullptr);
    void showEvent(QShowEvent* event) override;
public slots:
    void update_thrust(PidTunings tunings);
    void update_pitch(PidTunings tunings);
    void update_roll(PidTunings tunings);
    void update_yaw(PidTunings tunings);
private:
    QGridLayout* layout;
    PIDSettings* thrust_settings;
    PIDSettings* pitch_settings;
    PIDSettings* roll_settings;
    PIDSettings* yaw_settings;
    DroneModel* model;
};


#endif //DRONESIMPROJECT_PIDWINDOW_H
