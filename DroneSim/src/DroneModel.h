//
// Created by gero on 6/28/23.
//

#ifndef PLOTTING_DRONE_H
#define PLOTTING_DRONE_H

#include <cmath>
#include <algebra.h>
#include <DroneController.h>
#include "MotorDriverMock.h"
#include "SensorReaderMock.h"

class DroneModel {
public:
    Vector3 position{0.0, 0.0, 0.0};
    Vector3 velocity{0.0, 0.0, 0.0};
    Rotation rotation{0.0, 0.0, 0.0};
    Rotation angular_velocity{0.0, 0.0, 0.0};
//    Vector3 acceleration{0.0, 0.0, 0.0};
//    Vector3 angular_acceleration{0.0, 0.0, 0.0};
    const float arm_len = 0.1f;
    const float motor_max_thrust = 10.0f;//10N
    const float yaw_coeff = 0.01f;
    const float body_mass = 2.0f;
    const float body_radius = 0.03f;
    const float motor_mass = 0.03f;
    const float g = 9.8;

    const float rp_inertia_moment =
            ( 2.0f * body_mass * std::pow(body_radius, 2.0f) / 5.0f )
            + 2.0f * motor_mass * std::pow(arm_len, 2.0f);

    const float yaw_inertia_moment =
            ( 2.0f * body_mass * std::pow(body_radius, 2.0f) / 5.0f )
            + 4.0f * motor_mass * std::pow(arm_len, 2.0f);

    void update(float dt);

    MotorDriverMock fl_driver;
    MotorDriverMock fr_driver;
    MotorDriverMock bl_driver;
    MotorDriverMock br_driver;
    SensorReaderMock sensor_mock;
    DroneController controller{&fl_driver, &fr_driver, &bl_driver, &br_driver, &sensor_mock};
};


#endif //PLOTTING_DRONE_H
