//
// Created by gero on 7/1/23.
//

#include "DroneController.h"
#include <algorithm>

void DroneController::update(float dt) {
    //for now simple hover
    float altitude = sensor_reader->getAltitude();
    float pitch = 0.0, yaw = 0.0, roll = 0.0;

    float v_thrust = thrust_pid.update(altitude_setpoint, altitude);
    float v_pitch = pitch_pid.update(pitch_setpoint, pitch);
    float v_roll = roll_pid.update(roll_setpoint, roll);
    float v_yaw = yaw_pid.update(yaw_setpoint, yaw);

    front_left->set_speed(std::clamp(v_thrust + v_pitch - v_roll - v_yaw,0.0f, 1.0f));
    front_right->set_speed(std::clamp(v_thrust + v_pitch + v_roll + v_yaw,0.0f, 1.0f));
    back_left->set_speed(std::clamp(v_thrust - v_pitch - v_roll + v_yaw,0.0f, 1.0f));
    back_right->set_speed(std::clamp(v_thrust - v_pitch + v_roll - v_yaw,0.0f, 1.0f));
}