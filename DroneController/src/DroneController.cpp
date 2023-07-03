//
// Created by gero on 7/1/23.
//

#include "DroneController.h"
#include <algorithm>
#include <iostream>

void DroneController::update(float dt) {
    //for now simple hover
    float altitude = sensor_reader->getAltitude();
    auto [pitch_acc, yaw_acc, roll_acc] = sensor_reader->getAngularAcceleration();

    angular_velocity.pitch += pitch_acc * dt;
    angular_velocity.roll += roll_acc * dt;
    angular_velocity.yaw += yaw_acc * dt;

    rotation.pitch += angular_velocity.pitch * dt;
    rotation.roll += angular_velocity.roll * dt;
    rotation.yaw += angular_velocity.yaw * dt;

    float v_thrust = thrust_pid.update(altitude_setpoint, altitude);
    float v_pitch = pitch_pid.update(pitch_setpoint, rotation.pitch);
    float v_roll = roll_pid.update(roll_setpoint, rotation.roll);
    float v_yaw = yaw_pid.update(yaw_setpoint, rotation.yaw);

    std::cout << "Vpitch: " << v_pitch << "\n";

    front_left->set_speed(std::clamp(v_thrust + v_pitch - v_roll + v_yaw,0.0f, 1.0f));
    front_right->set_speed(std::clamp(v_thrust + v_pitch + v_roll - v_yaw,0.0f, 1.0f));
    back_left->set_speed(std::clamp(v_thrust - v_pitch - v_roll - v_yaw,0.0f, 1.0f));
    back_right->set_speed(std::clamp(v_thrust - v_pitch + v_roll + v_yaw,0.0f, 1.0f));
}