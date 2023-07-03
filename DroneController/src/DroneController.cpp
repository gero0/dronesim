//
// Created by gero on 7/1/23.
//

#include "DroneController.h"
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

    v_thrust = thrust_pid.update(altitude_setpoint, altitude);
    v_pitch = pitch_pid.update(pitch_setpoint, rotation.pitch);
    v_roll = roll_pid.update(roll_setpoint, rotation.roll);
    v_yaw = yaw_pid.update(yaw_setpoint, rotation.yaw);

    front_left->set_speed(std::clamp(v_thrust + v_pitch - v_roll + v_yaw, 0.0f, 1.0f));
    front_right->set_speed(std::clamp(v_thrust + v_pitch + v_roll - v_yaw, 0.0f, 1.0f));
    back_left->set_speed(std::clamp(v_thrust - v_pitch - v_roll - v_yaw, 0.0f, 1.0f));
    back_right->set_speed(std::clamp(v_thrust - v_pitch + v_roll + v_yaw, 0.0f, 1.0f));
}

PidValues DroneController::get_last_pid() {
    return {v_thrust, v_pitch, v_roll, v_yaw};
}

PidValues DroneController::get_setpoints() {
    return {altitude_setpoint, pitch_setpoint, roll_setpoint, yaw_setpoint};
}

void DroneController::set_yaw(float sp) {
    yaw_setpoint = sp;
}

void DroneController::set_roll(float sp) {
    roll_setpoint = sp;
}

void DroneController::set_pitch(float sp) {
    pitch_setpoint = sp;
}

void DroneController::set_altitude(float sp) {
    altitude_setpoint = sp;
}
