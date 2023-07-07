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

    v_thrust = thrust_pid.update(altitude_setpoint, altitude, dt);
    v_pitch = pitch_pid.update(pitch_setpoint, rotation.pitch, dt);
    v_roll = roll_pid.update(roll_setpoint, rotation.roll, dt);
    v_yaw = yaw_pid.update(yaw_setpoint, rotation.yaw, dt);

    front_left->set_speed(std::clamp(v_thrust + v_pitch - v_roll + v_yaw, 0.0f, 1.0f));
    front_right->set_speed(std::clamp(v_thrust + v_pitch + v_roll - v_yaw, 0.0f, 1.0f));
    back_left->set_speed(std::clamp(v_thrust - v_pitch - v_roll - v_yaw, 0.0f, 1.0f));
    back_right->set_speed(std::clamp(v_thrust - v_pitch + v_roll + v_yaw, 0.0f, 1.0f));
}

PidTunings DroneController::get_thrust_tunings() {
    return thrust_pid.get_tunings();
}

PidTunings DroneController::get_pitch_tunings() {
    return pitch_pid.get_tunings();
}

PidTunings DroneController::get_roll_tunings() {
    return roll_pid.get_tunings();
}

PidTunings DroneController::get_yaw_tunings() {
    return yaw_pid.get_tunings();
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

void DroneController::set_thrust_tunings(PidTunings tunings) {
    thrust_pid.set_tunings(tunings.Kp, tunings.Ki, tunings.Kd);
}

void DroneController::set_pitch_tunings(PidTunings tunings) {
    pitch_pid.set_tunings(tunings.Kp, tunings.Ki, tunings.Kd);
}

void DroneController::set_roll_tunings(PidTunings tunings) {
    roll_pid.set_tunings(tunings.Kp, tunings.Ki, tunings.Kd);
}

void DroneController::set_yaw_tunings(PidTunings tunings) {
    yaw_pid.set_tunings(tunings.Kp, tunings.Ki, tunings.Kd);
}

