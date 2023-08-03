//
// Created by gero on 7/1/23.
//

#include "DroneController.h"
#include <iostream>

void DroneController::update(float dt) {
    altitude = sensor_reader->getAltitude();
    angular_velocity += sensor_reader->getAngularAcceleration() * dt;

    rotation += angular_velocity * dt;
    //TODO: fix this and remove this workaround
    float yaw_og = rotation.yaw;
    rotation.normalize();
    rotation.yaw = yaw_og;

    Vector3 acceleration = sensor_reader->getAcceleration();
    Vector3 acceleration_global = body_to_earth(acceleration, rotation);
    velocity_global += acceleration_global * dt;
    position_global += velocity_global * dt;

    if(position_global.z <= 0){
        velocity_global.z = 0;
        position_global.z = 0;
    }

    if(roll_setpoint == 0.0f && pitch_setpoint == 0.0f){
        controlState = ControlState::PointHover;
    }else{
        controlState = ControlState::Direct;
    }

    control_update(dt);
}

void DroneController::control_update(float dt) {
    v_thrust = thrust_pid.update(altitude_setpoint, altitude, dt);
    v_yaw = yaw_pid.update(yaw_setpoint, rotation.yaw, dt);

    if(controlState==ControlState::PointHover){
        Vector3 position_local = rotate_flat(position_global, -rotation.yaw);
        Vector3 sp_local = rotate_flat(hover_setpoint, -rotation.yaw);
        auto px = position_x_pid.update(sp_local.x, position_local.x, dt);
        auto py = position_y_pid.update(sp_local.y, position_local.y, dt);
        v_pitch = pitch_pid.update(px * max_angle, rotation.pitch, dt);
        v_roll = roll_pid.update(py * max_angle, rotation.roll, dt);
    }else if (controlState == ControlState::Direct){
        v_pitch = pitch_pid.update(pitch_setpoint, rotation.pitch, dt);
        v_roll = roll_pid.update(roll_setpoint, rotation.roll, dt);
    }

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
//    yaw_setpoint = normalize_angle(sp);
    yaw_setpoint = sp;
}

void DroneController::set_roll(float sp) {
    roll_setpoint = normalize_angle(sp);
}

void DroneController::set_pitch(float sp) {
    pitch_setpoint = normalize_angle(sp);
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

void DroneController::level() {
    roll_setpoint = 0.0f;
    pitch_setpoint = 0.0f;
}

void DroneController::hover() {
    level();
    hover_setpoint = position_global;
    controlState = ControlState::PointHover;
}
void DroneController::RTO() {
    hover();
    hover_setpoint = {0.0f, 0.0f, 0.0f};
}



