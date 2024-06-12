//
// Created by gero on 7/1/23.
//

#include "DroneController.h"
#include <algorithm>
#include <array>

void DroneController::update(float dt) {
    sensor_reader->update(dt);
    rotation = sensor_reader->get_rotation();
    angular_rate = sensor_reader->get_angular_rate();

    Vector3 new_acceleration = sensor_reader->get_acceleration();
    Vector3 new_velocity = (acceleration + new_acceleration) * 0.5f * dt;

    position = (velocity + new_velocity) * 0.5f * dt;
    acceleration = new_acceleration;
    velocity = new_velocity;

    altitude = sensor_reader->get_altitude();
    vertical_speed = sensor_reader->get_vs();
    absolute_altitude = sensor_reader->get_absolute_altitude();
    radar_altitude = sensor_reader->get_radar_altitude();

    if (position.z <= 0) {
        velocity.z = 0;
        position.z = 0;
    }

    if(altitude < 0.1){
        yaw_setpoint = rotation.yaw;
    }

    if(altitude_setpoint < 0.0f){
        altitude_setpoint = 0.0f;
    }

    control_update(dt);
}

void DroneController::control_update(float dt) {
    float vs_setpoint = altitude_pid.update(altitude_setpoint, altitude, dt);
    v_thrust = vs_pid.update(vs_setpoint, (altitude - prev_altitude) / dt, dt);

    v_thrust = direct_thrust_value;

    if(mode == ControlMode::Angle){
        pitch_rate_setpoint = pitch_pid.update(pitch_setpoint, rotation.pitch, dt) * max_dps;
        roll_rate_setpoint =  roll_pid.update(roll_setpoint, rotation.roll, dt) * max_dps;
        yaw_rate_setpoint = yaw_pid.update(yaw_setpoint, rotation.yaw, dt) * max_dps_yaw;
    }else{
        pitch_pid.update(pitch_setpoint, rotation.pitch, dt);
        roll_pid.update(roll_setpoint, rotation.roll, dt);
        yaw_pid.update(yaw_setpoint, rotation.yaw, dt);
    }

    v_pitch = pitch_rate_pid.update(pitch_rate_setpoint, angular_rate.pitch, dt);
    v_roll = roll_rate_pid.update(roll_rate_setpoint, angular_rate.roll, dt);
    v_yaw = -yaw_rate_pid.update(yaw_rate_setpoint, angular_rate.yaw, dt);

//    v_yaw = yaw_raw;

    front_left->set_speed(std::clamp(v_thrust - v_pitch + v_roll + v_yaw, 0.0f, 1.0f));
    front_right->set_speed(std::clamp(v_thrust - v_pitch - v_roll - v_yaw, 0.0f, 1.0f));
    back_left->set_speed(std::clamp(v_thrust + v_pitch + v_roll - v_yaw, 0.0f, 1.0f));
    back_right->set_speed(std::clamp(v_thrust + v_pitch - v_roll + v_yaw, 0.0f, 1.0f));
}


PidTunings DroneController::get_altitude_tunings() {
    return altitude_pid.get_tunings();
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

PidTunings DroneController::get_pitch_rate_tunings() {
    return pitch_rate_pid.get_tunings();
}

PidTunings DroneController::get_roll_rate_tunings() {
    return roll_rate_pid.get_tunings();
}

PidTunings DroneController::get_yaw_rate_tunings() {
    return yaw_rate_pid.get_tunings();
}

PidTunings DroneController::get_vs_tunings() {
    return vs_pid.get_tunings();
}

PidValues DroneController::get_last_pid_output() {
    return {v_thrust, v_pitch, v_roll, v_yaw};
}

PidValues DroneController::get_setpoints() {
    return {altitude_setpoint, pitch_setpoint, roll_setpoint, yaw_setpoint};
}

void DroneController::yaw_raw_input(float input){
    yaw_raw = input * yaw_raw_constant;
}


void DroneController::set_yaw(float sp) {
    if(sp < -M_PI){
        sp += 2.0f * M_PI;
    }
    else if(sp > M_PI){
        sp -= 2.0f * M_PI;
    }
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

void DroneController::set_altitude_tunings(PidTunings tunings) {
    altitude_pid.set_tunings(tunings.Kp, tunings.Ki, tunings.Kd);
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

void DroneController::set_pitch_rate_tunings(PidTunings tunings) {
    pitch_rate_pid.set_tunings(tunings.Kp, tunings.Ki, tunings.Kd);
}

void DroneController::set_roll_rate_tunings(PidTunings tunings) {
    roll_rate_pid.set_tunings(tunings.Kp, tunings.Ki, tunings.Kd);
}

void DroneController::set_yaw_rate_tunings(PidTunings tunings) {
    yaw_rate_pid.set_tunings(tunings.Kp, tunings.Ki, tunings.Kd);
}

void DroneController::set_vs_tunings(PidTunings tunings) {
    vs_pid.set_tunings(tunings.Kp, tunings.Ki, tunings.Kd);
}

bool DroneController::is_stopped() {
    return is_stopped_v;
}

void DroneController::reset_pids(){
    vs_pid.reset();
    altitude_pid.reset();

    pitch_rate_pid.reset();
    yaw_rate_pid.reset();
    roll_rate_pid.reset();

    roll_pid.reset();
    pitch_pid.reset();
    yaw_pid.reset();
}

void DroneController::start(){
    reset_pids();

    front_left->enable();
    front_right->enable();
    back_right->enable();
    back_left->enable();

    is_stopped_v = false;
}

bool DroneController::stop(){
    //TODO: altitude check
    reset_pids();

    direct_thrust_value = 0;
    front_left->disable();
    front_right->disable();
    back_right->disable();
    back_left->disable();

    is_stopped_v = true;

    return true;
}

void DroneController::level() {
    roll_setpoint = 0.0f;
    pitch_setpoint = 0.0f;
}

void DroneController::hover() {
    level();
    hover_setpoint = position;
    controlState = ControlState::PointHover;
}

void DroneController::RTO() {
    hover();
    hover_setpoint = {0.0f, 0.0f, 0.0f};
}

Vector3 DroneController::get_position() {
    return position;
}

Rotation DroneController::get_rotation() {
    return rotation;
}

Vector3 DroneController::get_hover_setpoint() {
    return hover_setpoint;
}

void DroneController::reset_position() {
    position = {.0f, .0f, .0f};
}

void DroneController::set_hover_setpoint(Vector3 sp) {
    hover_setpoint = sp;
}

float DroneController::get_altitude() {
    return altitude;
}

float DroneController::get_radar_altitude() {
    return radar_altitude;
}

float DroneController::get_absolute_altitude() {
    return absolute_altitude;
}

std::array<float, 4> DroneController::get_motor_speeds() {
    return {front_left->get_speed(), front_right->get_speed(), back_left->get_speed(), back_right->get_speed()};
}

Rotation DroneController::get_rotation_setpoints() {
    return {pitch_setpoint, yaw_setpoint, roll_setpoint};
}

float DroneController::get_altitude_setpoint() const {
    return altitude_setpoint;
}

float DroneController::get_direct_thrust() const {
    return direct_thrust_value;
}

void DroneController::set_direct_thrust(float thrust) {
    direct_thrust_value = std::clamp(thrust, 0.0f, 1.0f);
}

Rotation DroneController::get_angular_rates() {
    return angular_rate;
}

float DroneController::get_vertical_speed() {
    return vertical_speed;
}

void DroneController::pitch_input(float input) {
    input = std::clamp(input, -1.0f, 1.0f);
    if(mode == ControlMode::Angle) {
        set_pitch(input * max_angle);
    }else{
        pitch_rate_setpoint = input * max_dps;
    }
}

void DroneController::roll_input(float input) {
    input = std::clamp(input, -1.0f, 1.0f);
    if(mode == ControlMode::Angle) {
        set_roll(input * max_angle);
    }else{
        roll_rate_setpoint = input * max_dps;
    }
}

void DroneController::yaw_input(float input) {
    if(std::abs(input) < 0.1f){
        input = 0.0f;
    }
    input = std::clamp(input, -1.0f, 1.0f);
    if(mode == ControlMode::Angle){
        set_yaw(yaw_setpoint - (input * yaw_constant));
    }else{
        yaw_rate_setpoint = -input * max_dps_yaw;
    }

//    yaw_rate_setpoint = -input * max_dps_yaw;
}

void DroneController::thrust_input(float input) {
    if( std::abs(input) < 0.1){
        input = 0;
    }
    set_direct_thrust(direct_thrust_value + input * thrust_input_const);
}

void DroneController::set_control_mode(ControlMode mode) {
    this->mode = mode;
}

Vector3 DroneController::get_acceleration(){
    return sensor_reader->get_acceleration();
}
