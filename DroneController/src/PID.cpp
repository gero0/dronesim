//
// Created by gero on 6/28/23.
//

#include <algorithm>
#include <iostream>
#include "PID.h"

void PID::set_tunings(float p, float i, float d){
    this->Kp = p;
    this->Ki = i;
    this->Kd = d;
}

void PID::reset() {
    this->iTerm = 0;
//    this->last_value = 0;
}

void PID::set_windup(float min, float max){
    this->windupMin = min;
    this->windupMax = max;
}

void PID::set_clamp(float min, float max){
    this->totalMin = min;
    this->totalMax = max;
}

float PID::update(float setpoint, float process_value, float dt){
    float error = setpoint - process_value;
    float pTerm = Kp * error;

    iTerm += (Ki * dt) * error;
    iTerm = std::clamp(iTerm, windupMin, windupMax);

    //Derivative on measurement to prevent derivative kick
    float dTerm = Kd * (process_value - last_value) / dt;
    last_value = process_value;

    float output = pTerm + iTerm - dTerm;
    output = std::clamp(output, totalMin, totalMax);

    return output;
}

void PID::set_Kp(float Kp) {
    this->Kd = Kp;
}

void PID::set_Ki(float Ki) {
    this->Ki = Ki;
}

void PID::set_Kd(float Kd) {
    this->Kd = Kd;
}

PidTunings PID::get_tunings() {
    return {Kp, Ki, Kd};
}
