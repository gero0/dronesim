//
// Created by gero on 6/28/23.
//

#ifndef PLOTTING_PID_H
#define PLOTTING_PID_H

//TODO: incorporate into constructor/set_tunings?
struct PidTunings{
    float Kp;
    float Ki;
    float Kd;
};

class PID {
public:
    PID(float Kp, float Ki, float Kd, float min, float max, bool angle_wrap = false)
            : Kp(Kp), Ki(Ki), Kd(Kd), totalMin(min), windupMin(min), totalMax(max), windupMax(max), angleWrap(angle_wrap) {}

    PID(float Kp, float Ki, float Kd, float min, float max, float windMin, float windMax, bool angle_wrap = false)
            : Kp(Kp), Ki(Ki), Kd(Kd), totalMin(min), windupMin(windMin), totalMax(max), windupMax(windMax), angleWrap(angle_wrap) {}

    float update(float setpoint, float process_value, float dt);

    void set_tunings(float p, float i, float d);
    PidTunings get_tunings();

    void set_windup(float min, float max);

    void set_clamp(float min, float max);

    void set_Kp(float Kp);

    void set_Ki(float Ki);

    void set_Kd(float Kd);

    void reset();

private:
    float last_value = 0.0f;
    float iTerm = 0.0f;
    float Kp = 1.0f;
    float Ki = 1.0f;
    float Kd = 1.0f;
    float dt = 0.001f;
    float windupMax = 1.0f;
    float windupMin = 0.0f;
    float totalMin = 0.0f;
    float totalMax = 1.0f;

    bool angleWrap;
};


#endif //PLOTTING_PID_H
