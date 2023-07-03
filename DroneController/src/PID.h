//
// Created by gero on 6/28/23.
//

#ifndef PLOTTING_PID_H
#define PLOTTING_PID_H


class PID {
public:
    PID(float Kp, float Ki, float Kd, float dt, float min, float max)
            : Kp(Kp), Ki(Ki), Kd(Kd), dt(dt), totalMin(min), windupMin(min), totalMax(max), windupMax(max) {}

    float update(float setpoint, float process_value);

    void set_tunings(float p, float i, float d);

    void set_dt(float dt);

    void set_windup(float min, float max);

    void set_clamp(float min, float max);

    void set_Kp(float Kp);

    void set_Ki(float Ki);

    void set_Kd(float Kd);

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
};


#endif //PLOTTING_PID_H
