//
// Created by gero on 7/1/23.
//

#ifndef DRONESIMPROJECT_DRONEVERTICALMODEL_H
#define DRONESIMPROJECT_DRONEVERTICALMODEL_H

#include "Model.h"

class DroneVerticalModel : public Model{
public:
    virtual float update(float dt, float pid_output) override {
        acceleration = (pid_output * motor_acc) - G;
        velocity += acceleration * dt;
        position += velocity * dt;
        if(position < 0.0f){
            position = 0.0f;
            velocity = 0.0f;
        }
        return position;
    }
private:
    float position = 0.0f;
    float velocity = 0.0f;
    float acceleration = 0.0f;
    const float G = 9.8f;
    float motor_acc = G * 3.0f;
};


#endif //DRONESIMPROJECT_DRONEVERTICALMODEL_H
