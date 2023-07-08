//
// Created by gero on 6/28/23.
//

#ifndef PLOTTING_ALGEBRA_H
#define PLOTTING_ALGEBRA_H

#include <cmath>

float normalize_angle( float angle );

struct Vector3 {
    float x;
    float y;
    float z;
};

struct Rotation {
    float pitch;
    float yaw;
    float roll;

    void normalize(){
        pitch = normalize_angle(pitch);
        yaw = normalize_angle(yaw);
        roll = normalize_angle(roll);
    }
};

#endif //PLOTTING_ALGEBRA_H
