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

    Vector3 operator*(float a) const{
        return {
            x * a,
            y * a,
            z * a,
        };
    }

    Vector3 operator+(const Vector3& b) const{
        return {
                x + b.x,
                y + b.y,
                z + b.z,
        };
    }

    Vector3 operator+(float a) const{
        return {
                x + a,
                y + a,
                z + a,
        };
    }

    Vector3 & operator+=(const Vector3 & b){
        this->x += b.x;
        this->y += b.y;
        this->z += b.z;
        return *this;
    }

    Vector3 & operator+=(float a){
        this->x += a;
        this->y += a;
        this->z += a;
        return *this;
    }

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

    Rotation operator*(float a) const{
        return {
                pitch * a,
                yaw * a,
                roll * a,
        };
    }

    Rotation operator+(const Rotation& b) const{
        return {
                pitch + b.pitch,
                yaw + b.yaw,
                roll + b.roll,
        };
    }

    Rotation operator+(float a) const{
        return {
                pitch + a,
                yaw + a,
                roll + a,
        };
    }

    Rotation operator/(const Rotation& b) const{
        return {
                pitch / b.pitch,
                yaw / b.yaw,
                roll / b.roll,
        };
    }

    Rotation operator/(float a) const{
        return {
                pitch / a,
                yaw / a,
                roll / a,
        };
    }

    Rotation& operator+=(const Rotation& b){
        this->pitch += b.pitch;
        this->yaw += b.yaw;
        this->roll += b.roll;
        return *this;
    }

    Rotation& operator+=(float a){
        this->pitch += a;
        this->yaw += a;
        this->roll += a;
        return *this;
    }
};

#endif //PLOTTING_ALGEBRA_H
