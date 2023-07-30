//
// Created by gero on 6/28/23.
//

#ifndef PLOTTING_ALGEBRA_H
#define PLOTTING_ALGEBRA_H

#include <cmath>

float normalize_angle(float angle);

struct Vector3 {
    float x;
    float y;
    float z;

    Vector3 &operator+=(const Vector3 &a);

    Vector3 operator+(const Vector3 &a) const;

    Vector3 &operator-=(const Vector3 &a);

    Vector3 operator-(const Vector3 &a) const;

    Vector3 &operator*=(const Vector3 &a);

    Vector3 operator*(const Vector3 &a) const;

    Vector3 &operator/=(const Vector3 &a);

    Vector3 operator/(const Vector3 &a) const;

    Vector3 &operator+=(float a);

    Vector3 &operator-=(float a);

    Vector3 &operator*=(float a);

    Vector3 &operator/=(float a);

    Vector3 operator*(float a) const;

    Vector3 operator/(float a) const;

    Vector3 operator+(float a) const;

    Vector3 operator-(float a) const;

};

Vector3 rotate_reference_frame(Vector3 v, float yaw_angle);

struct Rotation {
    float &pitch = inner.x;
    float &yaw = inner.z;
    float &roll = inner.y;

    Rotation(float pitch, float yaw, float roll) : inner(pitch, roll, yaw) {}

    Rotation& operator=(const Rotation &a){
        inner = a.inner;
        return *this;
    }

    Vector3 get();

    Rotation &operator+=(const Rotation &a);

    Rotation operator+(const Rotation &a);

    Rotation &operator-=(const Rotation &a);

    Rotation operator-(const Rotation &a);

    Rotation &operator*=(const Rotation &a);

    Rotation operator*(const Rotation &a);

    Rotation &operator/=(const Rotation &a);

    Rotation operator/(const Rotation &a);

    Rotation operator*(float a) const;

    Rotation operator/(float a) const;

    Rotation operator+(float a) const;

    Rotation operator-(float a) const;

    void normalize();

private:
    Vector3 inner;
};

#endif //PLOTTING_ALGEBRA_H
