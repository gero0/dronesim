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

struct Rotation {
    float pitch;
    float yaw;
    float roll;

    Rotation &operator+=(const Rotation &a);

    Rotation operator+(const Rotation &a) const;

    Rotation &operator-=(const Rotation &a);

    Rotation operator-(const Rotation &a) const;

    Rotation &operator*=(const Rotation &a);

    Rotation operator*(const Rotation &a) const;

    Rotation &operator/=(const Rotation &a);

    Rotation operator/(const Rotation &a) const;

    Rotation &operator+=(float a);
    Rotation &operator-=(float a);
    Rotation &operator*=(float a);
    Rotation &operator/=(float a);

    Rotation operator*(float a) const;

    Rotation operator/(float a) const;

    Rotation operator+(float a) const;

    Rotation operator-(float a) const;

    void normalize();

};

Vector3 body_to_earth(const Vector3 &v, const Rotation &r);
Vector3 earth_to_body(const Vector3 &v, const Rotation &r);
Vector3 rotate_flat(Vector3 v, float yaw_angle);

#endif //PLOTTING_ALGEBRA_H
