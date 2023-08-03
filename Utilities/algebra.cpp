//
// Created by gero on 7/8/23.
//

#include <iostream>
#include "algebra.h"

float normalize_angle(float angle) {
    const float full = 2.0f * M_PI;
    angle = fmodf(angle, full);
    if (angle > M_PI) {
        angle -= full;
    }
    if (angle < -M_PI) {
        angle += full;
    }
    return angle;
}

Vector3 body_to_earth(const Vector3 &v, const Rotation &r) {
    float xr = v.x * cosf(r.pitch) * cosf(r.yaw) + v.y * sinf(r.yaw) * cosf(r.pitch) - v.z * sinf(r.pitch);
    float yr = v.x * (sinf(r.pitch) * sinf(r.roll) * cosf(r.yaw) - sinf(r.yaw) * cosf(r.roll)) +
               v.y * (sinf(r.pitch) * sinf(r.roll) * sinf(r.yaw) + cosf(r.roll) * cosf(r.yaw)) +
               v.z * sinf(r.roll) * cosf(r.pitch);
    float zr = v.x * (sinf(r.pitch) * cosf(r.roll) * cosf(r.yaw) + sinf(r.roll) * sinf(r.yaw)) +
               v.y * (sinf(r.pitch) * sinf(r.yaw) * cosf(r.roll) - sinf(r.roll) * cosf(r.yaw)) +
               v.z * cosf(r.pitch) * cosf(r.roll);
    return {xr, yr, zr};
}

Vector3 earth_to_body(const Vector3 &v, const Rotation &r) {
    float xr = v.x * cosf(r.pitch) * cosf(r.yaw) +
               v.y * (sinf(r.pitch) * sinf(r.roll) * cosf(r.yaw) - sinf(r.yaw) * cosf(r.roll)) +
               v.z * (sinf(r.pitch) * cosf(r.roll) * cosf(r.yaw) + sinf(r.roll) * sinf(r.yaw));
    float yr = v.x * sinf(r.yaw) * cosf(r.pitch) +
               v.y * (sinf(r.pitch) * sinf(r.roll) * sinf(r.yaw) + cosf(r.roll) * cosf(r.yaw)) +
               v.z * (sinf(r.pitch) * sinf(r.yaw) * cosf(r.roll) - sinf(r.roll) * cosf(r.yaw));
    float zr = -v.x * sinf(r.pitch) + v.y * sinf(r.roll) * cosf(r.pitch) + v.z * cosf(r.pitch) * cosf(r.roll);
    return {xr, yr, zr};
}

Vector3 rotate_flat(Vector3 v, float yaw_angle) {
    return {
            v.x * cosf(yaw_angle) + v.y * sinf(yaw_angle),
            v.x * -sinf(yaw_angle) + v.y * cosf(yaw_angle),
            v.z
    };
}

Vector3 &Vector3::operator+=(const Vector3 &a) {
    this->x += a.x;
    this->y += a.y;
    this->z += a.z;
    return *this;
}

Vector3 Vector3::operator+(const Vector3 &a) const {
    return Vector3(*this) += a;
}

Vector3 &Vector3::operator-=(const Vector3 &a) {
    this->x -= a.x;
    this->y -= a.y;
    this->z -= a.z;
    return *this;
}

Vector3 Vector3::operator-(const Vector3 &a) const {
    return Vector3(*this) -= a;
}

Vector3 &Vector3::operator*=(const Vector3 &a) {
    this->x = x * a.x;
    this->y = y * a.y;
    this->z = z * a.z;
    return *this;
}

Vector3 Vector3::operator*(const Vector3 &a) const {
    return Vector3(*this) *= a;
}

Vector3 &Vector3::operator/=(const Vector3 &a) {
    this->x = x / a.x;
    this->y = y / a.y;
    this->z = z / a.z;
    return *this;
}

Vector3 Vector3::operator/(const Vector3 &a) const {
    return Vector3(*this) /= a;
}

Vector3 &Vector3::operator+=(float a) {
    this->x += a;
    this->y += a;
    this->z += a;
    return *this;
}

Vector3 &Vector3::operator-=(float a) {
    this->x -= a;
    this->y -= a;
    this->z -= a;
    return *this;
}

Vector3 &Vector3::operator*=(float a) {
    this->x *= a;
    this->y *= a;
    this->z *= a;
    return *this;
}

Vector3 &Vector3::operator/=(float a) {
    this->x /= a;
    this->y /= a;
    this->z /= a;
    return *this;
}

Vector3 Vector3::operator+(float a) const {
    return Vector3(*this) += a;
}

Vector3 Vector3::operator-(float a) const {
    return Vector3(*this) -= a;
}

Vector3 Vector3::operator*(float a) const {
    return Vector3(*this) *= a;
}

Vector3 Vector3::operator/(float a) const {
    return Vector3(*this) /= a;
}

Rotation &Rotation::operator+=(const Rotation &a) {
    this->pitch += a.pitch;
    this->yaw += a.yaw;
    this->roll += a.roll;
    return *this;
}

Rotation Rotation::operator+(const Rotation &a) const {
    return Rotation(*this) += a;
}

Rotation &Rotation::operator-=(const Rotation &a) {
    this->pitch -= a.pitch;
    this->yaw -= a.yaw;
    this->roll -= a.roll;
    return *this;
}

Rotation Rotation::operator-(const Rotation &a) const {
    return Rotation(*this) -= a;
}

Rotation &Rotation::operator*=(const Rotation &a) {
    this->pitch *= a.pitch;
    this->yaw *= a.yaw;
    this->roll *= a.roll;
    return *this;
}

Rotation Rotation::operator*(const Rotation &a) const {
    return Rotation(*this) *= a;
}

Rotation &Rotation::operator/=(const Rotation &a) {
    this->pitch /= a.pitch;
    this->yaw /= a.yaw;
    this->roll /= a.roll;
    return *this;
}

Rotation Rotation::operator/(const Rotation &a) const {
    return Rotation(*this) /= a;
}

Rotation &Rotation::operator+=(float a) {
    this->pitch += a;
    this->yaw += a;
    this->roll += a;
    return *this;
}

Rotation &Rotation::operator-=(float a) {
    this->pitch -= a;
    this->yaw -= a;
    this->roll -= a;
    return *this;
}

Rotation &Rotation::operator*=(float a) {
    this->pitch *= a;
    this->yaw *= a;
    this->roll *= a;
    return *this;
}

Rotation &Rotation::operator/=(float a) {
    this->pitch /= a;
    this->yaw /= a;
    this->roll /= a;
    return *this;
}

Rotation Rotation::operator+(float a) const {
    return Rotation(*this) += a;
}

Rotation Rotation::operator-(float a) const {
    return Rotation(*this) -= a;
}

Rotation Rotation::operator*(float a) const {
    return Rotation(*this) *= a;
}

Rotation Rotation::operator/(float a) const {
    return Rotation(*this) /= a;
}

void Rotation::normalize() {
    pitch = normalize_angle(pitch);
    yaw = normalize_angle(yaw);
    roll = normalize_angle(roll);
}
