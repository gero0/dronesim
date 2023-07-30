//
// Created by gero on 7/8/23.
//

#include "algebra.h"

float normalize_angle( float angle )
{
    const float full = 2.0f * M_PI;
    angle = fmodf(angle, full);
    if (angle > M_PI)
    {
        angle -= full;
    }
    if (angle < -M_PI)
    {
        angle += full;
    }
    return angle;
}

Vector3 rotate_vector(const Vector3 &v, const Rotation &r) {
    //https://en.wikipedia.org/wiki/Rotation_matrix
    float xr = v.x * cosf(r.yaw) * cosf(r.pitch)
               + v.y * (cosf(r.yaw) * sinf(r.pitch) * sinf(r.roll) - sinf(r.yaw)*cosf(r.roll))
               + v.z * (cosf(r.yaw) * sinf(r.pitch) * cosf(r.roll) + sinf(r.yaw)*sinf(r.roll));

    float yr = v.x * sinf(r.yaw) * cosf(r.pitch)
               + v.y * (sinf(r.yaw) * sinf(r.pitch) * sinf(r.roll) + cosf(r.yaw)*cosf(r.roll))
               + v.z * (sinf(r.yaw) * sinf(r.pitch) * cosf(r.roll) - cosf(r.yaw)*sinf(r.roll));

    float zr = v.x * -sinf(r.pitch) + v.y * cosf(r.pitch) * sinf(r.roll) + v.z * cosf(r.pitch) * cosf(r.roll);
    return {xr, yr, zr};
}

Vector3 rotate_reference_frame(Vector3 v, float yaw_angle) {
    return {
            v.x * cosf(yaw_angle) - v.y * sinf(yaw_angle),
            v.x * sinf(yaw_angle) + v.y * cosf(yaw_angle),
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

Vector3& Vector3::operator+=(float a){
    this->x += a;
    this->y += a;
    this->z += a;
    return *this;
}

Vector3& Vector3::operator-=(float a){
    this->x -= a;
    this->y -= a;
    this->z -= a;
    return *this;
}

Vector3& Vector3::operator*=(float a){
    this->x *= a;
    this->y *= a;
    this->z *= a;
    return *this;
}

Vector3& Vector3::operator/=(float a){
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
    this->inner += a.inner;
    return *this;
}

Rotation Rotation::operator+(const Rotation &a) {
    return Rotation(*this) += a;
}

Rotation &Rotation::operator-=(const Rotation &a) {
    this->inner -= a.inner;
    return *this;
}

Rotation Rotation::operator-(const Rotation &a) {
    return Rotation(*this) -= a;
}

Rotation &Rotation::operator*=(const Rotation &a) {
    this->inner *= a.inner;
    return *this;
}

Rotation Rotation::operator*(const Rotation &a) {
    return Rotation(*this) *= a;
}

Rotation &Rotation::operator/=(const Rotation &a) {
    this->inner /= a.inner;
    return *this;
}

Rotation Rotation::operator/(const Rotation &a) {
    return Rotation(*this) /= a;
}

Rotation Rotation::operator*(float a) const {
    auto r = Rotation(*this);
    r.inner *= a;
    return r;
}

Rotation Rotation::operator/(float a) const {
    auto r = Rotation(*this);
    r.inner /= a;
    return r;
}

Rotation Rotation::operator+(float a) const {
    auto r = Rotation(*this);
    r.inner += a;
    return r;
}

Rotation Rotation::operator-(float a) const {
    auto r = Rotation(*this);
    r.inner -= a;
    return r;
}

void Rotation::normalize() {
    pitch = normalize_angle(pitch);
    yaw = normalize_angle(yaw);
    roll = normalize_angle(roll);
}

Vector3 Rotation::get() {
    return {pitch, yaw, roll};
}
