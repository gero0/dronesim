//
// Created by gero on 6/28/23.
//

#include "DroneModel.h"

float DroneModel::update(float dt) {
    //Assume linear relation of motor power and thrust
    float speed_fl_sqr = std::pow(fl_driver.get_speed() * motor_max_speed, 2.0f);
    float speed_fr_sqr = std::pow(fr_driver.get_speed() * motor_max_speed, 2.0f);
    float speed_bl_sqr = std::pow(bl_driver.get_speed() * motor_max_speed, 2.0f);
    float speed_br_sqr = std::pow(br_driver.get_speed() * motor_max_speed, 2.0f);

    float roll_moment = arm_len * lift_coeff * (speed_fr_sqr + speed_br_sqr - speed_fl_sqr - speed_bl_sqr);
    float pitch_moment = arm_len * lift_coeff * (speed_fl_sqr + speed_fr_sqr - speed_bl_sqr - speed_br_sqr);
    float yaw_moment = yaw_coeff * (speed_fl_sqr + speed_br_sqr - speed_bl_sqr - speed_fr_sqr);

    float roll_acc = roll_moment / rp_inertia_moment;
    float pitch_acc = pitch_moment / rp_inertia_moment;
    float yaw_acc = yaw_moment / yaw_inertia_moment;

    angular_velocity.x += roll_acc * dt;
    angular_velocity.y += pitch_acc * dt;
    angular_velocity.z += yaw_acc * dt;

    rotation.x += angular_velocity.x * dt;
    rotation.y += angular_velocity.y * dt;
    rotation.z += angular_velocity.z * dt;

    float thrust = lift_coeff * (speed_fl_sqr + speed_fr_sqr + speed_bl_sqr + speed_br_sqr);
    float mass = (body_mass + 4.0f * motor_mass);
    Vector3 acceleration{
            cosf(rotation.z) * sinf(rotation.x) * thrust / mass,
            sinf(rotation.z) * sinf(rotation.y) * thrust / mass,
            (cosf(rotation.x) * cosf(rotation.y) * thrust / mass ) - g,
    };

    velocity.x += acceleration.x * dt;
    velocity.y += acceleration.y * dt;
    velocity.z += acceleration.z * dt;

    position.x += velocity.x * dt;
    position.y += velocity.y * dt;
    position.z += velocity.z * dt;

    if(position.z <= 0.0f){
        position.z = 0.0f;
        velocity.z = 0.0f;
    }

    sensor_mock.acceleration = acceleration;
    sensor_mock.angular_acceleration = {roll_acc, pitch_acc, yaw_acc};
    sensor_mock.altitude = position.z;

    controller.update(dt);
}