//
// Created by gero on 6/28/23.
//

#include <QDebug>
#include "DroneModel.h"



void DroneModel::update(float dt) {
    //Assume linear relation of motor power and thrust
    float thrust_fl = fl_driver.get_speed() * motor_max_thrust;
    float thrust_fr = fr_driver.get_speed() * motor_max_thrust;
    float thrust_bl = bl_driver.get_speed() * motor_max_thrust;
    float thrust_br = br_driver.get_speed() * motor_max_thrust;

    Rotation torque = {
            arm_len * (thrust_fl + thrust_fr - thrust_bl - thrust_br),
            yaw_coeff * (thrust_fl + thrust_br - thrust_bl - thrust_fr),
            arm_len * (thrust_fr + thrust_br - thrust_fl - thrust_bl),
    };

    Rotation angular_acceleration = torque / inertia_moment;
    angular_velocity += angular_acceleration * dt;
    rotation += angular_velocity * dt;
    rotation.normalize();

    float thrust = thrust_fl + thrust_fr + thrust_br + thrust_bl;
    float mass = (body_mass + 4.0f * motor_mass);

    acceleration = {
            (sinf(rotation.yaw) * sinf(rotation.pitch) * cosf(rotation.roll) -
             cosf(rotation.yaw) * sinf(rotation.roll)) * thrust / mass,
            -(cosf(rotation.yaw) * sinf(rotation.pitch) * cosf(rotation.roll) +
              sinf(rotation.yaw) * sinf(rotation.roll)) * thrust / mass,
            (cosf(rotation.roll) * cosf(rotation.pitch) * thrust / mass) - g,
    };

    acceleration = {acceleration.x - (1.0f / mass) * air_resistance_coeff * velocity.x,
                    acceleration.y - (1.0f / mass) * air_resistance_coeff * velocity.y,
                    acceleration.z - (1.0f / mass) * air_resistance_coeff * velocity.z,
    };

    velocity += acceleration * dt;
    position += velocity * dt;

    if (position.z <= 0.0f) {
        position.z = 0.0f;
        velocity.z = 0.0f;
        velocity_local.z = 0.0f;
    }

    acceleration_local = rotate_vector(acceleration, rotation * -1.0f);
    velocity_local = rotate_vector(velocity, rotation * -1.0f);

    sensor_mock.acceleration = acceleration_local;
    sensor_mock.angular_acceleration = angular_acceleration;
    sensor_mock.altitude = position.z;

    controller.update(dt);
}