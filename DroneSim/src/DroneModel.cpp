//
// Created by gero on 6/28/23.
//

#include "DroneModel.h"

void DroneModel::update(float dt) {
    //Assume linear relation of motor power and thrust
    float thrust_fl = fl_driver.get_speed() * motor_max_thrust;
    float thrust_fr = fr_driver.get_speed() * motor_max_thrust;
    float thrust_bl = bl_driver.get_speed() * motor_max_thrust;
    float thrust_br = br_driver.get_speed() * motor_max_thrust;

    float roll_moment = arm_len * (thrust_fr + thrust_br - thrust_fl - thrust_bl);
    float pitch_moment = arm_len * (thrust_fl + thrust_fr - thrust_bl - thrust_br);
    float yaw_moment = yaw_coeff * (thrust_fl + thrust_br - thrust_bl - thrust_fr);

    float roll_acc = roll_moment / rp_inertia_moment;
    float pitch_acc = pitch_moment / rp_inertia_moment;
    float yaw_acc = yaw_moment / yaw_inertia_moment;

    angular_velocity.roll += roll_acc * dt;
    angular_velocity.pitch += pitch_acc * dt;
    angular_velocity.yaw += yaw_acc * dt;

    rotation.roll += angular_velocity.roll * dt;
    rotation.pitch += angular_velocity.pitch * dt;
    rotation.yaw += angular_velocity.yaw * dt;

    rotation.normalize();

    float thrust = thrust_fl + thrust_fr + thrust_br + thrust_bl;
    float mass = (body_mass + 4.0f * motor_mass);
    Vector3 acceleration_local{
            -sinf(rotation.roll) * thrust / mass,
            -sinf(rotation.pitch) * thrust / mass,
            (cosf(rotation.roll) * cosf(rotation.pitch) * thrust / mass) - g,
    };

    Vector3 acceleration{
            (sinf(rotation.yaw) * sinf(rotation.pitch) * cosf(rotation.roll) -
             cosf(rotation.yaw) * sinf(rotation.roll)) * thrust / mass,
            -(cosf(rotation.yaw) * sinf(rotation.pitch) * cosf(rotation.roll) +
              sinf(rotation.yaw) * sinf(rotation.roll)) * thrust / mass,
            (cosf(rotation.roll) * cosf(rotation.pitch) * thrust / mass) - g,
    };

    //air resistance
    acceleration_local = {acceleration_local.x - (1.0f / mass) * air_resistance_coeff * velocity.x,
                          acceleration_local.y - (1.0f / mass) * air_resistance_coeff * velocity.y,
                          acceleration_local.z - (1.0f / mass) * air_resistance_coeff * velocity.z,
    };

    acceleration = {acceleration.x - (1.0f / mass) * air_resistance_coeff * velocity.x,
                    acceleration.y - (1.0f / mass) * air_resistance_coeff * velocity.y,
                    acceleration.z - (1.0f / mass) * air_resistance_coeff * velocity.z,
    };


    velocity.x += acceleration.x * dt;
    velocity.y += acceleration.y * dt;
    velocity.z += acceleration.z * dt;

    position.x += velocity.x * dt;
    position.y += velocity.y * dt;
    position.z += velocity.z * dt;

    if (position.z <= 0.0f) {
        position.z = 0.0f;
        velocity.z = 0.0f;
    }

    sensor_mock.acceleration = acceleration_local;
    sensor_mock.angular_acceleration = {pitch_acc, yaw_acc, roll_acc};
    sensor_mock.altitude = position.z;

    controller.update(dt);
}