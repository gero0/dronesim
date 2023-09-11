//
// Created by gero on 7/1/23.
//

#ifndef DRONESIMPROJECT_DRONECONTROLLER_H
#define DRONESIMPROJECT_DRONECONTROLLER_H

#include <array>
#include "SensorReader.h"
#include "MotorDriver.h"
#include "PID.h"

struct PidValues {
    float v_thrust;
    float v_pitch;
    float v_roll;
    float v_yaw;
};

enum class ControlState {
    PointHover,
    Direct,
};

class DroneController {
public:
    DroneController(MotorDriver *front_left, MotorDriver *front_right, MotorDriver *back_left,
                    MotorDriver *back_right, SensorReader *sensor_reader)
            : front_left(front_left), front_right(front_right), back_left(back_left), back_right(back_right),
              sensor_reader(sensor_reader) {
    }

    void update(float dt);

    PidValues get_last_pid();

    PidValues get_setpoints();

    PidTunings get_thrust_tunings();

    PidTunings get_pitch_tunings();

    PidTunings get_roll_tunings();

    PidTunings get_yaw_tunings();

    void set_thrust_tunings(PidTunings tunings);

    void set_pitch_tunings(PidTunings tunings);

    void set_roll_tunings(PidTunings tunings);

    void set_yaw_tunings(PidTunings tunings);

    void set_altitude(float sp);

    void set_pitch(float sp);

    void set_roll(float sp);

    void set_yaw(float sp);

    void level();

    void hover();

    void RTO();

    void reset_position();

    void set_hover_setpoint(Vector3 sp);

    Vector3 get_position();

    Vector3 get_hover_setpoint();

    Rotation get_rotation();

    Rotation get_rotation_setpoints();

    float get_altitude();

    float get_radar_altitude();

    float get_altitude_setpoint() const;

    std::array<float, 4> get_motor_speeds();

private:
    SensorReader *sensor_reader;

    float yaw_setpoint = 0.0f;
    float pitch_setpoint = 0.0f;
    float roll_setpoint = 0.0f;
    float altitude_setpoint = 0.0f;

    float v_thrust = 0.0f;
    float v_pitch = 0.0f;
    float v_roll = 0.0f;
    float v_yaw = 0.0f;

    float altitude = 0.0f;
    float radar_altitude = 0.0f;
    const float g = 9.81;

    const float max_angle = (45.0f / 180.0f) * M_PI;

    Vector3 position_global{0.0f, 0.0f, 0.0f};
    Vector3 hover_setpoint{0.0f, 0.0f, 0.0f};

    void control_update(float dt);

    MotorDriver *front_left;
    MotorDriver *front_right;
    MotorDriver *back_left;
    MotorDriver *back_right;

    PID position_x_pid{1.0f, 0.01f, 0.1f, -1.0f, 1.0f};
    PID position_y_pid{1.0f, 0.01f, 0.1f, -1.0f, 1.0f};

    PID thrust_pid{4.0f, 0.5f, 1.0f, 0.0f, 1.0f};
    PID pitch_pid{0.1f, 0.0f, 0.01f, -1.0f, 1.0f};
    PID roll_pid{0.1f, 0.0f, 0.01f, -1.0f, 1.0f};
    PID yaw_pid{0.1f, 0.0f, 0.01f, -1.0f, 1.0f};

    Vector3 velocity_global{0.0f, 0.0f, 0.0f};
    Rotation rotation{0.0f, 0.0f, 0.0f};
    Rotation angular_velocity{0.0f, 0.0f, 0.0f};

    ControlState controlState = ControlState::PointHover;
};

#endif //DRONESIMPROJECT_DRONECONTROLLER_H
