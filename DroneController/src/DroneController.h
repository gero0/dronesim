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

    PidValues get_last_pid_output();

    PidValues get_setpoints();

    PidTunings get_altitude_tunings();

    PidTunings get_pitch_tunings();

    PidTunings get_roll_tunings();

    PidTunings get_yaw_tunings();

    PidTunings get_pitch_rate_tunings();

    PidTunings get_roll_rate_tunings();

    PidTunings get_yaw_rate_tunings();

    PidTunings get_vs_tunings();

    void set_pitch_rate_tunings(PidTunings tunings);

    void set_roll_rate_tunings(PidTunings tunings);

    void set_yaw_rate_tunings(PidTunings tunings);

    void set_vs_tunings(PidTunings tunings);

    void set_altitude_tunings(PidTunings tunings);

    void set_pitch_tunings(PidTunings tunings);

    void set_roll_tunings(PidTunings tunings);

    void set_yaw_tunings(PidTunings tunings);

    void set_altitude(float sp);

    void set_pitch(float sp);

    void set_roll(float sp);

    void set_yaw(float sp);

    void level();

    bool is_stopped();

    void start();

    bool stop();

    void hover();

    void RTO();

    void reset_position();

    void set_hover_setpoint(Vector3 sp);

    Vector3 get_position();

    Vector3 get_hover_setpoint();

    Rotation get_rotation();

    Rotation get_angular_rates();

    Rotation get_rotation_setpoints();

    float get_vertical_speed();

    float get_altitude();

    float get_radar_altitude();

    float get_absolute_altitude();

    float get_altitude_setpoint() const;

    float get_direct_thrust() const;

    void set_direct_thrust(float thrust);

    std::array<float, 4> get_motor_speeds();

    void yaw_raw_input(float input);

private:
    SensorReader *sensor_reader;

    bool is_stopped_v = false;

    const float yaw_raw_constant = 0.1;
    float yaw_raw = 0.0f;
    float yaw_setpoint = 0.0f;
    float pitch_setpoint = 0.0f;
    float roll_setpoint = 0.0f;
    float altitude_setpoint = 0.0f;

    float yaw_rate_setpoint = 0.0f;
    float pitch_rate_setpoint = 0.0f;
    float roll_rate_setpoint = 0.0f;

    float v_thrust = 0.0f;
    float v_pitch = 0.0f;
    float v_roll = 0.0f;
    float v_yaw = 0.0f;

    float vertical_speed = 0.0f;
    float altitude = 0.0f;
    float radar_altitude = 0.0f;
    float absolute_altitude = 0.0f;
    float prev_altitude = 0.0f;
    float direct_thrust_value = 0.0f;

    void control_update(float dt);
    void reset_pids();

    MotorDriver *front_left;
    MotorDriver *front_right;
    MotorDriver *back_left;
    MotorDriver *back_right;

    PID position_x_pid{0.0f, 0.0f, 0.0f, -1.0f, 1.0f};
    PID position_y_pid{0.0f, 0.0f, 0.0f, -1.0f, 1.0f};

    PID altitude_pid{0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    PID vs_pid{0.0f, 0.0f, 0.0f, 0.0f, 1.0f};

    PID pitch_pid{0.00f, 0.0f, 0.0f, -1.0f, 1.0f, -1.0, 1.0};
    PID roll_pid{0.0f, 0.0f, 0.0f, -1.0f, 1.0f, -1.0, 1.0};
    PID yaw_pid{0.00f, 0.0f, 0.0f, -1.0f, 1.0f};

    PID pitch_rate_pid{0.000f, 0.0f, 0.0f, -1.0f, 1.0f, -1.0, 1.0};
    PID roll_rate_pid{0.000f, 0.0f, 0.0f, -1.0f, 1.0f, -1.0, 1.0};
    PID yaw_rate_pid{0.000f, 0.0f, 0.0f, -1.0f, 1.0f};

    Vector3 position{0.0f, 0.0f, 0.0f};
    Vector3 velocity{0.0f, 0.0f, 0.0f};
    Vector3 acceleration {0.0f, 0.0f, 0.0f};

    Rotation rotation{0.0f, 0.0f, 0.0f};
    Rotation angular_rate{0.0f, 0.0f, 0.0f};

    Vector3 hover_setpoint{0.0f, 0.0f, 0.0f};

//    ControlState controlState = ControlState::PointHover;
    ControlState controlState = ControlState::Direct;
};

#endif //DRONESIMPROJECT_DRONECONTROLLER_H
