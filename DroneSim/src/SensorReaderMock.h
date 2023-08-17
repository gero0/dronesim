//
// Created by gero on 7/2/23.
//

#ifndef DRONESIMPROJECT_SENSORREADERMOCK_H
#define DRONESIMPROJECT_SENSORREADERMOCK_H

#include <SensorReader.h>

class SensorReaderMock : public SensorReader{
public:
    Vector3 get_acceleration() override{
        return acceleration;
    }
    Rotation get_rotation() override{
        return rotation;
    }
//    float get_altitude() override{
//        return altitude;
//    }

    void update(float dt) override{
        angular_velocity += angular_acceleration * dt;
        rotation += angular_velocity * dt;
        //TODO: fix this and remove this workaround
        float yaw_og = rotation.yaw;
        rotation.normalize();
        rotation.yaw = yaw_og;
    }

    Rotation rotation {0,0,0};
    Vector3 acceleration {0.0f,0.0f,0.0f};
    Rotation angular_acceleration {0.0f,0.0f,0.0f};
    Rotation angular_velocity {0.0f,0.0f,0.0f};
    float altitude = 0.0f;
};


#endif //DRONESIMPROJECT_SENSORREADERMOCK_H
