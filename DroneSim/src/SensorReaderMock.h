//
// Created by gero on 7/2/23.
//

#ifndef DRONESIMPROJECT_SENSORREADERMOCK_H
#define DRONESIMPROJECT_SENSORREADERMOCK_H

#include <SensorReader.h>

class SensorReaderMock : public SensorReader{
public:
    Vector3 getAcceleration() override{
        return acceleration;
    }
    Vector3 getAngularAcceleration() override{
        return angular_acceleration;
    }
    float getAltitude() override{
        return altitude;
    }

    Vector3 acceleration {0.0f,0.0f,0.0f};
    Vector3 angular_acceleration {0.0f,0.0f,0.0f};
    float altitude = 0.0f;
};


#endif //DRONESIMPROJECT_SENSORREADERMOCK_H
