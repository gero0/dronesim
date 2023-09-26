//
// Created by gero on 7/1/23.
//

#ifndef DRONESIMPROJECT_SENSORREADER_H
#define DRONESIMPROJECT_SENSORREADER_H

#include "algebra.h"

class SensorReader {
public:
    virtual Vector3 get_acceleration() = 0;
    virtual Rotation get_rotation() = 0;
    virtual float get_altitude() = 0;
    virtual float get_radar_altitude() = 0;
    virtual void update(float dt) = 0;
};

#endif //DRONESIMPROJECT_SENSORREADER_H
