//
// Created by gero on 7/1/23.
//

#ifndef DRONESIMPROJECT_SENSORREADER_H
#define DRONESIMPROJECT_SENSORREADER_H

#include "algebra.h"

class SensorReader {
public:
    virtual Vector3 getAcceleration() = 0;
    virtual Rotation getAngularAcceleration() = 0;
    virtual float getAltitude() = 0;
};

#endif //DRONESIMPROJECT_SENSORREADER_H
