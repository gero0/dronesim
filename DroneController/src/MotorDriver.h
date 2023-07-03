//
// Created by gero on 7/1/23.
//

#ifndef DRONESIMPROJECT_MOTORDRIVER_H
#define DRONESIMPROJECT_MOTORDRIVER_H

class MotorDriver{
public:
    virtual void set_speed(float speed) = 0;
    virtual float get_speed() = 0;
};

#endif //DRONESIMPROJECT_MOTORDRIVER_H
