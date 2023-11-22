//
// Created by gero on 7/2/23.
//

#ifndef DRONESIMPROJECT_MOTORDRIVERMOCK_H
#define DRONESIMPROJECT_MOTORDRIVERMOCK_H

#include <MotorDriver.h>

class MotorDriverMock : public MotorDriver {
public:
    void set_speed(float speed) override {
        this->speed = speed;
    }
    float get_speed() override {
        return speed;
    }
private:
    float speed = 0.0f;
};


#endif //DRONESIMPROJECT_MOTORDRIVERMOCK_H
