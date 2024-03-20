//
// Created by gero on 11/26/23.
//

#ifndef BLDC_DRIVER_H
#define BLDC_DRIVER_H

#include <cstdint>
#include <stm32f4xx_hal.h>

#include "MotorDriver.h"

class BLDCController : public MotorDriver {
public:
    BLDCController(uint16_t* ccr_register, uint16_t off_value, uint16_t min_value, uint16_t max_value);
    void set_speed(float speed) override;
    float get_speed() override;

    void disable();
    void enable();
    bool is_enabled() const;
    void lock();
private:
    bool enabled = false;
    uint16_t off_value = 0;
    uint16_t min_value = 0;
    uint16_t max_value = 0;
    float speed = 0.0f;
    bool locked = false;

    uint16_t* ccr_register = nullptr;
};



#endif //BLDC_DRIVER_H
