//
// Created by gero on 11/26/23.
//

#include "BLDCController.h"

BLDCController::BLDCController(uint16_t* ccr_register, uint16_t off_value, uint16_t min_value,
                               uint16_t max_value) {
    this->off_value = off_value;
    this->min_value = min_value;
    this->max_value = max_value;
    this->ccr_register = ccr_register;
}

void BLDCController::set_speed(float speed) {
    const float reg_val = static_cast<float>(min_value)
        + speed * static_cast<float>(max_value - min_value);
    *ccr_register = static_cast<uint16_t>(reg_val);
}

float BLDCController::get_speed() {
    return speed;
}

void BLDCController::disable() {
    *ccr_register = off_value;
    enabled = false;
}

void BLDCController::enable() {
    *ccr_register = min_value;
    enabled = true;
}

bool BLDCController::is_enabled() const {
    return enabled;
}
