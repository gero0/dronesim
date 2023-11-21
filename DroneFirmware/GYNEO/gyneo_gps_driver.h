//
// Created by gero on 10/14/23.
//

#ifndef DRONEFIRMWARE_GYNEO_GPS_DRIVER_H
#define DRONEFIRMWARE_GYNEO_GPS_DRIVER_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>

void gyneo_byte_received(uint8_t byte);

void gyneo_init();

void gyneo_reset_starting_position();

float gyneo_get_x();

float gyneo_get_y();
//float gyneo_get_z();

#ifdef __cplusplus
};
#endif


#endif //DRONEFIRMWARE_GYNEO_GPS_DRIVER_H
