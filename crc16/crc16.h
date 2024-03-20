//
// Created by gero on 3/12/24.
//

#ifndef DRONEFIRMWARE_CRC16_H
#define DRONEFIRMWARE_CRC16_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif

    uint16_t crc16 (const uint8_t *nData, uint16_t wLength);

#ifdef __cplusplus
};
#endif

#endif //DRONEFIRMWARE_CRC16_H
