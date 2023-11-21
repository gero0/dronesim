//
// Created by gero on 10/14/23.
//
#include <memory.h>
#include <stdbool.h>
#include <minmea.h>
#include "gyneo_gps_driver.h"

#define UART_BUF_SIZE 512
uint8_t uart_buffer[UART_BUF_SIZE];
uint32_t uart_msg_len = 0;

bool message_to_process = false;

void message_received() {

}

void gyneo_byte_received(uint8_t byte) {
    if (message_to_process) {
        return;
    }
    uart_buffer[uart_msg_len] = byte;

    if (uart_buffer[uart_msg_len] == '\n') {
        message_to_process = true;
    } else if (uart_msg_len >= UART_BUF_SIZE) {
        uart_msg_len = 0;
        memset(uart_buffer, 0, UART_BUF_SIZE);
    } else {
        uart_msg_len += 1;
    }
}

void gyneo_init() {
}

void gyneo_process() {
    switch (minmea_sentence_id((const char *) uart_buffer, false)) {
        case MINMEA_SENTENCE_RMC: {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, (const char *) uart_buffer)) {
            }
        }
            break;

        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, (const char *) uart_buffer)) {
            }
        }
            break;

        case MINMEA_SENTENCE_GSV: {
            struct minmea_sentence_gsv frame;
            if (minmea_parse_gsv(&frame, (const char *) uart_buffer)) {
            }
        }
            break;
        default:
            break;
    }

}

void gyneo_update() {
    if (message_to_process) {
        //process message
        gyneo_process();
        uart_msg_len = 0;
        memset(uart_buffer, 0, UART_BUF_SIZE);
        message_to_process = false;
    }
}

void gyneo_reset_starting_position() {

}

float gyneo_get_x() {

}

float gyneo_get_y() {

}

