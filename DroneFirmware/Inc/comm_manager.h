//
// Created by gero on 11/13/23.
//

#ifndef DRONEFIRMWARE_COMM_MANAGER_H
#define DRONEFIRMWARE_COMM_MANAGER_H

#include <functional>
#include <utility>
#include "message.h"
#include "FreeRTOS.h"
#include "task.h"
#include "DroneController.h"
#include "semphr.h"

extern "C" {
#include "nRF24_Defs.h"
#include "nRF24.h"
}

enum class CommState {
    Init,
    Listen,
    Send,
    ConnLost,
    Error
};

class CommManager {
public:
    CommManager(
            DroneController *controller, SemaphoreHandle_t controller_mutex,
            SPI_HandleTypeDef *nrf_spi_handle, std::function<void()> stop_function = nullptr)
            : controller(controller), controller_mutex(controller_mutex),
            nrf_spi_handle(nrf_spi_handle), stop_function(std::move(stop_function))
            {

            }

    void update();
    CommState receive_message(Message *output_msg, TickType_t *last_contact_time);

private:
    bool init_transceiver();
    bool waitTXTimeout(uint32_t timeout);
    void emergency_stop();

    DroneController *controller;
    SemaphoreHandle_t controller_mutex;
    std::function<void()> stop_function;
    SPI_HandleTypeDef *nrf_spi_handle;

    CommState comm_state = CommState::Init;
    Message output_msg;
    Vector3 position_of_last_contact = {0.0f, 0.0f, 0.0f};
    TickType_t last_contact_time = xTaskGetTickCount();
};


#endif //DRONEFIRMWARE_COMM_MANAGER_H
