//
// Created by gero on 11/13/23.
//

#include <cstring>
#include "comm_manager.h"

void CommManager::update() {
    switch (comm_state) {
        case CommState::Init: {
            bool ok = init_transceiver();
            if (ok) {
                comm_state = CommState::Listen;
            } else {
                comm_state = CommState::Error;
            }
        }
            break;

        case CommState::Listen:
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, static_cast<GPIO_PinState>(0));
            comm_state = receive_message(&output_msg, &last_contact_time);
            break;

        case CommState::Send: {
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, static_cast<GPIO_PinState>(1));
            uint8_t output_buffer[NRF24_PAYLOAD_SIZE];
            nRF24_TX_Mode();
            serialize_msg(output_buffer, &output_msg);
            nRF24_WriteTXPayload(output_buffer, NRF24_PAYLOAD_SIZE);
            waitTXTimeout(10);
            nRF24_WaitTX();
            nRF24_RX_Mode();
            comm_state = CommState::Listen;
        }
            break;

        case CommState::ConnLost:
            xSemaphoreTake(controller_mutex, portMAX_DELAY);
            controller->hover();
            controller->set_hover_setpoint(position_of_last_contact);
            xSemaphoreGive(controller_mutex);
            break;

        case CommState::Error:
            emergency_stop();
            break;
    }
}

bool CommManager::init_transceiver() {
    vTaskDelay(100 / portTICK_RATE_MS);
    nRF24_Init(nrf_spi_handle);
    vTaskDelay(100 / portTICK_RATE_MS);
    nRF24_SetCRCLength(NRF24_CRC_WIDTH_2B);
    nRF24_SetRXAddress(0, (uint8_t *) "Dro");
    nRF24_SetTXAddress((uint8_t *) "Pil");
    nRF24_RX_Mode();
    return true;
}

CommState CommManager::receive_message(Message *output_msg, TickType_t *last_contact_time) {
    const TickType_t connlost_threshold = 3000;
    const float altitude_const = 0.1;
    const float max_angle = (35.0f / 180.0f) * M_PI;
    const float yaw_constant = 0.1f;

    static TickType_t last_angle_input;
    static TickType_t last_altitude_input;

    uint8_t input_buffer[NRF24_PAYLOAD_SIZE];

    if (nRF24_RXAvailible()) {
        *last_contact_time = xTaskGetTickCount();
        uint8_t read;
        nRF24_ReadRXPaylaod(input_buffer, &read);
        Message msg = parse_message(input_buffer);
        switch (msg.type) {
            case AnglesInput: {
                float pitch_input = *(float *) (&msg.data[0]);
                float yaw_input = *(float *) (&msg.data[sizeof(float)]);
                float roll_input = *(float *) (&msg.data[2 * sizeof(float)]);

                pitch_input = std::clamp(pitch_input, -1.0f, 1.0f);
                yaw_input = std::clamp(yaw_input, -1.0f, 1.0f);
                roll_input = std::clamp(roll_input, -1.0f, 1.0f);

                xSemaphoreTake(controller_mutex, portMAX_DELAY);
                controller->set_pitch(pitch_input * max_angle);
                controller->set_roll(roll_input * max_angle);

                auto timestamp = xTaskGetTickCount();
                if (timestamp > last_angle_input) {
                    last_angle_input = timestamp;
                    auto [pitch, yaw, roll] = controller->get_rotation_setpoints();
                    yaw = yaw + yaw_input * yaw_constant;
                    controller->set_yaw(yaw);
                }
                xSemaphoreGive(controller_mutex);
                return CommState::Listen;
            }
            case AltitudeInput: {
                float alt_input = *(float *) (&msg.data[0]);
                auto timestamp = xTaskGetTickCount();
                if (timestamp > last_altitude_input) {
                    last_altitude_input = timestamp;
                    float altitude_sp = controller->get_altitude_setpoint();
                    altitude_sp += alt_input * altitude_const;
                    xSemaphoreTake(controller_mutex, portMAX_DELAY);
                    controller->set_altitude(altitude_sp);
                    xSemaphoreGive(controller_mutex);
                    return CommState::Listen;
                }
            }
                break;
            case HoldCommand:
                xSemaphoreTake(controller_mutex, portMAX_DELAY);
                controller->hover();
                xSemaphoreGive(controller_mutex);
                return CommState::Listen;
            case RTOCommand:
                xSemaphoreTake(controller_mutex, portMAX_DELAY);
                controller->RTO();
                xSemaphoreGive(controller_mutex);
                return CommState::Listen;
            case GetAngles: {
                xSemaphoreTake(controller_mutex, portMAX_DELAY);
                Rotation rot = controller->get_rotation();
                xSemaphoreGive(controller_mutex);
                output_msg->type = GetAngles;
                memcpy(output_msg->data, &rot, sizeof(rot));
                return CommState::Send;
            }
            case GetPosition: {
                xSemaphoreTake(controller_mutex, portMAX_DELAY);
                Vector3 pos = controller->get_position();
                xSemaphoreGive(controller_mutex);
                output_msg->type = GetPosition;
                memcpy(output_msg->data, &pos, sizeof(pos));
                return CommState::Send;
            }
            case GetAltitude: {
                xSemaphoreTake(controller_mutex, portMAX_DELAY);
                float alt = controller->get_altitude();
                float radar = controller->get_radar_altitude();
                xSemaphoreGive(controller_mutex);
                output_msg->type = GetAltitude;
                memcpy(output_msg->data, &alt, sizeof(float));
                memcpy(output_msg->data + sizeof(float), &radar, sizeof(float));
                return CommState::Send;
            }
            case GetStatus: {
                xSemaphoreTake(controller_mutex, portMAX_DELAY);
                auto speeds = controller->get_motor_speeds();
                xSemaphoreGive(controller_mutex);
                uint8_t speeds_int[4];
                for (int i = 0; i < 4; i++) {
                    speeds_int[i] = static_cast<uint8_t>(speeds[i] * 100.0f);
                }
                output_msg->type = GetStatus;
                memcpy(output_msg->data, speeds_int, sizeof(uint8_t) * 4);
                return CommState::Send;
            }
        }
    }
    if (xTaskGetTickCount() - *last_contact_time > connlost_threshold) {
        return CommState::ConnLost;
    }
    return CommState::Listen;
}

bool CommManager::waitTXTimeout(uint32_t timeout) {
    auto begin = xTaskGetTickCount();
    auto now = begin;
    do {
        now = xTaskGetTickCount();
        if (nRF24_TXDone()) {
            return true;
        }
    } while ((now - begin) < timeout);
    return false;
}

void CommManager::emergency_stop(){
    if(stop_function != nullptr){
        stop_function();
    }else{
        while(1){
            //freeze here
        }
    }
}