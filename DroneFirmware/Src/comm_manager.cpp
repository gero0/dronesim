//
// Created by gero on 11/13/23.
//

#include <cstring>
#include "comm_manager.h"

void CommManager::update() {
    Message input_msg;
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
            receive_message(&input_msg, &last_contact_time);
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
    rtos_delay(100);
    nRF24_Init(nrf_spi_handle);
    rtos_delay(100);
    nRF24_SetCRCLength(NRF24_CRC_WIDTH_2B);
    nRF24_SetRXAddress(0, (uint8_t *) "Dro");
    nRF24_SetTXAddress((uint8_t *) "Pil");
    nRF24_RX_Mode();
    nRF24_FlushTX();
    nRF24_FlushRX();
    prepareResponse();
    return true;
}


void CommManager::prepareResponse() {
    Message output_msg;
    switch (currentMsgType) {
        case GetPosition: {
            xSemaphoreTake(controller_mutex, portMAX_DELAY);
            Vector3 pos = controller->get_position();
            Rotation rot = controller->get_rotation();
            float alt = controller->get_altitude();
            float radar = controller->get_radar_altitude();
            xSemaphoreGive(controller_mutex);
            output_msg.type = GetPosition;
            memcpy(&output_msg.data[0], &rot.pitch, sizeof(float));
            memcpy(&output_msg.data[4], &rot.yaw, sizeof(float));
            memcpy(&output_msg.data[8], &rot.roll, sizeof(float));
            memcpy(&output_msg.data[12], &pos.x, sizeof(float));
            memcpy(&output_msg.data[16], &pos.y, sizeof(float));
            memcpy(&output_msg.data[20], &alt, sizeof(float));
            memcpy(&output_msg.data[24], &radar, sizeof(float));
        }
        break;
        case GetStatus: {
            xSemaphoreTake(controller_mutex, portMAX_DELAY);
            auto speeds = controller->get_motor_speeds();
            auto setpoints = controller->get_setpoints();
            float abs = controller->get_absolute_altitude();
            xSemaphoreGive(controller_mutex);
            uint8_t speeds_int[4];
            for (int i = 0; i < 4; i++) {
                speeds_int[i] = static_cast<uint8_t>(speeds[i] * 100.0f);
            }
            output_msg.type = GetStatus;
            memcpy(&output_msg.data[0], speeds_int, sizeof(uint8_t) * 4);
            memcpy(&output_msg.data[4], &setpoints.v_thrust, sizeof(float));
            memcpy(&output_msg.data[8], &setpoints.v_pitch, sizeof(float));
            memcpy(&output_msg.data[12], &setpoints.v_yaw, sizeof(float));
            memcpy(&output_msg.data[16], &setpoints.v_roll, sizeof(float));
            memcpy(&output_msg.data[20], &abs, sizeof(float));
        }
        break;
        case GetTuningsPR:{
            xSemaphoreTake(controller_mutex, portMAX_DELAY);
            PidTunings Pitch = controller->get_pitch_tunings();
            PidTunings Roll = controller->get_roll_tunings();
            xSemaphoreGive(controller_mutex);
            output_msg.type = GetTuningsPR;
            memcpy(&output_msg.data[0], &Pitch.Kp, sizeof(float));
            memcpy(&output_msg.data[4], &Pitch.Ki, sizeof(float));
            memcpy(&output_msg.data[8], &Pitch.Kd, sizeof(float));
            memcpy(&output_msg.data[12], &Roll.Kp, sizeof(float));
            memcpy(&output_msg.data[16], &Roll.Ki, sizeof(float));
            memcpy(&output_msg.data[20], &Roll.Kd, sizeof(float));
        }
            break;
        case GetTuningsYT:{
            xSemaphoreTake(controller_mutex, portMAX_DELAY);
            PidTunings Yaw = controller->get_yaw_tunings();
            PidTunings Thrust = controller->get_thrust_tunings();
            xSemaphoreGive(controller_mutex);
            output_msg.type = GetTuningsYT;
            memcpy(&output_msg.data[0], &Yaw.Kp, sizeof(float));
            memcpy(&output_msg.data[4], &Yaw.Ki, sizeof(float));
            memcpy(&output_msg.data[8], &Yaw.Kd, sizeof(float));
            memcpy(&output_msg.data[12], &Thrust.Kp, sizeof(float));
            memcpy(&output_msg.data[16], &Thrust.Ki, sizeof(float));
            memcpy(&output_msg.data[20], &Thrust.Kd, sizeof(float));
        }
            break;
        default:
            currentMsgType = GetPosition;
            break;
    }
    uint8_t buffer[32];
    serialize_msg(buffer, &output_msg);
    nRF24_FlushTX();
    nRF24_WriteAckPayload(buffer, 32);
    currentMsgType = (MessageType)((int)(currentMsgType) + 1);
    if(currentMsgType == (MessageType)(8)){
        currentMsgType = GetPosition;
    }
}


CommState CommManager::receive_message(Message *output_msg, TickType_t *last_contact_time) {
    const TickType_t connlost_threshold = 3000;
    const float altitude_const = 0.4;
    const float thrust_const = 0.05;
    const float max_angle = (35.0f / 180.0f) * M_PI;
    const float yaw_constant = 0.1f;

    static TickType_t last_angle_input;
    static TickType_t last_altitude_input;

    uint8_t input_buffer[NRF24_PAYLOAD_SIZE];

    if (nRF24_RXAvailible()) {
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, static_cast<GPIO_PinState>(1));
        *last_contact_time = xTaskGetTickCount();
        uint8_t read;
        nRF24_ReadRXPaylaod(input_buffer, &read);
        Message msg = parse_message(input_buffer);
        switch (msg.type) {
            case Input: {
                float pitch_input = *(float *) (&msg.data[0]);
                float yaw_input = *(float *) (&msg.data[4]);
                float roll_input = *(float *) (&msg.data[8]);
                float alt_input = *(float *) (&msg.data[12]);
                uint8_t commands = *(uint8_t *) (&msg.data[16]);

                pitch_input = std::clamp(pitch_input, -1.0f, 1.0f);
                yaw_input = std::clamp(yaw_input, -1.0f, 1.0f);
                roll_input = std::clamp(roll_input, -1.0f, 1.0f);
                if( std::abs(alt_input) < 0.1){
                    alt_input = 0;
                }

                auto timestamp = xTaskGetTickCount();

                xSemaphoreTake(controller_mutex, portMAX_DELAY);

                if(commands & MSG_LAND_CMD){
                    //controller->auto_land();
                }
                if(commands & MSG_ESTOP_CMD){
                    emergency_stop();
                }
                if(commands & MSG_HOLD_CMD){
                    //controller->hold()
                }
                if(commands & MSG_RTO_CMD){
                    //controller->RTO();
                }

                controller->set_pitch(pitch_input * max_angle);
                controller->set_roll(roll_input * max_angle);
                controller->yaw_raw_input(yaw_input);

//                if (timestamp > last_angle_input) {
//                    last_angle_input = timestamp;
//                    auto [pitch, yaw, roll] = controller->get_rotation_setpoints();
//                    yaw = yaw + yaw_input * yaw_constant;
//                    controller->set_yaw(yaw);
//                }

                if (timestamp > last_altitude_input) {
//                    last_altitude_input = timestamp;
//                    float altitude_sp = controller->get_altitude_setpoint();
//                    altitude_sp += alt_input * altitude_const;
//                    controller->set_altitude(altitude_sp);
                    float thrust = controller->get_direct_thrust();
                    thrust += alt_input * thrust_const;
                    thrust = std::clamp(thrust, 0.0f, 1.0f);
                    controller->set_direct_thrust(thrust);
                }
                xSemaphoreGive(controller_mutex);
            }
            break;
            case DataRequest:
                //RESERVED
                break;
            case SetTuningsPR: {
                float Pkp = *(float *) (&msg.data[0]);
                float Pki = *(float *) (&msg.data[4]);
                float Pkd = *(float *) (&msg.data[8]);
                float Rkp = *(float *) (&msg.data[12]);
                float Rki = *(float *) (&msg.data[16]);
                float Rkd = *(float *) (&msg.data[20]);

                xSemaphoreTake(controller_mutex, portMAX_DELAY);
                controller->set_pitch_tunings({Pkp, Pki, Pkd});
                controller->set_roll_tunings({Rkp, Rki, Rkd});
                xSemaphoreGive(controller_mutex);
            }
                break;
            case SetTuningsYT: {
                float Ykp = *(float *) (&msg.data[0]);
                float Yki = *(float *) (&msg.data[4]);
                float Ykd = *(float *) (&msg.data[8]);
                float Tkp = *(float *) (&msg.data[12]);
                float Tki = *(float *) (&msg.data[16]);
                float Tkd = *(float *) (&msg.data[20]);

                xSemaphoreTake(controller_mutex, portMAX_DELAY);
                controller->set_yaw_tunings({Ykp, Yki, Ykd});
                controller->set_thrust_tunings({Tkp, Tki, Tkd});
                xSemaphoreGive(controller_mutex);
            }
                break;
            default:
                break;
        }
        prepareResponse();
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, static_cast<GPIO_PinState>(0));
    }
     if (xTaskGetTickCount() - *last_contact_time > connlost_threshold) {
         emergency_stop();
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