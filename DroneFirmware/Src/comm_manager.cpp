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
            emergency_stop(4);
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
            ControlMode cmode = controller->get_current_cmode();
            ThrustMode tmode = controller->get_current_tmode();
            bool ok = controller->is_all_nominal();
            xSemaphoreGive(controller_mutex);
            uint8_t speeds_int[4];
            for (int i = 0; i < 4; i++) {
                speeds_int[i] = static_cast<uint8_t>(speeds[i] * 100.0f);
            }
            output_msg.type = GetStatus;

            char ok_c = ok ? '^' : '!';
            char cmode_c = (cmode == ControlMode::Angle) ? 'A' : 'R';
            char tmode_c = (tmode == ThrustMode::Direct) ? 'D' : 'H';
            char pad = '-';
            char status_str[5] = {ok_c, cmode_c, tmode_c, pad, 0};

            memcpy(&output_msg.data[0], speeds_int, sizeof(uint8_t) * 4);
            memcpy(&output_msg.data[4], &setpoints.v_thrust, sizeof(float));
            memcpy(&output_msg.data[8], &setpoints.v_pitch, sizeof(float));
            memcpy(&output_msg.data[12], &setpoints.v_yaw, sizeof(float));
            memcpy(&output_msg.data[16], &setpoints.v_roll, sizeof(float));
            memcpy(&output_msg.data[20], &abs, sizeof(float));
            memcpy(&output_msg.data[24], &status_str, sizeof(status_str));
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
        case GetTuningsYA:{
            xSemaphoreTake(controller_mutex, portMAX_DELAY);
            PidTunings Yaw = controller->get_yaw_tunings();
            PidTunings Thrust = controller->get_altitude_tunings();
            xSemaphoreGive(controller_mutex);
            output_msg.type = GetTuningsYA;
            memcpy(&output_msg.data[0], &Yaw.Kp, sizeof(float));
            memcpy(&output_msg.data[4], &Yaw.Ki, sizeof(float));
            memcpy(&output_msg.data[8], &Yaw.Kd, sizeof(float));
            memcpy(&output_msg.data[12], &Thrust.Kp, sizeof(float));
            memcpy(&output_msg.data[16], &Thrust.Ki, sizeof(float));
            memcpy(&output_msg.data[20], &Thrust.Kd, sizeof(float));
        }
            break;
        case GetTuningsPrRr:{
            xSemaphoreTake(controller_mutex, portMAX_DELAY);
            PidTunings pr = controller->get_pitch_rate_tunings();
            PidTunings rr = controller->get_roll_rate_tunings();
            xSemaphoreGive(controller_mutex);
            output_msg.type = GetTuningsPrRr;
            memcpy(&output_msg.data[0], &pr.Kp, sizeof(float));
            memcpy(&output_msg.data[4], &pr.Ki, sizeof(float));
            memcpy(&output_msg.data[8], &pr.Kd, sizeof(float));
            memcpy(&output_msg.data[12], &rr.Kp, sizeof(float));
            memcpy(&output_msg.data[16], &rr.Ki, sizeof(float));
            memcpy(&output_msg.data[20], &rr.Kd, sizeof(float));
        }
            break;
        case GetTuningsYrVs:{
            xSemaphoreTake(controller_mutex, portMAX_DELAY);
            PidTunings yr = controller->get_yaw_rate_tunings();
            PidTunings vs = controller->get_vs_tunings();
            xSemaphoreGive(controller_mutex);
            output_msg.type = GetTuningsYrVs;
            memcpy(&output_msg.data[0], &yr.Kp, sizeof(float));
            memcpy(&output_msg.data[4], &yr.Ki, sizeof(float));
            memcpy(&output_msg.data[8], &yr.Kd, sizeof(float));
            memcpy(&output_msg.data[12], &vs.Kp, sizeof(float));
            memcpy(&output_msg.data[16], &vs.Ki, sizeof(float));
            memcpy(&output_msg.data[20], &vs.Kd, sizeof(float));
        }
            break;
        case GetRates:{
            xSemaphoreTake(controller_mutex, portMAX_DELAY);
            Rotation rates = controller->get_angular_rates();
            float vs = controller->get_vertical_speed();
            xSemaphoreGive(controller_mutex);
            output_msg.type = GetRates;
            memcpy(&output_msg.data[0], &rates, sizeof(float) * 3);
            memcpy(&output_msg.data[12], &vs, sizeof(float));
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
    if(response_state == ResponseState::Basic){
        if(currentMsgType == (MessageType)(9)){
            currentMsgType = GetPosition;
        }
    }else{
        if(currentMsgType == (MessageType)(13)){
            currentMsgType = GetPosition;
            response_state = ResponseState::Basic;
        }
    }
}


CommState CommManager::receive_message(Message *output_msg, TickType_t *last_contact_time) {
    const TickType_t connlost_threshold = 2000;
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

                auto timestamp = xTaskGetTickCount();

                xSemaphoreTake(controller_mutex, portMAX_DELAY);

                if(commands & MSG_LAND_CMD){
                    //controller->auto_land();
                }
                if(commands & MSG_ESTOP_CMD){
                    emergency_stop(5);
                }
                if(commands & MSG_HOLD_CMD){
                    //controller->hold()
                }
                if(commands & MSG_RTO_CMD){
                    //controller->RTO();
                }
                if(commands & MSG_STOP_CMD){
                    if(timestamp - last_stop_cmd_received >= stop_cmd_threshold){
                        if(controller->is_stopped()){
                            controller->start();
                        }else{
                            controller->stop();
                        }
                        last_stop_cmd_received = timestamp;
                    }
                }
                if(commands & MSG_SWITCHALT_CMD){
                    ThrustMode mode = controller->get_current_tmode();
                    if(mode == ThrustMode::Direct){
                        controller->switch_to_althold();
                    }else{
                        controller->switch_to_direct_thrust();
                    }
                }

                controller->pitch_input(pitch_input);
                controller->roll_input(roll_input);
                controller->yaw_input(yaw_input);
                controller->thrust_input(alt_input);

                xSemaphoreGive(controller_mutex);
            }
            break;
            case DataRequest:
                response_state = ResponseState::Full;
                break;
//            case SetTuningsPR: {
//                float Pkp = *(float *) (&msg.data[0]);
//                float Pki = *(float *) (&msg.data[4]);
//                float Pkd = *(float *) (&msg.data[8]);
//                float Rkp = *(float *) (&msg.data[12]);
//                float Rki = *(float *) (&msg.data[16]);
//                float Rkd = *(float *) (&msg.data[20]);
//
//                xSemaphoreTake(controller_mutex, portMAX_DELAY);
//                controller->set_pitch_tunings({Pkp, Pki, Pkd});
//                controller->set_roll_tunings({Rkp, Rki, Rkd});
//                xSemaphoreGive(controller_mutex);
//            }
//                break;
//            case SetTuningsYA: {
//                float Ykp = *(float *) (&msg.data[0]);
//                float Yki = *(float *) (&msg.data[4]);
//                float Ykd = *(float *) (&msg.data[8]);
//                float Tkp = *(float *) (&msg.data[12]);
//                float Tki = *(float *) (&msg.data[16]);
//                float Tkd = *(float *) (&msg.data[20]);
//
//                xSemaphoreTake(controller_mutex, portMAX_DELAY);
//                controller->set_yaw_tunings({Ykp, Yki, Ykd});
//                controller->set_altitude_tunings({Tkp, Tki, Tkd});
//                xSemaphoreGive(controller_mutex);
//            }
//                break;
//            case SetTuningsPrRr: {
//                float Prkp = *(float *) (&msg.data[0]);
//                float Prki = *(float *) (&msg.data[4]);
//                float Prkd = *(float *) (&msg.data[8]);
//                float Rrkp = *(float *) (&msg.data[12]);
//                float Rrki = *(float *) (&msg.data[16]);
//                float Rrkd = *(float *) (&msg.data[20]);
//
//                xSemaphoreTake(controller_mutex, portMAX_DELAY);
//                controller->set_pitch_rate_tunings({Prkp, Prki, Prkd});
//                controller->set_roll_rate_tunings({Rrkp, Rrki, Rrkd});
//                xSemaphoreGive(controller_mutex);
//            }
//                break;
//            case SetTuningsYrVs: {
//                float Yrkp = *(float *) (&msg.data[0]);
//                float Yrki = *(float *) (&msg.data[4]);
//                float Yrkd = *(float *) (&msg.data[8]);
//                float Vskp = *(float *) (&msg.data[12]);
//                float Vski = *(float *) (&msg.data[16]);
//                float Vskd = *(float *) (&msg.data[20]);
//
//                xSemaphoreTake(controller_mutex, portMAX_DELAY);
//                controller->set_yaw_rate_tunings({Yrkp, Yrki, Yrkd});
//                controller->set_vs_tunings({Vskp, Vski, Vskd});
//                xSemaphoreGive(controller_mutex);
//            }
//                break;
            default:
                break;
        }
        prepareResponse();
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, static_cast<GPIO_PinState>(0));
    }
     if (xTaskGetTickCount() - *last_contact_time > connlost_threshold) {
         emergency_stop(6);
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

void CommManager::emergency_stop(uint8_t code){
    if(stop_function != nullptr){
        stop_function(code);
    }else{
        while(1){
            //freeze here
        }
    }
}