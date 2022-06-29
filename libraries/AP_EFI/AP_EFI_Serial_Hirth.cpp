/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <AP_HAL/AP_HAL.h>

#include "AP_EFI_Serial_Hirth.h"

#if HAL_EFI_ENABLED
#include <AP_SerialManager/AP_SerialManager.h>
#include "SRV_Channel/SRV_Channel.h"

/* PCK : Testing*/
#include <GCS_MAVLink/GCS.h>


/* Constructor with initialization */
AP_EFI_Serial_Hirth::AP_EFI_Serial_Hirth(AP_EFI &_frontend) : AP_EFI_Backend(_frontend) {
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI, 0);

    req_data.code = 0;
    send_request_status();

    response_complete = true;
    response_counter = 0;
}


void AP_EFI_Serial_Hirth::update() {
    if (port != nullptr) {

        /* parse response if any */
        num_bytes = port->available();

        if (num_bytes == get_quantity()) {
            checksum = 0;
            checksum += res_data.quantity = port->read();
            checksum += res_data.code = port->read();

            if (res_data.quantity == get_quantity()) {

                // Read data from status response
                if (res_data.code == req_data.code) {
                    for (int i = 0; i < (res_data.quantity - QUANTITY_REQUEST_STATUS); i++) {
                        checksum += raw_data[i] = port->read();
                    }
                }

                res_data.checksum = port->read();

                if (res_data.checksum != (256 - checksum)) {

                    gcs().send_text(MAV_SEVERITY_INFO, "Hirth::CRC Fail %x %x %x", raw_data[0], raw_data[1], raw_data[2]);

                    port->discard_input();
                }
                else {
                    decode_data();
                }

                response_complete = true;
                response_counter = 0;
            }
        }

        /* send requests */
        if (response_complete) {
            new_throttle = (uint16_t)SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);

            if (new_throttle != old_throttle) {
                send_target_values(new_throttle);
            }
            else {
                send_request_status();
            }

            old_throttle = new_throttle;
        }

        /* handle missed responses from Hirth ECU */
        if (response_counter++ > 5) {
            port->discard_input();
            response_complete = true;
            response_counter = 0;
        }
    }
}


uint8_t AP_EFI_Serial_Hirth::get_quantity() {
    uint8_t quantity = 0;
    switch (req_data.code)
    {
    case CODE_REQUEST_STATUS_1:
        quantity = QUANTITY_RESPONSE_STATUS_1;
        break;
    case CODE_REQUEST_STATUS_2:
        quantity = QUANTITY_RESPONSE_STATUS_2;
        break;
    case CODE_REQUEST_STATUS_3:
        quantity = QUANTITY_RESPONSE_STATUS_3;
        break;
    case CODE_SET_VALUE:
        quantity = QUANTITY_ACK_SET_VALUES;
        break;
    }

    return quantity;
}


void AP_EFI_Serial_Hirth::send_target_values(uint16_t throttle) {
    int idx = 0;
    checksum = 0;

    checksum += raw_data[idx++] = QUANTITY_SET_VALUE;
    checksum += raw_data[idx++] = CODE_SET_VALUE;
    checksum += raw_data[idx++] = (throttle >> 0x08) & 0xFF;
    checksum += raw_data[idx++] = throttle & 0xFF;

    for (; idx < QUANTITY_SET_VALUE - 5; idx++) {
        /* value-0 has no impact on checksum */
        raw_data[idx] = 0;
    }

    raw_data[QUANTITY_SET_VALUE - 1] = (256 - checksum);

    port->write(raw_data, QUANTITY_SET_VALUE);

    req_data.code = CODE_SET_VALUE;

    response_complete = false;

    /* PCK : Testing */
    gcs().send_text(MAV_SEVERITY_INFO, "Hirth::Throttle req : Q - %d, C - %d, T - %d %d", (int)raw_data[0], (int)raw_data[1], (int)raw_data[2], (int)raw_data[3]);
}


void AP_EFI_Serial_Hirth::send_request_status() {

    switch (req_data.code)
    {
    case CODE_REQUEST_STATUS_1:
        req_data.quantity = QUANTITY_REQUEST_STATUS;
        req_data.code = CODE_REQUEST_STATUS_2;
        req_data.checksum = CHECKSUM_REQUEST_STATUS_2;
        break;
    case CODE_REQUEST_STATUS_2:
        req_data.quantity = QUANTITY_REQUEST_STATUS;
        req_data.code = CODE_REQUEST_STATUS_3;
        req_data.checksum = CHECKSUM_REQUEST_STATUS_3;
        break;
    case CODE_REQUEST_STATUS_3:
        req_data.quantity = QUANTITY_REQUEST_STATUS;
        req_data.code = CODE_REQUEST_STATUS_1;
        req_data.checksum = CHECKSUM_REQUEST_STATUS_1;
        break;
    default:
        req_data.quantity = QUANTITY_REQUEST_STATUS;
        req_data.code = CODE_REQUEST_STATUS_1;
        req_data.checksum = CHECKSUM_REQUEST_STATUS_1;
        break;
    }

    raw_data[0] = req_data.quantity;
    raw_data[1] = req_data.code;
    raw_data[2] = req_data.checksum;

    port->write(raw_data, QUANTITY_REQUEST_STATUS);

    response_complete = false;

    gcs().send_text(MAV_SEVERITY_INFO, "Hirth::Status req : Q - %d, C - %d, Ch - %d", (int)raw_data[0], (int)raw_data[1], (int)raw_data[2]);
}


void AP_EFI_Serial_Hirth::decode_data() {
    
    switch (res_data.code) {
    case CODE_REQUEST_STATUS_1:
        internal_state.engine_state = (Engine_State)(raw_data[8] << 0x08 & raw_data[9]);
        internal_state.engine_speed_rpm = (raw_data[10] << 0x08 & raw_data[11]);
        internal_state.cylinder_status->injection_time_ms = (raw_data[32] << 0x08 & raw_data[33]);
        internal_state.cylinder_status->ignition_timing_deg = (raw_data[34] << 0x08 & raw_data[35]);
        internal_state.cylinder_status->cylinder_head_temperature = (raw_data[74] << 0x08 & raw_data[75]);
        // internal_state.battery_voltage = raw_data[0];
        internal_state.cylinder_status->exhaust_gas_temperature = (raw_data[78] << 0x08 & raw_data[79]);
        internal_state.crankshaft_sensor_status = (Crankshaft_Sensor_Status)raw_data[82];

        gcs().send_text(MAV_SEVERITY_INFO, "Hirth::decode - state - %d, rpm - %lu, time - %f, ", (int)internal_state.engine_state, internal_state.engine_speed_rpm, internal_state.cylinder_status->injection_time_ms);
        gcs().send_text(MAV_SEVERITY_INFO, "deg - %f, cht - %f, egt - %f, stat - %d", internal_state.cylinder_status->ignition_timing_deg, internal_state.cylinder_status->cylinder_head_temperature, internal_state.cylinder_status->exhaust_gas_temperature, (int)internal_state.crankshaft_sensor_status);

        break;
    case CODE_REQUEST_STATUS_2:
        // internal_state.injection_rate = raw_data[0];
        internal_state.fuel_consumption_rate_cm3pm = (raw_data[52] << 0x08 & raw_data[53]); // l/h
        internal_state.throttle_position_percent = (raw_data[62] << 0x08 & raw_data[63]);
        // internal_state.injector_opening_time = raw_data[0];

        gcs().send_text(MAV_SEVERITY_INFO, "Hirth::decode - injRate - , rate - %f, thr - %d, ", internal_state.fuel_consumption_rate_cm3pm, internal_state.throttle_position_percent);

        break;
    case CODE_REQUEST_STATUS_3:
        // internal_state.Temp1 = raw_data[0];
        // internal_state.Temp2 = raw_data[0];
        // internal_state.MixingRatio1 = raw_data[0];
        // internal_state.MixingRatio2 = raw_data[0];
        // internal_state.fuel_pump = raw_data[0];
        // internal_state.exhaust_valve = raw_data[0];
        // internal_state.air_vane = raw_data[0];
        break;
    case CODE_SET_VALUE:
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth::throttle ACK %x %x %x", raw_data[0], raw_data[1], raw_data[2]);
        break;
    }
}

#endif // HAL_EFI_ENABLED
