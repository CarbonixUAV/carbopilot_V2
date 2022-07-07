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
}


void AP_EFI_Serial_Hirth::update() {
    bool status = false;
    uint32_t now = AP_HAL::millis();

    if (port != nullptr) {

        if (now - last_response_ms > SERIAL_WAIT_TIMEOUT) {
            waiting_response = false;
            port->discard_input();
            last_response_ms = now;
            gcs().send_text(MAV_SEVERITY_INFO, "Hirth::timeout %ld", (now - last_response_ms));
        }

        if (waiting_response) {
            num_bytes = port->available();

            gcs().send_text(MAV_SEVERITY_INFO, "Hirth::bytes %d, %d", expected_bytes, (int)num_bytes);

            if (num_bytes >= expected_bytes) {
                
                // read data from buffer
                uint8_t computed_checksum = 0;
                computed_checksum += res_data.quantity = port->read();
                computed_checksum += res_data.code = port->read();

                if (res_data.code == req_data.code) {
                    for (int i = 0; i < (res_data.quantity - QUANTITY_REQUEST_STATUS); i++) {
                        computed_checksum += raw_data[i] = port->read();
                    }
                }

                res_data.checksum = port->read();

                gcs().send_text(MAV_SEVERITY_INFO, "#### Hirth:: %x %x %x %x", res_data.quantity, res_data.code, res_data.checksum, computed_checksum);

                if (res_data.checksum != (256 - computed_checksum)) {

                    gcs().send_text(MAV_SEVERITY_INFO, "Hirth::CRC Fail %x %x %x", raw_data[0], raw_data[1], raw_data[2]);

                    port->discard_input();
                }
                else {
                    decode_data();
                }

                waiting_response = false;
            }
        }
        else {
            new_throttle = (uint16_t)SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);

            if (new_throttle != old_throttle) {
                status = send_target_values(new_throttle);
                old_throttle = new_throttle;
            }
            else {
                status = send_request_status();
            }

            get_quantity();            

            if (status == true) {
                waiting_response = true;
                last_response_ms = now;
            }
        }
    }
}


void AP_EFI_Serial_Hirth::get_quantity() {
    switch (req_data.code)
    {
    case CODE_REQUEST_STATUS_1:
        expected_bytes = QUANTITY_RESPONSE_STATUS_1;
        break;
    case CODE_REQUEST_STATUS_2:
        expected_bytes = QUANTITY_RESPONSE_STATUS_2;
        break;
    case CODE_REQUEST_STATUS_3:
        expected_bytes = QUANTITY_RESPONSE_STATUS_3;
        break;
    case CODE_SET_VALUE:
        expected_bytes = QUANTITY_ACK_SET_VALUES;
        break;
    }
}


bool AP_EFI_Serial_Hirth::send_target_values(uint16_t thr) {
    bool status = false;
    int idx = 0;
    uint8_t computed_checksum = 0;

    // Get throttle value
    uint16_t throttle = thr * 10;

    // set Quantity + Code + "20 bytes of records to set" + Checksum
    computed_checksum += raw_data[idx++] = req_data.quantity = QUANTITY_SET_VALUE;
    computed_checksum += raw_data[idx++] = req_data.code = CODE_SET_VALUE;
    computed_checksum += raw_data[idx++] = throttle & 0xFF;
    computed_checksum += raw_data[idx++] = (throttle >> 0x08) & 0xFF;

    for (; idx < QUANTITY_SET_VALUE - 5; idx++) {
        // 0 has no impact on checksum
        raw_data[idx] = 0;
    }

    raw_data[QUANTITY_SET_VALUE - 1] = (256 - computed_checksum);

    port->write(raw_data, QUANTITY_SET_VALUE);

    status = true;

    /* PCK : Testing */
    gcs().send_text(MAV_SEVERITY_INFO, "Hirth::Throttle req : Q-%d, C-%d, Ch-%d, T-%d %d", (int)raw_data[0], (int)raw_data[1], (int)raw_data[QUANTITY_SET_VALUE - 1], (int)raw_data[2], (int)raw_data[3]);

    return status;
}


bool AP_EFI_Serial_Hirth::send_request_status() {

    bool status = false;

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

    status = true;

    gcs().send_text(MAV_SEVERITY_INFO, "Hirth::Status req : Q - %d, C - %d, Ch - %d", (int)raw_data[0], (int)raw_data[1], (int)raw_data[2]);

    return status;
}


void AP_EFI_Serial_Hirth::decode_data() {
    switch (res_data.code)
    {
    case CODE_REQUEST_STATUS_1:
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - Status:%d %d", raw_data[8], raw_data[9]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - RPM   :%d%d", raw_data[11], raw_data[10]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - NIImpl:%d%d", raw_data[24], raw_data[25]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - SError:%d%d", raw_data[30], raw_data[31]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - InjTim:%d%d", raw_data[32], raw_data[33]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - IgnAng:%d%d", raw_data[34], raw_data[35]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - VolThr:%d %d", raw_data[38], raw_data[39]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - VEngT :%d%d", raw_data[44], raw_data[45]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - VAirT :%d%d", raw_data[46], raw_data[47]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - VIAirT:%d%d", raw_data[50], raw_data[51]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - ThrtlP:%d %d", raw_data[73], raw_data[72]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - EngTmp:%d%d", raw_data[74], raw_data[75]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - BatVol:%d%d", raw_data[76], raw_data[77]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - AitTmp:%d%d", raw_data[78], raw_data[79]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S1 - Sensor:%d %d", raw_data[82], raw_data[83]);
        break;
    case CODE_REQUEST_STATUS_2:
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S2 - BInjR :%d%d", raw_data[12], raw_data[13]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S2 - TotTim:%d%d%d%d", raw_data[44], raw_data[45], raw_data[46], raw_data[47]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S2 - TotRot:%d%d%d%d", raw_data[48], raw_data[49], raw_data[50], raw_data[51]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S2 - FuelC :%d%d", raw_data[52], raw_data[53]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S2 - VIThrT:%d %d", raw_data[56], raw_data[57]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S2 - PosThT:%d %d", raw_data[60], raw_data[61]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S2 - PosThP:%d %d", raw_data[62], raw_data[63]);
        break;
    case CODE_REQUEST_STATUS_3:
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S3 - VETem :%d%d %d%d %d%d %d%d %d%d", raw_data[0], raw_data[1], raw_data[2], raw_data[3], raw_data[4], raw_data[5], raw_data[6], raw_data[7], raw_data[8], raw_data[9]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S3 -  ETem :%d%d %d%d %d%d %d%d %d%d", raw_data[16], raw_data[17], raw_data[18], raw_data[19], raw_data[20], raw_data[21], raw_data[22], raw_data[23], raw_data[24], raw_data[25]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S3 - EETemC:%d%d %d%d %d%d %d%d     ", raw_data[33], raw_data[32], raw_data[35], raw_data[34], raw_data[37], raw_data[36], raw_data[39], raw_data[38]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S3 - ErrETB:%d%d", raw_data[47], raw_data[46]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S3 - OVe-Th:%d %d", raw_data[61], raw_data[60]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S3 - STime :%d %d %d %d", raw_data[67], raw_data[66], raw_data[65], raw_data[64]);
        gcs().send_text(MAV_SEVERITY_INFO, "Hirth:S3 - TarRPM:%d%d", raw_data[71], raw_data[70]);
        break;
    case CODE_SET_VALUE:
        gcs().send_text(MAV_SEVERITY_INFO, "##### Hirth::THROTTLE ACK %x %x %x  - %ld", res_data.quantity, res_data.code, res_data.checksum, last_response_ms - AP_HAL::millis());
        break;
    }

    /*
    switch (res_data.code) {
    case CODE_REQUEST_STATUS_1:
        internal_state.engine_state = (Engine_State)(raw_data[8] | raw_data[9] << 0x08);
        internal_state.engine_speed_rpm = (raw_data[10] | raw_data[11] << 0x08);
        internal_state.cylinder_status->injection_time_ms = (raw_data[32] | raw_data[33] << 0x08);
        internal_state.cylinder_status->ignition_timing_deg = (raw_data[34] | raw_data[35] << 0x08);
        internal_state.cylinder_status->cylinder_head_temperature = (raw_data[74] | raw_data[75] << 0x08);
        // internal_state.battery_voltage = raw_data[0];
        internal_state.cylinder_status->exhaust_gas_temperature = (raw_data[78] | raw_data[79] << 0x08);
        internal_state.crankshaft_sensor_status = (Crankshaft_Sensor_Status)raw_data[82];

        gcs().send_text(MAV_SEVERITY_INFO, "Hirth::decode - state - %d, rpm - %lu, time - %f, ", (int)internal_state.engine_state, internal_state.engine_speed_rpm, internal_state.cylinder_status->injection_time_ms);
        gcs().send_text(MAV_SEVERITY_INFO, "deg - %f, cht - %f, egt - %f, stat - %d", internal_state.cylinder_status->ignition_timing_deg, internal_state.cylinder_status->cylinder_head_temperature, internal_state.cylinder_status->exhaust_gas_temperature, (int)internal_state.crankshaft_sensor_status);

        break;
    case CODE_REQUEST_STATUS_2:
        // internal_state.injection_rate = raw_data[0];
        internal_state.fuel_consumption_rate_cm3pm = (raw_data[52] | raw_data[53] << 0x08); // l/h
        internal_state.throttle_position_percent = (raw_data[62] | raw_data[63] << 0x08);
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
    */
}

#endif // HAL_EFI_ENABLED
