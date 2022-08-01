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


/**
 * @brief Constructor with port initialization
 * 
 * @param _frontend 
 */
AP_EFI_Serial_Hirth::AP_EFI_Serial_Hirth(AP_EFI &_frontend) : AP_EFI_Backend(_frontend) {
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI, 0);
}


/**
 * @brief checks for response from or makes requests to Hirth ECU periodically
 * 
 */
void AP_EFI_Serial_Hirth::update() {
    bool status = false;
    uint32_t now = AP_HAL::millis();

    if ((port != nullptr) && (now - last_response_ms >= SERIAL_WAIT_DURATION)) {

        // reset request if not response for SERIAL_WAIT_TIMEOUT-ms
        if (now - last_response_ms > SERIAL_WAIT_TIMEOUT) {
            waiting_response = false;
            port->discard_input();
            last_response_ms = now;
        }

        if (waiting_response) {
            num_bytes = port->available();

            // if already requested
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

                // discard further bytes if checksum is not matching
                if (res_data.checksum != (CHECKSUM_MAX - computed_checksum)) {

                    port->discard_input();
                }
                else {
                    internal_state.last_updated_ms = now;
                    decode_data();
                    copy_to_frontend();
                }

                waiting_response = false;
            }
        }
        else {
            // Send requests only if response is completed

            new_throttle = (uint16_t)SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);

            if (new_throttle != old_throttle) {
                // if new throttle value, send throttle request
                status = send_target_values(new_throttle);
                old_throttle = new_throttle;
                
            }
            else {
                // request Status request, if no throttle commands
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


/**
 * @brief updates the current quantity that will be expected
 * 
 */
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


/**
 * @brief sends the new throttle command to Hirth ECU
 * 
 * @param thr - new throttle value given by SRV_Channel::k_throttle
 * @return true - if success
 * @return false - currently not implemented
 */
bool AP_EFI_Serial_Hirth::send_target_values(uint16_t thr) {
    bool status = false;
    int idx = 0;
    uint8_t computed_checksum = 0;

    // Get throttle value
    uint16_t throttle = thr * THROTTLE_POSITION_FACTOR;

    // set Quantity + Code + "20 bytes of records to set" + Checksum
    computed_checksum += raw_data[idx++] = req_data.quantity = QUANTITY_SET_VALUE;
    computed_checksum += raw_data[idx++] = req_data.code = CODE_SET_VALUE;
    computed_checksum += raw_data[idx++] = throttle & 0xFF;
    computed_checksum += raw_data[idx++] = (throttle >> 0x08) & 0xFF;

    for (; idx < QUANTITY_SET_VALUE - 2; idx++) {
        // 0 has no impact on checksum
        raw_data[idx] = 0;
    }

    raw_data[QUANTITY_SET_VALUE - 1] = (256 - computed_checksum);

    port->write(raw_data, QUANTITY_SET_VALUE);

    status = true;

    return status;
}


/**
 * @brief cyclically sends different Status requests to Hirth ECU
 * 
 * @return true - when successful
 * @return false  - not implemented
 */
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

    return status;
}


/**
 * @brief parses the response from Hirth ECU and updates the internal state instance
 * 
 */
void AP_EFI_Serial_Hirth::decode_data() {
    int engine_status = 0;
    static int prev_engine_status = 9;
    static uint8_t prev_sensor_status = 255;

    int excess_temp1 = 0;
    int excess_temp2 = 0;
    int excess_temp3 = 0;
    int excess_temp4 = 0;
    // int excess_temp5 = 0;

    int excess_temp_error = 0;

    // static int prev_excess_temp1 = 0;
    // static int prev_excess_temp2 = 0;
    // static int prev_excess_temp3 = 0;
    // static int prev_excess_temp4 = 0;
    // static int prev_excess_temp5 = 0;

    // static int prev_excess_temp_error = 0;
    static int count_temp_sensor = 0;

    switch (res_data.code)
    {
    case CODE_REQUEST_STATUS_1:
        engine_status = (raw_data[8] | raw_data[9] << 0x08);
        internal_state.engine_state = (engine_status >= ENGINE_RUNNING) ? Engine_State::RUNNING : Engine_State::STARTING;
        internal_state.engine_speed_rpm = (raw_data[10] | raw_data[11] << 0x08);
        internal_state.cylinder_status->injection_time_ms = ((raw_data[32] | raw_data[33] << 0x08)) * INJECTION_TIME_RESOLUTION;
        internal_state.cylinder_status->ignition_timing_deg = (raw_data[34] | raw_data[35] << 0x08);
        internal_state.cylinder_status->cylinder_head_temperature = (raw_data[74] | raw_data[75] << 0x08) + KELVIN_CONVERSION_CONSTANT;
        // internal_state.battery_voltage = (raw_data[76] | raw_data[77] << 0x08); // TBD - internal state addition
        internal_state.cylinder_status->exhaust_gas_temperature = (raw_data[78] | raw_data[79] << 0x08) + KELVIN_CONVERSION_CONSTANT;
        internal_state.crankshaft_sensor_status = ((raw_data[82] & CRANK_SHAFT_SENSOR_OK) == CRANK_SHAFT_SENSOR_OK) ? Crankshaft_Sensor_Status::OK : Crankshaft_Sensor_Status::ERROR;
        
        // gcs().send_text(MAV_SEVERITY_INFO, "************************************");
        gcs().send_text(MAV_SEVERITY_INFO, "HE: In Flash = %d", (raw_data[1] | raw_data[2] << 0x08));

        if (prev_engine_status != engine_status) {
            if (engine_status == 0)
                gcs().send_text(MAV_SEVERITY_INFO, "HE: 0 - Eng tune-up");
            else if (engine_status == 1)
                gcs().send_text(MAV_SEVERITY_INFO, "HE: 1 - Igni On,No Crank");
            else if (engine_status == 2)
                gcs().send_text(MAV_SEVERITY_INFO, "HE: 2 - Igni On,Low RPM");
            else if (engine_status == 3)
                gcs().send_text(MAV_SEVERITY_INFO, "HE: 3 - Warming-up phase");
            else
                gcs().send_text(MAV_SEVERITY_INFO, "HE: 4 - Engine running");

            prev_engine_status = engine_status;
        }
        if (prev_sensor_status != raw_data[82]) {
            if (!(raw_data[82] & 0x01))
                gcs().send_text(MAV_SEVERITY_INFO, "HE: Eng Temp ERROR");
            if (!(raw_data[82] & 0x02))
                gcs().send_text(MAV_SEVERITY_INFO, "HE: Air Temp ERROR");
            if (!(raw_data[82] & 0x04))
                gcs().send_text(MAV_SEVERITY_INFO, "HE: Air Pre ERROR");
            if (!(raw_data[82] & 0x08))
                gcs().send_text(MAV_SEVERITY_INFO, "HE: Throttle ERROR");
            prev_engine_status = raw_data[82];

        }
        gcs().send_text(MAV_SEVERITY_INFO, "------------------------------------");
        gcs().send_text(MAV_SEVERITY_INFO, "HE: Air Temp = %f", internal_state.cylinder_status->exhaust_gas_temperature - 273.15);
        gcs().send_text(MAV_SEVERITY_INFO, "HE: Eng Temp = %f", internal_state.cylinder_status->cylinder_head_temperature - 273.15);

        break;

    case CODE_REQUEST_STATUS_2:
        // internal_state.injection_rate = ((raw_data[16] | raw_data[17] << 0x08)) * INJECTION_TIME_RESOLUTION; // TBD - internal state addition
        internal_state.fuel_consumption_rate_cm3pm = (raw_data[52] | raw_data[53] << 0x08) / FUEL_CONSUMPTION_RESOLUTION;
        internal_state.throttle_position_percent = (raw_data[62] | raw_data[63] << 0x08) / THROTTLE_POSITION_RESOLUTION;
        // internal_state.injector_opening_time = (raw_data[70] | raw_data[71] << 0x08) * INJECTION_TIME_RESOLUTION // TBD - internal state addition

        break;

    case CODE_REQUEST_STATUS_3: // TBD - internal state addition
        // internal_state.Temp1 = ((raw_data[2] | raw_data[3] << 0x08) * 5 / 1024); // TBD - Average of Voltage Excess Temp1 to Tmep5
        // internal_state.Temp2 = (raw_data[16] | raw_data[17] << 0x08); // TBD - Average of Excess Temp1 to Tmep5
        // internal_state.MixingRatio1 = 1 / (raw_data[48] | raw_data[49] << 0x08);
        // internal_state.MixingRatio2 = 1 / (raw_data[50] | raw_data[51] << 0x08);
        // internal_state.fuel_pump = (raw_data[54] | raw_data[55] << 0x08);
        // internal_state.exhaust_valve = (raw_data[56] | raw_data[57] << 0x08);
        // internal_state.air_vane = (raw_data[58] | raw_data[59] << 0x08);
        excess_temp1 = (raw_data[16] | raw_data[17] << 0x08);
        excess_temp2 = (raw_data[18] | raw_data[19] << 0x08);
        excess_temp3 = (raw_data[20] | raw_data[21] << 0x08);
        excess_temp4 = (raw_data[22] | raw_data[23] << 0x08);
        // excess_temp5 = (raw_data[24] | raw_data[25] << 0x08);

        excess_temp_error = (raw_data[46] | raw_data[47] << 0x08);

        // int enrichment1 = (raw_data[74] | raw_data[75] << 0x08);
        // int enrichment2 = (raw_data[76] | raw_data[77] << 0x08);
        // int enrichment3 = (raw_data[78] | raw_data[79] << 0x08);
        // int enrichment4 = (raw_data[80] | raw_data[81] << 0x08);

        if (count_temp_sensor++ > 10)
        {
            
            gcs().send_text(MAV_SEVERITY_INFO, "HE: CHT1 = %d", excess_temp1);
            gcs().send_text(MAV_SEVERITY_INFO, "HE: CHT2 = %d", excess_temp2);
            gcs().send_text(MAV_SEVERITY_INFO, "HE: EGT1 = %d", excess_temp3);
            gcs().send_text(MAV_SEVERITY_INFO, "HE: EGT2 = %d", excess_temp4);
            gcs().send_text(MAV_SEVERITY_INFO, "------------------------------------");
            gcs().send_text(MAV_SEVERITY_INFO, "HE: C1 L= %d H= %d A= %d", (excess_temp_error & 0x01),(excess_temp_error & 0x02),(excess_temp_error & 0x04));
            gcs().send_text(MAV_SEVERITY_INFO, "HE: C2 L= %d H= %d A= %d", (excess_temp_error & 0x08),(excess_temp_error & 0x10),(excess_temp_error & 0x20));
            gcs().send_text(MAV_SEVERITY_INFO, "HE: E1 L= %d H= %d A= %d", (excess_temp_error & 0x40),(excess_temp_error & 0x80),(excess_temp_error & 0x100));
            gcs().send_text(MAV_SEVERITY_INFO, "HE: E2 L= %d H= %d A= %d", (excess_temp_error & 0x200),(excess_temp_error & 0x400),(excess_temp_error & 0x800));
        }

        gcs().send_text(MAV_SEVERITY_INFO, "########################################");

        break;
        
    // case CODE_SET_VALUE:
    //     // Do nothing for now
    //     break;
    }
}

#endif // HAL_EFI_ENABLED
