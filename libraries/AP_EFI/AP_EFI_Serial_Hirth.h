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


#pragma once


#include "AP_EFI.h"
#include "AP_EFI_Serial_Hirth.h"


#define HIRTH_MAX_PKT_SIZE 100
#define HIRTH_MAX_RAW_PKT_SIZE 103

#define BYTE_RANGE_MAX 256

#define SERIAL_WAIT_DURATION 100
#define SERIAL_WAIT_TIMEOUT 100

#define ENGINE_RUNNING 4
#define THROTTLE_POSITION_FACTOR 10
#define CRANK_SHAFT_SENSOR_OK 0x0F
#define INJECTION_TIME_RESOLUTION 0.8
#define FUEL_CONSUMPTION_RESOLUTION 10.0
#define THROTTLE_POSITION_RESOLUTION 0.1
#define KELVIN_CONVERSION_CONSTANT 273.5
#define VOLTAGE_RESOLUTION 0.0049       /* 5/1024 */
#define ADC_CALIBRATION 5.0/1024.0
#define MAP_HPA_PER_VOLT_FACTOR 248.2673
#define HPA_TO_KPA 0.1
#define TPS_SCALE 0.70

const uint8_t QUANTITY_REQUEST_STATUS    = 0x03;
const uint8_t QUANTITY_SET_VALUE         = 0x17;

const uint8_t CODE_REQUEST_STATUS_1      = 0x04;
const uint8_t CODE_REQUEST_STATUS_2      = 0x0B;
const uint8_t CODE_REQUEST_STATUS_3      = 0x0D;
const uint8_t CODE_SET_VALUE             = 0xC9;

const uint8_t CHECKSUM_REQUEST_STATUS_1  = 0xF9;
const uint8_t CHECKSUM_REQUEST_STATUS_2  = 0xF2;
const uint8_t CHECKSUM_REQUEST_STATUS_3  = 0xF0;
const uint8_t CHECKSUM_SET_VALUE         = 0x34;

const uint8_t QUANTITY_RESPONSE_STATUS_1 = 0x57;
const uint8_t QUANTITY_RESPONSE_STATUS_2 = 0x65;
const uint8_t QUANTITY_RESPONSE_STATUS_3 = 0x67;
const uint8_t QUANTITY_ACK_SET_VALUES    = 0x03;



/*!
 * Data mapping between rawBytes and Telemetry packets
 */
typedef struct
{
    uint8_t quantity;
    uint8_t code;
    uint8_t checksum;
} data_set_t;


/*!
 * class definition for Hirth 4103 ECU
 */
class AP_EFI_Serial_Hirth: public AP_EFI_Backend
{
public:
    AP_EFI_Serial_Hirth(AP_EFI &_frontend);

    void update() override;

    void decode_data();

    bool send_request_status();

    bool send_target_values(uint16_t);

    void get_quantity();

    float get_avg_fuel_consumed(float fuel_consumed);

private:
    // serial port instance
    AP_HAL::UARTDriver *port;

    // periodic refresh 
    uint32_t last_response_ms;
    uint32_t last_loop_ms;
    uint32_t last_uptime;
    uint32_t last_req_send_throttle;

    // raw bytes - max size
    uint8_t raw_data[BYTE_RANGE_MAX];

    // request and response data 
    data_set_t req_data;
    data_set_t res_data;

    // number of bytes 
    uint32_t num_bytes;

    // TRUE - Request is sent; waiting for response
    // FALSE - Response is already received
    bool waiting_response;

    // Expected bytes from response
    uint8_t expected_bytes;

    // Throttle values
    uint16_t new_throttle;
    uint16_t old_throttle;

    float fuel_consumption_rate_average;
    float fuel_consumption_rate_raw;
    float fuel_consumption_rate;
    float total_fuel_consumed;
    float instance_fuel_reading[BYTE_RANGE_MAX];
    uint8_t fuel_avg_count;
    uint8_t fuel_avg_config;


    uint8_t data_send;

    float throttle_scaling_factor;
};