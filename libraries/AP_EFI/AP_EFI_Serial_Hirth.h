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
#define HIRTH_MAX_RAW_PKT_SIZE 103 //


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
    /* Constructor with initialization */
    AP_EFI_Serial_Hirth(AP_EFI &_frontend);

    /* To update the state structure */
    void update() override;

    /**/
    void decode_data();

    /**/
    void send_request_status();

    /**/
    void send_target_values(uint16_t);

    /**/
    uint8_t get_quantity();

private:
    //
    AP_HAL::UARTDriver *port;

    // periodic refresh 
    uint32_t last_response_ms;

    // Raw bytes - max size
    uint8_t raw_data[HIRTH_MAX_RAW_PKT_SIZE];

    //
    uint8_t data_size = HIRTH_MAX_PKT_SIZE;

    //
    uint8_t num_bytes;

    //
    uint8_t checksum;

    //
    uint16_t new_throttle;
    uint16_t old_throttle;

    bool response_complete;
    uint8_t response_counter;

    //
    data_set_t req_data;
    data_set_t res_data;
};
