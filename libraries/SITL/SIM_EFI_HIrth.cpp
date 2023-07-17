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
/*
  simulate Hirth EFI system
*/

#include "SIM_Aircraft.h"
#include <SITL/SITL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>
#include "SIM_EFI_Hirth.h"

using namespace SITL;

void EFI_Hirth::update_receive()
{
    const ssize_t num_bytes_read = read_from_autopilot((char*)&receive_buf[receive_buf_ofs], ARRAY_SIZE(receive_buf) - receive_buf_ofs);
    if (num_bytes_read < 1) {
        return;
    }
    receive_buf_ofs += num_bytes_read;

    if (receive_buf_ofs < 1) {
        return;
    }

    const uint8_t expected_bytes_in_message = receive_buf[0];

    if (expected_bytes_in_message == 0) {
        AP_HAL::panic("zero bytes expected is unexpected");
    }

    if (expected_bytes_in_message > ARRAY_SIZE(receive_buf)) {
        AP_HAL::panic("Unexpectedly large byte count");
    }

    if (receive_buf_ofs < expected_bytes_in_message) {
        return;
    }

    // checksum is sum of all bytes except the received checksum:
    const uint8_t expected_checksum = 256 - sum_of_bytes_in_buffer_mod_256(receive_buf, expected_bytes_in_message-1);
    const uint8_t received_checksum = receive_buf[expected_bytes_in_message-1];
    if (expected_checksum == received_checksum) {
        PacketCode received_packet_code = PacketCode(receive_buf[1]);
        if (received_packet_code == PacketCode::SetValues) {
            // do this synchronously for now
            handle_set_values();
        } else {
            assert_receive_size(3);
            if (requested_data_record.time_ms != 0) {
                AP_HAL::panic("Requesting too fast?");
            }
            requested_data_record.code = received_packet_code;
            requested_data_record.time_ms = AP_HAL::millis();
        }
    } else {
        AP_HAL::panic("checksum failed");
        // simply throw these bytes away.  What the actual device does in the
        // face of weird data is unknown.
    }
    memmove(&receive_buf[0], &receive_buf[expected_bytes_in_message], receive_buf_ofs - expected_bytes_in_message);
    receive_buf_ofs -= expected_bytes_in_message;
}

void EFI_Hirth::assert_receive_size(uint8_t receive_size)
{
    if (receive_buf[0] != receive_size) {
        AP_HAL::panic("Expected %u message size, got %u message size", receive_size, receive_buf[0]);
    }
}

void EFI_Hirth::handle_set_values()
{
    assert_receive_size(23);
    static_assert(sizeof(settings) == 20, "correct number of bytes in settings");
    memcpy((void*)&settings, &receive_buf[2], sizeof(settings));
}

void EFI_Hirth::update_send()
{
    if (requested_data_record.time_ms == 0) {
        // no outstanding request
        return;
    }
    if (AP_HAL::millis() - requested_data_record.time_ms < 20) {
        // 20ms to service a request
        return;
    }
    requested_data_record.time_ms = 0;

    switch (requested_data_record.code) {
    case PacketCode::DataRecord1:
        send_record1();
        break;
    case PacketCode::DataRecord2:
        send_record2();
        break;
    case PacketCode::DataRecord3:
        send_record3();
        break;
    default:
        AP_HAL::panic("Unknown data record (%u) requested", (unsigned)requested_data_record.code);
    }
}

void EFI_Hirth::update()
{
    auto sitl = AP::sitl();
    if (!sitl || sitl->efi_type != SIM::EFI_TYPE_HIRTH) {
        return;
    }

    // update throttle; interim thing to make life a little more interesting
    // gcs().send_text(MAV_SEVERITY_INFO, "using throttle %f", settings.throttle/10.0);
    throttle = 0.9 * throttle + 0.1 * (settings.throttle)/10.0;

    update_receive();
    update_send();
}

void SITL::EFI_Hirth::send_record1()
{
    const auto *sitl = AP::sitl();

    auto &r = packed_record1.record;

    r.rpm = sitl->state.rpm[0];
    gcs().send_text(MAV_SEVERITY_INFO, "rpm in: %u", r.rpm);
    r.air_temperature = AP::baro().get_temperature();

    packed_record1.update_checksum();

    static_assert(sizeof(record1) == 84, "correct number of bytes in record1");
    write_to_autopilot((char*)&packed_record1, sizeof(packed_record1));
}

void SITL::EFI_Hirth::send_record2()
{
    packed_record2.record.throttle = uint16_t(throttle * 10.0);
    packed_record2.update_checksum();

    static_assert(sizeof(record2) == 98, "correct number of bytes in record2");
    write_to_autopilot((char*)&packed_record2, sizeof(packed_record2));
}


void SITL::EFI_Hirth::send_record3()
{
    packed_record3.update_checksum();

    static_assert(sizeof(record3) == 100, "correct number of bytes in record3");
    write_to_autopilot((char*)&packed_record3, sizeof(packed_record3));
}