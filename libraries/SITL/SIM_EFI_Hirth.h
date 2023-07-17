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

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduPlane -A --uartF=sim:hirth --speedup=1
param set SERIAL5_PROTOCOL 24
param set SIM_EFI_TYPE 6
param set EFI_TYPE 6
reboot
status EFI_STATUS

./Tools/autotest/autotest.py --gdb --debug build.Plane test.Plane.Hirth

*/

#pragma once

#include <SITL/SITL.h>
#include <AP_HAL/utility/Socket.h>
#include "SIM_SerialDevice.h"

namespace SITL {

class EFI_Hirth : public SerialDevice {
public:

    using SerialDevice::SerialDevice;

    void update();

private:

    void update_receive();
    void update_send();

    void assert_receive_size(uint8_t receive_size);

    void handle_set_values();

    // maps from an on-wire number to a record number:
    enum class PacketCode : uint8_t {
        DataRecord1 = 4,
        DataRecord2 = 11,
        DataRecord3 = 13,
        SetValues = 201,
    };

    template <typename T>
    class PACKED PackedRecord {
    public:
        PackedRecord(PacketCode _code, T _record) :
            code(uint8_t(_code)),
            record(_record)
        { }
            const uint8_t length { sizeof(T) + 3 };  // 1 each of length, code and checksum
        const uint8_t code;
        T record;
        uint8_t checksum;

        void update() {
            record.update();
            update_checksum();
        }

        void update_checksum() {
            checksum = 256 - sum_of_bytes_in_buffer_mod_256((uint8_t*)this, length-1);
        }
    };

    void send_record1();
    void send_record2();
    void send_record3();

    class PACKED Record1 {
    public:
        uint8_t unknown1[10];
        uint16_t rpm;
        uint8_t unknown2[66];
        uint16_t air_temperature;
        uint8_t unknown3[4];

        void update();
    };

    class PACKED Record2 {
    public:
        uint8_t unknown1[62];
        uint16_t throttle;  // percent * 10
        uint8_t unknown2[34];
    };

    class PACKED Record3 {
    public:
        uint8_t unknown1[100];
    };

    class PACKED SetValues {
    public:
        uint16_t throttle;  // percent * 10
        uint8_t unknown1[18];
    };

    // these records are just used for initial values of the fields;
    // they aren't used past that.
    Record1 record1;
    Record2 record2;
    Record3 record3;


    SetValues settings;

    PackedRecord<Record1> packed_record1{PacketCode::DataRecord1, record1};
    PackedRecord<Record2> packed_record2{PacketCode::DataRecord2, record2};
    PackedRecord<Record3> packed_record3{PacketCode::DataRecord3, record3};

    struct {
        PacketCode code;  // code which was requested by driver
        uint32_t time_ms;  // time that code was requested by driver
    } requested_data_record;

    uint8_t receive_buf[32];
    uint8_t receive_buf_ofs;

    float throttle;
};

}