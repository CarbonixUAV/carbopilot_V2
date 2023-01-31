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



#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

#ifndef HAL_EFI_ENABLED
#define HAL_EFI_ENABLED !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024
#endif

#if HAL_EFI_ENABLED
#include "AP_EFI_Backend.h"
#include "AP_EFI_State.h"


/*
 * This library aims to read data from Electronic Fuel Injection 
 * or Engine Control units. It is focused around the generic
 * internal combustion engine state message provided by the 
 * UAVCAN protocol due to its comprehensiveness, but is extensible
 * to use other forms of data transfer besides UAVCAN. 
 * 
 *
 *
 * Authors: Sriram Sami and David Ingraham
 * With direction from Andrew Tridgell, Robert Lefebvre, Francisco Ferreira and
 * Pavel Kirienko.
 * Thanks to Yonah, SpektreWorks Inc, and HFE International.
 */

class AP_EFI {
public:
    friend class AP_EFI_Backend;

    // For parameter initialization
    AP_EFI();

    // Initializes backend
    void init(void);

    // Requests backend to update the frontend. Should be called at 10Hz.
    void update();
    
    // Returns the Engine Load Perct
    uint8_t get_load_prct() const { return state.engine_load_percent; }

    // Returns the RPM
    uint32_t get_rpm() const { return state.engine_speed_rpm; }

    // Returns the atmospheric_pressure_kpa
    float get_atm_press() const { return state.atmospheric_pressure_kpa; }
    
    // Returns the intake_manifold_pressure_kpa
    float get_in_mani_press() const { return state.intake_manifold_pressure_kpa; }

    // Returns the fuel_consumption_rate_cm3pm
    float get_fuel_consumption_rate() const { return state.fuel_consumption_rate_cm3pm; }

    // Returns the Engine state
    uint8_t get_engine_state() const { return uint8_t(state.engine_state); }


    
    // Returns the k_throttle
    float get_k_throttle() const { return state.k_throttle; }
    
    // Returns the thr_pos
    float get_thr_pos() const { return state.thr_pos; }
    
 

    // Returns the crc_fail_cnt
    uint32_t get_crc_fail_cnt() const { return state.crc_fail_cnt; }

    //get_temperature
    bool get_temperature(float &air_temp, float &eng_temp, float &cht1_temp, float &cht2_temp, float &egt1_temp, float &egt2_temp) {
        air_temp= state.air_temp;
        eng_temp= state.eng_temp;
        cht1_temp= state.cht1_temp;
        cht2_temp= state.cht2_temp;
        egt1_temp= state.egt1_temp;
        egt2_temp= state.egt2_temp;
        return true;
    }

    // engine_temperature_sensor_status
    bool get_sensor_status(uint8_t &engine_temperature_sensor_status, uint8_t &air_temperature_sensor_status, uint8_t &air_pressure_sensor_status,uint8_t &throttle_sensor_status, uint8_t &CHT_1_error_excess_temperature_status, uint8_t &CHT_2_error_excess_temperature_status, uint8_t &EGT_1_error_excess_temperature_status, uint8_t &EGT_2_error_excess_temperature_status) {

        engine_temperature_sensor_status = state.engine_temperature_sensor_status;
        air_temperature_sensor_status = state.air_temperature_sensor_status;
        air_pressure_sensor_status = state.air_pressure_sensor_status;
        throttle_sensor_status = state.throttle_sensor_status;
        CHT_1_error_excess_temperature_status = state.CHT_1_error_excess_temperature_status;
        CHT_2_error_excess_temperature_status = state.CHT_2_error_excess_temperature_status;
        EGT_1_error_excess_temperature_status = state.EGT_1_error_excess_temperature_status;
        EGT_2_error_excess_temperature_status = state.EGT_2_error_excess_temperature_status;
        return true;
    }

    // returns enabled state of EFI
    bool enabled() const { return type != Type::NONE; }

    bool is_healthy() const;

    // Parameter info
    static const struct AP_Param::GroupInfo var_info[];

    // Backend driver types
    enum class Type : uint8_t {
        NONE       = 0,
        MegaSquirt = 1,
        NWPMU     = 2,
        Lutan     = 3,
        // LOWEHEISER = 4,
        Hirth = 6,
    };

    static AP_EFI *get_singleton(void) {
        return singleton;
    }

    // send EFI_STATUS
    void send_mavlink_status(mavlink_channel_t chan);

protected:

    // Back end Parameters
    AP_Float coef1;
    AP_Float coef2;

    EFI_State state;

private:
    // Front End Parameters
    AP_Enum<Type> type;

    // Tracking backends
    AP_EFI_Backend *backend;
    static AP_EFI *singleton;

    // write to log
    void log_status();
};

namespace AP {
    AP_EFI *EFI();
};

#endif // HAL_EFI_ENABLED