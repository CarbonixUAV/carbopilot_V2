/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Charlie Johnson
 */

#pragma once

#include <Filter/LowPassFilter.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#include <Filter/Filter.h>                     // Filter library
#include <Filter/AverageFilter.h>      // AverageFilter class (inherits from Filter class)

// ADC Calibration for CX31021020 Rev D Voltage Divider for ADC #3
#define CPN_ADC_5V_SLP 797.76
#define CPN_ADC_5V_OFF 1.11

// Quadratic fit for LS800 sensor readings to convert to actual fuel in tank
#define LS800_FIT_FIRST_COEFF 1.0015
#define LS800_FIT_SECOND_COEFF 0.2419
#define LS800_FIT_THIRD_COEFF -0.0213
#define LS800_FIT_OFFSET 0.1766

#define FILTER_MAX 60 //battery monitor called at 1Hz

class AP_BattMonitor_FuelLevel_Analog : public AP_BattMonitor_Backend
{
public:

    // Constructor
    AP_BattMonitor_FuelLevel_Analog(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    static const struct AP_Param::GroupInfo var_info[];

    // Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    // returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return true; }

    // returns true if battery monitor provides current info
    bool has_current() const override { return true; }

    void init(void) override {}

private:

    AP_Float _fuel_level_empty_voltage;
    AP_Float _fuel_level_voltage_mult;
    AP_Float _fuel_level_filter_frequency;    
    AP_Int8  _pin;

    AP_Float _ls800_max_fuel_level_litres;

    AP_HAL::AnalogSource *_analog_source;

    LowPassFilterFloat _voltage_filter;
    AverageFilter<float,float,FILTER_MAX> _ls800_filter;
};