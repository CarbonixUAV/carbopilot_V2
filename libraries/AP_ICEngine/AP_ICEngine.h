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
  control of internal combustion engines (starter, ignition and choke)
 */
#pragma once

#include "AP_ICEngine_config.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/I2CDevice.h>

#if AP_ICENGINE_ENABLED

#include <AP_Param/AP_Param.h>
#include <Filter/LowPassFilter.h>

class AP_ICEngine {
public:
    // constructor
    AP_ICEngine(const class AP_RPM &_rpm);

    static const struct AP_Param::GroupInfo var_info[];

    // update engine state. Should be called at 10Hz or more
    void update(void);

    // check for throttle override
    bool throttle_override(float &percent, const float base_throttle);

    enum ICE_State {
        ICE_OFF=0,
        ICE_START_HEIGHT_DELAY=1,
        ICE_START_DELAY=2,
        ICE_STARTING=3,
        ICE_RUNNING=4
    };

    // get current engine control state
    ICE_State get_state(void) const { return state; }

    // handle DO_ENGINE_CONTROL messages via MAVLink or mission
    bool engine_control(float start_control, float cold_start, float height_delay);

    // update min throttle for idle governor
    void update_idle_governor(int8_t &min_throttle);

    static AP_ICEngine *get_singleton() { return _singleton; }

    bool allow_throttle_disarmed() const;

private:
    static AP_ICEngine *_singleton;

    /*
     * TCA9554 output register mapping for PMB Rev E
     * P0 = PMU_EN - PMU output ON/OFF  (CN6 pin 2)
     * P1 = ECU_EN - Unused (previously Engine Kill Switch)
     * P2 = I2C_P2 - Unused
     * P3 = LED (active low)
     * P4 = PMU_START - Crank Direction (CN6 pin 5)
     * P5 = PMU_ARM  - Crank Signal (CN6 pin 6)
     * P6 = PMU_STAT_IN - Unused
     * P7 = PMU_STAT - Unused
     */

    enum TCA9554_state_t {
        IGN_OFF_STR_OFF = 0x30,	 		//output register - 0011 0000
        IGN_ON_STR_OFF = 0x30,		 	//output register - 0011 0000
		IGN_ON_STR_ON_DIR_ON = 0x11, 	//output register - 0001 0001
    } TCA9554_state = IGN_OFF_STR_OFF;

    enum i2c_init_t {
        I2C_UNINITIALIZED = 0,
        I2C_FAILED = 1,
        I2C_SUCCESS = 2
    } i2c_state = I2C_UNINITIALIZED;

    bool TCA9554_init();

    void TCA9554_set(TCA9554_state_t value);

    void control_ign_str(TCA9554_state_t value);
    void ignition_relay_set(bool on);

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_TCA9554;

    const class AP_RPM &rpm;

    enum ICE_State state;

    // filter for RPM value
    LowPassFilterFloat _rpm_filter;
    float filtered_rpm_value;

    // enable library
    AP_Int8 enable;

    // channel for pilot to command engine start, 0 for none
    AP_Int8 start_chan;

    // min pwm on start channel for engine stop
    AP_Int16 start_chan_min_pwm;
    
    // which RPM instance to use
    AP_Int8 rpm_instance;
    
    // time to run starter for (seconds)
    AP_Float starter_time;

    // delay between start attempts (seconds)
    AP_Float starter_delay;
    
    // pwm values 
    AP_Int16 pwm_ignition_on;
    AP_Int16 pwm_ignition_off;
    AP_Int16 pwm_starter_on;
    AP_Int16 pwm_starter_off;
    
    // RPM above which engine is considered to be running
    AP_Int32 rpm_threshold;

    // time when we started the starter
    uint32_t starter_start_time_ms;

    // time when we last ran the starter
    uint32_t starter_last_run_ms;

    // throttle percentage for engine start
    AP_Int8 start_percent;

    // throttle percentage for engine idle
    AP_Int8 idle_percent;

    // Idle Controller RPM setpoint
    AP_Int16 idle_rpm;

    // Idle Controller RPM deadband
    AP_Int16 idle_db;

    // Idle Controller Slew Rate
    AP_Float idle_slew;
    
    // relay number for ignition
    AP_Int8 ignition_relay;
    
    // height when we enter ICE_START_HEIGHT_DELAY
    float initial_height;

    // height change required to start engine
    float height_required;

    // we are waiting for valid height data
    bool height_pending:1;

    // idle governor
    float idle_governor_integrator;

    enum class Options : uint16_t {
        DISABLE_IGNITION_RC_FAILSAFE=(1U<<0),
        DISABLE_REDLINE_GOVERNOR = (1U << 1),
        THROTTLE_WHILE_DISARMED = (1U << 2),
    };
    AP_Int16 options;

    bool option_set(Options option) const {
        return (options & uint16_t(option)) != 0;
    }

    // start_chan debounce
    uint16_t start_chan_last_value = 1500;
    uint32_t start_chan_last_ms;

    // redline rpm
    AP_Int32 redline_rpm;
    struct {
        bool flag;
        float governor_integrator;
        float throttle_percentage;
    } redline;
};


namespace AP {
    AP_ICEngine *ice();
};

#endif  // AP_ICENGINE_ENABLED
