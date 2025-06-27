--[[ 
Internal Combustion Engine (ICE) simulation for SITL.

This script models the behavior of the Hirth 4103 engine used on Ottano and
implements a scripting backend EFI to report the simulated data (CHT, fuel
consuption, etc). The script also adds parameters to simulate various failure
behaviors like overheat (on either cylinder), or degradation of thrust.

It also simulates the ignition and starter control. The script listens to
ArduPilot's existing simulated GPIO pins. You must configure a starter relay and
an ignition relay with pin assignments that match whatever is set in
SIM_ICE_IGN_PIN and SIM_ICE_STRT_PIN. The script then simulates the idle and
starter behavior by adjusting the throttle servo's min and max values. You can
fail the engine by setting the ignition pin to -1, and you can similarly fail
the starter motor too.
--]]

local SCRIPT_NAME       = "ICEngine: SITL"

-- Ensure the script is loaded in SITL only
assert(param:get('SIM_OPOS_LAT') ~= nil, string.format('%s was designed for SITL', SCRIPT_NAME))

-- Bind parameter utilities
local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- Set up EFI parameters
PARAM_TABLE_PREFIX = 'SIM_ICE_'
PARAM_TABLE_KEY = 36
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add ' .. string.sub(PARAM_TABLE_PREFIX, 1, -2) .. ' param table')
--[[
  // @Param: SIM_ICE_CHT1_INC
  // @DisplayName: CHT1 Increase
  // @Description: Causes the temperature of cylinder 1 to increase by this much
  // @Range: -100 100
  // @Units: degC
--]]
local CHT1_INCREASE = bind_add_param('CHT1_INC', 1, 0)
--[[
  // @Param: SIM_ICE_CHT2_INC
  // @DisplayName: CHT2 Increase
  // @Description: Causes the temperature of cylinder 2 to increase by this much
  // @Range: -100 100
  // @Units: degC
--]]
local CHT2_INCREASE = bind_add_param('CHT2_INC', 2, 0)
--[[
  // @Param: SIM_ICE_IDLE_PWM
  // @DisplayName: Idle PWM
  // @Description: PWM value for idle throttle
  // @Range: 1000 2000
  // @Units: us
--]]
local IDLE_PWM = bind_add_param('IDL_PWM', 3, 1100)
--[[
  // @Param: SIM_ICE_STRT_PWM
  // @DisplayName: Starter PWM
  // @Description: PWM value to simulate when the starter motor is running
  // @Range: 1000 2000
  // @Units: us
--]]
local STRT_PWM = bind_add_param('STRT_PWM', 4, 1150)
--[[
  // @Param: SIM_ICE_OFF_PWM
  // @DisplayName: Off PWM
  // @Description: PWM value to simulate when the engine is off
  // @Range: 1000 2000
  // @Units: us
--]]
local OFF_PWM = bind_add_param('OFF_PWM', 5, 1000)
--[[
  // @Param: SIM_ICE_MAX_PWM
  // @DisplayName: Max PWM
  // @Description: Maximum PWM value for the throttle
  // @Range: 1000 2000
  // @Units: us
--]]
local MAX_PWM = bind_add_param('MAX_PWM', 6, 2000)
--[[
  // @Param: SIM_ICE_IGN_PIN
  // @DisplayName: Ignition Pin
  // @Description: Simulated GPIO pin for ignition control, i.e., which bit to
  //  check in the SIM_PIN_MASK to see if ignition is enabled. Set to -1 to
  //  fail the ignition.
  // @Range -1 31
--]]
local IGN_PIN = bind_add_param('IGN_PIN', 7, 0)
--[[
  // @Param: SIM_ICE_STRT_PIN
  // @DisplayName: Starter Pin
  // @Description: Simulated GPIO pin for starter control, i.e., which bit to
  //  check in the SIM_PIN_MASK to see if the starter is running. Set to -1 to
  //  fail the starter.
  // @Range -1 31
--]]
local STRT_PIN = bind_add_param('STRT_PIN', 8, 1)
--[[
  // @Param: SIM_ICE_EGT1_INC
  // @DisplayName: EGT1 Increase
  // @Description: Causes the temperature of exhaust gas 1 to increase by this much
  // @Range: -300 300
  // @Units: degC
--]]
local EGT1_INCREASE = bind_add_param('EGT1_INC', 9, 0)
--[[
  // @Param: SIM_ICE_EGT2_INC
  // @DisplayName: EGT2 Increase
  // @Description: Causes the temperature of exhaust gas 2 to increase by this much
  // @Range: -300 300
  // @Units: degC
--]]
local EGT2_INCREASE = bind_add_param('EGT2_INC', 10, 0)

-- We look at RPM2 to see if another source (RealFlight) should provide the RPM
-- instead of us making one up based on the throttle PWM.
local RPM_TYPE = bind_param('RPM2_TYPE')

local UPDATE_HZ = 4

-- behavior-tuning constants
local NOMINAL_VALUES = {
    IDLE_RPM = 1500,          -- Minimum RPM for delta T calculation
    FULL_RPM = 7100,          -- Maximum RPM for delta T calculation
    CRUISE_SPEED = 25,        -- Cruise speed (m/s)
    CRUISE_RPM = 5000,        -- Nominal RPM for burn rate
}

local CHT_FIT_VALUES = {
    FULL_DT_CRUISE = 232.4,   -- Full throttle delta temperature at cruise speed (°C)
    IDLE_DT_CRUISE = 92.2,    -- Idle throttle delta temperature at cruise speed (°C)
    IDLE_DT_HOVER = 116.8,    -- Idle throttle delta temperature while stationary (°C)
    TCONST_CRUISE = 18.9,     -- Time constant for thermal inertia at cruise speed (s)
    KAPPA = 7.84,             -- Scaling factor for thermal inertia changes with airspeed
}

local CHT2_FIT_VALUES = {
    FULL_DT_CRUISE = 203.2,   -- Full throttle delta temperature at cruise speed (°C)
    IDLE_DT_CRUISE = 83.9,    -- Idle throttle delta temperature at cruise speed (°C)
    IDLE_DT_HOVER = 115.5,    -- Idle throttle delta temperature while stationary (°C)
    TCONST_CRUISE = 28.6,     -- Time constant for thermal inertia at cruise speed (s)
    KAPPA = 2.00,             -- Scaling factor for thermal inertia changes with airspeed
}

local EGT_FIT_VALUES = {
    FULL_DT_CRUISE = 580.0,   -- Full throttle delta temperature at cruise speed (°C)
    IDLE_DT_CRUISE = 350.0,   -- Idle throttle delta temperature at cruise speed (°C)
    IDLE_DT_HOVER = 350.0,    -- Idle throttle delta temperature while stationary (°C)
    TCONST_CRUISE = 6.0,      -- Time constant for thermal inertia at cruise speed (s)
    KAPPA = 0.0,              -- Scaling factor for thermal inertia changes with airspeed
}

local EGT2_FIT_VALUES = {
    FULL_DT_CRUISE = 590.0,   -- Full throttle delta temperature at cruise speed (°C)
    IDLE_DT_CRUISE = 355.0,   -- Idle throttle delta temperature at cruise speed (°C)
    IDLE_DT_HOVER = 355.0,    -- Idle throttle delta temperature while stationary (°C)
    TCONST_CRUISE = 5.0,      -- Time constant for thermal inertia at cruise speed (s)
    KAPPA = 0.0,              -- Scaling factor for thermal inertia changes with airspeed
}

local FUEL_FIT_VALUES = {
    BURN_RATE = 0.816,        -- Nominal burn rate at cruise RPM (kg/hr)
}

-- Utility functions
local function constrain(value, min_val, max_val)
    return math.min(math.max(value, min_val), max_val)
end

local function c_to_kelvin(temp)
    return temp + 273.15
end

-- The script hacks the throttle servo parameters to emulate ignition control
-- First, find the throttle servo channel
throttle_channel = SRV_Channels:find_channel(70)
if not throttle_channel then
    gcs:send_text(0, "Could not find throttle channel")
    return
end
throttle_channel = throttle_channel + 1

local SERVO_THR_MIN = bind_param('SERVO' .. throttle_channel .. '_MIN')
local SERVO_THR_MAX = bind_param('SERVO' .. throttle_channel .. '_MAX')
local SERVO_THR_TRIM = bind_param('SERVO' .. throttle_channel .. '_TRIM')
local SIM_PIN_MASK = bind_param('SIM_PIN_MASK')
local SIM_TEMP_START = bind_param('SIM_TEMP_START')

local efi_backend = nil

--- Exponentially decay a temperature toward the steady state
---@param current_cht number Current CHT
---@param cht_steady number Steady state CHT that we are decaying towards
---@param airspeed number Airspeed in m/s
---@param tconst_cruise number Time constant for thermal inertia at cruise speed (s)
---@param kappa number Scaling factor for thermal inertia changes with airspeed
---@return number
local function update_cht(current_cht, cht_steady, airspeed, tconst_cruise, kappa)
    --[[
    We model the thermal inertia of the engine as a first-order system:
    
            T = T_steady + (T_last - T_steady) * (1 - exp(-t / τ))
    
    where T is the temperature, T_steady is the steady-state temperature, T_last
    is the temperature when we last updated, and t is the time since the last
    update. The time constant, τ, is a calibration parameter, but it too
    depends on airspeed. We model this as:

                        τ ~ 1 / (1 + κ * sqrt(v))

    where v is the airspeed, and κ is another fit parameter. Usually the time
    constant is easiest to guess at the cruise speed, as that's where we have
    the most data. Then, you can just play around with κ until the warmup on
    the ground feels right.
    --]]

    local tau = tconst_cruise * (1 + kappa * math.sqrt(NOMINAL_VALUES.CRUISE_SPEED)) / (1 + kappa * math.sqrt(airspeed))
    local decay_factor = 1 - math.exp(-1 / tau / UPDATE_HZ)
    return current_cht * (1 - decay_factor) + cht_steady * decay_factor
end

local function get_air_temperature()
    -- Standard lapse rate is 6.5°C per 1000m
    local start_temp = SIM_TEMP_START:get() or 15
    local altitude = - ahrs:get_relative_position_D_home() * 0.01 or 0
    return start_temp - altitude * 0.0065
end

-- Engine control object
local function engine_control()
    local self = {}

    -- Build up the EFI_State that is passed into the EFI Scripting backend
    local efi_state = EFI_State()
    local cylinder_state = Cylinder_Status()
    local rpm = 0
    local air_pressure = 0
    local fuel_consumption_lph = 0
    local fuel_total_l = 0
    local temps = {
        cht = {get_air_temperature(), get_air_temperature()}, -- Cylinder head temperatures
        egt = {get_air_temperature(), get_air_temperature()}, -- Exhaust gas temperatures
        imt = get_air_temperature(), -- Intake manifold temperature
    }
    local is_running = false
    local crank_start_ms = nil -- Timestamp of when the starter first turned on

    -- Build and set the EFI_State that is passed into the EFI Scripting backend
    function self.set_EFI_State()
        if not efi_backend then
            return
        end

        -- Cylinder_Status
        cylinder_state:cylinder_head_temperature(c_to_kelvin(temps.cht[1]))
        cylinder_state:cylinder_head_temperature2(c_to_kelvin(temps.cht[2]))
        cylinder_state:exhaust_gas_temperature(c_to_kelvin(temps.egt[1]))
        cylinder_state:exhaust_gas_temperature2(c_to_kelvin(temps.egt[2]))

        efi_state:engine_speed_rpm(uint32_t(rpm))

        efi_state:fuel_consumption_rate_cm3pm(fuel_consumption_lph * 1000.0 / 60.0)
        efi_state:estimated_consumed_fuel_volume_cm3(fuel_total_l * 1000.0)
        efi_state:intake_manifold_pressure_kpa(air_pressure)
        efi_state:intake_manifold_temperature(c_to_kelvin(temps.imt))
        local throttle_pwm = SRV_Channels:get_output_pwm(70) or 1000
        local throttle = (throttle_pwm - 1000) / (MAX_PWM:get() - 1000)
        throttle = constrain(throttle, 0, 1)
        efi_state:throttle_position_percent(math.floor(throttle * 100))

        -- copy cylinder_state to efi_state
        efi_state:cylinder_status(cylinder_state)

        efi_state:last_updated_ms(millis())

        -- Set the EFI_State into the EFI scripting driver
        efi_backend:handle_scripting(efi_state)
    end

    ---Calculate the steady state CHT for a given airspeed, fit parameters,
    ---and the user-supplied cht_increase parameter (for simulating overheat).
    ---@param airspeed number
    ---@param params table
    ---@param cht_increase number
    ---@return number -- Steady state CHT
    function self.calculate_cht_steady(airspeed, params, cht_increase)
        --[[
        This thermal model is based on King's law, which is an empirical model
        used for calibrating hotwire anemometers. The thermal power convected
        away from a hot object in moving air follows this relation:

                        P = ΔT * C1 * [1 + C2 * sqrt(v)]

        where P is the power, ΔT is the temperature difference between the
        object and the air, v is the airspeed, and C1 and C2 are calibration
        constants.

        At steady state, the power generated by the engine is equal to the power
        convected away by the air, and the power generated by the engine will be
        assumed to be proportional to the RPM cubed, plus some additional
        constant power. So we can rewrite the above equation as:
            ΔT_steady = (C3 * RPM^3 + C4) / [1 + C2 * sqrt(v)] / C1
        where C3 is an additional calibration constant. However, these
        calibration constants are arbitrary, so we can define C1' = C3 / C1 and
        C3' = C4 / C1, and rewrite the equation as:

                    ΔT_steady = (C1' * RPM^3 + C3') / [1 + C2 * sqrt(v)]

        after which I will drop the ' mark for simplicity.

        We could do some curve fitting to find values for C1-C3, but
        I wanted to define constants that had an easier intuitive meaning, and
        derive them from the intuitive values. We define the following system of
        equations:

        1. ΔT_full_cruise = (C1 * RPM_full^3 + C3) / [1 + C2 * sqrt(v_cruise)]
        2. ΔT_idle_cruise = (C1 * RPM_idle^3 + C3) / [1 + C2 * sqrt(v_cruise)]
        3. ΔT_idle_hover  =  C1 * RPM_idle^3 + C3
        
        where RPM_full is the max RPM of the engine, RPM_idle is the idle
        RPM, and v_cruise is the cruise speed. These are arbitrary nominal
        values for which the new ΔT constants are defined.
        
        Anyone familiar with flying that engine can easily spitball these ΔT
        values. I've gone the extra mile of fitting them to log data with a
        python script, but having an intuitive starting point helps with
        that, and it makes it easier to sanity check the results.
        
        From these equations, we can derive the following:
        4. ΔT_full_hover =  C1 * RPM_full^3 + C3
        5. ΔT_full_hover = ΔT_full_cruise * (1 + C2 * sqrt(v_cruise))
        6. C2 = (ΔT_idle_hover / ΔT_idle_cruise - 1) / sqrt(v_cruise)
        7. C1 = (ΔT_full_hover - ΔT_idle_hover) / (RPM_full^3 - RPM_idle^3)
        8. C3 = ΔT_full_hover - C1 * RPM_full^3

        We also allow for simulating overheat. To do this, we essentially
        apply an offset to the full-throttle cruise delta temperature. This is
        more realistic than just adding a constant to cht_steady.
        --]]

        -- First, we see if we need to recalculate these constants, which only
        -- needs to happen if cht_increase changes.
        if params.last_cht_increase ~= cht_increase then
            params.last_cht_increase = cht_increase
            -- Solve for C2
            params.C2 = (params.IDLE_DT_HOVER / params.IDLE_DT_CRUISE - 1) / math.sqrt(NOMINAL_VALUES.CRUISE_SPEED)
            -- Derive the full-throttle delta temperature for stationary condition
            -- (we also add in the "CHT increase" user parameter here)
            params.FULL_DT_HOVER = (params.FULL_DT_CRUISE + cht_increase) * (1 + params.C2 * math.sqrt(NOMINAL_VALUES.CRUISE_SPEED))
            -- Use that to solve for C1 and C3
            params.C1 = (params.FULL_DT_HOVER - params.IDLE_DT_HOVER) / (NOMINAL_VALUES.FULL_RPM^3 - NOMINAL_VALUES.IDLE_RPM^3)
            params.C3 = params.FULL_DT_HOVER - params.C1 * NOMINAL_VALUES.FULL_RPM^3
        end

        -- Calculate the steady state CHT at our actual airspeed
        local dt_steady = (params.C1 * rpm^3 + params.C3) / (1 + params.C2 * math.sqrt(airspeed))

        -- Handle engine off condition
        if rpm == 0 then
            dt_steady = 0
        end

        return dt_steady + temps.imt
    end

    -- Simulate engine behavior
    function self.simulate_engine()
        -- If the RPM sensor is None, or EFI (prevent circular dependency), then we need to make up the RPM
        if RPM_TYPE:get() == 3 or RPM_TYPE:get() == 0 then
            local thr = (SRV_Channels:get_output_pwm(70) - OFF_PWM:get()) / (MAX_PWM:get() - OFF_PWM:get())
            thr = constrain(thr, 0, 1)
            rpm = math.sqrt(thr) * NOMINAL_VALUES.FULL_RPM
        else
            rpm = constrain(RPM:get_rpm(1) or 0, 0, 50000)
        end
        air_pressure = baro:get_pressure() / 100 or 0
        temps.imt = get_air_temperature()
        local airspeed = ahrs:airspeed_estimate() or 0

        -- Simulate CHT1 and CHT2
        for i = 1, 2 do
            local params = (i == 1) and CHT_FIT_VALUES or CHT2_FIT_VALUES
            local cht_increase = (i == 1) and CHT1_INCREASE:get() or CHT2_INCREASE:get()

            -- Calculate the steady state CHT at our current rpm and airspeed
            local cht_steady = self.calculate_cht_steady(airspeed, params, cht_increase)

            -- Update CHT towards steady state
            temps.cht[i] = update_cht(temps.cht[i], cht_steady, airspeed, params.TCONST_CRUISE, params.KAPPA)
        end

        -- Simulate EGT1 and EGT2
        for i = 1, 2 do
            local params = (i == 1) and EGT_FIT_VALUES or EGT2_FIT_VALUES
            local egt_increase = (i == 1) and EGT1_INCREASE:get() or EGT2_INCREASE:get()

            -- Calculate the steady state EGT at our current rpm and airspeed
            local egt_steady = self.calculate_cht_steady(airspeed, params, egt_increase)

            -- Update EGT towards steady state
            temps.egt[i] = update_cht(temps.egt[i], egt_steady, airspeed, params.TCONST_CRUISE, params.KAPPA)
        end

        -- Simulate fuel consumption
        local fuel_factor = rpm / NOMINAL_VALUES.CRUISE_RPM
        fuel_consumption_lph = FUEL_FIT_VALUES.BURN_RATE * fuel_factor^3
        fuel_total_l = fuel_total_l + fuel_consumption_lph / 3600 / UPDATE_HZ
    end

    ---Update servo parameters to simulate ignition and starter
    ---@param ignition boolean
    ---@param start boolean
    function self.handle_ignition_and_starter(ignition, start)
        -- Track how long the engine has had a chance to start. The starter has
        -- to run, while the ignition is also on, for a minimum of 1.5 seconds.
        if (not start) or (not ignition)  then
            crank_start_ms = nil
        elseif not crank_start_ms then
            crank_start_ms = millis()
        end
        if crank_start_ms and millis() - crank_start_ms > 1500 then
            is_running = true
        end

        -- Kill the engine if the ignition cuts off
        if is_running and not ignition then
            is_running = false
        end

        -- Mess with the throttle servo parameters to simulate the running/starting
        local servo_thr_min
        if start then
            servo_thr_min = STRT_PWM:get() or 1200
        elseif is_running then
            servo_thr_min = IDLE_PWM:get() or 1100
        else
            servo_thr_min = OFF_PWM:get() or 1000
        end
        SERVO_THR_MIN:set(servo_thr_min)
        SERVO_THR_TRIM:set(servo_thr_min)

        if is_running then
            SERVO_THR_MAX:set(MAX_PWM:get() or 2000)
        else
            SERVO_THR_MAX:set(SERVO_THR_MIN:get() + 1 or 1001)
        end
    end

    -- Return engine control instance
    return self
end

local engine = engine_control()

local function update()
    if not efi_backend then
        efi_backend = efi:get_backend(0)
        if not efi_backend then
           return
        end
     end

    engine.simulate_engine()
    engine.set_EFI_State()

    -- Emulate ignition and starter
    local ignition = false
    local start = false
    if IGN_PIN:get() >= 0 then
        ignition = SIM_PIN_MASK:get() & (1 << IGN_PIN:get()) ~= 0
    end
    if STRT_PIN:get() >= 0 then
        start = SIM_PIN_MASK:get() & (1 << STRT_PIN:get()) ~= 0
    end
    engine.handle_ignition_and_starter(ignition, start)
end

-- Wrapper to handle errors
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(0, "Internal Error: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, 1000 / UPDATE_HZ
end

-- Start the update loop
return protected_wrapper()
