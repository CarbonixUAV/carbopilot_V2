--[[
ESC telemetry simulator for SITL.

Simulates basic ESC telemetry: voltage, current, temperature, and RPM.
Additional parameters are supplied for simulating telemetry failures and
overheating.
--]]

local SCRIPT_NAME = "ESC: SITL"
local UPDATE_HZ = 50
-- Ensure the script is loaded in SITL only
assert(param:get('SIM_OPOS_LAT') ~= nil, string.format('%s was designed for SITL', SCRIPT_NAME))

-- Set up ESC parameters
local PARAM_TABLE_KEY = 20
local PARAM_TABLE_PREFIX = 'SIM_ESC_'

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

assert(false or param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), 'could not add ' .. string.sub(PARAM_TABLE_PREFIX, 1, -2) .. ' param table')

--[[
  // @Param: SIM_ESC_TLM_FAIL
  // @DisplayName: Simulated ESC telemetry failure mask
  // @Description: Telemetry will stop being sent for the ESCs with the corresponding bits set in this mask
  // @Bitmask: 0:ESC1, 1:ESC2, 2:ESC3, 3:ESC4, 4:ESC5, 5:ESC6, 6:ESC7, 7:ESC8
--]]
local TELEM_FAIL_MASK = bind_add_param('TLM_FAIL', 1, 0)
--[[
    // @Param: SIM_ESC_OVERHEAT
    // @DisplayName: Simulated overheat additional temperature
    // @Description: The ESCs indicated by SIM_ESC_OH_MASK will have this temperature added to them
    // @Range: 0 100
    // @Increment: 1
    // @Units: degC
--]]
local OVERHEAT_TEMP = bind_add_param('OVERHEAT', 2, 0)
--[[
    // @Param: SIM_ESC_OH_MASK
    // @DisplayName: Simulated overheat mask
    // @Description: The ESCs with the corresponding bits set in this mask will have the temperature added to them
    // @Bitmask: 0:ESC1, 1:ESC2, 2:ESC3, 3:ESC4, 4:ESC5, 5:ESC6, 6:ESC7, 7:ESC8
--]]
local OVERHEAT_MASK = bind_add_param('OH_MASK', 3, 0)

-- Constants for Ottano
-- (these could be parameters, but they don't need to change in flight, and the
-- defaults.parm files can't update defaults for script parameters).
-- TODO: make some constants for Volanti, and make a way for the script to know
-- which aircraft it's running on
local BAT_IDX = 8
local CABLE_RESISTANCE = 0.003 -- Ohms
local CURRENT_NOISE = 5 -- Amps
local VOLTAGE_NOISE = 0.1 -- Volts
local RPM_NOISE = 1.5 -- % of RPM
local NOMINAL_CURRENT = 105 -- Amps
local DELTA_TEMP = 64 -- degC, steady-state temp difference at NOMINAL_CURRENT
local TIME_CONSTANT = 41 -- seconds

local SIM_ENGINE_FAIL = bind_param('SIM_ENGINE_FAIL')
local SIM_ENGINE_MUL = bind_param('SIM_ENGINE_MUL')
local SIM_TEMP_START = bind_param('SIM_TEMP_START')
local MAX_RPM = param:get('SIM_VIB_MOT_MAX') * 60
assert(MAX_RPM > 0, 'SIM_VIB_MOT_MAX must be set to get RPM telemetry')
local MIN_RPM = 500
local Q_M_PWM_MIN = param:get('Q_M_PWM_MIN') or 1000
local Q_M_PWM_MAX = param:get('Q_M_PWM_MAX') or 2000
local Q_M_SPIN_ARM = param:get('Q_M_SPIN_MIN') or 0.1

local function get_air_temperature()
    -- Standard lapse rate is 6.5°C per 1000m
    local start_temp = SIM_TEMP_START:get() or 15
    local altitude = - ahrs:get_relative_position_D_home() * 0.01 or 0
    return start_temp - altitude * 0.0065
end

-- Gaussian noise approximation generator
local function gaussian_noise(std_dev)
    local sum = 0
    for _ = 1, 12 do
        sum = sum + math.random() - 0.5
    end
    return sum * std_dev
end

local function move_temp_toward_steady(temp, temp_steady)
    local decay_factor = 1 - math.exp(-1 / TIME_CONSTANT / UPDATE_HZ)
    return temp * (1 - decay_factor) + temp_steady * decay_factor
end

local function get_output_scaled(servo_func)
    local pwm = SRV_Channels:get_output_pwm(servo_func)
    local chan = SRV_Channels:find_channel(servo_func)
    -- An occasional race condition can cause either of these to return nil
    -- return a nil to let the caller know to try again next iteration
    if not pwm or not chan then
        return nil
    end
    if SIM_ENGINE_FAIL:get() & (1 << chan) ~= 0 then
        pwm = pwm * SIM_ENGINE_MUL:get()
    end
    local actuator = (pwm - Q_M_PWM_MIN) / (Q_M_PWM_MAX - Q_M_PWM_MIN)
    return math.min(1, math.max(0, actuator))
end

-- Table of ESCs detected in our configuration parameters
local ESCs = {}
-- Constructor to return an ESC information table
local function new_esc(servo_func)
    local self = {}
    self.servo_func = servo_func
    self.thrust = 0
    self.rpm = 0
    self.temp = get_air_temperature()
    self.telem_data = ESCTelemetryData()
    return self
end

-- Send telemetry for an ESC
local function send_esc_telem(i, esc)
    local data_mask = 0x0D -- voltage, current, temperature
    esc_telem:update_telem_data(i-1, esc.telem_data, data_mask)
    esc_telem:update_rpm(i-1, esc.rpm, 0)
end

-- Scan the config parameters and set up the table of detected ESCs
local function setup()
    local ice_enable = param:get('ICE_ENABLE')
    assert(ice_enable, 'Could not find ICE_ENABLE parameter')
    for i = 1, 8 do
        local servo_func = 32 + i
        -- Detect how many motors we have
        if not SRV_Channels:find_channel(servo_func) then
            break
        end
        table.insert(ESCs, new_esc(servo_func))
    end
    assert(#ESCs >= 4, 'Could not find at least 4 ESCs')
    -- Add the pusher as the last ESC if we do not have an engine
    if ice_enable == 0 then
        assert(SRV_Channels:find_channel(70), 'Could not find pusher servo channel')
        table.insert(ESCs, new_esc(70))
    end
end


local function update()
    -- Get the voltage and current from the battery
    local voltage = battery:voltage(BAT_IDX or 0) or 0
    local total_current = battery:current_amps(BAT_IDX or 0) or 0

    -- Calculate the thrust for each ESC
    local sum_of_thrust_sq = 0
    for _, esc in ipairs(ESCs) do
        local actuator = get_output_scaled(esc.servo_func)
        if actuator then -- keep the old thrust value if the get_output race condition bytes us this iteration
            esc.thrust = motors:actuator_to_thrust(actuator)
            -- Add the small amount of thrust from the motor spinning at min
            esc.thrust = esc.thrust * (1 - (MIN_RPM / MAX_RPM)^2) + math.min(actuator, Q_M_SPIN_ARM)*(MIN_RPM / MAX_RPM)^2/Q_M_SPIN_ARM
        end
        sum_of_thrust_sq = sum_of_thrust_sq + esc.thrust * esc.thrust
    end

    -- Calculate the rpm, current, voltage, and temperature for each ESC
    for i, esc in ipairs(ESCs) do
        esc.rpm = math.sqrt(esc.thrust) * MAX_RPM
        esc.rpm = esc.rpm + gaussian_noise(RPM_NOISE * esc.rpm / 100)
        esc.rpm = math.max(esc.rpm, 0)
        esc.rpm = math.floor(esc.rpm)
        local esc_current = 0
        if sum_of_thrust_sq > 0 then
            esc_current = total_current * esc.thrust^2 / sum_of_thrust_sq + gaussian_noise(CURRENT_NOISE)
        end
        local esc_voltage = voltage - esc_current * CABLE_RESISTANCE + gaussian_noise(VOLTAGE_NOISE)
        local steady_temp = get_air_temperature() + (esc_current^2) / (NOMINAL_CURRENT^2) * DELTA_TEMP
        if OVERHEAT_MASK:get() & (1 << (i - 1)) ~= 0 then
            steady_temp = steady_temp + OVERHEAT_TEMP:get()
        end
        esc.temp = move_temp_toward_steady(esc.temp, steady_temp)

        -- Set the telemetry data
        esc.telem_data:voltage(esc_voltage)
        esc.telem_data:current(esc_current)
        local t = math.floor(esc.temp * 100)
        -- Round off to make it a bit blockier
        esc.telem_data:temperature_cdeg(t - t % 20)
    end

    -- Send the telemetry
    for i, esc in ipairs(ESCs) do
        if TELEM_FAIL_MASK:get() & (1 << (i - 1)) == 0 then
            send_esc_telem(i, esc)
        end
    end
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

-- Set up the script
setup()

-- Start the update loop
return protected_wrapper()
