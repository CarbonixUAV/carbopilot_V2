--[[
    File Name: cx_built_in_test.lua
    Description: This script is Carbonix Continuous Built in Test used to do multiple lua functionality of Carbonix Aircrafts, Ottano and ESC.
    Functionality: 
        - ESC Status Check and Fault Detection
        - EFI Status Check and Fault Detection
    Owner: [Carbonix - Software Team]
]]


-- ******************* Macros *******************

local SCRIPT_NAME = 'CX_BIT'
local CX_SANITY_SCRIPT_VERSION = 1.0 -- Script Version
local CX_PILOT_MIN_FW_VERSION = 5.0 -- Minimum Firmware Version Supported e.g CxPilot 5.0.0 should be mentioned as 5.0

--------  MAVLINK/AUTOPILOT 'CONSTANTS'  --------
-- local MAV_SEVERITY_EMERGENCY=0 --/* System is unusable. This is a "panic" condition. | */
-- local MAV_SEVERITY_ALERT=1     --/* Action should be taken immediately. Indicates error in non-critical systems. | */
local MAV_SEVERITY_CRITICAL=2  --/* Action must be taken immediately. Indicates failure in a primary system. | */
local MAV_SEVERITY_ERROR=3     --/* Indicates an error in secondary/redundant systems. | */
local MAV_SEVERITY_WARNING=4   --/* Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. | */
-- local MAV_SEVERITY_NOTICE=5    --/* An unusual event has occurred though not an error condition. This should be investigated for the root cause. | */
local MAV_SEVERITY_INFO=6      --/* Normal operational messages. Useful for logging. No action is required for these messages. | */
-- local MAV_SEVERITY_DEBUG=7     --/* Useful non-operational messages that can assist in debugging. These should not occur during normal operation. | */
-- local MAV_SEVERITY_ENUM_END=8  --/*  | */

local MSG_NORMAL = 1    -- 1 for normal status messages
local MSG_DEBUG  = 2    -- 2 for additional debug messages
local VERBOSE_MODE = 2  -- 0 to suppress all GCS messages, 1 for normal status messages, 2 for additional debug messages

local M1 = 0
local M2 = 1
local M3 = 2
local M4 = 3
local M5 = 4
local WARMING_MSG_TIMEOUT = 15

local UNKNOWN = 0
local VOLANTI = 1
local OTTANO = 2

local HIRTH_EFI_TYPE = 8
-- ******************* Variables *******************

local aircraft_type = UNKNOWN

local pre_arm_init = false

local esc_prev_ts = {0, 0, 0, 0, 0, 0, 0} -- Initialize to 0 for each ESC

local last_motor_lost = -1

-- ******************* Objects *******************

local auth_id = arming:get_aux_auth_id()
assert(auth_id, SCRIPT_NAME .. ": could not get a prearm check auth id")

local params = {
    -- CAN_D1_UC_ESC_BM = Parameter(),
    -- CAN_D2_UC_ESC_BM = Parameter(),
    EFI_TYPE = Parameter()
}

-- ******************* Functions *******************

local function get_time_sec()
    return millis():tofloat() * 0.001
end

-- wrapper for gcs:send_text()
local function gcs_msg(msg_type, severity, txt)
    if type(msg_type) == 'string' then
    -- allow just a string to be passed for simple/routine messages
        txt      = msg_type
        msg_type = MSG_NORMAL
        severity = MAV_SEVERITY_INFO
    end
    if msg_type <= VERBOSE_MODE then
        gcs:send_text(severity, string.format('%s: %s', SCRIPT_NAME, txt))
    end
end

local function check_aircraft()
    if params.EFI_TYPE:get() == HIRTH_EFI_TYPE then
        aircraft_type = OTTANO
        gcs_msg(MSG_DEBUG, MAV_SEVERITY_INFO, "Aircraft Type: Ottano")
    else
        aircraft_type = VOLANTI
        gcs_msg(MSG_DEBUG, MAV_SEVERITY_INFO, "Aircraft Type: Volanti")
    end
    return 0
end

local function init_param()
    for paramName, param in pairs(params) do
        if not param:init(paramName) then
            arming:set_aux_auth_failed(auth_id, "LUA: " .. paramName .. " Param fetch failed")
            return -1
        end
    end
    gcs_msg(MSG_DEBUG, MAV_SEVERITY_INFO, "Param Init")
    return 0
end

local function all_telemetry_check()
    local escs = {M1, M2, M3, M4, M5}

    if aircraft_type == UNKNOWN then
        gcs_msg(MSG_DEBUG, MAV_SEVERITY_WARNING, "Aircraft Type Unknown")
        return -1
    end

    for i = 1, #escs do
        local current_ts = esc_telem:get_last_telem_data_ms(escs[i]):toint()
        if (i < 5 or aircraft_type == VOLANTI) and (current_ts == 0 or current_ts - esc_prev_ts[i] > 1000) then
            return i
        end
        if i == 5 and aircraft_type == OTTANO then
            current_ts =  efi.get_last_update_ms():toint()
            if current_ts == 0 or current_ts - esc_prev_ts[i] > 1000 then
                return i
            end
        end
        esc_prev_ts[i] = current_ts
    end
    return 0
end

local function pre_arm_check()
    if pre_arm_init then
        local act_return = all_telemetry_check()
        if act_return == 5 and aircraft_type == OTTANO then
            arming:set_aux_auth_failed(auth_id, "EFI Not Responding")
        elseif act_return ~= 0 then
            arming:set_aux_auth_failed(auth_id, "ESC " .. act_return .. " Not Responding")
        else
            arming:set_aux_auth_passed(auth_id)
        end
        return
    end
    if not auth_id or init_param() ~= 0 or check_aircraft() ~= 0 then
        local message = not auth_id and "Pre Arm Not functional" or
                        init_param() ~= 0 and "Parameter Init Failed" or
                        "Aircraft Type Check Failed"
        gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, message)
        return
    end
    gcs_msg(MSG_DEBUG, MAV_SEVERITY_INFO, "Pre Arm Init Done")
    pre_arm_init = true
end

local function vtol_failure_check()
    local lost_index = MotorsMatrix:get_thrust_boost() and MotorsMatrix:get_lost_motor() or -1

    if lost_index ~= last_motor_lost then
        local message = lost_index == -1 and "Motors Thrust: recovered" or "Motor ".. lost_index + 1 .." Thrust: lost"
        local severity = lost_index == -1 and MAV_SEVERITY_INFO or MAV_SEVERITY_CRITICAL

        gcs_msg(MSG_NORMAL, severity, message)
        last_motor_lost = lost_index
    end
end

local last_fail_msg_time = 0
local telem_failure = false

local function during_arm_check()
    local act_return = all_telemetry_check()
    local current_time = get_time_sec()

    if current_time - last_fail_msg_time >= WARMING_MSG_TIMEOUT then
        if act_return > 0 then
            if act_return == 5 and aircraft_type == OTTANO then
                gcs_msg(MSG_NORMAL, MAV_SEVERITY_CRITICAL, "EFI Telemetry Not Responding")
            else
                gcs_msg(MSG_NORMAL, MAV_SEVERITY_CRITICAL, "ESC " .. act_return .. " Telemetry Not Responding")
            end
            last_fail_msg_time = current_time
            telem_failure = true
        end
    end
    if telem_failure and act_return == 0 then
        gcs_msg(MSG_NORMAL, MAV_SEVERITY_CRITICAL, "Telemetry Recovered")
        telem_failure = false
    end
    vtol_failure_check()
end


local function update()
    local arming = require("arming")
    if not arming:is_armed() then
        pre_arm_check()
    else
        if pre_arm_init == false then
            return
        end
        during_arm_check()
    end
end

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs_msg(MSG_NORMAL, MAV_SEVERITY_ERROR, "Internal Error: " .. err)
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, 500
end

local function script_exit()
    gcs_msg(MSG_NORMAL, MAV_SEVERITY_CRITICAL, "LUA SCRIPT EXIT")
end

-- ******************* Main *******************
-- Check Script uses a miniumum firmware version
local FWVersion = require("FWVersion")
local cx_version = FWVersion:string()
local version = cx_version:match("(%d+%.%d+)")

if cx_version:match("CxPilot") and version then
    version = tonumber(version)
    if CX_PILOT_MIN_FW_VERSION > version then
        gcs_msg(MSG_NORMAL, MAV_SEVERITY_INFO, string.format('%s Requires: %.1f. Found Version: %s', SCRIPT_NAME, CX_PILOT_MIN_FW_VERSION, version))
    else
        gcs_msg(MSG_NORMAL, MAV_SEVERITY_INFO, "Script Version " .. CX_SANITY_SCRIPT_VERSION .. " Initialized on " .. cx_version)
        return protected_wrapper()
    end
else
    gcs_msg(MSG_NORMAL, MAV_SEVERITY_INFO, string.format('Requires: CxPilot Code Base. Found Version: %s', FWVersion:string()))
end

script_exit()