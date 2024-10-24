--[[
    File Name: cx_built_in_test.lua
    Description: This script performs a continuous built-in test for various functionalities on Carbonix Aircrafts, focusing on ESC status check and fault detection.
    Owner: [Carbonix - Software Team]
]]

-- ******************* Macros *******************

local SCRIPT_NAME = 'CX_BIT'
local SCRIPT_VERSION = 1.1 -- Script Version


--------  MAVLINK/AUTOPILOT 'CONSTANTS'  --------
-- MAVLink Severity Levels
local MAV_SEVERITY_CRITICAL  = 2 -- Action must be taken immediately. Indicates failure in a primary system.
local MAV_SEVERITY_ERROR     = 3 -- Indicates an error in secondary/redundant systems.
local MAV_SEVERITY_WARNING   = 4 -- Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
local MAV_SEVERITY_INFO      = 6 -- Normal operational messages. Useful for logging. No action is required for these messages.

-- gcs_sendany error_type less than this 
local min_severity = MAV_SEVERITY_WARNING

-- Engine Types
local HIRTH_EFI_TYPE = 8

-- ESC Constants
local ESC_WARMUP_TIME = 3000
local SERVO_OUT_THRESHOLD = 1010
local ESC_RPM_THRESHOLD = 10
-- ******************* Variables *******************

local number_of_esc = 5 --default value for Volanti

-- Add a new table to store the warm-up end times for each ESC
local esc_warmup_end_time = {}

local srv_prv_telem_ms = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

local srv_number = {
    [1] = {"Motor1", 33},
    [2] = {"Motor2", 34},
    [3] = {"Motor3", 35},
    [4] = {"Motor4", 36},
    [5] = {"Motor5", 70},
    [6] = {"Motor6", 38},
    [7] = {"Elevator", 19},
    [8] = {"Rudder", 21},
    [9] = {"GPIO", -1},
    [10] = {"Script1", 94},
    [11] = {"Aileron", 4}
}

-- Due to ArduPilot's limitation of handling only 3 auth-IDs and 1 message during pre-arm checks, 
-- it is necessary to prioritize and manage the message that is displayed.
-- Error Types (prioritized in ascending order)
local ERR_TYPE = {
    PRM_TBL_ADD  = 1,    -- Parameter table addition error
    PRM_CFGADD   = 2,    -- Configuration parameter addition error
    PRM_INIT     = 3,    -- Parameter initialization error
    PRM_AC_TYPE  = 4,    -- Parameter aircraft type unknown error
    ESC_TLM_LST  = 5,    -- ESC telemetry lost error
    ESC_RPM_NIL  = 6,    -- ESC RPM nil error
    ESC_SRV_OUT  = 7,    -- ESC servo output error
    ESC_RPM_DROP = 8,    -- ESC RPM drop error
    ENG_CHT_NIL  = 9,    -- Engine CHT nil error
    ENG_CHT_LOW  = 10,   -- Engine CHT low error
    ENG_CHT_HIGH = 11,   -- Engine CHT high error
    GPS_SATCOUNT = 12    -- GPS satellite count low error
}

-- PARAM 1 : Error_type  - index of the error defined in ERR_TYPE
-- PARAM 2 : status      - holds status of multple instances (e.g. ESCs 1, 2, 3, 4 and 5)
-- PARAM 3 : prev_status - holds previous status of multple instances (e.g. ESCs 1, 2, 3, 4 and 5)
-- PARAM 4 : fltr        - filter(debounce) for the error. should be set to 0
-- PARAM 5 : fltr_th     - filter threshold, show how continuous the error should be before displaying it
-- PARAM 6 : severity    - severity of the error. Helps in setting the error message frequency
-- PARAM 7 : Message     - error message
local ERROR_TABLE = {
    {Error_type = ERR_TYPE.PRM_TBL_ADD,  mod = "PRM", status = 0, prev_status = 0, fltr = 0, fltr_th = 0, severity = MAV_SEVERITY_INFO, Message = "TABLE init err"},
    {Error_type = ERR_TYPE.PRM_CFG_ADD,  mod = "PRM", status = 0, prev_status = 0, fltr = 0, fltr_th = 0, severity = MAV_SEVERITY_INFO, Message = "CFG add err"},
    {Error_type = ERR_TYPE.PRM_INIT,     mod = "PRM", status = 0, prev_status = 0, fltr = 0, fltr_th = 0, severity = MAV_SEVERITY_INFO, Message = "INIT error"},
    {Error_type = ERR_TYPE.PRM_AC_TYPE,  mod = "PRM", status = 0, prev_status = 0, fltr = 0, fltr_th = 0, severity = MAV_SEVERITY_INFO, Message = "ACType unknown"},
    {Error_type = ERR_TYPE.ESC_TLM_LST,  mod = "ESC", status = 0, prev_status = 0, fltr = 0, fltr_th = 3, severity = MAV_SEVERITY_CRITICAL, Message = "TELEM LOST"},
    {Error_type = ERR_TYPE.ESC_RPM_NIL,  mod = "ESC", status = 0, prev_status = 0, fltr = 0, fltr_th = 3, severity = MAV_SEVERITY_CRITICAL, Message = "RPM nil"},
    {Error_type = ERR_TYPE.ESC_SRV_OUT,  mod = "ESC", status = 0, prev_status = 0, fltr = 0, fltr_th = 3, severity = MAV_SEVERITY_CRITICAL, Message = "Srv Out nil"},
    {Error_type = ERR_TYPE.ESC_RPM_DROP, mod = "ESC", status = 0, prev_status = 0, fltr = 0, fltr_th = 3, severity = MAV_SEVERITY_CRITICAL, Message = "RPM Drop"},
    {Error_type = ERR_TYPE.ENG_CHT_NIL,  mod = "CYL", status = 0, prev_status = 0, fltr = 0, fltr_th = 0, severity = MAV_SEVERITY_INFO, Message = "cht nil"},
    {Error_type = ERR_TYPE.ENG_CHT_LOW,  mod = "CYL", status = 0, prev_status = 0, fltr = 0, fltr_th = 0, severity = MAV_SEVERITY_INFO, Message = "cht low"},
    {Error_type = ERR_TYPE.ENG_CHT_HIGH, mod = "CYL", status = 0, prev_status = 0, fltr = 0, fltr_th = 0, severity = MAV_SEVERITY_INFO, Message = "cht high"},
    {Error_type = ERR_TYPE.GPS_SATCOUNT, mod = "GPS", status = 0, prev_status = 0, fltr = 0, fltr_th = 0, severity = MAV_SEVERITY_INFO, Message = "Satcount low"}
}

-- auxiliary authorisation id for this script
local auth_id = arming:get_aux_auth_id()
assert(auth_id, SCRIPT_NAME .. ": could not get a prearm check auth id")

-- ******************* Functions *******************

-- wrapper for gcs:send_text()
local function gcs_msg(severity, txt)
    if type(severity) == 'string' then
    -- allow just a string to be passed for simple/routine messages
        txt      = severity
        severity = MAV_SEVERITY_INFO
    end
    gcs:send_text(severity, string.format('%s: %s', SCRIPT_NAME, txt))
end

-- this function returns instances which had set_error()
local function convert(num)
    local result = {}
    local pos = 0
    
    while num > 0 do
        if (num & 1) == 1 then
            table.insert(result, pos)
        end
        num = num >> 1
        pos = pos + 1
    end
    return table.concat(result, ",")
end

-- sets error status, 
-- implements dbounce filter and 
-- sends error message if severity is greater than min_severity
local function set_error(error_type, index)
    local mask = 1 << index
    if (ERROR_TABLE[error_type].status & mask) == 0 then
        ERROR_TABLE[error_type].status = ERROR_TABLE[error_type].status | mask
        -- send error message if severity is greater than min_severity
        if ERROR_TABLE[error_type].severity < min_severity then
            if ERROR_TABLE[error_type].fltr >= ERROR_TABLE[error_type].fltr_th then
                severity = MAV_SEVERITY_INFO
            else
                severity = ERROR_TABLE[error_type].severity
            end
            gcs_msg(ERROR_TABLE[error_type].severity, ERROR_TABLE[error_type].mod .. " {" .. convert(ERROR_TABLE[error_type].status) .. "} FAILED - " .. ERROR_TABLE[error_type].Message)
            ERROR_TABLE[error_type].fltr = ERROR_TABLE[error_type].fltr + 1
        end
    end
end

-- clears error status
-- sends cleared message if severity is greater than min_severity
-- note - clearing error does not implement debounce filter
local function clear_error(error_type, index)
    local mask = (1 << index)
    if (ERROR_TABLE[error_type].status & mask) ~= 0 then
        ERROR_TABLE[error_type].status = ERROR_TABLE[error_type].status & ~mask
        if ERROR_TABLE[error_type].severity < min_severity then
            gcs_msg(ERROR_TABLE[error_type].severity, ERROR_TABLE[error_type].mod .. " {" .. convert(ERROR_TABLE[error_type].status) .. "} CLEARED - " .. ERROR_TABLE[error_type].Message)
        end
        ERROR_TABLE[error_type].fltr = 0
    end
end

-- raise prearm authentication error messages, if any
-- only high priority error_type message is displayed due to limitation in aux_auth functionality
-- bad_message length cannot exceed 34 characters (Total auth msg length = 42; from - "PreArm: ", 8)
local function print_prearm_error()
    local bad_messages = ""
    for _, error_data in ipairs(ERROR_TABLE) do
        if error_data.status ~= 0 then
            bad_messages = error_data.mod .. " {" .. convert(error_data.status) .. "} FAIL - " .. error_data.Message
            break
        end
    end
    if bad_messages ~= "" then
        arming:set_aux_auth_failed(auth_id, bad_messages)
    else
        arming:set_aux_auth_passed(auth_id)
    end
end

-- print error summary
local function print_error_summary()
    for _, error_data in ipairs(ERROR_TABLE) do
        if error_data.status ~= error_data.prev_status then
            if error_data.status ~= 0 then
                bad_messages = error_data.mod .. " {" .. convert(error_data.status) .. "} FAIL - " .. error_data.Message
                gcs_msg(MAV_SEVERITY_CRITICAL, bad_messages)
            else
                gcs_msg(MAV_SEVERITY_INFO, error_data.mod .. " CLEARED - " .. error_data.Message)
            end
            error_data.prev_status = error_data.status
        end
    end
end

-- main error handling function
local function check_errors()
    -- check for any prearm errors
    if not arming:is_armed() then
        print_prearm_error()
    end
    -- print error summary
    print_error_summary()
end

-- checks if aircraft has enginetype defined as Hirth EFI. This needs to be changed in the future.
local function check_aircraft_type()
    if params.EFI_TYPE:get() == HIRTH_EFI_TYPE then
        number_of_esc = 4
    else
        number_of_esc = 5
    end
    return true
end

-- Initialise config parameters used in this script
local function init_param()
    -- read all config parameters
    for param_name, param in pairs(params) do
        if not param:init(param_name) then
            set_error(ERR_TYPE.PRM_INIT, 1)
            gcs_msg(MAV_SEVERITY_WARNING, "Parameter" .. param_name .. " Init Failed")
            return false
        end
    end
    clear_error(ERR_TYPE.PRM_INIT, 1)

    return true
end

-- initialise script parameters
local function arming_check_init()
    -- check param fetch
    if not init_param() then
        return false
    end

    -- check aircraft Type
    if not check_aircraft_type() then
        set_error(ERR_TYPE.PRM_AC_TYPE, 1)
        gcs_msg(MAV_SEVERITY_WARNING, "Aircraft Type Check Failed")
        return false
    end
    
    gcs_msg(MAV_SEVERITY_INFO, "Script Version " .. SCRIPT_VERSION .. " Initialized")
    
    return true
end

-- Call this function whenever a motor starts running
local function esc_is_started(i)
    -- Set the warm-up end time for this ESC to 3 seconds from now
    esc_warmup_end_time[i] = millis() + ESC_WARMUP_TIME
end

-- Call this function whenever a motor stops running
local function esc_is_stopped(i)
    -- Clear the warm-up end time for this ESC
    esc_warmup_end_time[i] = nil
end

-- check all escs in a loop
local function esc_check_loop()
    for i = 1, number_of_esc  do
        local esc_last_telem_data_ms = esc_telem:get_last_telem_data_ms(i-1):toint()
        local esc_rpm = esc_telem:get_rpm(i-1)
        local servo_out = SRV_Channels:get_output_pwm(srv_number[i][2])
        -- Telem data timestamp check
        if not esc_last_telem_data_ms or esc_last_telem_data_ms == 0 or esc_last_telem_data_ms == srv_prv_telem_ms[i] then
            set_error(ERR_TYPE.ESC_TLM_LST, i)
        -- Nil check for RPM reading
        elseif not esc_rpm then
            set_error(ERR_TYPE.ESC_RPM_NIL, i)
        -- Nil check for servo output
        elseif not servo_out then
            set_error(ERR_TYPE.ESC_SRV_OUT, i)
        -- Telemetry data is fresh and valid
        else
            clear_error(ERR_TYPE.ESC_TLM_LST, i)
            clear_error(ERR_TYPE.ESC_RPM_NIL, i)
            clear_error(ERR_TYPE.ESC_SRV_OUT, i)
            -- If armed, check that the motor is actually turning when it is commanded to
            if arming:is_armed() then
                -- If the PWM is below the threshold, it is okay for the motor to be stopped
                if servo_out < SERVO_OUT_THRESHOLD then
                    esc_is_stopped(i)
                -- If the PWM has just gone above the threshold, start the warm-up timer
                elseif servo_out > SERVO_OUT_THRESHOLD and not esc_warmup_end_time[i]  then
                    esc_is_started(i)
                -- If the motor is running, and the warmup timer has expired, check that the motor is spinning
                elseif esc_warmup_end_time[i] and millis() > esc_warmup_end_time[i] then
                    if servo_out > SERVO_OUT_THRESHOLD and esc_rpm < ESC_RPM_THRESHOLD then
                        set_error(ERR_TYPE.ESC_RPM_DROP, i)
                    else
                        clear_error(ERR_TYPE.ESC_RPM_DROP, i)
                    end
                end
            end
        end
        -- Update srv_prv_telem_ms[i] if it had valid data this loop
        if esc_last_telem_data_ms and esc_last_telem_data_ms ~= 0 then
            srv_prv_telem_ms[i] = esc_last_telem_data_ms
        end
    end
end


-- main function to check status and gcs_send() if any error
local function update()
    efi_check_loop()
    esc_check_loop()

    check_errors()
end

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs_msg(MAV_SEVERITY_ERROR, "Internal Error: " .. err)
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, 200
end

local function script_exit()
    -- pre arm failure SCRIPT_NAME not Running
    arming:set_aux_auth_failed(auth_id, SCRIPT_NAME .. " Not Running")
    gcs_msg(MAV_SEVERITY_CRITICAL, "LUA SCRIPT EXIT   ... Need Reboot to Reinitialize")
end


-- ******************* Main *******************
if arming_check_init() then
    return protected_wrapper, 10000
end

script_exit()
