-- MACROS
SCRIPT_NAME = 'CX_BIT'

local aircraft_type = require("aircraft")


local cx_msg = {
    -- MAVLink severity level definitions
    MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7},
}

-- wrapper for gcs:send_text(). Helps identify bit messages
function cx_msg:send(severity, txt)
    if type(severity) == 'string' then
        -- allow just a string to be passed for simple/routine messages
        txt      = severity
        severity = self.MAV_SEVERITY.INFO
    end
    gcs:send_text(severity, string.format('%s: %s', SCRIPT_NAME, txt))
end


local ESC = {
    name = "ESC",

    number_of_esc = 5,

    -- CONSTANTS
    ESC_WARMUP_TIME = 3000,
    ESC_RPM_THRESHOLD = 10,
    SERVO_OUT_THRESHOLD = 1010,

    -- Add a new table to store the warm-up end times for each ESC
    esc_warmup_end_time = {},

    srv_prv_telem_ms = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    srv_telem_in_err_status  = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false},
    srv_rpm_in_err_status  = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false},

    -- Counters to debounce nil checks on esc rpm and servo output, this is a
    -- workaround to avoid giving the pilot a critical warning for an unexplained
    -- one-loop dropout we saw recently
    NIL_WARN_THRESHOLD = 3,
    esc_rpm_nil_counter = {0, 0, 0, 0, 0},
    servo_out_nil_counter = {0, 0, 0, 0, 0},

    srv_number = {
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
    },
}

-- Get the number of ESCs based on the aircraft type
function ESC:get_num_esc()
    if aircraft_type == "Volanti" then
        self.number_of_esc = 5
    elseif aircraft_type == "Ottano" then
        self.number_of_esc = 4
    else
        assert(false, "ESC init failed: Aircraft type not set")
    end
end

-- Initialize the ESC module
function ESC:init()
    self:get_num_esc()
    if self.number_of_esc == 0 then
        cx_msg:send(cx_msg.MAV_SEVERITY.CRITICAL, "ESC init failed: Aircraft type not set")
        return
    end
    for i = 1, self.number_of_esc do
        self.esc_warmup_end_time[i] = nil
        self.srv_prv_telem_ms[i] = 0
    end
    cx_msg:send(cx_msg.MAV_SEVERITY.INFO, "ESC init (" .. aircraft_type .. ": " .. self.number_of_esc .. " ESCs)")
end


-- Call this function whenever a motor starts running
function ESC:esc_is_started(i)
    -- Set the warm-up end time for this ESC to 3 seconds from now
    self.esc_warmup_end_time[i] = millis() + self.ESC_WARMUP_TIME
end

-- Call this function whenever a motor stops running
function ESC:esc_is_stopped(i)
    -- Clear the warm-up end time for this ESC
    self.esc_warmup_end_time[i] = nil
end

function ESC:update()
    -- When the safety is engaged, the ESCs do not output telemetry
    if SRV_Channels:get_safety_state() then
        -- Reset all the counters and flags
        for i = 1, self.number_of_esc do
            self.esc_warmup_end_time[i] = nil
            self.srv_prv_telem_ms[i] = 0
            self.srv_telem_in_err_status[i] = false
            self.srv_rpm_in_err_status[i] = false
            self.esc_rpm_nil_counter[i] = 0
            self.servo_out_nil_counter[i] = 0
        end
        return
    end

    -- check for errors
    for i = 1, self.number_of_esc  do
        local esc_last_telem_data_ms = esc_telem:get_last_telem_data_ms(i-1):toint()
        local esc_rpm = esc_telem:get_rpm(i-1)
        local servo_out = SRV_Channels:get_output_pwm(self.srv_number[i][2])
        -- Telem data timestamp check
        if not esc_last_telem_data_ms or esc_last_telem_data_ms == 0 or esc_last_telem_data_ms == self.srv_prv_telem_ms[i] then
            if self.srv_telem_in_err_status[i] == false then
                cx_msg:send(cx_msg.MAV_SEVERITY.CRITICAL, "ESC " .. i .. " Telemetry Lost")
                self.srv_telem_in_err_status[i] = true
            end
        -- Nil check for RPM reading
        elseif not esc_rpm then
            self.esc_rpm_nil_counter[i] = self.esc_rpm_nil_counter[i] + 1
            if self.esc_rpm_nil_counter[i] >= self.NIL_WARN_THRESHOLD and self.srv_rpm_in_err_status[i] == false then
                cx_msg:send(cx_msg.MAV_SEVERITY.CRITICAL, "ESC " .. i .. " Telemetry Lost")
                self.srv_telem_in_err_status[i] = true
            end
        -- Nil check for servo output
        elseif not servo_out then
            self.servo_out_nil_counter[i] = self.servo_out_nil_counter[i] + 1
            if self.servo_out_nil_counter[i] >= self.NIL_WARN_THRESHOLD and self.srv_rpm_in_err_status[i] == false then
                cx_msg:send(cx_msg.MAV_SEVERITY.CRITICAL, "ESC " .. i .. " Telemetry Lost")
                self.srv_telem_in_err_status[i] = true
            end
        -- Telemetry data is fresh and valid
        else
            self.servo_out_nil_counter[i] = 0
            self.esc_rpm_nil_counter[i] = 0
            if self.srv_telem_in_err_status[i] == true then
                cx_msg:send(cx_msg.MAV_SEVERITY.INFO, "ESC " .. i .. " Telemetry Recovered")
                self.srv_telem_in_err_status[i] = false
            end
            -- If armed, check that the motor is actually turning when it is commanded to
            if arming:is_armed() then
                -- If the PWM is below the threshold, it is okay for the motor to be stopped
                if servo_out < self.SERVO_OUT_THRESHOLD then
                    self:esc_is_stopped(i)
                -- If the PWM has just gone above the threshold, start the warm-up timer
                elseif servo_out > self.SERVO_OUT_THRESHOLD and not self.esc_warmup_end_time[i]  then
                    self:esc_is_started(i)
                -- If the motor is running, and the warmup timer has expired, check that the motor is spinning
                elseif self.esc_warmup_end_time[i] and millis() > self.esc_warmup_end_time[i] then
                    if servo_out > self.SERVO_OUT_THRESHOLD and esc_rpm < self.ESC_RPM_THRESHOLD then
                        if self.srv_rpm_in_err_status[i] == false then
                            cx_msg:send(cx_msg.MAV_SEVERITY.CRITICAL, "ESC " .. i .. " RPM Drop")
                            self.srv_rpm_in_err_status[i] = true
                        end
                    else
                        if self.srv_rpm_in_err_status[i] == true then
                            cx_msg:send(cx_msg.MAV_SEVERITY.INFO, "ESC " .. i .. " RPM Recovered")
                            self.srv_rpm_in_err_status[i] = false
                        end
                    end
                end
            else
                self:esc_is_stopped(i)
            end
        end
        -- Update srv_prv_telem_ms[i] if it had valid data this loop
        if esc_last_telem_data_ms and esc_last_telem_data_ms ~= 0 then
            self.srv_prv_telem_ms[i] = esc_last_telem_data_ms
        end
    end
end

-- Return error messages
function ESC:check_for_errors()
    for _, status in ipairs(self.srv_telem_in_err_status) do
        if status then
            return {"ESC Telemetry Lost"}
        end
    end
    return {}
end


local GPS = {
    name = "GPS",

    N_GPS = 2,

    MIN_SATS = 18,
    MAX_DIFF = 8,

    sat_count = {0, 0},
    fix_type = {0, 0},
}

function GPS:init()
    cx_msg:send(cx_msg.MAV_SEVERITY.INFO, self.name .. " init")
end

function GPS:update()
    for i = 1, self.N_GPS do
        if i > gps:num_sensors() then
            self.sat_count[i] = 0
            self.fix_type[i] = 0
        else
            self.sat_count[i] = gps:num_sats(i - 1)
            self.fix_type[i] = gps:status(i - 1)
        end
    end
end

function GPS:check_for_errors()
    local max_sat_count = 0
    for i = 1, self.N_GPS do
        if self.sat_count[i] > max_sat_count then
            max_sat_count = self.sat_count[i]
        end
    end
    local low_sat_count = {}
    for i = 1, self.N_GPS do
        -- We don't need to complain about sat count if we don't have a fix.
        -- ArduPilot's existing checks will handle that for us.
        if self.fix_type[i] >= gps.GPS_OK_FIX_3D then
            if self.sat_count[i] < self.MIN_SATS or max_sat_count - self.sat_count[i] > self.MAX_DIFF then
                table.insert(low_sat_count, i)
            end
        end
    end
    if #low_sat_count == 0 then
        return {}
    elseif #low_sat_count == 1 then
        return {self.name .. " " .. low_sat_count[1] .. " low satellite count"}
    else
        return {self.name .. " low satellite counts"}
    end
end

-- Add subsystems that require Built-in-test (implemented in subsystems)
-- Each subsystem should have the following functions:
-- 1. init() - initialize the subsystem
-- 2. update() - update the subsystem
-- 3. check_for_errors() - returns pre-arm checks/errors in the subsystem
--                       - built in test errors are managed by each subsystem
local subsystems = {
    cx_esc,
    cx_gps,
}

-- auth id for prearm check
local prearm_msg = nil
local auth_id = arming:get_aux_auth_id()
assert(auth_id, SCRIPT_NAME .. ": could not get prearm check auth id")

-- ******************* Functions *******************
-- get time in seconds since boot
local function get_time()
    return millis():tofloat() * 0.001
end

local function set_prearm_error(txt)
    if (not prearm_msg) or (prearm_msg ~= txt) then
        prearm_msg = txt
        arming:set_aux_auth_failed(auth_id, txt)
    end
end

local function clear_prearm_error()
    prearm_msg = nil
    arming:set_aux_auth_passed(auth_id)
end

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
PARAM_TABLE_PREFIX = 'BIT_'
PARAM_TABLE_KEY = 1
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1), 'could not add ' .. string.sub(PARAM_TABLE_PREFIX, 1, -2) .. ' param table')
--[[
  // @Param: BIT_PREARM_DIS
  // @DisplayName: Built-In-Test Prearm Bypass Mask
  // @Description: Allows bypassing prearm checks for individual subsystems
  // @Bitmask: 0:ESC, 1:GPS
--]]
local PREARM_BYPASS = bind_add_param('PREARM_DIS', 1, 0)

-- initialize function
local function init()
    -- initialize all subsystems that are part of constructor
    for _, subsystem in pairs(subsystems) do
        subsystem:init()
    end

    cx_msg:send(cx_msg.MAV_SEVERITY.INFO, "LUA script initialized")
    return true
end

-- Pre-arm status check before arming
local last_prearm_msg_s = 0 -- timestamp of last message sent
local prearm_messages = {} -- Set of all unique messages seen since we last sent
local function check_prearm_status()
    -- Track errors in subsystems
    local subsystems_with_errors = {}
    local msg = ""
    local disabled_mask = PREARM_BYPASS:get() or 0
    for i, subsystem in pairs(subsystems) do
        local errors = {}
        if disabled_mask & (1 << (i - 1)) == 0 then
            errors = subsystem:check_for_errors()
        end
        if #errors > 0 then
            table.insert(subsystems_with_errors, subsystem.name)
            for _, error in pairs(errors) do
                prearm_messages[error] = true
            end
            if #errors == 1 then
                msg = errors[1]
            else
                msg = errors .. " " .. subsystem.name .. " errors. Check messages."
            end
        end
    end

    -- Handle prearm
    if #subsystems_with_errors == 0 then
        clear_prearm_error()
    elseif #subsystems_with_errors == 1 then
        set_prearm_error(msg)
    else
        msg = ""
        for _, subsystem in pairs(subsystems_with_errors) do
            msg = msg .. subsystem .. ", "
        end
        msg = msg:sub(1, -3) .. " failing. Check messages."
        set_prearm_error(msg)
    end

    -- Every 2 seconds, send a message with all the unique errors seen. This
    -- helps the operator see the specific errors if there are more than one
    -- (since the prearm library only allows one error message for all scripts
    -- to share)
    if get_time() - last_prearm_msg_s > 2 or get_time() < last_prearm_msg_s then
        -- Count the number of unique errors (# operator doesn't work on sets)
        local num_prearm_errors = 0
        for _ in pairs(prearm_messages) do
            num_prearm_errors = num_prearm_errors + 1
        end
        if num_prearm_errors > 1 then
            for err, _ in pairs(prearm_messages) do
                gcs:send_text(cx_msg.MAV_SEVERITY.CRITICAL, "Prearm: " .. err)
            end
        end
        last_prearm_msg_s = get_time()
        prearm_messages = {}
    end
end

-- update function
local function update()
    -- update all subsystems
    for _, subsystem in pairs(subsystems) do
        subsystem:update()
    end

    -- check for any prearm errors
    if not arming:is_armed() then
	    check_prearm_status()
	end
end

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        cx_msg:send(cx_msg.MAV_SEVERITY.ERROR, "Internal Error: " .. err)
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, 200
end

-- exit function
local function script_exit()
    -- pre arm failure SCRIPT_NAME not Running
    arming:set_aux_auth_failed(auth_id, SCRIPT_NAME .. " Not Running")
    cx_msg:send(cx_msg.MAV_SEVERITY.CRITICAL, "LUA SCRIPT EXIT   ... Need Reboot to Reinitialize")
end


-- ******************* Main *******************
if init() then
    return protected_wrapper, 10000
end

script_exit()
