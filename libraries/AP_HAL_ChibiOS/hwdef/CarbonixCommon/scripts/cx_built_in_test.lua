local cx_msg = require("msg")
local cx_esc = require("bit_esc")
local cx_gps = require("bit_gps")
local cx_engine = require("bit_engine")

-- Add subsystems that require Built-in-test (implemented in subsystems)
-- Each subsystem should have the following functions:
-- 1. init() - initialize the subsystem
-- 2. update() - update the subsystem, send in-flight error messages if needed
-- 3. check_for_errors() - returns pre-arm checks/errors in the subsystem
local subsystems = {
    cx_esc,
    cx_gps,
    cx_engine,
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
  // @Bitmask: 0:ESC, 1:GPS, 2:Engine
--]]
local PREARM_BYPASS = bind_add_param('PREARM_DIS', 1, 0)

-- fetches the aircraft configuration from the config file and prints the model name and version
local function get_aircraft_config(file_name)
    local model, model_version
    local file = io.open(file_name,"r")
    if not file then
        cx_msg:send(cx_msg.MAV_SEVERITY.INFO, "config file not found")
        return
    end

    logger:log_file_content(file_name)
    for line in file:lines() do
        if not model then
            local m = string.match(line, "<model>(.-)</model>")
            if m then model = m end
        end
        if not model_version then
            local v = string.match(line, "<model_version>(.-)</model_version>")
            if v then model_version = v end
        end
        if model and model_version then
            cx_msg:send(cx_msg.MAV_SEVERITY.INFO, "config (" .. model .. "_" .. model_version .. ".xml)")
            break
        end
    end
    file:close()

    if not model or not model_version then
        cx_msg:send(cx_msg.MAV_SEVERITY.INFO, "Incomplete model info in config")
    end
end 
    
-- initialize function
local AIRCRAFT_CONFIG_PATH = "@ROMFS/AircraftConfiguration.xml"

local function init()
    get_aircraft_config(AIRCRAFT_CONFIG_PATH)

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
