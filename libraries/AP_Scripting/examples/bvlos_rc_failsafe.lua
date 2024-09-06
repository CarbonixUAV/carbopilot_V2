--[[
    File Name: bvlos_rc_failsafe.lua
    Description: This script provides relaxed handling of RC failsafe in BVLOS scenarios where the RC link is likely to have shorter range than the primary telemetry link, but at large distances, you are less likely to use the RC link.
]]

local MAV_SEVERITY_EMERGENCY = 0 -- System is unusable. This is a "panic" condition.
local MAV_SEVERITY_WARNING = 4 -- Indicates about a possible future error if this is not resolved within a given timeframe.
local MAV_SEVERITY_INFO = 6 -- Normal operational messages. Useful for logging. No action is required for these messages.

local PARAM_TABLE_KEY = 98
local PARAM_TABLE_PREFIX = "THR_FAIL_"

local UPDATE_RATE_HZ = 2

local auto_modes = {
    10, -- Auto
    12, -- Loiter
    14, -- Avoid_ADSB
    15, -- Guided
    25, -- Loiter to QLand
}

-- bind a parameter to a variable
local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

local function contains(table, element)
    for _, value in pairs(table) do
        if value == element then
            return true
        end
    end
    return false
end

-- get time in seconds since boot
local function get_time()
    return millis():tofloat() * 0.001
end

 -- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 13), 'could not add param table')

local THR_FAILSAFE = bind_param('THR_FAILSAFE')

--[[
  // @Param: THR_FAIL_RANGE
  // @DisplayName: Throttle Failsafe Range
  // @Description: Range, in meters, for which the throttle failsafe will be active
  // @Units: m
  // @Range: 0 100000
  // @User: Standard
--]]
local RANGE  = bind_add_param('RANGE', 1, -1)

-- flag to track if failsafe should be enabled
local failsafe_enabled = false

-- timestamp of last mode warning
local last_mode_warning = 0

-- update function, called at UPDATE_RATE_HZ
local function update()
    -- if range is negative, this feature is disabled. This does not restore the
    -- THR_FAILSAFE parameter to its original value. It just essentially
    -- disables the script. (this way the script doesn't interfere with the
    -- user's configuration by default)
    if RANGE:get() < 0 then
        return
    end

    local distance = 0
    local home = ahrs:get_home()
    local cur_loc = ahrs:get_location()

    if home and cur_loc then
        distance = home:get_distance(cur_loc)
    end

    local should_enable = true

    -- If we are far away, disable RC failsafe
    if distance > RANGE:get() then
        should_enable = false
        -- send a warning if we are far away and in a non-auto mode
        if get_time() - last_mode_warning > 10 then
            if not contains(auto_modes, vehicle:get_mode()) then
                gcs:send_text(MAV_SEVERITY_WARNING, "Warning: beyond RC range")
                last_mode_warning = get_time()
            end
        end
    end

    if should_enable and not failsafe_enabled then
        -- enable failsafe
        gcs:send_text(MAV_SEVERITY_INFO, "RC Failsafe Enabled")
        failsafe_enabled = true
        THR_FAILSAFE:set(1)
    elseif not should_enable and failsafe_enabled then
        -- disable failsafe
        gcs:send_text(MAV_SEVERITY_INFO, "RC Failsafe Disabled")
        failsafe_enabled = false
        THR_FAILSAFE:set(2)
    end
end

local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY_EMERGENCY, "Internal Error: " .. err)
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, 1000/UPDATE_RATE_HZ
end

-- start running update loop
return protected_wrapper()
