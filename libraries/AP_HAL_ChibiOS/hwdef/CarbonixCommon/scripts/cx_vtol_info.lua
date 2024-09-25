--[[
   Send VTOL timer and VTOL state to GCS using named floats
--]]

local RATE_HZ = 1

local vtol_time_s = 0
local last_arm_state = false

local last_warning_s = 0

local MAV_SEVERITY_ERROR = 3
local MAV_SEVERITY_WARNING = 4
local MAV_SEVERITY_INFO = 6

-- get time in seconds since boot
local function get_time()
    return millis():tofloat() * 0.001
end

local function update()
    local vtol_state = quadplane:in_vtol_mode()
    local assist_state = quadplane:in_assisted_flight()
    local arm_state = arming:is_armed()

    -- Clear timer on transition from disarmed to armed
    if arm_state and not last_arm_state then
        vtol_time_s = 0
    end
    last_arm_state = arm_state

    -- Increment timer if in VTOL mode
    if vehicle:get_likely_flying() and arm_state then
        if vtol_state then
            vtol_time_s = vtol_time_s + 1 / RATE_HZ
        -- Increment at half-rate if in assisted mode
        elseif assist_state then
            vtol_time_s = vtol_time_s + 1 / (RATE_HZ * 2)
        end
    end

    -- Send VTOL state and timer to GCS
    gcs:send_named_float("VTOLState", vtol_state and 1 or 0)
    gcs:send_named_float("VTOLTime", vtol_time_s)

    -- Send warning every 10s if in assisted mode
    if assist_state and (get_time() - last_warning_s) > 10 then
        gcs:send_text(MAV_SEVERITY_WARNING, "QASSIST")
        last_warning_s = get_time()
    end
end

gcs:send_text(MAV_SEVERITY_INFO, "VTOL Info loaded")

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY_ERROR, "Internal Error: " .. err)
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, math.floor(1000 / RATE_HZ)
end

-- start running update loop
return protected_wrapper()
