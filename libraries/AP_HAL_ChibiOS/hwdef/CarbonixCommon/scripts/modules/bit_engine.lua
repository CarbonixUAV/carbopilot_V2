local cx_msg = require("msg")
local aircraft_type = require("aircraft")

KELVIN_CELSIUS_DIFF = 273.15

local Engine = {
    name = "Engine",

    has_engine = false, -- defaults off, can be set to true by init()
    has_started = false, -- whether the engine has started at least once

    -- Limits from the integration manual for the Hirth 4103. The deviation
    -- limits have a max for long-term and a higher max for up to 30 seconds.
    -- For simplicity, I have opted to use only the long-term max.
    CHT_MIN = 100,
    CHT_MAX = 280,
    CHT_DEVIATION = 50,
    EGT_MAX = 720,
    EGT_DEVIATION = 50,

    -- Threshold for the full throttle ground runup
    RUNUP_RPM_THRESHOLD = 6700,

    cht1 = 0,
    cht2 = 0,
    egt1 = 0,
    egt2 = 0,
    max_rpm = 0, -- max RPM seen

    WARN_TIME_MS = 15000, -- time to wait between in-flight warnings
    last_warn_time_ms = 0, -- last time we sent a warning message
}

-- Initialize the Engine module
function Engine:init()
    -- Volanti does not have Engine module
    if aircraft_type == "Volanti" then
        return
    end
    self.has_engine = true
    cx_msg:send(cx_msg.MAV_SEVERITY.INFO, self.name .. " init (" .. aircraft_type .. ")")
end

-- Update loop for Engine module
function Engine:update()
    if not self.has_engine then
        return
    end

    local efi_state = efi:get_state()
    local cylinder_status = efi_state:cylinder_status()

    self.cht1 = cylinder_status:cylinder_head_temperature() - KELVIN_CELSIUS_DIFF
    self.cht2 = cylinder_status:cylinder_head_temperature2() - KELVIN_CELSIUS_DIFF
    self.egt1 = cylinder_status:exhaust_gas_temperature() - KELVIN_CELSIUS_DIFF
    self.egt2 = cylinder_status:exhaust_gas_temperature2() - KELVIN_CELSIUS_DIFF

    if not self.has_started then
        self.has_started = efi_state:engine_speed_rpm() > 0
    end

    if efi_state:engine_speed_rpm() > self.max_rpm then
        self.max_rpm = efi_state:engine_speed_rpm():toint()
    end

    -- Send in-flight warnings if any errors are returned from check_for_errors()
    if arming:is_armed() and (millis() - self.last_warn_time_ms) > self.WARN_TIME_MS then
        local error_msg = self:check_for_errors()
        if #error_msg > 0 then
            self.last_warn_time_ms = millis()
            cx_msg:send(cx_msg.MAV_SEVERITY.ERROR, error_msg[1])
        else
            self.last_warn_time_ms = 0 -- reset if no errors
        end
    end
end

-- Generate warning messages for pre-arm checks and in-flight errors
-- Called by cx_built_in_test.lua and by Engine:update()
function Engine:check_for_errors()
    if not self.has_engine then
        return {}
    end

    -- Don't run the prearm checks until the engine has initially started.
    if not self.has_started then
        return {}
    end

    local msgs = ""

    if math.max(self.cht1, self.cht2) > self.CHT_MAX or math.max(self.egt1, self.egt2) > self.EGT_MAX then
        msgs = self.name .. " hot"
    elseif math.min(self.cht1, self.cht2) < self.CHT_MIN then
        msgs = self.name .. " cold"
    elseif math.abs(self.cht1 - self.cht2) > self.CHT_DEVIATION then
        local cold_cylinder = (self.cht1 < self.cht2) and "1" or "2"
        msgs = self.name .. " CHT" .. cold_cylinder .. " cold"
    elseif math.abs(self.egt1 - self.egt2) > self.EGT_DEVIATION then
        local cold_cylinder = (self.egt1 < self.egt2) and "1" or "2"
        msgs = self.name .. " EGT" .. cold_cylinder .. " cold"
    elseif (not arming:is_armed()) and (self.max_rpm < self.RUNUP_RPM_THRESHOLD) then
        msgs = self.name .. " needs runup to " .. self.RUNUP_RPM_THRESHOLD .. " RPM"
    end

    if msgs == "" then
        return {}
    end
    return {msgs}
end

return Engine
