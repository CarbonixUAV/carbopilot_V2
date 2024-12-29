--[[ GPS Prearm Checks

This module is scrictly for pre-arm checks. No warning messages are sent to the
GCS. GPS status messages have enough information, and update quickly enough, to
handle alerts on the GCS side.

--]]

local cx_msg = require("msg")

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

return GPS
