local cx_msg = require("msg")
local aircraft_type = require("aircraft")

KELVIN_CELSIUS_DIFF = 273.15

local Engine = {
    name = "Engine",

    -- These range is for pre-arm checks only. 
    -- Post-arm checks are set in the Mission-Planner 
    --   -> warning -> SETUP -> Advanced -> Warning Manager
    efi_chtmin = 100,   -- in celcius
    efi_chtmax = 150,   -- in celcius

    cht1 = 0,
    cht2 = 0,
}

-- Initialize the Engine module
function Engine:init()
    -- Volanti does not have Engine module
    if aircraft_type == "Volanti" then
        return
    end
    cx_msg:send(cx_msg.MAV_SEVERITY.INFO, self.name .. " init (" .. aircraft_type .. ")")
end

-- Update loop for Engine module
function Engine:update()
    
    local efi_state = efi:get_state()
    local cylinder_status = efi_state:cylinder_status()

    self.cht1 = cylinder_status:cylinder_head_temperature() - KELVIN_CELSIUS_DIFF
    self.cht2 = cylinder_status:cylinder_head_temperature2() - KELVIN_CELSIUS_DIFF

    return
end

-- Return error messages
function Engine:check_for_errors()
    -- Volanti does not have Engine module
    if aircraft_type == "Volanti" then
        return {}
    end
    
    local msgs = ""
    if self.cht1 > self.efi_chtmax or self.cht2 > self.efi_chtmax then
        msgs = self.name .. " CHT too hot"
    elseif self.cht1 < self.efi_chtmin or self.cht2 < self.efi_chtmin then
        msgs = self.name .. " CHT too cold"
    end

    if msgs == "" then
        return {}
    end
    return {msgs}
end

return Engine
