local cx_msg = {}
cx_msg.__index = cx_msg

-- MACROS
SCRIPT_NAME = 'CX_BIT'

-- MAVLink severity level definitions
cx_msg.MAV_SEVERITY_CRITICAL = 2
cx_msg.MAV_SEVERITY_ERROR = 3
cx_msg.MAV_SEVERITY_WARNING = 4
cx_msg.MAV_SEVERITY_INFO = 6

-- auth id for prearm check
function cx_msg.new()
    local self = setmetatable({}, cx_msg)
    self.prearm_msg = "nil"
    self.auth_id = arming:get_aux_auth_id()
    assert(self.auth_id, SCRIPT_NAME .. ": could not get prearm check auth id")
    return self
end

function cx_msg:set_prearm_error(msg)
    if self.prearm_msg == nil or self.prearm_msg ~= msg then
        self.prearm_msg = msg
        arming:set_aux_auth_failed(self.auth_id, msg)
        self:send(self.MAV_SEVERITY_WARNING, msg)
    end
end

function cx_msg:clear_prearm_error()
    if self.prearm_msg ~= nil then
        self.prearm_msg = nil
        arming:set_aux_auth_passed(self.auth_id)
        self:send(self.MAV_SEVERITY_INFO, "Prearm check passed")
    end
end

-- wrapper for gcs:send_text(). Helps identify cx_bit messages
function cx_msg:send(severity, txt)
    if type(severity) == 'string' then
        -- allow just a string to be passed for simple/routine messages
        txt      = severity
        severity = self.MAV_SEVERITY_INFO
    end
    gcs:send_text(severity, string.format('%s: %s', SCRIPT_NAME, txt))
end

return cx_msg
