-- MACROS
SCRIPT_NAME = 'CX_BIT'

local msg = {
    -- MAVLink severity level definitions
    MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7},
}

-- wrapper for gcs:send_text(). Helps identify bit messages
function msg:send(severity, txt)
    if type(severity) == 'string' then
        -- allow just a string to be passed for simple/routine messages
        txt      = severity
        severity = self.MAV_SEVERITY.INFO
    end
    gcs:send_text(severity, string.format('%s: %s', SCRIPT_NAME, txt))
end

return msg
