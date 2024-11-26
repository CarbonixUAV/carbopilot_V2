local cx_msg = require("scripts.cx_utils.cx_msg")
local cx_esc = require("scripts.cx_modules.esc")

-- Create instances of modules
cx_msg = cx_msg.new()

-- Add modules that require Built-in-test (implemented in cx_modules)
-- Each module should have the following functions:
-- 1. new() - constructor
-- 2. init() - initialize the module
-- 3. update() - update the module
-- 4. check_for_errors() - returns pre-arm checks/errors in the module
--                       - built in test errors are managed by each module
local modules = {
    esc = cx_esc.new()
}

-- ******************* Functions *******************
-- initialize function
local function init()
    -- initialize all modules
    for _, module in pairs(modules) do
        module:init()
    end

    cx_msg:send(cx_msg.MAV_SEVERITY_INFO, "LUA script initialized")
    return true
end

-- Pre-arm status check before arming
local function check_prearm_status()
    -- Track errors in modules
    local modules_with_errors = {}
    local total_errors = 0

    for _, module in pairs(modules) do
        local error_count = module:check_for_errors()
        if error_count > 0 then
            table.insert(modules_with_errors, { name = module.name, errors = error_count })
            total_errors = total_errors + error_count
        end
    end

    -- Handle error cases
    if #modules_with_errors == 0 then
        -- No modules have errors, clear error flag
        cx_msg:clear_prearm_error()
    elseif #modules_with_errors == 1 then
        -- Only one module has errors
        local module = modules_with_errors[1]
        msg = module.name .. " (" .. module.errors .. " checks fail)"
        cx_msg:set_prearm_error(msg)
    else
        -- More than one module has errors
        msg = #modules_with_errors .. " modules (" .. total_errors .. " checks) failing. Check messages"
        cx_msg:set_prearm_error(msg)
    end
end

-- update function
local function update()
    -- update all modules
    for _, module in pairs(modules) do
        module:update()
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
        cx_msg:send(cx_msg.MAV_SEVERITY_ERROR, "Internal Error: " .. err)
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
    cx_msg:send(cx_msg.MAV_SEVERITY_CRITICAL, "LUA SCRIPT EXIT   ... Need Reboot to Reinitialize")
end


-- ******************* Main *******************
if init() then
    return protected_wrapper, 10000
end

script_exit()
