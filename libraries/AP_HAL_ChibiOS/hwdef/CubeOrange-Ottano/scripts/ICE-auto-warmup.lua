-- This Script runs through an warmup up sequence for the ICE engine. 
-- Once the engine is started it will use the Idle governor to modify the engine RPM to desired value until CHT's reach desired value.
-- Once the CHT's reach desired value the script will switch to the normal governor and set the desired RPM to the normal value.

local UPDATE_HZ = 1

local MAV_SEVERITY_INFO = 6
local MAV_SEVERITY_NOTICE = 5
local MAV_SEVERITY_WARNING = 4
local MAV_SEVERITY_ERROR = 3
local MAV_SEVERITY_CRITICAL = 2
local MAV_SEVERITY_ALERT = 1
local MAV_SEVERITY_EMERGENCY = 0

gcs:send_text(MAV_SEVERITY_INFO, "ICE Warmup: Loaded")

--Track State of Warmup as to not print too many messages
local STATUS_NOT_DONE = 0
local STATUS_IGNITION = 1
local STATUS_IDLE = 2
local STATUS_STEP = 3
local STATUS_DONE = 4

local Warmup_Status = STATUS_NOT_DONE

-- Bind Param Utilities
local PARAM_TABLE_KEY = 69
local PARAM_TABLE_PREFIX = "WARMUP_"
local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

--Setup Warmup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 13), 'could not add param table')

--[[
  // @Param: WARMUP_ENABLED
  // @DisplayName: Warmup Enabled
  // @Description: Enables the automatic warmup sequence
  // @Range: 0 1
  // @Units: degC
--]]
local WARMUP_ENABLED  = bind_add_param('ENABLED', 1, 1)
--[[
  // @Param: WARMUP_STEP
  // @DisplayName: Warmup Step
  // @Description: Sets the temperature at which the engine will increase RPM to WARMUP_RPM
  // @Range: 25 100
  // @Units: degC
--]]
local WARMUP_STEP = bind_add_param('STEP', 2, 60)
--[[
  // @Param: WARMUP_DONE
  // @DisplayName: WARMUP_DONE
  // @Description: Temperature at which the warmup sequence is done
  // @Range: 100 130
  // @Units: degC
--]]
local WARMUP_DONE  = bind_add_param('DONE', 3, 120)
--[[
  // @Param: WARMUP_RPM
  // @DisplayName: WARMUP_RPM
  // @Description: RPM to increase to when WARMUP_STEP temperature is reached
  // @Range: 2400 4000
  // @Units: degC
--]]
local WARMUP_RPM  = bind_add_param('RPM', 4, 3000)

local ICE_IDLE_RPM = bind_param("ICE_IDLE_RPM")
local idle = ICE_IDLE_RPM:get()


local function Kelvin_to_C (temp)
    return (temp - 273.15)
end

local function Warmup(min_cht, Warmup_Status)
    local step_rpm = WARMUP_RPM:get()
    local first_step = WARMUP_STEP:get()
    local final_temp = WARMUP_DONE:get()

    if ICE_IDLE_RPM:get() == nil or step_rpm == nil then
        gcs:send_text(MAV_SEVERITY_INFO, "ICE Warmup: Check Idle governor params")
        return STATUS_NOT_DONE
    end

    if min_cht < first_step and Warmup_Status <= STATUS_IGNITION then
        gcs:send_text(MAV_SEVERITY_INFO, "ICE Warmup: Idle RPM")
        ICE_IDLE_RPM:set(idle)
        return STATUS_IDLE
    elseif min_cht >= first_step and min_cht < final_temp and Warmup_Status <= STATUS_IDLE then
        gcs:send_text(MAV_SEVERITY_INFO, string.format("ICE Warmup: %d RPM", step_rpm))
        ICE_IDLE_RPM:set(step_rpm)
        return STATUS_STEP
    elseif min_cht >= final_temp and Warmup_Status <= STATUS_STEP then
        gcs:send_text(MAV_SEVERITY_INFO, "ICE Warmup: Done")
        ICE_IDLE_RPM:set(idle)
        return STATUS_DONE
    end
    return Warmup_Status

end

-- main update function
local function update()
    local engine = efi:get_state()
    local cylinder_status = engine:cylinder_status()
    local cht1 = Kelvin_to_C(cylinder_status:cylinder_head_temperature())
    local cht2 = Kelvin_to_C(cylinder_status:cylinder_head_temperature2())
    local min_cht = math.min(cht1, cht2)
    local rpm = engine:engine_speed_rpm()

    --Only run if warmup is enabled and Disarmed
    if WARMUP_ENABLED:get() == 0 or arming:is_armed() then
        return
    end

    if cht1 == nil or cht2 == nil then
        gcs:send_text(MAV_SEVERITY_INFO, "ICE Warmup: CHT not available")
        return
    end
    --Check if engine is running
    if rpm > 2000 and Warmup_Status == STATUS_NOT_DONE then
        Warmup_Status = STATUS_IGNITION
        gcs:send_text(MAV_SEVERITY_INFO, "ICE Warmup: Started")
        return
    elseif rpm < 2000 then
        --Reset state if Engine Stops
        Warmup_Status = STATUS_NOT_DONE
    end
    
    Warmup_Status = Warmup(min_cht, Warmup_Status)
end

--wrap to handle errors
function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY_ERROR, "Internal Error: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, math.floor(1000 / UPDATE_HZ)
end

--start update loop
return protected_wrapper()
