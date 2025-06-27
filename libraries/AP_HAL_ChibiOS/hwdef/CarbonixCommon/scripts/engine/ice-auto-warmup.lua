--[[
Automatic warmup sequence for the ICE engine. 

Once the engine is started, this script will modify the Idle governor setpoint
to control the engine RPM until the CHT (minimum of the two cylinders) reached
the desired value. Once the warmup CHT is reached the script reverts back to the
nominal governor setpoint.
--]]

local UPDATE_HZ = 1

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

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

local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

--Setup Warmup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'could not add param table')

--[[
  // @Param: WARMUP_ENABLED
  // @DisplayName: Warmup Enabled
  // @Description: Enables the automatic warmup sequence
  // @Values: 0:Disabled, 1:Enabled
--]]
local WARMUP_ENABLED  = bind_add_param('ENABLED', 1, 1)

--[[
  // @Param: WARMUP_MIDTEMP
  // @DisplayName: Warmup Mid Temp
  // @Description: Temperature at which the engine will increase RPM to WARMUP_HIGHRPM
  // @Range: 25 100
  // @Units: degC
--]]
local WARMUP_MIDTEMP = bind_add_param('MIDTEMP', 2, 60)

--[[
  // @Param: WARMUP_ENDTEMP
  // @DisplayName: Warmup End Temp
  // @Description: Temperature at which the warmup sequence is complete
  // @Range: 100 130
  // @Units: degC
--]]
local WARMUP_ENDTEMP  = bind_add_param('ENDTEMP', 3, 120)

--[[
  // @Param: WARMUP_RPM
  // @DisplayName: Warmup RPM
  // @Description: RPM to increase to when between WARMUP_MIDTEMP and WARMUP_ENDTEMP
  // @Range: 2400 4000
  // @Units: RPM
--]]
local WARMUP_RPM  = bind_add_param('RPM', 4, 3200)


local ICE_IDLE_RPM = bind_param("ICE_IDLE_RPM")
local idle_rpm = ICE_IDLE_RPM:get() or 2400


local function Kelvin_to_C (temp)
    return (temp - 273.15)
end

--- Handle the warmup state machine. This function determins when to move on to
--- the next state of the warmup process, updates the governor setpoint, and
--- notifies the GCS on each state change.
---@param cht number The cylinder head temperature in degrees Celsius.
---@param state_in number The current state of the warmup process (e.g., STATUS_IDLE, STATUS_STEP).
---@return number state_out next warmup state (e.g., STATUS_IDLE, STATUS_DONE).
local function Warmup(cht, state_in)
    -- Do a very early return if done
    if state_in == STATUS_DONE then
        return STATUS_DONE
    end

    local warmup_rpm = WARMUP_RPM:get()
    local mid_temp = WARMUP_MIDTEMP:get()
    local end_temp = WARMUP_ENDTEMP:get()

    -- These can't actually be nil with the way we constructed the param variables,
    -- but this keeps the linter happy
    assert(warmup_rpm, "ICE Warmup: error reading parameters")

    if state_in == STATUS_NOT_DONE then
        gcs:send_text(MAV_SEVERITY.INFO, "ICE Warmup: Started")
        -- Don't return yet; handle the first state
    end
    if cht < mid_temp and state_in <= STATUS_IGNITION then
        gcs:send_text(MAV_SEVERITY.INFO, "ICE Warmup: Idle RPM")
        ICE_IDLE_RPM:set(idle_rpm)
        return STATUS_IDLE
    elseif cht >= mid_temp and cht < end_temp and state_in <= STATUS_IDLE then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("ICE Warmup: %d RPM", warmup_rpm))
        ICE_IDLE_RPM:set(warmup_rpm)
        return STATUS_STEP
    elseif cht >= end_temp and state_in <= STATUS_STEP then
        gcs:send_text(MAV_SEVERITY.INFO, "ICE Warmup: Done")
        ICE_IDLE_RPM:set(idle_rpm)
        return STATUS_DONE
    end
    return state_in
end

-- main update function
local function update()

    --Only run if warmup is enabled and Disarmed
    if (WARMUP_ENABLED:get() == 0 or arming:is_armed()) then
        if Warmup_Status ~= STATUS_DONE then
            Warmup_Status = STATUS_DONE
            ICE_IDLE_RPM:set(idle_rpm)
        end
        return
    end

    local engine = efi:get_state()
    local cylinder_status = engine:cylinder_status()
    local cht1 = Kelvin_to_C(cylinder_status:cylinder_head_temperature())
    local cht2 = Kelvin_to_C(cylinder_status:cylinder_head_temperature2())
    local min_cht = math.min(cht1, cht2)
    local engine_running = engine:engine_speed_rpm() > 300

    --Check if engine is running
    if engine_running then
        Warmup_Status = Warmup(min_cht, Warmup_Status)
    else
        --Reset state if Engine Stops
        Warmup_Status = STATUS_NOT_DONE
    end
end

--wrap to handle errors
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "Internal Error: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, math.floor(1000 / UPDATE_HZ)
end

gcs:send_text(MAV_SEVERITY.INFO, "ICE Warmup: Loaded")

--start update loop
return protected_wrapper()
