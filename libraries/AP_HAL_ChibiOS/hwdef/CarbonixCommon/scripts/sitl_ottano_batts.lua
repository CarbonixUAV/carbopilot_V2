--[[
Ottano battery simulation script for SITL

Simulates constant voltage for the avionics battery and the two monitors that
measure the 28V bus. It does not simulate current draw or drain. Instructors
can set these voltages directly during simulation to simulate various issues.

Generator failure (belt breakage or otherwise)
- Lower the avionics battery voltage in flight
- When avionics battery reaches a low enough value, drop the 28V bus to 25.2V
  - This simulates what happens when the backup battery kicks in
- Stop reducing the avionics battery voltage and start dropping the 28V bus
--]]
UPDATE_RATE_HZ = 10

-- Ensure the script is loaded in SITL only
assert(param:get('SIM_OPOS_LAT') ~= nil, string.format('%s was designed for SITL', SCRIPT_NAME))

-- Bind parameter utilities
local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- Set up EFI parameters
PARAM_TABLE_PREFIX = 'SIM_'
PARAM_TABLE_KEY = 21
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 9), 'could not add ' .. string.sub(PARAM_TABLE_PREFIX, 1, -2) .. ' param table')

-- Bind parameters
--[[
  // @Param: SIM_AV_BAT_VOLT
  // @DisplayName: Simulated Avionics Battery Voltage
  // @Description: Voltage of the avionics battery
  // @Range: 19 25
  // @Increment: 0.1
  // @Units: V
--]]
local AV_BAT_VOLT = bind_add_param('AV_BAT_VOLT', 1, 24.2)

--[[
  // @Param: SIM_BUS_28_VOLT
  // @DisplayName: Simulated 28V Bus Voltage
  // @Description: Voltage of the 28V bus
  // @Range: 24 30
  // @Increment: 0.1
  // @Units: V
--]]
local BUS_28_VOLT = bind_add_param('BUS_28_VOLT', 2, 28.0)

AV_BAT_IDX = 1
BUS_28_IDX1 = 5
BUS_28_IDX2 = 6

local function update()
    local state = BattMonitorScript_State()
    state:healthy(true)
    state:voltage(AV_BAT_VOLT:get() or 0)
    battery:handle_scripting(AV_BAT_IDX, state)
    state:voltage(BUS_28_VOLT:get() or 0)
    battery:handle_scripting(BUS_28_IDX1, state)
    battery:handle_scripting(BUS_28_IDX2, state)
end

-- Wrapper to handle errors
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(0, "Internal Error: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, 1000 / UPDATE_RATE_HZ
end

-- Start the update loop
return protected_wrapper()
