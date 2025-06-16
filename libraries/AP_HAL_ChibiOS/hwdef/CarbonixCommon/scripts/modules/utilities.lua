--[[
Common utility functions for Lua scripts.

This module provides a number of commonly repeated functions in ArduPilot Lua
scripts. It is intended to reduce the amount of boilerplate code that needs to
be written in each script, and reduces the likelihood of errors.
]]
local utilities = {}

-- Binds a parameter to a variable
---@param name string -- The name of the parameter.
---@return Parameter_ud -- The initialized parameter object.
function utilities.bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    assert(p:get(), string.format('could not get a numeric value for %s', name))
    return p
end

-- Create a parameter table
---@param table_key number -- Arbitrary id. Must be unique among all loaded scripts.
---@param prefix string -- Prefix string for all parameters in the table.
---@param num_params number -- Number of parameters to reserve for the table.
function utilities.param_add_table(table_key, prefix, num_params)
    assert(utilities.param_table_key == nil and utilities.param_table_prefix == nil,
           'not supported: each script can only have one parameter table')
    assert(param:add_table(table_key, prefix, num_params),
           string.format('could not add table %s', table_key))
    utilities.param_table_key = table_key
    utilities.param_table_prefix = prefix
    utilities.param_index_max = num_params
    utilities.param_index = 1
end

-- Add a parameter and bind it to a variable. Must be called after
-- param_add_table.
---@param name string -- The name of the parameter.
---@param default_value number -- The default value of the parameter.
---@return Parameter_ud -- The initialized parameter object.
function utilities.bind_add_param(name, default_value)
    assert(utilities.param_table_key ~= nil and utilities.param_table_prefix ~= nil,
           'parameter table not initialized')
    assert(utilities.param_index <= utilities.param_index_max,
           'not enough parameters allocated')
    assert(param:add_param(utilities.param_table_key, utilities.param_index, name, default_value),
           string.format('could not add param %s', name))
    utilities.param_index = utilities.param_index + 1
    return utilities.bind_param(utilities.param_table_prefix .. name)
end

-- Get the current time in seconds
---@return number -- The current time in seconds.
function utilities.get_time_sec()
    return millis():tofloat() * 0.001
end

-- Wrap an angle to the range [-180, 180).
---@param angle number -- The angle to wrap.
---@return number -- The wrapped angle.
function utilities.wrap_180(angle)
    return math.fmod(angle + 180, 360) - 180
end

-- Partial reimplementation of the logic in Plane::relative_ground_altitude.
-- Someone should really add a binding for this.
---@param use_rangefinder boolean -- Whether to use the rangefinder if available.
---@param use_terrain boolean -- Whether to use terrain data if available.
---@return number -- The relative ground altitude in meters.
function utilities.relative_ground_altitude(use_rangefinder,
                                            use_terrain)
    local ROTATION_PITCH_270 = 25
    local RANGE_STATUS_GOOD = 4
    local in_range = rangefinder:status_orient(ROTATION_PITCH_270) == RANGE_STATUS_GOOD
    if use_rangefinder and in_range then
        return rangefinder:distance_cm_orient(ROTATION_PITCH_270) * 0.01
    end

    local height_above_terrain = terrain:height_above_terrain(false)
    if use_terrain and height_above_terrain then
        return height_above_terrain
    end

    -- We can't use the height_above_target() part of
    -- Plane::relative_ground_altitude because it needs to check that we are not
    -- quadplane.landing_with_fixed_wing_spiral_approach(), which is not bound
    -- in lua.
    local location = ahrs:get_location()
    -- If we can't get relative altitude, there's nothing we can do. Return 0.
    if not location or not location:change_alt_frame(1) then
        return 0
    end

    return location:alt() * 0.01
end

return utilities
