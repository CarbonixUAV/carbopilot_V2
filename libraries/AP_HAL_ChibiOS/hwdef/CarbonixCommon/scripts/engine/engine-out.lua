--[[
Engine Failsafe Script

This script provides a best-effort return mechanism in case of a forward motor
loss on an aircraft. Upon detecting engine failure, characterized by drops in
RPM and vibration below certain thresholds, the script initiates a series of
actions to safely handle the situation.

Prearm checks:
- Check that the configured glide speed is within bounds.
- Check that the engine is running.

Engine-Out Bahavior:
- Send a warning to the ground station.
- Prioritize airspeed and set an optimal target speed for glide.
- Initiate an automatic return-to-launch (RTL) sequence to the nearest rally
  point or to the home position. The following actions are taken automatically:
    - Disable RTL_AUTOLAND to send the aircraft to a rally point or home.
    - Set Q_ASSIST_ALT to match Q_RTL_ALT.
    - Deactivate the "level transition" feature for more roll authority during
      Q_ASSIST, and additionally allowing VTOL motors to manage altitude.
    - When Q_ASSIST activates, the aircraft waits for the speed to drop below a
      threshold before switching to QLand or QRTL.
    - Should the aircraft arrive at the landing aread with excess altitude, it
      will orbit the area until it descends low enough to trigger Q_ASSIST.
    - When orbiting in Q_ASSIST, the aircraft will gradually spiral inward while
      reducing speed, transitioning to QRTL once it slows down sufficiently.
    - Safe landing areas can be manually indicated by switching to Guided mode.
      The script will immediately switch back to RTL, but override its
      destination with that of the Guided command.
- If manual control is assumed in any mode other than Guided, all automatic
  behaviors are suspended and the pilot assumes full responsibility for the rest
  of the landing unless they switch back to RTL or Guided.

Parameters:

    // @Param: ENGOUT_FS_ENABLE
    // @DisplayName: Engine-out failsafe enable
    // @Description: Enable/disable automated actions during engine out.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard

    // @Param: ENGOUT_GLIDE_SPD
    // @DisplayName: Engine-out glide speed
    // @Description: Optimal airspeed for gliding
    // @Range: 5 50
    // @Units: m/s
    // @User: Standard

    // @Param: ENGOUT_DELAY
    // @DisplayName: Engine-out detection threshold
    // @Description: Delay for detecting changes to the engine state.
    // @Range: 0.2 5
    // @Units: s
    // @User: Standard

    // @Param: ENGOUT_RPM_CHAN
    // @DisplayName: Engine-out RPM channel
    // @Description: Instance number of the engine's RPM sensor.
    // @Values: 0:None,1:RPM1,2:RPM2
    // @User: Standard

    // @Param: ENGOUT_RPM_THRSH
    // @DisplayName: Engine-out low-RPM threshold
    // @Description: RPM threshold for detecting engine stop.
    // @Range: 0 2000
    // @User: Standard

    // @Param: ENGOUT_VIB_THRSH
    // @DisplayName: Engine-out low-vibration threshold
    // @Description: Vibration threshold for detecting engine stop.
    // @Values: 10000:Disabled
    // @Range: 2 10
    // @Units: m/s/s
    // @User: Standard

    // @Param: ENGOUT_QAST_TIME
    // @DisplayName: Engine-out Q_ASSIST timeout
    // @Description: How long Q_ASSIST can be active before switching to QLand.
    // @Range: 5 300
    // @Units: s
    // @User: Standard

    // @Param: ENGOUT_QAST_GSPD
    // @DisplayName: Engine-out Q_ASSIST minimum ground speed
    // @Description: Low-ground speed threshold for switching to QLand or QRTL.
    // @Range: 0 15
    // @Units: m/s
    // @User: Standard

    // @Param: ENGOUT_QRTL_TIME
    // @DisplayName: Engine-out QRTL timeout
    // @Description: How long to wait for QRTL to start descending before aborting to QLand.
    // @Range: 20 300
    // @Units: s
    // @User: Standard

    // @Param: ENGOUT_AUX_FUNC
    // @DisplayName: Engine-out RC function
    // @Description: RCn_OPTION to assign to override engine-out detection. A low position means engine is definitely stopped, a high position means engine is definitely running, and mid position means detect as normal.
    // @Values: 0:Disabled,300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @User: Standard
--]]

-- Script initialization and parameter binding
local SCRIPT_NAME = "Engine Failsafe Script"
local PARAM_TABLE_KEY = 62 -- Arbitrary, but must be unique among all scripts loaded
local PARAM_TABLE_PREFIX = "ENGOUT_"
local utilities = require("utilities")
utilities.param_add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10)

-- Script parameters for engine failsafe behavior
local FS_ENABLE = utilities.bind_add_param("FS_ENABLE", 1)  -- Enable/disable automated actions during engine out
local GLIDE_SPD = utilities.bind_add_param("GLIDE_SPD", 0)  -- Optimal glide airspeed in m/s
local DELAY = utilities.bind_add_param("DELAY", 2)  -- Time to consider engine stopped after low RPM/vibe
local RPM_CHAN = utilities.bind_add_param("RPM_CHAN", 1)  -- RPM sensor channel
local RPM_THRSH = utilities.bind_add_param("RPM_THRSH", 500)  -- RPM threshold for engine stop
local VIB_THRSH = utilities.bind_add_param("VIB_THRSH", 4)  -- Vibration threshold for engine stop
local QAST_TIME = utilities.bind_add_param("QAST_TIME", 45)  -- Timeout to switch to QLand from Q_ASSIST
local QAST_GSPD = utilities.bind_add_param("QAST_GSPD", 5)  -- Minimum ground speed to switch to QRTL or QLand from Q_ASSIST
local QRTL_TIME = utilities.bind_add_param("QRTL_TIME", 60)  -- Timeout to switch to QLand from QRTL
local AUX_FUNC = utilities.bind_add_param("AUX_FUNC", 0)  -- RCn_OPTION number for overriding engine-out detection: stopped/auto/running

-- Existing parameters read by the script
local Q_RTL_ALT = utilities.bind_param("Q_RTL_ALT")  -- Altitude threshold landing
local Q_FW_LND_APR_RAD = utilities.bind_param("Q_FW_LND_APR_RAD")  -- Distance threshold for QRTL switch
local Q_OPTIONS = utilities.bind_param("Q_OPTIONS")  -- Q_OPTIONS bitmask, for adjusting the "level transition" option
local AIRSPEED_MAX = utilities.bind_param("AIRSPEED_MAX")  -- Max airspeed for glide check
local AIRSPEED_MIN = utilities.bind_param("AIRSPEED_MIN")  -- Min airspeed for glide check

-- Pre-arm-check backend for checking engine running and glide speed is within bounds
local arming_auth_id = arming:get_aux_auth_id()

-- Engine and failsafe status tracking
local q_rtl_start_time, q_assist_start_time, engine_change_time
local engine_stopped, triggered_failsafe = false, false

-- Location of the guided-mode override point
local guided_override

-- Flight mode constants
local MODE_RTL, MODE_GUIDED, MODE_QLAND, MODE_QRTL = 11, 15, 20, 21

-- Parameter group class for managing parameter backup and restoration
---@class ParamGroup
---@field params table<string, Parameter_ud> -- Storage for parameter objects
---@field backups table<string, number> -- Storage for backup values
local ParamGroup = {}
ParamGroup.__index = ParamGroup
function ParamGroup.new()
    local self = setmetatable({}, ParamGroup)
    self.params = {}  -- Storage for parameter objects
    self.backups = {}  -- Storage for backup values
    return self
end

-- Set a parameter's value and backup the original.
---@param name string -- Parameter name
---@param value number -- New value for the parameter
function ParamGroup:set_parameter(name, value)
    self:backup(name)
    self.params[name]:set(value)
end

-- Retrieve the backup value for a parameter (or the current value if no backup).
---@param name string -- Parameter name
---@return number|nil
function ParamGroup:get_backup(name)
    if not self.params[name] then
        self.params[name] = utilities.bind_param(name)
    end
    if self.backups[name] then
        return self.backups[name]
    else
        return self.params[name]:get()
    end
end

-- Backup the current value of a parameter.
---@param name string -- Parameter name
function ParamGroup:backup(name)
    if not self.params[name] then
        self.params[name] = utilities.bind_param(name)
    end
    if not self.backups[name] then
        self.backups[name] = self.params[name]:get()
    end
end

-- Restore a parameter from its backup.
---@param name string -- Parameter name
function ParamGroup:restore(name)
    if self.params[name] and self.backups[name] then
        self.params[name]:set(self.backups[name])
        self.backups[name] = nil
    end
end

-- Restore all parameters in the group from their backups.
function ParamGroup:restore_all()
    for name, _ in pairs(self.params) do
        self:restore(name)
    end
end

-- Instantiate parameter groups for different failsafe aspects
local tecsParamGroup, engineFailsafeParamGroup = ParamGroup.new(), ParamGroup.new()

-- Handle changing the `engine_stopped` flag, including delay.
---@param stop_detected boolean -- Whether the engine has been detected as stopped
local function set_engine_state(stop_detected)
    if engine_stopped == stop_detected then
        engine_change_time = nil
        return
    else
        engine_change_time = engine_change_time or utilities.get_time_sec()
    end
    -- Change the detected state of the engine after the timeout period has passed
    if utilities.get_time_sec() - engine_change_time > DELAY:get() then
        gcs:send_text(2, "Engine " .. (stop_detected and "out" or "running"))
        engine_stopped = stop_detected
        engine_change_time = nil
    end
end

local function check_engine()
    --[[
    Called periodically to check the engine status based on RPM and vibration
    readings. Updates the engine_stopped flag.

    Also, if configured, check if the pilot has manually indicated that they
    have manually determined that the engine is off or that it is on, overriding
    the automatic detection.
    --]]
    if not arming:is_armed() then
        set_engine_state(false) -- Consider the engine running if not armed
        return
    end

    -- Check if the pilot has manually overridden the engine detection
    local aux_func = AUX_FUNC:get()
    local aux_cached = aux_func and rc:get_aux_cached(aux_func)
    -- Pilot is sure that the engine is stopped
    if aux_cached and aux_cached == 0 then
        set_engine_state(true)
        return
    -- Pilot is sure that the engine is running
    elseif aux_cached and aux_cached == 2 then
        set_engine_state(false)
        return
    end

    local rpm = RPM:get_rpm(RPM_CHAN:get() - 1)
    local vibe = ahrs:get_vibration():length()

    -- If either RPM is high or vibe is high then assume engine is running
    if (rpm and (rpm > RPM_THRSH:get())) or (vibe > VIB_THRSH:get()) then
        set_engine_state(false)
        return
    end
    -- Otherwise, the engine is definitely not running
    set_engine_state(true)
end

local function switch_qland(reason)
    --[[
    Switch to QLand mode and send a warning to the ground station and an
    optional description of the specific reason for the switch.
    --]]
    if reason then
        gcs:send_text(6, reason .. ", switching to QLand")
    end
    gcs:send_text(2, "Failsafe: Landing QLand")
    vehicle:set_mode(MODE_QLAND)
end

local function switch_qrtl(reason)
    --[[
    Switch to QRTL mode and send a warning to the ground station and an optional
    description of the specific reason for the switch.
    
    We also override the target location with the current target location,
    because we may be switching from an RTL with an overridden target location.
    --]]

    if reason then
        gcs:send_text(6, reason .. ", switching to QRTL")
    end

    gcs:send_text(2, "Failsafe: Landing QRTL")
    vehicle:set_mode(MODE_QRTL)
end

local function do_engine_failsafe()
    --[[
    Trigger the RTL upon engine failure. This includes reconfiguring some
    parameters. This should only be called once to initiate the failsafe.
    --]]

    -- We don't want to do the mission based autoland, so disable RTL_AUTOLAND
    engineFailsafeParamGroup:set_parameter("RTL_AUTOLAND", 0)
    -- We don't want any automatic QRTL behavior
    engineFailsafeParamGroup:set_parameter("Q_RTL_MODE", 0)

    -- If we are already low, then RTL is a bad idea. Instead, switch to QLand
    local height_agl = utilities.relative_ground_altitude(true, true)
    if height_agl and height_agl < Q_RTL_ALT:get() then
        switch_qland("Engine out and below minimum altitude")
        return
    end

    -- Trigger an RTL to start bringing the vehicle or to the nearest rally point
    -- if set or go to home if no rally point available within the RALLY_LIMIT_KM
    vehicle:set_mode(MODE_RTL)
end

local function reset_target_alt()
    --[[ 
    Reset the target altitude to Q_RTL_ALT, and set it to the terrain altitude
    frame. Do this for QRTL and RTL during Q_ASSIST.

    For some reason, setting `TERRAIN_FOLLOW` to 1 does not guarantee that Q_RTL
    will target the correct altitude. This seems to be a bug, but I can't figure
    out the cause.
    --]]

    -- Don't reset the altitude during a final descent
    if quadplane:in_vtol_land_descent() then
        return
    end

    -- Only do this for QRTL, RTL with Q_ASSIST
    local mode = vehicle:get_mode()
    if mode ~= MODE_QRTL and not (mode == MODE_RTL and quadplane:in_assisted_flight()) then
        return
    end

    local old_target = vehicle:get_target_location()
    if not old_target then return end

    -- Check if the target is already at the correct altitude
    old_target:change_alt_frame(3)
    if old_target:alt() > Q_RTL_ALT:get() * 99 and old_target:alt() < Q_RTL_ALT:get() * 101 then
        return
    end

    gcs:send_text(6, "Resetting target altitude to " .. Q_RTL_ALT:get() .. "m")

    local new_target = old_target:copy()
    new_target:alt(Q_RTL_ALT:get() * 100)
    vehicle:update_target_location(old_target, new_target)
end

local function check_qrtl_timeout()
    --[[
    Called periodically to check if the vehicle should switch to QLand to abort
    a QRTL that is taking too long to switch to its descent phase.
    
    We only only perform this check when we are
      1. armed
      2. in QRTL
      3. and the VTOLs are spooled up
      4. the engine is stopped
    
    The state of the engine should not matter, but this function is called every
    update and this script should confine itself to cases where the engine is
    stopped.
    --]]

    -- Track how long we have been in QRTL with the engine stopped
    local now = utilities.get_time_sec()
    local should_run_timer = arming:is_armed()
                             and engine_stopped
                             and vehicle:get_mode() == MODE_QRTL 
                             and (quadplane:in_assisted_flight() or quadplane:in_vtol_mode())
    if should_run_timer then
        q_rtl_start_time = q_rtl_start_time or now
    else
        q_rtl_start_time = nil
        return
    end

    -- If we are in the descent phase, we don't need to check the timeout
    if quadplane:in_vtol_land_descent() then
        return
    end

    -- If we are in QRTL for too long, switch to QLand
    if q_rtl_start_time and (now - q_rtl_start_time) > QRTL_TIME:get() then
        switch_qland("QRTL for too long")
        return
    end
end

local function check_qland_qrtl()
    --[[
    Check if the vehicle should switch to QLand or QRTL. This function is called
    periodically when an engine failsafe has triggered and we are in RTL

    The vehicle will switch to QLand or QRTL in the following cases:
      1. If Q_ASSIST is active, and groundspeed is low. This is the intended
         typical case.
      2. If there is a problem reading the position or target, and the vehicle
         is low.
      3. If Q_ASSIST is active for an unusually long time.

    This function also controls the inward-spiral at the end of the descent by
    scaling the RTL_RADIUS down as the groundspeed reduces.
    --]]
    local height_agl = utilities.relative_ground_altitude(true, true)
    if not height_agl then
        --[[
        This should be unreachable as long as we have even relative altitude.
        Something is seriously wrong; I would rather we not try anything than
        try to land without knowing where we are or how high we are.
        --]]
        return
    end

    local target = vehicle:get_target_location() or ahrs:get_home()
    local pos = ahrs:get_location()

    -- If for some reason, we don't have a position, we can't do anything but go
    -- to QLand if we get below the minimum altitude.
    if not pos then
        if height_agl < 1.05*Q_RTL_ALT:get() then
            switch_qland("Missing position and below minimum altitude")
        end
        return
    end

    local groundspeed = ahrs:groundspeed_vector():length()
    local dist = pos and target:get_distance(pos)

    --[[
        If Q_ASSIST is active, start scaling down RTL_RADIUS with the reducing
        groundspeed, going toward zero radius when groundspeed equals the QRTL
        threshold. This lets us spiral inward as we gradually slow down. We
        scale it by the square of the groundspeed, going to zero when we reach
        the groundspeed threshold.
    --]]
    if q_assist_start_time then
        local original_radius = engineFailsafeParamGroup:get_backup("RTL_RADIUS")
        local ratio = (groundspeed^2 - QAST_GSPD:get()^2) / (GLIDE_SPD:get()^2 - QAST_GSPD:get()^2)
        local radius = original_radius * math.min(ratio, 1)
        engineFailsafeParamGroup:set_parameter("RTL_RADIUS", radius)

        --[[
        If we are inside of RTL_RADIUS (and the desired radius is too) and we
        are pointing toward the target, then we should switch to QRTL
        --]]
        if dist < original_radius and radius < original_radius then
            local bearing = math.deg(pos:get_bearing(target))
            local track = math.deg(ahrs:groundspeed_vector():angle())
            local angle_diff = utilities.wrap_180(bearing - track)
            if math.abs(angle_diff) < 20 then
                switch_qrtl("Close and pointing at target")
                return
            end
        end
    end

    local threshold = Q_FW_LND_APR_RAD:get()
    -- If the user has not set a fixed-wing approach radius, use RTL_RADIUS
    if threshold == 0 then
        threshold = engineFailsafeParamGroup:get_backup("RTL_RADIUS") * 1.25
    end
    threshold = math.abs(threshold or 0)
    local in_range = dist and dist < threshold

    --[[
    Land if we are in Q_ASSIST for too long, no matter what altitude. This is
    bad, but better than waiting forever. This is to catch situations where we
    are waiting forever for some condition to be met, either due to a bug or an
    unhandled edge-case.
    --]]
    local now = utilities.get_time_sec()
    if q_assist_start_time and (now - q_assist_start_time) > QAST_TIME:get() then
        if in_range then
            switch_qrtl("Q_ASSIST for too long")
        else
            switch_qland("Q_ASSIST for too long")
        end
        return
    end

    -- If we are in Q_ASSIST not making progress, then land
    if q_assist_start_time and (now - q_assist_start_time) > 2
       and groundspeed < QAST_GSPD:get() then
        if in_range then
            switch_qrtl("Q_ASSIST and no progress")
        else
            switch_qland("Q_ASSIST and no progress")
        end
        return
    end
end

local function pre_arm_checks()
    --[[
        Perform pre-arm checks:
          - Check that the glide speed is within bounds.
          - Check that the vibration threshold is not too low (making it very
            likely to incorrectly detect engine running).
          - Check that both RPM and vibration sensors detect the engine as
            running.
    --]]
    if arming:is_armed() or not arming_auth_id then
        return
    end

    -- Check if the glide speed is within bounds
    if GLIDE_SPD:get() <= 0 or GLIDE_SPD:get() < AIRSPEED_MIN:get() or GLIDE_SPD:get() > AIRSPEED_MAX:get() then
        local failure_message =
            PARAM_TABLE_PREFIX .. "GLIDE_SPD out of bounds"
        arming:set_aux_auth_failed(arming_auth_id, failure_message)
        return
    end

    -- Check if VIB_THRSH is unusually low
    if VIB_THRSH:get() <= 1 then
        local failure_message =
            PARAM_TABLE_PREFIX .. "VIB_THRSH is too low"
        arming:set_aux_auth_failed(arming_auth_id, failure_message)
        return
    end

    -- Check if the engine is running. If we are using two sources of engine
    -- detection, then we want to confirm both of them detect that the engine is
    -- running. This is the opposite of the in-flight check, where we only need
    -- one to detect the engine as running.
    local is_running = true
    local failure_message = "Engine not running: "
    if RPM_CHAN:get() > 0 then
        local rpm = RPM:get_rpm(RPM_CHAN:get() - 1)
        if (not rpm) or rpm <= RPM_THRSH:get() then
            failure_message = failure_message .. "low RPM"
            is_running = false
        end
    end
    if VIB_THRSH:get() < 200 then
        local vibe = ahrs:get_vibration():length()
        if vibe <= VIB_THRSH:get() then
            failure_message = failure_message .. (is_running and "" or ", ") .. "low vibration"
            is_running = false
        end
    end

    if not is_running then
        arming:set_aux_auth_failed(arming_auth_id, failure_message)
        return
    end

    -- If we get here, all checks passed
    arming:set_aux_auth_passed(arming_auth_id)
end

-- Configure TECS for glide, or restore it to normal operation
---@param glide boolean -- Whether to configure TECS for glide
local function configure_tecs(glide)
    if not glide then
        tecsParamGroup:restore_all()
        return
    end

    --[[
    Glide. We do this by setting the THR_MAX to 0. This is better than setting
    TECS_SPDWEIGHT to 2 for a couple reasons:
      - It sets the glide flag in the TECS, which should prevent the underspeed
        flag (which locks the airspeed to min) from triggering. 
      - It locks the throttle to idle. If this is somehow a false positive, the
        TECS behavior is weird when TECS_SPDWEIGHT is 2 and the engine is
        running. It's actually better to just idle the engine and glide.
    --]]
    tecsParamGroup:set_parameter("THR_MAX", 0)

    --[[
    We need to enable terrain *following* or else Q_ASSIST_ALT will not use the
    terrain altitude frame. This parameter does not relieve the script's burden
    of manually resetting all target altitudes into the terrain frame though.
    --]]
    tecsParamGroup:set_parameter("TERRAIN_FOLLOW", 1)

    --[[
    We need to set the "Disable Approach" bit to allow us to override the target
    location in QRTL. The target does not seem to be overridable during the
    approach phase (not sure if that's a bug or a feature).
    
    We also need to clear the "level transition" bit in Q_OPTIONS. This does two
    things:
      1. It allows the aircraft to bank to its normal maximum angle, which helps
         during our inward spiral in Q_ASSIST.
      2. It allows the VTOL motors to fully control the aircraft's desired
         altitude. This allows the aircraft to climb and descend with terrain,
         and prevents dangerous accidental descents.
    --]]
    local q_options = Q_OPTIONS:get()
    if q_options then
        q_options = q_options & (~0x1) -- Clear bit 0:level_transition
        q_options = q_options | 0x10000 -- Set bit 16:disable_approach
        tecsParamGroup:set_parameter("Q_OPTIONS", q_options)
    else
        -- Should be unreachable, but nil checking makes the linter happy
        gcs:send_text(5, 'Error: could not read ' .. 'Q_OPTIONS')
    end

    -- This script relies on Q_ASSIST_ALT and Q_ASSIST_ANGLE, but they will
    -- not be used if Q_ASSIST_SPEED is <= 0. Ensure it is > 0.
    local q_assist_speed = tecsParamGroup:get_backup("Q_ASSIST_SPEED") or 0
    tecsParamGroup:set_parameter("Q_ASSIST_SPEED", math.max(q_assist_speed, 0.1))

    -- Set Q_ASSIST_ALT to Q_RTL_ALT
    local q_rtl_alt = Q_RTL_ALT:get() or 60 -- Shouldn't be possible to be nil
    tecsParamGroup:set_parameter("Q_ASSIST_ALT", q_rtl_alt)

    --[[
    Set AIRSPEED_MIN to the the threshold speed for switching to land. This
    helps get unstuck from Q-Assist, and also allows us to lower the target
    airspeed to facilitate airbraking.
    --]]
    local threshold_speed = QAST_GSPD:get() or 1
    tecsParamGroup:set_parameter("AIRSPEED_MIN", threshold_speed)

    -- If in Q_ASSIST, set the cruise speed to the threshold speed to facilitate
    -- airbraking, otherwise set it to the optimal glide speed.
    if q_assist_start_time then
        tecsParamGroup:set_parameter("AIRSPEED_CRUISE", threshold_speed)
    else
        -- Set the cruise speed to the optimal glide speed
        local glide_speed = GLIDE_SPD:get()
        if glide_speed then
            tecsParamGroup:set_parameter("AIRSPEED_CRUISE", glide_speed)
        else
            -- Should be unreachable, but nil checking makes the linter happy
            gcs:send_text(5, 'Error: could not read ' .. 'GLIDE_SPD')
        end
    end
end

local function update()
    --[[
    Main update loop to check engine status, trigger failsafe actions, and
    manage parameter changes for glide configuration.
    --]]

    -- Run pre-arm checks
    pre_arm_checks()

    -- Check engine status
    check_engine()

    -- Configure TECS for glide if engine is stopped
    configure_tecs(engine_stopped)

    -- Do not allow QRTL to take too long before descending
    check_qrtl_timeout()

    -- Make sure QRTL is landing at the override location
    if vehicle:get_mode() == MODE_QRTL and guided_override then
        local old_target = vehicle:get_target_location()
        if old_target and old_target:get_distance(guided_override) > 1 then
            if vehicle:update_target_location(old_target, guided_override) then
                gcs:send_text(6, "Restored guided override")
            end
        end
    end

    -- Ensure the target altitude in QRTL and RTL is appropriate
    reset_target_alt()

    -- Track how long Q_ASSIST has been active
    local now = utilities.get_time_sec()
    if quadplane:in_assisted_flight() then
        q_assist_start_time = q_assist_start_time or now
    else
        q_assist_start_time = nil
    end

    -- If enabled, and the engine is stopped while we are armed in forward
    -- flight, then we need to trigger the failsafe
    local need_failsafe = FS_ENABLE:get() > 0 and engine_stopped and arming:is_armed() and not quadplane:in_vtol_mode()

    -- Only trigger once
    if need_failsafe and not triggered_failsafe then
        triggered_failsafe = true
        do_engine_failsafe()
        gcs:send_text(2, "Failsafe: Engine Out")
    end

    -- If we triggered a failsafe that is no longer needed, restore parameters.
    if triggered_failsafe and not need_failsafe then
        triggered_failsafe = false
        engineFailsafeParamGroup:restore_all()
    end

    -- If a failsafe has triggered, we need to periodically check these things
    if triggered_failsafe then
        -- Allow the pilot to override the location by switching to guided
        if vehicle:get_mode() == MODE_GUIDED then
            guided_override = vehicle:get_target_location():copy()
            if guided_override then
                guided_override:change_alt_frame(3)
                guided_override:alt(Q_RTL_ALT:get() * 100)
                vehicle:set_mode(MODE_RTL)
                local old_target = vehicle:get_target_location()
                if old_target then
                    vehicle:update_target_location(old_target, guided_override)
                end
            end
        -- If we are in any mode but RTL, QRTL, then clear the guided_override
        elseif vehicle:get_mode() ~= MODE_RTL and vehicle:get_mode() ~= MODE_QRTL then
            guided_override = nil
        end

        -- If we are in RTL, check if we should switch to QLand or QRTL
        if vehicle:get_mode() == MODE_RTL then
            check_qland_qrtl()
        end
    end
end

-- Script is now loaded, print success message and start the protected loop
gcs:send_text(6, SCRIPT_NAME .. string.format(" loaded"))
local function protected_wrapper()
    --[[
    Wrapper around the update function to catch and handle errors, preventing
    script termination.
    --]]
    local success, err = pcall(update)
    if not success then
        gcs:send_text(0, "Internal Error: " .. err)
        return protected_wrapper, 1000  -- Retry after 1 second on error
    end
    return protected_wrapper, 200 -- Normal update rate at 5Hz
end

return protected_wrapper() -- Start the update loop
