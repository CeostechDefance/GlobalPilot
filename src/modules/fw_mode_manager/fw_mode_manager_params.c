/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * NPFG period
 *
 * Period of NPFG control law.
 *
 * @unit s
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_PERIOD, 10.0f);

/**
 * NPFG damping ratio
 *
 * Damping ratio of NPFG control law.
 *
 * @min 0.10
 * @max 1.00
 * @decimal 2
 * @increment 0.01
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_DAMPING, 0.7f);

/**
 * Enable automatic lower bound on the NPFG period
 *
 * Avoids limit cycling from a too aggressively tuned period/damping combination.
 * If false, also disables upper bound NPFG_PERIOD_UB.
 *
 * @boolean
 * @group FW NPFG Control
 */
PARAM_DEFINE_INT32(NPFG_LB_PERIOD, 1);

/**
 * Enable automatic upper bound on the NPFG period
 *
 * Adapts period to maintain track keeping in variable winds and path curvature.
 *
 * @boolean
 * @group FW NPFG Control
 */
PARAM_DEFINE_INT32(NPFG_UB_PERIOD, 1);

/**
 * Roll time constant
 *
 * Time constant of roll controller command / response, modeled as first order delay.
 * Used to determine lower period bound. Setting zero disables automatic period bounding.
 *
 * @unit s
 * @min 0.00
 * @max 2.00
 * @decimal 2
 * @increment 0.05
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_ROLL_TC, 0.5f);

/**
 * NPFG switch distance multiplier
 *
 * Multiplied by the track error boundary to determine when the aircraft switches
 * to the next waypoint and/or path segment. Should be less than 1.
 *
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_SW_DST_MLT, 0.32f);

/**
 * Period safety factor
 *
 * Multiplied by period for conservative minimum period bounding (when period lower
 * bounding is enabled). 1.0 bounds at marginal stability.
 *
 * @min 1.0
 * @max 10.0
 * @decimal 1
 * @increment 0.1
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_PERIOD_SF, 1.5f);


/**
 * Minimum pitch angle setpoint
 *
 * Applies in any altitude controlled flight mode.
 *
 * @unit deg
 * @min -60.0
 * @max 0.0
 * @decimal 1
 * @increment 0.5
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_P_LIM_MIN, -30.0f);

/**
 * Maximum pitch angle setpoint
 *
 * Applies in any altitude controlled flight mode.
 *
 * @unit deg
 * @min 0.0
 * @max 80.0
 * @decimal 1
 * @increment 0.5
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_P_LIM_MAX, 30.0f);

/**
 * Maximum roll angle setpoint
 *
 * Applies in any altitude controlled flight mode.
 *
 * @unit deg
 * @min 35.0
 * @max 75.0
 * @decimal 1
 * @increment 0.5
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_R_LIM, 50.0f);

/**
 * Throttle limit max
 *
 * Applies in any altitude controlled flight mode.
 * Should be set accordingly to achieve FW_T_CLMB_MAX.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_THR_MAX, 1.0f);

/**
 * Throttle limit min
 *
 * Applies in any altitude controlled flight mode.
 * Usually set to 0 but can be increased to prevent the motor from stopping when
 * descending, which can increase achievable descent rates.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_THR_MIN, 0.0f);

/**
 * Idle throttle
 *
 * This is the minimum throttle while on the ground ("landed") in auto modes.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_THR_IDLE, 0.0f);

/**
 * Maximum landing slope angle
 *
 * Typically the desired landing slope angle when landing configuration (flaps, airspeed) is enabled.
 * Set this value within the vehicle's performance limits.
 *
 * @unit deg
 * @min 1.0
 * @max 45.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_ANG, 5.0f);

/**
 * Minimum pitch during takeoff.
 *
 * @unit deg
 * @min -5.0
 * @max 80.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_FLOAT(FW_TKO_PITCH_MIN, 10.0f);

/**
 * Runway takeoff exit blend time
 *
 * When the runway takeoff climbout ends, the pitch floor (FW_TKO_PITCH_MIN) and the fixed takeoff
 * throttle (RWTO_MAX_THR) are released over this time instead of at once: the pitch floor ramps down to
 * FW_P_LIM_MIN, the throttle range opens from the takeoff throttle to FW_THR_MIN..FW_THR_MAX and the
 * flaps retract from FW_FLAPS_TO_SCL. This avoids a step in pitch, throttle and lift at the end of the takeoff.
 *
 * Set to 0 to release both at once.
 *
 * @unit s
 * @min 0.0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_FLOAT(FW_TKO_BLEND_T, 0.0f);

/**
 * Takeoff Airspeed
 *
 * The calibrated airspeed setpoint during the takeoff climbout.
 *
 * If set <= 0, FW_AIRSPD_MIN will be set by default.
 *
 * @unit m/s
 * @min -1.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_FLOAT(FW_TKO_AIRSPD, -1.0f);

/**
 * Landing flare altitude (relative to landing altitude)
 *
 * NOTE: max(FW_LND_FLALT, FW_LND_FL_TIME * |z-velocity|) is taken as the flare altitude
 *
 * @unit m
 * @min 0.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FLALT, 0.5f);

/**
 * Use terrain estimation during landing.
 *
 * This is critical for detecting when to flare, and should be enabled if possible.
 *
 * If enabled and no measurement is found within a given timeout, the landing waypoint altitude will be used OR the landing
 * will be aborted, depending on the criteria set in FW_LND_ABORT.
 *
 * If disabled, FW_LND_ABORT terrain based criteria are ignored.
 *
 * @min 0
 * @max 2
 * @value 0 Disable the terrain estimate
 * @value 1 Use the terrain estimate to trigger the flare (only)
 * @value 2 Calculate landing glide slope relative to the terrain estimate
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_USETER, 1);

/**
 * Early landing configuration deployment
 *
 * Allows to deploy the landing configuration (flaps, landing airspeed, etc.) already in
 * the loiter-down waypoint before the final approach.
 * Otherwise is enabled only in the final approach.
 *
 * @boolean
 *
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_EARLYCFG, 0);

/**
 * Flare, minimum pitch
 *
 * Minimum pitch during landing flare.
 *
 * @unit deg
 * @min -5
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_PMIN, 2.5f);

/**
 * Flare, maximum pitch
 *
 * Maximum pitch during landing flare.
 *
 * @unit deg
 * @min 0
 * @max 45.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_PMAX, 15.0f);

/**
 * Landing airspeed
 *
 * The calibrated airspeed setpoint during landing.
 *
 * If set <= 0, landing airspeed = FW_AIRSPD_MIN by default.
 *
 * @unit m/s
 * @min -1.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_AIRSPD, -1.f);

/**
 * Altitude time constant factor for landing and low-height flight
 *
 * The TECS altitude time constant (FW_T_ALT_TC) is multiplied by this value.
 *
 * @unit
 * @min 0.2
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_THRTC_SC, 1.0f);

/**
 * Speed <--> Altitude weight
 *
 * Adjusts the amount of weighting that the pitch control
 * applies to speed vs height errors.
 * 0 -> control height only
 * 2 -> control speed only (gliders)
 *
 * @min 0.0
 * @max 2.0
 * @decimal 1
 * @increment 1.0
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_T_SPDWEIGHT, 1.0f);

/**
 * Custom stick configuration
 *
 * Applies in manual Position and Altitude flight modes.
 *
 * @min 0
 * @max 3
 * @bit 0 Alternative stick configuration (height rate on throttle stick, airspeed on pitch stick)
 * @bit 1 Enable airspeed setpoint via sticks in altitude and position flight mode
 * @group FW General
 */
PARAM_DEFINE_INT32(FW_POS_STK_CONF, 2);

/**
 * Default target climbrate.
 *
 * In auto modes: default climb rate output by controller to achieve altitude setpoints.
 * In manual modes: maximum climb rate setpoint.
 *
 * @unit m/s
 * @min 0.1
 * @decimal 2
 * @increment 0.01
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_T_CLMB_R_SP, 3.0f);

/**
 * Default target sinkrate.
 *
 * In auto modes: default sink rate output by controller to achieve altitude setpoints.
 * In manual modes: maximum sink rate setpoint.
 *
 * @unit m/s
 * @min 0.1
 * @decimal 2
 * @increment 0.01
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_T_SINK_R_SP, 2.0f);

/**
 * GPS failure loiter time
 *
 * The time the system should do open loop loiter and wait for GPS recovery
 * before it starts descending. Set to 0 to disable. Roll angle is set to FW_GPSF_R.
 * Does only apply for fixed-wing vehicles or VTOLs with NAV_FORCE_VT set to 0.
 *
 * @unit s
 * @min 0
 * @group FW General
 */
PARAM_DEFINE_INT32(FW_GPSF_LT, 30);

/**
 * GPS failure fixed roll angle
 *
 * Roll angle in GPS failure loiter mode.
 *
 * @unit deg
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @increment 0.5
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_GPSF_R, 15.0f);


/**
 * The aircraft's wing span (length from tip to tip).
 *
 * This is used for limiting the roll setpoint near the ground. (if multiple wings, take the longest span)
 *
 * @unit m
 * @min 0.1
 * @decimal 1
 * @increment 0.1
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_WING_SPAN, 3.0);

/**
 * Height (AGL) of the wings when the aircraft is on the ground.
 *
 * This is used to constrain a minimum altitude below which we keep wings level to avoid wing tip strike. It's safer
 * to give a slight margin here (> 0m)
 *
 * @unit m
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_WING_HEIGHT, 0.5);

/**
 * Landing flare time
 *
 * Multiplied by the descent rate to calculate a dynamic altitude at which
 * to trigger the flare.
 *
 * NOTE: max(FW_LND_FLALT, FW_LND_FL_TIME * descent rate) is taken as the flare altitude
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_TIME, 1.0f);

/**
 * Landing touchdown time (since flare start)
 *
 * This is the time after the start of flaring that we expect the vehicle to touch the runway.
 * At this time, a 0.5s clamp down ramp will engage, constraining the pitch setpoint to RWTO_PSP.
 * If enabled, ensure that RWTO_PSP is configured appropriately for full gear contact on ground roll.
 *
 * Set to -1.0 to disable touchdown clamping. E.g. it may not be desirable to clamp on belly landings.
 *
 * The touchdown time will be constrained to be greater than or equal to the flare time (FW_LND_FL_TIME).
 *
 * @unit s
 * @min -1.0
 * @max 5.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_TD_TIME, -1.0f);

/**
 * Landing flare sink rate
 *
 * TECS will attempt to control the aircraft to this sink rate via pitch angle (throttle killed during flare)
 *
 * @unit m/s
 * @min 0.0
 * @max 2
 * @decimal 2
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_SINK, 0.25f);

/**
 * Landing flare law
 *
 * How the sink rate, pitch limits and throttle limit move from the approach values to the touchdown values
 * during the flare of a straight (runway) landing. Circular landings always use the time based ramp.
 *
 * 0: time based ramps over FW_LND_FL_TIME (PX4 default).
 * 1: height based linear sink rate (exponential flare). The sink rate setpoint goes linearly with height from the
 *    sink rate at flare start to FW_LND_FL_SINK at FW_LND_FL_HOLD, so height decays exponentially in time.
 *    Pitch limits are blended with the same height fraction. Throttle goes to idle over FW_LND_FL_TIME as in 0.
 * 2: constant vertical deceleration (circular arc flare). The sink rate goes down linearly in time from the sink
 *    rate at flare start to FW_LND_FL_SINK at FW_LND_FL_HOLD. Limits are blended as in 1.
 *
 * 3: autoland style. Sink rate profile as in 1, but the sink rate error drives the pitch setpoint directly through a
 *    PI controller (FW_LND_FL_KP, FW_LND_FL_KI) around the pitch angle at flare start, bypassing TECS for pitch.
 *    Pitch is constrained by the blended limits of 1; throttle goes to idle over FW_LND_FL_TIME.
 * 4: power-on attitude landing (seaplane glassy water technique, FAA-H-8083-23 ch. 6). At the flare height the pitch is
 *    brought to FW_LND_FL_ATT over FW_LND_FL_TATT and held; throttle controls the sink rate (FW_LND_FL_SNK1 above
 *    FW_LND_FL_HOLD, FW_LND_FL_SINK below) with a PI (FW_LND_FL_TKP, FW_LND_FL_TKI) around FW_LND_FL_TFF. With the
 *    angle of attack held and lift equal to weight, the touchdown airspeed depends only on the attitude and the weight,
 *    not on the pattern or the flare height. The throttle goes to idle at touchdown (land detector). Needs a height
 *    above ground at decimetre level: use a downward range finder (FW_LND_USETER 1).
 * 5: airliner style. Sink rate FW_LND_FL_SNK1, FW_LND_FL_SINK below FW_LND_FL_HOLD, held by pitch (PI FW_LND_FL_KP/KI
 *    around the pitch at flare start, up to FW_LND_FL_PMAX). Throttle holds FW_LND_FL_VATT (weight scaled) with a PI
 *    (FW_LND_FL_TKP/TKI per m/s, centre FW_LND_FL_TFF), moving at FW_LND_FL_TSLW, and goes to idle below
 *    FW_LND_FL_HOLD (retard). Unlike 3 the throttle is not cut at flare start.
 * 6: as 5, for an unknown weight. The 1 g stall speed is estimated on the approach from the synthetic angle of attack
 *    (FW_LND_VS_CL0/CLA/CLMX); the approach airspeed is FW_LND_VAPP_K times it (also the lower airspeed limit, so
 *    WEIGHT_GROSS does not hold the aircraft fast). The flare throttle starts from the approach throttle and holds an
 *    airspeed falling with height to FW_LND_VTD_K times the stall speed at FW_LND_FL_HOLD, until touchdown (no retard).
 *
 * @value 0 Time based (PX4)
 * @value 1 Height based exponential
 * @value 2 Constant deceleration
 * @value 3 Exponential, direct pitch (autoland style)
 * @value 4 Power-on attitude (throttle controls the sink rate)
 * @value 5 Airliner style (pitch holds the sink rate, throttle holds FW_LND_FL_VATT, retard below FW_LND_FL_HOLD)
 * @value 6 Airliner style with estimated stall speed (weight independent)
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_FL_LAW, 0);

/**
 * Landing flare hold-off height
 *
 * Height above the touchdown point at which the height based flare laws (FW_LND_FL_LAW 1-3) reach
 * FW_LND_FL_SINK and the touchdown pitch limits; laws 4-6 hold FW_LND_FL_SNK1 above it. Below it the sink rate
 * stays at FW_LND_FL_SINK.
 *
 * @unit m
 * @min 0.0
 * @max 5.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_HOLD, 1.0f);

/**
 * Landing flare speed weight
 *
 * TECS speed weight during the flare of the height based flare laws (FW_LND_FL_LAW 1 and 2).
 * 0 makes pitch control only the sink rate while the airspeed bleeds off at idle throttle.
 * Negative keeps FW_T_SPDWEIGHT.
 *
 * @min -1.0
 * @max 2.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_SPDW, -1.0f);

/**
 * Landing flare sink rate to pitch proportional gain
 *
 * Flare laws 3-6 (law 4 while decelerating) and the water scoop flare. Pitch change per sink rate error.
 *
 * @unit deg
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_KP, 3.0f);

/**
 * Landing flare sink rate to pitch integral gain
 *
 * Flare laws 3-6 (law 4 while decelerating) and the water scoop flare. Pitch change per second per sink rate error.
 *
 * @unit deg
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_KI, 1.0f);

/**
 * Landing attitude of flare law 4
 *
 * Pitch held from the end of the attitude transition to touchdown. It sets the angle of attack and so the touchdown
 * airspeed for a given weight.
 *
 * @unit deg
 * @min 0.0
 * @max 12.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_ATT, 6.0f);

/**
 * Attitude transition time of flare law 4
 *
 * Time to bring the pitch from its value at flare start to FW_LND_FL_ATT, and the sink rate setpoint from its value at
 * flare start to FW_LND_FL_SNK1.
 *
 * @unit s
 * @min 0.5
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_TATT, 4.0f);

/**
 * Sink rate of flare law 4 above FW_LND_FL_HOLD
 *
 * Below FW_LND_FL_HOLD the sink rate setpoint is FW_LND_FL_SINK.
 *
 * @unit m/s
 * @min 0.1
 * @max 3.0
 * @decimal 2
 * @increment 0.05
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_SNK1, 0.75f);

/**
 * Throttle feedforward of flare law 4
 *
 * Throttle at the start of the attitude hold and the PI's centre (user decision 2026-11-10: start at 20 %).
 * From there the throttle moves at FW_LND_FL_TSLW.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_TFF, 0.20f);

/**
 * Throttle proportional gain of flare law 4
 *
 * Throttle per m/s of sink rate error (sinking faster than the setpoint adds throttle).
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_TKP, 0.15f);

/**
 * Throttle integral gain of flare law 4
 *
 * Throttle per m of accumulated sink rate error.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_TKI, 0.10f);

/**
 * Minimum airspeed of flare law 4 at WEIGHT_BASE
 *
 * Scaled with sqrt(WEIGHT_GROSS / WEIGHT_BASE). Below it the landing attitude is lowered by up to 2 deg (1 deg/s) and
 * restored at 0.5 deg/s once the airspeed recovers. 0 disables the guard.
 *
 * @unit m/s
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_VMIN, 0.0f);

/**
 * Attitude hold airspeed of flare law 4 at WEIGHT_BASE
 *
 * Flare law 4 decelerates at idle (pitch holds the sink rate) until the airspeed falls to this value, scaled with
 * sqrt(WEIGHT_GROSS / WEIGHT_BASE), then holds FW_LND_FL_ATT and controls the sink rate with throttle. Set it to the
 * equilibrium airspeed at the landing attitude. 0: switch when the pitch reaches FW_LND_FL_ATT.
 *
 * @unit m/s
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_VATT, 0.0f);

/**
 * Throttle slew rate of flare law 4
 *
 * Maximum throttle change per second in the flare, also from idle at the start of the attitude hold. The thrust line
 * lies above the CG: a fast throttle change pitches the aircraft before the elevator can hold the attitude at landing
 * airspeed. 0 disables the limit.
 *
 * @unit 1/s
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_TSLW, 0.10f);

/**
 * Landing lift coefficient at zero synthetic angle of attack (flare law 6)
 *
 * Lift coefficient with landing flaps at pitch = flight path angle, for the stall speed estimate. Fit from approach
 * data of known weight: CL = 2 m g cos(roll) / (rho V^2 S) against pitch - asin(-vz / V).
 *
 * @min 0.0
 * @max 3.0
 * @decimal 3
 * @increment 0.01
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_VS_CL0, 0.97f);

/**
 * Landing lift curve slope (flare law 6)
 *
 * Per radian of synthetic angle of attack.
 * @min 1.0
 * @max 8.0
 * @decimal 2
 * @increment 0.05
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_VS_CLA, 4.7f);

/**
 * Landing maximum lift coefficient (flare law 6)
 *
 * Lift coefficient at the stall with landing flaps; defines the estimated stall speed V sqrt(CL / CLMX).
 *
 * @min 0.5
 * @max 4.0
 * @decimal 3
 * @increment 0.01
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_VS_CLMX, 1.83f);

/**
 * Approach airspeed over the estimated stall speed (flare law 6)
 *
 * @min 1.1
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_VAPP_K, 1.30f);

/**
 * Touchdown airspeed over the estimated stall speed (flare law 6)
 *
 * The flare throttle holds an airspeed falling linearly with height from the flare start airspeed to this times the
 * estimated stall speed at FW_LND_FL_HOLD. Same ratio, same angle of attack: the touchdown pitch does not depend on
 * the weight.
 *
 * @min 1.0
 * @max 1.5
 * @decimal 2
 * @increment 0.01
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_VTD_K, 1.12f);

/**
 * Flare pitch rate limit (flare law 6)
 *
 * The flare pitch command moves at most this fast, up and down, so the nose is raised slowly. The sink rate PI does not
 * integrate while the command is held by the rate limit in the error's direction. 0 disables the limit.
 *
 * @unit deg/s
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_PRT, 3.0f);

/**
 * Maximum lateral position offset for the touchdown point
 *
 * @unit m
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_TD_OFF, 3.0);

/**
 * Landing touchdown nudging option.
 *
 * Approach angle nudging: shifts the touchdown point laterally while keeping the approach entrance point constant
 * Approach path nudging: shifts the touchdown point laterally along with the entire approach path
 *
 * This is useful for manually adjusting the landing point in real time when map or GNSS errors cause an offset from the
 * desired landing vector. Nudging is done with yaw stick, constrained to FW_LND_TD_OFF (in meters) and the direction is
 * relative to the vehicle heading (stick deflection to the right = land point moves to the right as seen by the vehicle).
 *
 * @min 0
 * @max 2
 * @value 0 Disable nudging
 * @value 1 Nudge approach angle
 * @value 2 Nudge approach path
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_NUDGE, 2);

/**
 * Bit mask to set the automatic landing abort conditions.
 *
 * Terrain estimation:
 * bit 0: Abort if terrain is not found
 * bit 1: Abort if terrain times out (after a first successful measurement)
 *
 * The last estimate is always used as ground, whether the last valid measurement or the land waypoint, depending on the
 * selected abort criteria, until an abort condition is entered. If FW_LND_USETER == 0, these bits are ignored.
 *
 * TODO: Extend automatic abort conditions
 * e.g. glide slope tracking error (horizontal and vertical)
 *
 * @min 0
 * @max 3
 * @bit 0 Abort if terrain is not found (only applies to mission landings)
 * @bit 1 Abort if terrain times out (after a first successful measurement)
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_ABORT, 3);

/**
 * Fixed-wing launch detection
 *
 * Enables automatic launch detection based on measured acceleration. Use for hand- or catapult-launched vehicles.
 * Not compatible with runway takeoff.
 *
 * @boolean
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_INT32(FW_LAUN_DETCN_ON, 0);

/**
 * Flaps setting during take-off
 *
 * Sets a fraction of full flaps during take-off.
 * Also applies to flaperons if enabled in the mixer/allocation.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_FLOAT(FW_FLAPS_TO_SCL, 0.0f);

/**
 * Flaps setting during landing
 *
 * Sets a fraction of full flaps during landing.
 * Also applies to flaperons if enabled in the mixer/allocation.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_FLAPS_LND_SCL, 1.0f);

/**
 * Spoiler landing setting
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_SPOILERS_LND, 0.f);

/**
 * Water scoop: touchdown airspeed over the estimated stall speed
 *
 * Flare law 6 on the water (MAV_CMD_USER_1 scoop item). Higher than FW_LND_VTD_K: lower nose at touchdown, the aft
 * hull cone touches the water at 6-7 deg with the step 5-10 cm under.
 *
 * @min 1.0
 * @max 1.5
 * @decimal 2
 * @increment 0.01
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_VTD_K, 1.15f);

/**
 * Water scoop: flare pitch upper limit
 *
 * @unit deg
 * @min 0.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_PMAX, 6.0f);

/**
 * Water scoop: pitch held while scooping on the water
 *
 * Inside the hull step range (2-7 deg).
 *
 * @unit deg
 * @min -5.0
 * @max 10.0
 * @decimal 1
 * @increment 0.5
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_PITCH, 4.0f);

/**
 * Water scoop: takeoff pitch from the water
 *
 * @unit deg
 * @min 0.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_TO_PITCH, 6.0f);

/**
 * Water scoop: upper limit of the scoop airspeed band
 *
 * The tank only fills between the item's lower limit (param4) and this (water_tank.txt speed_max).
 *
 * @unit m/s
 * @min 10.0
 * @max 50.0
 * @decimal 1
 * @increment 0.1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_VMAX, 27.5f);

/**
 * Water scoop: throttle proportional gain on the airspeed error while scooping
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_TKP, 0.10f);

/**
 * Water scoop: throttle integral gain on the airspeed error while scooping
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_TKI, 0.05f);

/**
 * Water scoop: go-around height above the water
 *
 * After an abort the aircraft climbs straight to this height, then holds (AUTO.LOITER); the planner decides.
 *
 * @unit m
 * @min 20.0
 * @max 300.0
 * @decimal 0
 * @increment 5
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_GA_ALT, 60.0f);

/**
 * Water scoop gate: largest cross-track error at the flare start
 *
 * @unit m
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_G_XTRK, 15.0f);

/**
 * Water scoop gate: largest course error at the flare start
 *
 * @unit deg
 * @min 0.5
 * @max 45.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_G_CRS, 3.0f);

/**
 * Water scoop gate: largest roll angle at the flare start
 *
 * @unit deg
 * @min 0.5
 * @max 30.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_G_ROLL, 3.0f);

/**
 * Water scoop gate: largest airspeed error at the flare start
 *
 * @unit m/s
 * @min 0.5
 * @max 10.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_G_SPD, 2.0f);

/**
 * Water scoop: go-around above this roll angle during the flare
 *
 * @unit deg
 * @min 1.0
 * @max 30.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_A_ROLL, 5.0f);

/**
 * Water scoop: go-around above this cross-track error during the flare
 *
 * @unit m
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_A_XTRK, 20.0f);

/**
 * Water scoop: own go-around checks only above this height (below it the touchdown comes first)
 *
 * @unit m
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_A_HMIN, 1.0f);

/**
 * Water scoop: go-around after this long out of the abort band (FW_SC_A_VMIN..FW_SC_A_VMAX) once scooping started
 *
 * @unit s
 * @min 0.5
 * @max 30.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_OOB_T, 3.0f);

/**
 * Water scoop: go-around when the scoop phase lasts this much longer than the item duration
 *
 * @unit s
 * @min 1.0
 * @max 120.0
 * @decimal 0
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_T_MARG, 15.0f);

/**
 * Water scoop: the scoop is done this high over the water after the takeoff
 *
 * @unit m
 * @min 3.0
 * @max 200.0
 * @decimal 0
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_DONE_H, 20.0f);

/**
 * Water scoop: climb height over the water while the navigator is still on a water waypoint
 *
 * @unit m
 * @min 5.0
 * @max 300.0
 * @decimal 0
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_CLB_H, 30.0f);

/**
 * Water scoop: go-around from the water with direct takeoff pitch/throttle up to this height, then the altitude controller
 *
 * @unit m
 * @min 1.0
 * @max 50.0
 * @decimal 0
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_GA_H, 10.0f);

/**
 * Flare law 6 touchdown: range finder reading below this
 *
 * @unit m
 * @min 0.1
 * @max 5.0
 * @decimal 2
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_TD_RNG, 1.2f);

/**
 * Flare law 6 touchdown: sink rate below this
 *
 * @unit m/s
 * @min 0.05
 * @max 2.0
 * @decimal 2
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_TD_VZ, 0.35f);

/**
 * Flare law 6 touchdown: both conditions held this long
 *
 * @unit s
 * @min 0.05
 * @max 5.0
 * @decimal 2
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_TD_T, 0.5f);

/**
 * Flare law 6 stall speed estimate: samples only above this height (ground effect)
 *
 * @unit m
 * @min 0.0
 * @max 500.0
 * @decimal 0
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_VS_HMIN, 20.0f);

/**
 * Flare law 6 stall speed estimate: samples only below this roll angle
 *
 * @unit deg
 * @min 1.0
 * @max 30.0
 * @decimal 1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_VS_ROLL, 5.0f);

/**
 * Flare law 6 stall speed estimate: samples only after this long wings level
 *
 * @unit s
 * @min 0.0
 * @max 30.0
 * @decimal 1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_VS_LVL, 2.0f);

/**
 * Flare law 6 stall speed estimate: filter time constant (also the settling time)
 *
 * @unit s
 * @min 0.5
 * @max 60.0
 * @decimal 1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_VS_TAU, 5.0f);

/**
 * Water scoop: airspeed prediction time for the throttle
 *
 * The throttle works on the airspeed predicted this far ahead (filtered airspeed rate), so it opens while still
 * decelerating towards the band (water_leg THROTTLE_LOOKAHEAD_S).
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_LOOK, 2.5f);

/**
 * Water scoop: throttle feedforward on the water at FW_SC_FF_VS stall speed
 *
 * Throttle that holds about 25 m/s on the water (water_leg THROTTLE_BASE, 800 kg); scaled with (Vs_est / FW_SC_FF_VS)^2.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_THR_FF, 0.70f);

/**
 * Water scoop: stall speed at which FW_SC_THR_FF holds the scoop airspeed
 *
 * @unit m/s
 * @min 1.0
 * @max 60.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_FF_VS, 21.6f);

/**
 * Water scoop: fill rate for the momentum drag feedforward
 *
 * Scooped water is brought to the aircraft speed: drag fill_rate x v (water_tank.txt fill_rate).
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_FILL, 40.0f);

/**
 * Water scoop: maximum thrust (thrust proportional to throttle squared)
 *
 * @unit N
 * @min 1.0
 * @max 100000.0
 * @decimal 0
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_TMAX, 5002.0f);

/**
 * Water scoop: pitch feedforward per unit throttle above the feedforward throttle
 *
 * The thrust line is above the CG: more throttle pitches the nose down, the pitch command is raised accordingly.
 *
 * @unit deg
 * @min -10.0
 * @max 10.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_P_THR, 4.0f);

/**
 * Water scoop: height over the water at which the takeoff run hands over to the climb
 *
 * Below it the takeoff run is flown with direct pitch and throttle; above it TECS climbs at the trim airspeed to
 * FW_SC_DONE_H (no step at the end of the scoop).
 *
 * @unit m
 * @min 0.5
 * @max 20.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_LIFT_H, 2.0f);

/**
 * Water scoop: airspeed setpoint decrease after liftoff
 *
 * The climb starts at the liftoff airspeed and slows to the trim airspeed at this rate.
 *
 * @unit m/s^2
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_CLB_DEC, 1.0f);

/**
 * Water scoop: low speed guard margin
 *
 * Full throttle while the predicted airspeed is less than this above the band floor (item param4).
 *
 * @unit m/s
 * @min 0.0
 * @max 5.0
 * @decimal 1
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_V_MARG, 0.8f);

/**
 * Water scoop: abort band lower limit
 *
 * The throttle keeps the airspeed in the control band (item param4 .. FW_SC_VMAX, 80-100 km/h); only leaving this
 * wider abort band for longer than FW_SC_OOB_T aborts the scoop. 19.44 m/s = 70 km/h (user).
 *
 * @unit m/s
 * @min 10.0
 * @max 50.0
 * @decimal 2
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_A_VMIN, 19.44f);

/**
 * Water scoop: abort band upper limit
 *
 * 34.72 m/s = 125 km/h (user).
 *
 * @unit m/s
 * @min 10.0
 * @max 60.0
 * @decimal 2
 * @group FW Water Scoop
 */
PARAM_DEFINE_FLOAT(FW_SC_A_VMAX, 34.72f);
