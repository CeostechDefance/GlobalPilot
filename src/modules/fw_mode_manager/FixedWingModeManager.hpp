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
 * @file FixedWingModeManager.hpp
 * Implementation of various fixed-wing control modes.
 */

#ifndef FIXEDWINGMODEMANAGER_HPP_
#define FIXEDWINGMODEMANAGER_HPP_

#include "launchdetection/LaunchDetector.h"
#include "runway_takeoff/RunwayTakeoff.h"
#include "ControllerConfigurationHandler.hpp"

#include <float.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/atmosphere/atmosphere.h>
#include <lib/npfg/DirectionalGuidance.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRate.hpp>
#include <lib/sticks/Sticks.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/uORB.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/fixed_wing_lateral_setpoint.h>
#include <uORB/topics/fixed_wing_lateral_guidance_status.h>
#include <uORB/topics/fixed_wing_longitudinal_setpoint.h>
#include <uORB/topics/fixed_wing_runway_control.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/launch_detection_status.h>
#include <uORB/topics/normalized_unsigned_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_controller_landing_status.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/debug_key_value.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/orbit_status.h>

#ifdef CONFIG_FIGURE_OF_EIGHT
#include "figure_eight/FigureEight.hpp"
#include <uORB/topics/figure_eight_status.h>
#endif // CONFIG_FIGURE_OF_EIGHT

using namespace launchdetection;
using namespace runwaytakeoff;
using namespace time_literals;

using matrix::Vector2d;
using matrix::Vector2f;

// [m] initial distance of waypoint in front of plane in heading hold mode
static constexpr float HDG_HOLD_DIST_NEXT = 3000.0f;

// [rad/s] max yawrate at which plane locks yaw for heading hold mode
static constexpr float HDG_HOLD_YAWRATE_THRESH = 0.15f;

// [.] max manual roll/yaw normalized input from user which does not change the locked heading
static constexpr float HDG_HOLD_MAN_INPUT_THRESH = 0.01f;

// [us] time after which we abort landing if terrain estimate is not valid. this timer start whenever the terrain altitude
// was previously valid, and has changed to invalid.
static constexpr hrt_abstime TERRAIN_ALT_TIMEOUT = 1_s;

// [us] within this timeout, if a distance sensor measurement not yet made, the land waypoint altitude is used for terrain
// altitude. this timer starts at the beginning of the landing glide slope.
static constexpr hrt_abstime TERRAIN_ALT_FIRST_MEASUREMENT_TIMEOUT = 10_s;

// [.] max throttle from user which will not lead to motors spinning up in altitude controlled modes
static constexpr float THROTTLE_THRESH = -.9f;

// [us] time after which the wind estimate is disabled if no longer updating
static constexpr hrt_abstime WIND_EST_TIMEOUT = 10_s;

// [s] minimum time step between auto control updates
static constexpr float MIN_AUTO_TIMESTEP = 0.01f;

// [s] maximum time step between auto control updates
static constexpr float MAX_AUTO_TIMESTEP = 0.05f;

// [rad] minimum pitch while airspeed has not yet reached a controllable value in manual position controlled takeoff modes
static constexpr float MIN_PITCH_DURING_MANUAL_TAKEOFF = 0.0f;

// [m] arbitrary buffer altitude added to clearance altitude setpoint during takeoff to ensure aircraft passes the clearance
// altitude while waiting for navigator to flag it exceeded
static constexpr float kClearanceAltitudeBuffer = 10.0f;

// [m/s] maximum rate at which the touchdown position can be nudged
static constexpr float MAX_TOUCHDOWN_POSITION_NUDGE_RATE = 4.0f;

// [.] normalized deadzone threshold for manual nudging input
static constexpr float MANUAL_TOUCHDOWN_NUDGE_INPUT_DEADZONE = 0.15f;

// [s] time interval after touchdown for ramping in runway clamping constraints (touchdown is assumed at FW_LND_TD_TIME after start of flare)
static constexpr float POST_TOUCHDOWN_CLAMP_TIME = 0.5f;

// [] Stick deadzon
static constexpr float kStickDeadBand = 0.06f;

class FixedWingModeManager final : public ModuleBase<FixedWingModeManager>, public ModuleParams,
	public px4::WorkItem
{
public:
	FixedWingModeManager();
	~FixedWingModeManager() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _wind_sub{ORB_ID(wind)};
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<position_controller_landing_status_s>	_pos_ctrl_landing_status_pub{ORB_ID(position_controller_landing_status)};
	uORB::Publication<launch_detection_status_s> _launch_detection_status_pub{ORB_ID(launch_detection_status)};
	uORB::PublicationMulti<orbit_status_s> _orbit_status_pub{ORB_ID(orbit_status)};
	uORB::Publication<landing_gear_s> _landing_gear_pub {ORB_ID(landing_gear)};
	uORB::Publication<normalized_unsigned_setpoint_s> _flaps_setpoint_pub{ORB_ID(flaps_setpoint)};
	uORB::Publication<normalized_unsigned_setpoint_s> _spoilers_setpoint_pub{ORB_ID(spoilers_setpoint)};
	uORB::PublicationData<fixed_wing_lateral_setpoint_s> _lateral_ctrl_sp_pub{ORB_ID(fixed_wing_lateral_setpoint)};
	uORB::PublicationData<fixed_wing_longitudinal_setpoint_s> _longitudinal_ctrl_sp_pub{ORB_ID(fixed_wing_longitudinal_setpoint)};
	uORB::Publication<fixed_wing_lateral_guidance_status_s> _fixed_wing_lateral_guidance_status_pub{ORB_ID(fixed_wing_lateral_guidance_status)};
	uORB::Publication<fixed_wing_runway_control_s> _fixed_wing_runway_control_pub{ORB_ID(fixed_wing_runway_control)};

	position_setpoint_triplet_s _pos_sp_triplet{};
	vehicle_control_mode_s _control_mode{};
	vehicle_local_position_s _local_pos{};
	vehicle_status_s _vehicle_status{};

	CombinedControllerConfigurationHandler _ctrl_configuration_handler;

	Vector2f _lpos_where_backtrans_started;

	bool _position_setpoint_previous_valid{false};
	bool _position_setpoint_current_valid{false};
	bool _position_setpoint_next_valid{false};

	perf_counter_t _loop_perf; // loop performance counter

	// [us] Last absolute time position control has been called
	hrt_abstime _last_time_position_control_called{0};

	uint8_t _position_sp_type{0};

	enum FW_POSCTRL_MODE {
		FW_POSCTRL_MODE_AUTO,
		FW_POSCTRL_MODE_AUTO_ALTITUDE,
		FW_POSCTRL_MODE_AUTO_CLIMBRATE,
		FW_POSCTRL_MODE_AUTO_TAKEOFF,
		FW_POSCTRL_MODE_AUTO_TAKEOFF_NO_NAV,
		FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT,
		FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR,
		FW_POSCTRL_MODE_AUTO_PATH,
		FW_POSCTRL_MODE_MANUAL_POSITION,
		FW_POSCTRL_MODE_MANUAL_ALTITUDE,
		FW_POSCTRL_MODE_TRANSITION_TO_HOVER_LINE_FOLLOW,
		FW_POSCTRL_MODE_TRANSITION_TO_HOVER_HEADING_HOLD,
		FW_POSCTRL_MODE_OTHER
	} _control_mode_current{FW_POSCTRL_MODE_OTHER}; // used to check if the mode has changed

	enum StickConfig {
		STICK_CONFIG_SWAP_STICKS_BIT = (1 << 0),
		STICK_CONFIG_ENABLE_AIRSPEED_SP_MANUAL_BIT = (1 << 1)
	};


	Sticks _sticks{this};

	// VEHICLE STATES

	double _current_latitude{0};
	double _current_longitude{0};
	float _current_altitude{0.f};

	float _pitch{0.0f};
	float _roll{0.0f};

	// flare law 6: stall speed (1 g, landing flaps) estimated on the approach from the synthetic angle of attack
	// (pitch - flight path angle) and FW_LND_VS_CL0/CLA/CLMX, so the landing speeds do not need the weight
	float _vs_est{NAN}; // [m/s] filtered estimate, frozen at flare start
	float _vs_est_settled{0.0f}; // [s] time the estimate has been filtered for
	hrt_abstime _vs_est_last_call{0};
	hrt_abstime _vs_est_last_sample{0};
	hrt_abstime _vs_wings_level_since{0}; // samples only after this long wings level (turn exit bias)
	float _yaw{0.0f};
	float _yawrate{0.0f};

	float _body_acceleration_x{0.f};
	float _body_velocity_x{0.f};

	MapProjection _global_local_proj_ref{};

	float _reference_altitude{NAN}; // [m AMSL] altitude of the local projection reference point

	bool _landed{true};

	// MANUAL MODES

	// indicates whether we have completed a manual takeoff in a position control mode
	bool _completed_manual_takeoff{false};

	// [rad] yaw setpoint for manual position mode heading hold
	float _hdg_hold_yaw{0.0f};

	bool _hdg_hold_enabled{false}; // heading hold enabled
	bool _yaw_lock_engaged{false}; // yaw is locked for heading hold

	Vector2f _hdg_hold_position{}; // position where heading hold started

	// [.] normalized setpoint for manual altitude control [-1,1]; -1,0,1 maps to min,zero,max height rate commands
	float _manual_control_setpoint_for_height_rate{0.0f};

	// [.] normalized setpoint for manual airspeed control [-1,1]; -1,0,1 maps to min,cruise,max airspeed commands
	float _manual_control_setpoint_for_airspeed{0.0f};

	// [m/s] airspeed setpoint for manual modes commanded via MAV_CMD_DO_CHANGE_SPEED
	float _commanded_manual_airspeed_setpoint{NAN};

	// AUTO TAKEOFF

	// [m] ground altitude AMSL where the plane was launched
	float _takeoff_ground_alt{0.0f};

	// runway takeoff exit blend (FW_TKO_BLEND_T): last time in climbout and the limits held there
	hrt_abstime _time_takeoff_climbout_last{0};
	float _takeoff_exit_pitch_min{0.0f};
	float _takeoff_exit_throttle{0.0f};

	/**
	 * Releases the runway takeoff pitch floor and throttle over FW_TKO_BLEND_T after the climbout ends.
	 * Tightens the limits of the current cycle only; called after the mode has set its own.
	 */
	void apply_takeoff_exit_blend(const hrt_abstime now);

	// class handling launch detection methods for fixed-wing takeoff
	LaunchDetector _launchDetector;

	// true if a launch, specifically using the launch detector, has been detected
	bool _launch_detected{false};

	// [deg] global position of the vehicle at the time launch is detected (using launch detector) or takeoff is started (runway)
	Vector2d _takeoff_init_position{0, 0};

	// [rad] current vehicle yaw at the time the launch is detected
	float _launch_current_yaw{0.f};

	// class handling runway takeoff for fixed-wing UAVs with steerable wheels
	RunwayTakeoff _runway_takeoff;

	bool _skipping_takeoff_detection{false};

	// AUTO LANDING

	// corresponds to param FW_LND_NUDGE
	enum LandingNudgingOption {
		kNudgingDisabled = 0,
		kNudgeApproachAngle,
		kNudgeApproachPath
	};

	// [us] Start time of the landing approach. If a fixed-wing landing pattern is used, this timer starts *after any
	// orbit to altitude only when the aircraft has entered the final *straight approach.
	hrt_abstime _time_started_landing{0};

	// [m] lateral touchdown position offset manually commanded during landing
	float _lateral_touchdown_position_offset{0.0f};

	// [m] relative vector from land point to approach entrance (NE)
	Vector2f _landing_approach_entrance_offset_vector{};

	// [m] relative height above land point
	float _landing_approach_entrance_rel_alt{0.0f};

	uint8_t _landing_abort_status{position_controller_landing_status_s::NOT_ABORTED};

	// organize flare states XXX: need to split into a separate class at some point!
	struct FlareStates {
		bool flaring{false};
		hrt_abstime start_time{0}; // [us]
		float initial_height_rate_setpoint{0.0f}; // [m/s]
		float start_height{0.0f}; // [m] height above the touchdown point when the flare started
		float start_pitch{0.0f}; // [rad] pitch angle when the flare started (flare law 3)
		float sink_rate_integral{0.0f}; // [rad] integral term of the flare law 3 pitch command
		float throttle_integral{0.0f}; // [-] integral term of the flare law 4 throttle command
		float attitude_offset{0.0f}; // [rad] flare law 4 landing attitude reduction by the airspeed guard (<= 0)
		bool attitude_hold{false}; // flare law 4: false = decelerating (pitch holds the sink rate, idle throttle)
		float hold_pitch{0.0f}; // [rad] flare law 4 pitch command, rate limited towards the landing attitude
		float throttle_cmd{0.0f}; // [-] flare law 4 throttle command, slew rate limited
		float throttle_start{0.0f}; // [-] flare law 6 throttle at flare start (centre of the airspeed PI)
		float start_airspeed{0.0f}; // [m/s] flare law 6 airspeed at flare start
		float pitch_cmd{0.0f}; // [rad] flare law 6 rate limited pitch command
		bool touchdown{false}; // flare law 6: touchdown latched (sink rate collapsed near the ground)
		float touchdown_timer{0.0f}; // [s] flare law 6 time the touchdown condition has held
		float last_range{NAN}; // [m] flare law 6 last valid range finder reading (water: hull under the radar)
		hrt_abstime last_update{0}; // [us] last flare law 4 update
	} _flare_states;

	// Flare law 6 step (shared by the runway landing and the water scoop: same algorithm, surface profile)
	struct Law6Profile {
		float vtd_k;            // touchdown airspeed / estimated stall speed
		float pitch_max_rad;    // flare pitch upper limit
		bool water;             // true: no retard and no runway pitch after touchdown (scoop continues)
	};

	struct Law6Output {
		float pitch;            // [rad]
		float pitch_min;        // [rad]
		float pitch_max;        // [rad]
		float throttle;         // [-]
		float sink_setpoint;    // [m/s] down positive
		float airspeed_setpoint; // [m/s]
		bool touched_down;
	};

	void law6Start(FlareStates &fs, const hrt_abstime now, const float height);
	Law6Output law6Step(FlareStates &fs, const hrt_abstime now, const float height, const Law6Profile &profile);

	// GASI water scoop (mission item MAV_CMD_USER_1 + two waypoints at the water surface): approach on a glide slope,
	// flare law 6 (water profile), touchdown with power, hold the scoop airspeed, take off. Go-around by
	// MAV_CMD_DO_GO_AROUND or own checks; afterwards AUTO.LOITER, the planner decides (MISSION_SET_CURRENT).
	enum class ScoopPhase : uint8_t { OFF = 0, APPROACH = 1, GATE_PASSED = 2, FLARE = 3, TOUCHDOWN = 4, SCOOP = 5,
					  TAKEOFF = 6, DONE = 7, ABORT = 8 };

	struct ScoopState {
		bool armed{false};
		float target_speed{25.f};
		float duration{10.f};
		float speed_min{22.22f};
		float water_amsl{NAN};
		ScoopPhase phase{ScoopPhase::OFF};
		Vector2f entrance{};    // local, approach line start
		Vector2f touchdown{};   // local, touchdown point (first water waypoint)
		float entrance_alt{0.f};
		hrt_abstime phase_start{0};
		hrt_abstime last_update{0};
		hrt_abstime last_status{0};
		float scoop_time{0.f};  // [s] in the speed band on the water
		float out_of_band{0.f}; // [s] out of the band after the scoop started
		float out_of_abort_band{0.f}; // [s] out of the abort band (FW_SC_A_VMIN..FW_SC_A_VMAX) after the scoop started
		float speed_lo{INFINITY};
		float speed_hi{0.f};
		bool scoop_started{false};
		float pitch_cmd{0.f};
		float throttle_cmd{0.f};
		float throttle_integral{0.f};
		bool loiter_sent{false};
		bool on_water{false}; // touchdown latched: the radar may be under the water line, height from the altitude
		float last_airspeed{0.f};
		float accel_filt{0.f}; // [m/s^2] filtered airspeed rate on the water
		float liftoff_airspeed{0.f};
		hrt_abstime liftoff_time{0};
		FlareStates flare{};
	} _scoop;

	orb_advert_t _mavlink_log_pub{nullptr};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<debug_key_value_s> _debug_key_value_pub{ORB_ID(debug_key_value)};

	static const char *scoopPhaseName(ScoopPhase phase);
	void setScoopPhase(const hrt_abstime now, ScoopPhase phase, const char *detail = nullptr);
	void scoopAbort(const hrt_abstime now, const char *reason);
	bool scoopEngage(const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr);
	void control_auto_scoop(const hrt_abstime &now, const float control_interval, const Vector2f &ground_speed,
				const position_setpoint_s &pos_sp_curr);

	// [m] last terrain estimate which was valid
	float _last_valid_terrain_alt_estimate{0.0f};

	// [us] time at which we had last valid terrain alt
	hrt_abstime _last_time_terrain_alt_was_valid{0};

	enum TerrainEstimateUseOnLanding {
		kDisableTerrainEstimation = 0,
		kTriggerFlareWithTerrainEstimate,
		kFollowTerrainRelativeLandingGlideSlope
	};

	// AIRSPEED

	float _airspeed_eas{0.f};
	bool _airspeed_valid{false};

	// [us] last time airspeed was received. used to detect timeouts.
	hrt_abstime _time_airspeed_last_valid{0};
	// [us] last accepted airspeed sample. _time_airspeed_last_valid is never set in this version (upstream), so
	// _airspeed_valid is always false; flare law 4 uses this one instead and leaves the upstream logic as it is.
	hrt_abstime _time_airspeed_sample{0};

	// WIND

	// [m/s] wind velocity vector
	Vector2f _wind_vel{0.0f, 0.0f};

	bool _wind_valid{false};

	hrt_abstime _time_wind_last_received{0}; // [us]

	// VTOL / TRANSITION
	matrix::Vector2d _transition_waypoint{(double)NAN, (double)NAN};
	float _backtrans_heading{NAN};	// used to lock the initial heading for backtransition with no position control

	// ESTIMATOR RESET COUNTERS
	uint8_t _xy_reset_counter{0};
	uint64_t _time_last_xy_reset{0};

	// LATERAL-DIRECTIONAL GUIDANCE

	// CLosest point on path to track
	matrix::Vector2f _closest_point_on_path;

	// nonlinear path following guidance - lateral-directional position control
	DirectionalGuidance _directional_guidance;

	// LANDING GEAR
	int8_t _new_landing_gear_position{landing_gear_s::GEAR_KEEP};

	// FLAPS/SPOILERS
	float _flaps_setpoint{0.f};
	float _spoilers_setpoint{0.f};

	hrt_abstime _time_in_fixed_bank_loiter{0}; // [us]
	float _min_current_sp_distance_xy{FLT_MAX};

#ifdef CONFIG_FIGURE_OF_EIGHT
	/* Loitering */
	FigureEight _figure_eight;
	uORB::Publication<figure_eight_status_s> _figure_eight_status_pub {ORB_ID(figure_eight_status)};
	/**
	 * Vehicle control for the autonomous figure 8 mode.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos the current 2D absolute position of the vehicle in [deg].
	 * @param ground_speed the 2D ground speed of the vehicle in [m/s].
	 * @param pos_sp_curr the current position setpoint.
	 */
	void controlAutoFigureEight(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
				    const position_setpoint_s &pos_sp_curr);

	void publishFigureEightStatus(const position_setpoint_s pos_sp);
#endif // CONFIG_FIGURE_OF_EIGHT

	// Update our local parameter cache.
	void parameters_update();

	// Update subscriptions
	void airspeed_poll();

	void manual_control_setpoint_poll();
	void vehicle_attitude_poll();
	void vehicle_command_poll();
	void vehicle_control_mode_poll();

	void wind_poll(const hrt_abstime now);

	void landing_status_publish();

	void publishLocalPositionSetpoint(const position_setpoint_s &current_waypoint);

	/**
	 * @brief Sets the landing abort status and publishes landing status.
	 *
	 * @param new_abort_status Either 0 (not aborted) or the singular bit >0 which triggered the abort
	 */
	void updateLandingAbortStatus(const uint8_t new_abort_status = position_controller_landing_status_s::NOT_ABORTED);

	/**
	 * @brief Checks if the automatic abort bitmask (from FW_LND_ABORT) contains the given abort criterion.
	 *
	 * @param automatic_abort_criteria_bitmask Bitmask containing all active abort criteria
	 * @param landing_abort_criterion The specifc criterion we are checking for
	 * @return true if the bitmask contains the criterion
	 */
	bool checkLandingAbortBitMask(const uint8_t automatic_abort_criteria_bitmask, uint8_t landing_abort_criterion);

	/**
	 * @brief Maps the manual control setpoint (pilot sticks) to height rate commands
	 *
	 * @return Manual height rate setpoint [m/s]
	 */
	float getManualHeightRateSetpoint();

	/**
	 * @brief Updates a state indicating whether a manual takeoff has been completed.
	 *
	 * Criteria include passing an airspeed threshold and not being in a landed state. VTOL airframes always pass.
	 */
	void updateManualTakeoffStatus();

	/**
	 * @brief Updates timing information for landed and in-air states.
	 *
	 * @param now Current system time [us]
	 */
	void update_in_air_states(const hrt_abstime now);

	/**
	 * @brief Moves the current position setpoint to a value far ahead of the current vehicle yaw when in  a VTOL
	 * transition.
	 *
	 * @param[in,out] current_sp Current position setpoint
	 */
	void move_position_setpoint_for_vtol_transition(position_setpoint_s &current_sp);

	/**
	 * @brief Changes the position setpoint type to achieve the desired behavior in some instances.
	 *
	 * @param pos_sp_curr Current position setpoint
	 * @return Adjusted position setpoint type
	 */
	uint8_t handle_setpoint_type(const position_setpoint_s &pos_sp_curr,
				     const position_setpoint_s &pos_sp_next);

	/* automatic control methods */

	/**
	 * @brief Automatic position control for waypoints, orbits, and velocity control
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_prev previous position setpoint
	 * @param pos_sp_curr current position setpoint
	 * @param pos_sp_next next position setpoint
	 */
	void control_auto(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
			  const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr, const position_setpoint_s &pos_sp_next);

	/**
	 * @brief Controls altitude and airspeed for a fixed-bank loiter.
	 *
	 * Used as a failsafe mode after a lateral position estimate failure.
	 */
	void control_auto_fixed_bank_alt_hold();

	/**
	 * @brief Control airspeed with a fixed descent rate and roll angle.
	 *
	 * Used as a failsafe mode after a lateral position estimate failure.
	 */
	void control_auto_descend();

	/**
	 * @brief Vehicle control for position waypoints.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_prev previous position setpoint
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_position(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
				   const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr);

	/**
	 * @brief Vehicle control for loiter waypoints.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_curr current position setpoint
	 * @param pos_sp_next next position setpoint
	 */
	void control_auto_loiter(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
				 const position_setpoint_s &pos_sp_curr, const position_setpoint_s &pos_sp_next);


	/**
	 * @brief Vehicle control for following a path.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_prev previous position setpoint
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_path(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
			       const position_setpoint_s &pos_sp_curr);

	/**
	 * @brief Controls a desired airspeed, bearing, and height rate.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_velocity(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
				   const position_setpoint_s &pos_sp_curr);

	/**
	 * @brief Controls automatic takeoff.
	 *
	 * @param now Current system time [us]
	 * @param control_interval Time since last position control call [s]
	 * @param global_position Vechile global position [deg]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_takeoff(const hrt_abstime &now, const float control_interval, const Vector2d &global_position,
				  const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr);

	/**
	 * @brief Controls automatic takeoff without navigation.
	 *
	 * @param now Current system time [us]
	 * @param control_interval Time since last position control call [s]
	 * @param altitude_setpoint_amsl Altitude setpoint, AMSL [m]
	 */
	void control_auto_takeoff_no_nav(const hrt_abstime &now, const float control_interval,
					 const float altitude_setpoint_amsl);

	/**
	 * @brief Controls automatic landing with straight approach.
	 *
	 * To be used in Missions that contain a loiter down followed by a land waypoint.
	 *
	 * @param now Current system time [us]
	 * @param control_interval Time since last position control call [s]
	 * @param control_interval Time since the last position control update [s]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_prev previous position setpoint
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_landing_straight(const hrt_abstime &now, const float control_interval, const Vector2f &ground_speed,
					   const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr);

	/**
	 * @brief Controls automatic landing with circular final appraoch.
	 *
	 * To be used outside of Mission landings. Vehicle will orbit down around the landing position setpoint until flaring.
	 *
	 * @param now Current system time [us]
	 * @param control_interval Time since last position control call [s]
	 * @param control_interval Time since the last position control update [s]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_landing_circular(const hrt_abstime &now, const float control_interval, const Vector2f &ground_speed,
					   const position_setpoint_s &pos_sp_curr);

	/* manual control methods */

	/**
	 * @brief Controls altitude and airspeed, user commands roll setpoint.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 */
	void control_manual_altitude(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed);

	/**
	 * @brief Controls user commanded altitude, airspeed, and bearing.
	 *
	 * @param now Current system time [us]
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 */
	void control_manual_position(const hrt_abstime now, const float control_interval, const Vector2d &curr_pos,
				     const Vector2f &ground_speed);

	/**
	 * @brief Holds the initial heading during the course of a transition to hover. Used when there is no local
	 * position to do line following.
	 */
	void control_backtransition_heading_hold();

	/**
	 * @brief Controls flying towards a transition waypoint and then transitioning to MC mode.
	 *
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_curr current position setpoint
	 */
	void control_backtransition_line_follow(const Vector2f &ground_speed,
						const position_setpoint_s &pos_sp_curr);

	float get_manual_airspeed_setpoint();

	void reset_takeoff_state();
	void reset_landing_state();

	/**
	 * @brief Decides which control mode to execute.
	 *
	 * May also change the position setpoint type depending on the desired behavior.
	 *
	 * @param now Current system time [us]
	 */
	void set_control_mode_current(const hrt_abstime &now);

	void publishOrbitStatus(const position_setpoint_s pos_sp);

	float getMaxRollAngleNearGround(const float altitude, const float terrain_altitude) const;

	/**
	 * @brief Calculates the touchdown position for landing with optional manual lateral adjustments.
	 *
	 * Manual inputs (from the remote) are used to command a rate at which the position moves and the integrated
	 * position is bounded. This is useful for manually adjusting the landing point in real time when map or GNSS
	 * errors cause an offset from the desired landing vector.
	 *
	 * @param control_interval Time since the last position control update [s]
	 * @param local_land_position Originally commanded local land position (NE) [m]
	 * @return (Nudged) Local touchdown position (NE) [m]
	 */
	Vector2f calculateTouchdownPosition(const float control_interval, const Vector2f &local_land_position);

	/**
	 * @brief Calculates the vector from landing approach entrance to touchdown point
	 *
	 * NOTE: calculateTouchdownPosition() MUST be called before this method
	 *
	 * @return Landing approach vector [m]
	 */
	Vector2f calculateLandingApproachVector() const;

	/**
	 * @brief Returns a terrain altitude estimate with consideration of altimeter measurements.
	 *
	 * @param now Current system time [us]
	 * @param land_point_altitude Altitude (AMSL) of the land point [m]
	 * @param abort_on_terrain_measurement_timeout Abort if distance to ground estimation doesn't get valid when we expect it to
	 * @param abort_on_terrain_timeout Abort if distance to ground estimation is invalid after being valid before
	 * @return Terrain altitude (AMSL) [m]
	 */
	/**
	 * @brief Updates the flare law 6 stall speed estimate on the landing approach (frozen while flaring).
	 *
	 * @param height Height above the touchdown point [m]
	 * @return Estimated 1 g stall speed with landing flaps [m/s], NAN until settled
	 */
	float updateStallSpeedEstimate(const hrt_abstime now, const float height);

	float getLandingTerrainAltitudeEstimate(const hrt_abstime &now, const float land_point_altitude,
						const bool abort_on_terrain_measurement_timeout, const bool abort_on_terrain_timeout);

	/**
	 * @brief Initializes landing states
	 *
	 * @param now Current system time [us]
	 * @param pos_sp_prev Previous position setpoint
	 * @param land_point_alt Landing point altitude setpoint AMSL [m]
	 * @param local_position Local aircraft position (NE) [m]
	 * @param local_land_point Local land point (NE) [m]
	 */
	void initializeAutoLanding(const hrt_abstime &now, const position_setpoint_s &pos_sp_prev,
				   const float land_point_alt, const Vector2f &local_position, const Vector2f &local_land_point);

	/*
	 * Waypoint handling logic following closely to the ECL_L1_Pos_Controller
	 * method of the same name. Takes two waypoints, steering the vehicle to track
	 * the line segment between them.
	 *
	 * @param[in] start_waypoint Segment starting position in local coordinates. (N,E) [m]
	 * @param[in] end_waypoint Segment end position in local coordinates. (N,E) [m]
	 * @param[in] vehicle_pos Vehicle position in local coordinates. (N,E) [m]
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 */
	DirectionalGuidanceOutput navigateWaypoints(const matrix::Vector2f &start_waypoint,
			const matrix::Vector2f &end_waypoint,
			const matrix::Vector2f &vehicle_pos, const matrix::Vector2f &ground_vel,
			const matrix::Vector2f &wind_vel);

	/*
	 * Takes one waypoint and steers the vehicle towards this.
	 *
	 * NOTE: this *will lead to "flowering" behavior if no higher level state machine or
	 * switching condition changes the waypoint.
	 *
	 * @param[in] waypoint_pos Waypoint position in local coordinates. (N,E) [m]
	 * @param[in] vehicle_pos Vehicle position in local coordinates. (N,E) [m]
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 */
	DirectionalGuidanceOutput navigateWaypoint(const matrix::Vector2f &waypoint_pos, const matrix::Vector2f &vehicle_pos,
			const matrix::Vector2f &ground_vel, const matrix::Vector2f &wind_vel);

	/*
	 * Line (infinite) following logic. Two points on the line are used to define the
	 * line in 2D space (first to second point determines the direction). Determines the
	 * relevant parameters for evaluating the NPFG guidance law, then updates control setpoints.
	 *
	 * @param[in] point_on_line_1 Arbitrary first position on line in local coordinates. (N,E) [m]
	 * @param[in] point_on_line_2 Arbitrary second position on line in local coordinates. (N,E) [m]
	 * @param[in] vehicle_pos Vehicle position in local coordinates. (N,E) [m]
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 */
	DirectionalGuidanceOutput navigateLine(const Vector2f &point_on_line_1, const Vector2f &point_on_line_2,
					       const Vector2f &vehicle_pos,
					       const Vector2f &ground_vel, const Vector2f &wind_vel);

	/*
	 * Line (infinite) following logic. One point on the line and a line bearing are used to define
	 * the line in 2D space. Determines the relevant parameters for evaluating the NPFG guidance law,
	 * then updates control setpoints.
	 *
	 * @param[in] point_on_line Arbitrary position on line in local coordinates. (N,E) [m]
	 * @param[in] line_bearing Line bearing [rad] (from north)
	 * @param[in] vehicle_pos Vehicle position in local coordinates. (N,E) [m]
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 */
	DirectionalGuidanceOutput navigateLine(const Vector2f &point_on_line, const float line_bearing,
					       const Vector2f &vehicle_pos,
					       const Vector2f &ground_vel, const Vector2f &wind_vel);

	/*
	 * Loitering (unlimited) logic. Takes loiter center, radius, and direction and
	 * determines the relevant parameters for evaluating the NPFG guidance law,
	 * then updates control setpoints.
	 *
	 * @param[in] loiter_center The position of the center of the loiter circle [m]
	 * @param[in] vehicle_pos Vehicle position in local coordinates. (N,E) [m]
	 * @param[in] radius Loiter radius [m]
	 * @param[in] loiter_direction_counter_clockwise Specifies loiter direction
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 */
	DirectionalGuidanceOutput navigateLoiter(const matrix::Vector2f &loiter_center, const matrix::Vector2f &vehicle_pos,
			float radius, bool loiter_direction_counter_clockwise, const matrix::Vector2f &ground_vel,
			const matrix::Vector2f &wind_vel);

	/*
	 * Path following logic. Takes poisiton, path tangent, curvature and
	 * then updates control setpoints to follow a path setpoint.
	 *
	 * TODO: deprecate this function with a proper API to NPFG.
	 *
	 * @param[in] vehicle_pos vehicle_pos Vehicle position in local coordinates. (N,E) [m]
	 * @param[in] position_setpoint closest point on a path in local coordinates. (N,E) [m]
	 * @param[in] tangent_setpoint unit tangent vector of the path [m]
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 * @param[in] curvature of the path setpoint [1/m]
	 */
	DirectionalGuidanceOutput navigatePathTangent(const matrix::Vector2f &vehicle_pos,
			const matrix::Vector2f &position_setpoint,
			const matrix::Vector2f &tangent_setpoint,
			const matrix::Vector2f &ground_vel, const matrix::Vector2f &wind_vel, const float &curvature);

	/*
	 * Navigate on a fixed bearing.
	 *
	 * This only holds a certain (ground relative) direction and does not perform
	 * cross track correction. Helpful for semi-autonomous modes.
	 *
	 * @param[in] vehicle_pos vehicle_pos Vehicle position in local coordinates. (N,E) [m]
	 * @param[in] bearing Bearing angle [rad]
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 */
	DirectionalGuidanceOutput navigateBearing(const matrix::Vector2f &vehicle_pos, float bearing,
			const matrix::Vector2f &ground_vel,
			const matrix::Vector2f &wind_vel);

	void control_idle();
	void publish_lateral_guidance_status(const hrt_abstime now);

	float rollAngleToLateralAccel(float roll_body) const;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_R_LIM>) _param_fw_r_lim,

		(ParamFloat<px4::params::NPFG_PERIOD>) _param_npfg_period,
		(ParamFloat<px4::params::NPFG_DAMPING>) _param_npfg_damping,
		(ParamBool<px4::params::NPFG_LB_PERIOD>) _param_npfg_en_period_lb,
		(ParamBool<px4::params::NPFG_UB_PERIOD>) _param_npfg_en_period_ub,
		(ParamFloat<px4::params::NPFG_ROLL_TC>) _param_npfg_roll_time_const,
		(ParamFloat<px4::params::NPFG_SW_DST_MLT>) _param_npfg_switch_distance_multiplier,
		(ParamFloat<px4::params::NPFG_PERIOD_SF>) _param_npfg_period_safety_factor,

		(ParamFloat<px4::params::FW_LND_AIRSPD>) _param_fw_lnd_airspd,
		(ParamFloat<px4::params::FW_LND_ANG>) _param_fw_lnd_ang,
		(ParamFloat<px4::params::FW_LND_FL_PMAX>) _param_fw_lnd_fl_pmax,
		(ParamFloat<px4::params::FW_LND_FL_PMIN>) _param_fw_lnd_fl_pmin,
		(ParamFloat<px4::params::FW_LND_FLALT>) _param_fw_lnd_flalt,
		(ParamBool<px4::params::FW_LND_EARLYCFG>) _param_fw_lnd_earlycfg,
		(ParamInt<px4::params::FW_LND_USETER>) _param_fw_lnd_useter,

		(ParamFloat<px4::params::FW_P_LIM_MAX>) _param_fw_p_lim_max,
		(ParamFloat<px4::params::FW_P_LIM_MIN>) _param_fw_p_lim_min,
		(ParamFloat<px4::params::FW_T_CLMB_R_SP>) _param_climbrate_target,
		(ParamFloat<px4::params::FW_T_SINK_R_SP>) _param_sinkrate_target,
		(ParamFloat<px4::params::FW_THR_IDLE>) _param_fw_thr_idle,
		(ParamFloat<px4::params::FW_THR_MAX>) _param_fw_thr_max,
		(ParamFloat<px4::params::FW_THR_MIN>) _param_fw_thr_min,
		(ParamFloat<px4::params::FW_FLAPS_LND_SCL>) _param_fw_flaps_lnd_scl,
		(ParamFloat<px4::params::FW_FLAPS_TO_SCL>) _param_fw_flaps_to_scl,
		(ParamFloat<px4::params::FW_SPOILERS_LND>) _param_fw_spoilers_lnd,
		(ParamInt<px4::params::FW_POS_STK_CONF>) _param_fw_pos_stk_conf,
		(ParamInt<px4::params::FW_GPSF_LT>) _param_nav_gpsf_lt,
		(ParamFloat<px4::params::FW_GPSF_R>) _param_nav_gpsf_r,
		(ParamFloat<px4::params::FW_T_SPDWEIGHT>) _param_t_spdweight,

		// external parameters
		(ParamBool<px4::params::FW_USE_AIRSPD>) _param_fw_use_airspd,
		(ParamFloat<px4::params::NAV_LOITER_RAD>) _param_nav_loiter_rad,
		(ParamFloat<px4::params::FW_TKO_PITCH_MIN>) _takeoff_pitch_min,
		(ParamFloat<px4::params::NAV_FW_ALT_RAD>) _param_nav_fw_alt_rad,
		(ParamFloat<px4::params::FW_WING_SPAN>) _param_fw_wing_span,
		(ParamFloat<px4::params::FW_WING_HEIGHT>) _param_fw_wing_height,
		(ParamBool<px4::params::RWTO_NUDGE>) _param_rwto_nudge,
		(ParamFloat<px4::params::FW_LND_FL_TIME>) _param_fw_lnd_fl_time,
		(ParamFloat<px4::params::FW_LND_FL_SINK>) _param_fw_lnd_fl_sink,
		(ParamInt<px4::params::FW_LND_FL_LAW>) _param_fw_lnd_fl_law,
		(ParamFloat<px4::params::FW_LND_FL_HOLD>) _param_fw_lnd_fl_hold,
		(ParamFloat<px4::params::FW_LND_FL_SPDW>) _param_fw_lnd_fl_spdw,
		(ParamFloat<px4::params::FW_LND_FL_KP>) _param_fw_lnd_fl_kp,
		(ParamFloat<px4::params::FW_LND_FL_KI>) _param_fw_lnd_fl_ki,
		(ParamFloat<px4::params::FW_LND_FL_ATT>) _param_fw_lnd_fl_att,
		(ParamFloat<px4::params::FW_LND_FL_TATT>) _param_fw_lnd_fl_tatt,
		(ParamFloat<px4::params::FW_LND_FL_SNK1>) _param_fw_lnd_fl_snk1,
		(ParamFloat<px4::params::FW_LND_FL_TFF>) _param_fw_lnd_fl_tff,
		(ParamFloat<px4::params::FW_LND_FL_TKP>) _param_fw_lnd_fl_tkp,
		(ParamFloat<px4::params::FW_LND_FL_TKI>) _param_fw_lnd_fl_tki,
		(ParamFloat<px4::params::FW_LND_FL_VMIN>) _param_fw_lnd_fl_vmin,
		(ParamFloat<px4::params::FW_LND_FL_VATT>) _param_fw_lnd_fl_vatt,
		(ParamFloat<px4::params::FW_LND_FL_TSLW>) _param_fw_lnd_fl_tslw,
		(ParamFloat<px4::params::FW_LND_VS_CL0>) _param_fw_lnd_vs_cl0,
		(ParamFloat<px4::params::FW_LND_VS_CLA>) _param_fw_lnd_vs_cla,
		(ParamFloat<px4::params::FW_LND_VS_CLMX>) _param_fw_lnd_vs_clmx,
		(ParamFloat<px4::params::FW_LND_VAPP_K>) _param_fw_lnd_vapp_k,
		(ParamFloat<px4::params::FW_LND_VTD_K>) _param_fw_lnd_vtd_k,
		(ParamFloat<px4::params::FW_LND_FL_PRT>) _param_fw_lnd_fl_prt,
		(ParamFloat<px4::params::FW_SC_VTD_K>) _param_fw_sc_vtd_k,
		(ParamFloat<px4::params::FW_SC_PMAX>) _param_fw_sc_pmax,
		(ParamFloat<px4::params::FW_SC_PITCH>) _param_fw_sc_pitch,
		(ParamFloat<px4::params::FW_SC_TO_PITCH>) _param_fw_sc_to_pitch,
		(ParamFloat<px4::params::FW_SC_VMAX>) _param_fw_sc_vmax,
		(ParamFloat<px4::params::FW_SC_TKP>) _param_fw_sc_tkp,
		(ParamFloat<px4::params::FW_SC_TKI>) _param_fw_sc_tki,
		(ParamFloat<px4::params::FW_SC_GA_ALT>) _param_fw_sc_ga_alt,
		(ParamFloat<px4::params::FW_SC_LOOK>) _param_fw_sc_look,
		(ParamFloat<px4::params::FW_SC_THR_FF>) _param_fw_sc_thr_ff,
		(ParamFloat<px4::params::FW_SC_FF_VS>) _param_fw_sc_ff_vs,
		(ParamFloat<px4::params::FW_SC_FILL>) _param_fw_sc_fill,
		(ParamFloat<px4::params::FW_SC_TMAX>) _param_fw_sc_tmax,
		(ParamFloat<px4::params::FW_SC_P_THR>) _param_fw_sc_p_thr,
		(ParamFloat<px4::params::FW_SC_LIFT_H>) _param_fw_sc_lift_h,
		(ParamFloat<px4::params::FW_SC_CLB_DEC>) _param_fw_sc_clb_dec,
		(ParamFloat<px4::params::FW_SC_V_MARG>) _param_fw_sc_v_marg,
		(ParamFloat<px4::params::FW_SC_A_VMIN>) _param_fw_sc_a_vmin,
		(ParamFloat<px4::params::FW_SC_A_VMAX>) _param_fw_sc_a_vmax,
		(ParamFloat<px4::params::FW_SC_G_XTRK>) _param_fw_sc_g_xtrk,
		(ParamFloat<px4::params::FW_SC_G_CRS>) _param_fw_sc_g_crs,
		(ParamFloat<px4::params::FW_SC_G_ROLL>) _param_fw_sc_g_roll,
		(ParamFloat<px4::params::FW_SC_G_SPD>) _param_fw_sc_g_spd,
		(ParamFloat<px4::params::FW_SC_A_ROLL>) _param_fw_sc_a_roll,
		(ParamFloat<px4::params::FW_SC_A_XTRK>) _param_fw_sc_a_xtrk,
		(ParamFloat<px4::params::FW_SC_A_HMIN>) _param_fw_sc_a_hmin,
		(ParamFloat<px4::params::FW_SC_OOB_T>) _param_fw_sc_oob_t,
		(ParamFloat<px4::params::FW_SC_T_MARG>) _param_fw_sc_t_marg,
		(ParamFloat<px4::params::FW_SC_DONE_H>) _param_fw_sc_done_h,
		(ParamFloat<px4::params::FW_SC_CLB_H>) _param_fw_sc_clb_h,
		(ParamFloat<px4::params::FW_SC_GA_H>) _param_fw_sc_ga_h,
		(ParamFloat<px4::params::FW_LND_TD_RNG>) _param_fw_lnd_td_rng,
		(ParamFloat<px4::params::FW_LND_TD_VZ>) _param_fw_lnd_td_vz,
		(ParamFloat<px4::params::FW_LND_TD_T>) _param_fw_lnd_td_t,
		(ParamFloat<px4::params::FW_LND_VS_HMIN>) _param_fw_lnd_vs_hmin,
		(ParamFloat<px4::params::FW_LND_VS_ROLL>) _param_fw_lnd_vs_roll,
		(ParamFloat<px4::params::FW_LND_VS_LVL>) _param_fw_lnd_vs_lvl,
		(ParamFloat<px4::params::FW_LND_VS_TAU>) _param_fw_lnd_vs_tau,
		(ParamFloat<px4::params::WEIGHT_BASE>) _param_weight_base,
		(ParamFloat<px4::params::WEIGHT_GROSS>) _param_weight_gross,
		(ParamFloat<px4::params::FW_LND_TD_TIME>) _param_fw_lnd_td_time,
		(ParamFloat<px4::params::FW_LND_TD_OFF>) _param_fw_lnd_td_off,
		(ParamInt<px4::params::FW_LND_NUDGE>) _param_fw_lnd_nudge,
		(ParamInt<px4::params::FW_LND_ABORT>) _param_fw_lnd_abort,
		(ParamFloat<px4::params::FW_TKO_AIRSPD>) _param_fw_tko_airspd,
		(ParamFloat<px4::params::FW_TKO_BLEND_T>) _param_fw_tko_blend_t,
		(ParamFloat<px4::params::RWTO_PSP>) _param_rwto_psp,
		(ParamBool<px4::params::FW_LAUN_DETCN_ON>) _param_fw_laun_detcn_on,
		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamFloat<px4::params::FW_T_CLMB_MAX>) _param_fw_t_clmb_max
	)
};

#endif // FIXEDWINGMODEMANAGER_HPP_
