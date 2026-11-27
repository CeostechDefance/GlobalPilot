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

#include "FixedWingModeManager.hpp"

#include <px4_platform_common/events.h>
#include <uORB/topics/longitudinal_control_configuration.h>

using math::constrain;
using math::max;
using math::min;
using math::radians;

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector2d;
using matrix::Vector3f;
using matrix::wrap_pi;

const fixed_wing_lateral_setpoint_s empty_lateral_control_setpoint = {.timestamp = 0, .course = NAN, .airspeed_direction = NAN, .lateral_acceleration = NAN};
const fixed_wing_longitudinal_setpoint_s empty_longitudinal_control_setpoint = {.timestamp = 0, .altitude = NAN, .height_rate = NAN, .equivalent_airspeed = NAN, .pitch_direct = NAN, .throttle_direct = NAN};

FixedWingModeManager::FixedWingModeManager() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_launchDetector(this),
	_runway_takeoff(this)
#ifdef CONFIG_FIGURE_OF_EIGHT
	, _figure_eight(_directional_guidance, _wind_vel)
#endif // CONFIG_FIGURE_OF_EIGHT
{
	// limit to 50 Hz
	_local_pos_sub.set_interval_ms(20);

	_pos_ctrl_landing_status_pub.advertise();
	_launch_detection_status_pub.advertise();
	_landing_gear_pub.advertise();
	_flaps_setpoint_pub.advertise();
	_spoilers_setpoint_pub.advertise();
	_fixed_wing_lateral_guidance_status_pub.advertise();
	_fixed_wing_runway_control_pub.advertise();

	parameters_update();
}

FixedWingModeManager::~FixedWingModeManager()
{
	perf_free(_loop_perf);
}

bool
FixedWingModeManager::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
FixedWingModeManager::parameters_update()
{
	updateParams();

	_directional_guidance.setPeriod(_param_npfg_period.get());
	_directional_guidance.setDamping(_param_npfg_damping.get());
	_directional_guidance.enablePeriodLB(_param_npfg_en_period_lb.get());
	_directional_guidance.enablePeriodUB(_param_npfg_en_period_ub.get());
	_directional_guidance.setRollTimeConst(_param_npfg_roll_time_const.get());
	_directional_guidance.setSwitchDistanceMultiplier(_param_npfg_switch_distance_multiplier.get());
	_directional_guidance.setPeriodSafetyFactor(_param_npfg_period_safety_factor.get());
}

void
FixedWingModeManager::vehicle_control_mode_poll()
{
	if (_control_mode_sub.updated()) {
		const bool was_armed = _control_mode.flag_armed;

		if (_control_mode_sub.copy(&_control_mode)) {

			// reset state when arming
			if (!was_armed && _control_mode.flag_armed) {
				reset_takeoff_state();
				reset_landing_state();
			}
		}
	}
}

void
FixedWingModeManager::vehicle_command_poll()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND) {
			bool accepted = false;

			// water scoop: any phase up to the takeoff can be aborted by the planner / operator
			if (_scoop.phase >= ScoopPhase::APPROACH && _scoop.phase <= ScoopPhase::TAKEOFF) {
				scoopAbort(hrt_absolute_time(), "OPERATOR");
				accepted = true;

			// only abort landing before point of no return (horizontal and vertical)
			} else if (_control_mode.flag_control_auto_enabled &&
				   _position_setpoint_current_valid &&
				   (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND)) {

				updateLandingAbortStatus(position_controller_landing_status_s::ABORTED_BY_OPERATOR);
				accepted = true;
			}

			// commander leaves DO_GO_AROUND unacknowledged: answer here so the planner knows it arrived
			publishCommandAck(vehicle_command, accepted ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED
					  : vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);

		} else if (vehicle_command.command == VEHICLE_CMD_SCOOP) {
			// MAV_CMD_USER_1 water scoop (mission item or COMMAND_LONG): param1 1 arm / 0 disarm, param2 airspeed, param3
			// duration, param4 lower band limit, param7 water surface AMSL
			if (vehicle_command.param1 > 0.5f && PX4_ISFINITE(vehicle_command.param7)) {
				_scoop = ScoopState{};
				_scoop.armed = true;
				_scoop.target_speed = (vehicle_command.param2 > 5.f) ? vehicle_command.param2 : 25.f;
				_scoop.duration = (vehicle_command.param3 > 0.5f) ? vehicle_command.param3 : 10.f;
				_scoop.speed_min = (vehicle_command.param4 > 5.f) ? vehicle_command.param4 : 22.22f;
				_scoop.water_amsl = (float)vehicle_command.param7;
				mavlink_log_info(&_mavlink_log_pub, "SU: KURULDU hiz=%.1f sure=%.0f bant=%.2f-%.2f su=%.1f\t",
						 (double)_scoop.target_speed, (double)_scoop.duration, (double)_scoop.speed_min,
						 (double)_param_fw_sc_vmax.get(), (double)_scoop.water_amsl);

			} else {
				if (_scoop.phase >= ScoopPhase::APPROACH && _scoop.phase <= ScoopPhase::TAKEOFF) {
					scoopAbort(hrt_absolute_time(), "OPERATOR");
				}

				_scoop.armed = false;
				mavlink_log_info(&_mavlink_log_pub, "SU: KAPATILDI\t");
			}

			publishCommandAck(vehicle_command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

		} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED) {

			if ((static_cast<uint8_t>(vehicle_command.param1 + .5f) == vehicle_command_s::SPEED_TYPE_AIRSPEED)) {
				if (vehicle_command.param2 > FLT_EPSILON) {	// param2 is an equivalent airspeed setpoint
					if (_control_mode_current == FW_POSCTRL_MODE_AUTO) {
						_pos_sp_triplet.current.cruising_speed = vehicle_command.param2;

					} else if (_control_mode_current == FW_POSCTRL_MODE_MANUAL_ALTITUDE
						   || _control_mode_current == FW_POSCTRL_MODE_MANUAL_POSITION) {
						_commanded_manual_airspeed_setpoint = vehicle_command.param2;
					}

				}
			}

		}
	}
}

void
FixedWingModeManager::airspeed_poll()
{
	airspeed_validated_s airspeed_validated;

	if (_param_fw_use_airspd.get() && _airspeed_validated_sub.update(&airspeed_validated)) {

		// do not use synthetic airspeed as it's for the use here not reliable enough
		if (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
		    && airspeed_validated.airspeed_source != airspeed_validated_s::SOURCE_SYNTHETIC) {

			_airspeed_eas = airspeed_validated.calibrated_airspeed_m_s;
			_time_airspeed_sample = hrt_absolute_time();
		}
	}

	// no airspeed updates for one second --> declare invalid
	// this flag is used for some logic like: exiting takeoff, flaps retraction
	_airspeed_valid = hrt_elapsed_time(&_time_airspeed_last_valid) < 1_s;
}

void
FixedWingModeManager::wind_poll(const hrt_abstime now)
{
	if (_wind_sub.updated()) {
		wind_s wind;
		_wind_sub.update(&wind);

		// assumes wind is valid if finite
		_wind_valid = PX4_ISFINITE(wind.windspeed_north)
			      && PX4_ISFINITE(wind.windspeed_east);

		_time_wind_last_received = now;

		_wind_vel(0) = wind.windspeed_north;
		_wind_vel(1) = wind.windspeed_east;

	} else {
		// invalidate wind estimate usage (and correspondingly NPFG, if enabled) after subscription timeout
		_wind_valid = _wind_valid && (now - _time_wind_last_received) < WIND_EST_TIMEOUT;
	}

	if (!_wind_valid) {
		_wind_vel(0) = 0.f;
		_wind_vel(1) = 0.f;
	}
}

void
FixedWingModeManager::manual_control_setpoint_poll()
{
	_sticks.checkAndUpdateStickInputs();

	_manual_control_setpoint_for_height_rate = _sticks.getPitch();
	_manual_control_setpoint_for_airspeed = _sticks.getThrottleZeroCentered();

	if (_param_fw_pos_stk_conf.get() & STICK_CONFIG_SWAP_STICKS_BIT) {
		/* Alternate stick allocation (similar concept as for multirotor systems:
		 * demanding up/down with the throttle stick, and move faster/break with the pitch one.
		 */

		_manual_control_setpoint_for_height_rate = -_sticks.getThrottleZeroCentered();
		_manual_control_setpoint_for_airspeed = _sticks.getPitch();
	}
}

void
FixedWingModeManager::vehicle_attitude_poll()
{
	vehicle_attitude_s vehicle_attitude;

	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&angular_velocity);
		const Vector3f rates{angular_velocity.xyz};

		Dcmf R{Quatf(vehicle_attitude.q)};

		// if the vehicle is a tailsitter we have to rotate the attitude by the pitch offset
		// between multirotor and fixed wing flight
		if (_vehicle_status.is_vtol_tailsitter) {
			const Dcmf R_offset{Eulerf{0.f, M_PI_2_F, 0.f}};
			R = R * R_offset;

			_yawrate = rates(0);

		} else {
			_yawrate = rates(2);
		}

		const Eulerf euler_angles(R);
		_pitch = euler_angles(1);
		_roll = euler_angles(0);
		_yaw = euler_angles(2);

		const Vector3f body_acceleration = R.transpose() * Vector3f{_local_pos.ax, _local_pos.ay, _local_pos.az};
		_body_acceleration_x = body_acceleration(0);

		const Vector3f body_velocity = R.transpose() * Vector3f{_local_pos.vx, _local_pos.vy, _local_pos.vz};
		_body_velocity_x = body_velocity(0);
	}
}

float
FixedWingModeManager::get_manual_airspeed_setpoint()
{
	float manual_airspeed_setpoint = NAN;

	if (_param_fw_pos_stk_conf.get() & STICK_CONFIG_ENABLE_AIRSPEED_SP_MANUAL_BIT) {
		// neutral throttle corresponds to trim airspeed
		manual_airspeed_setpoint = math::interpolateNXY(_manual_control_setpoint_for_airspeed,
		{-1.f, 0.f, 1.f},
		{_param_fw_airspd_min.get(), _param_fw_airspd_trim.get(), _param_fw_airspd_max.get()});

	} else if (PX4_ISFINITE(_commanded_manual_airspeed_setpoint)) {
		// override stick by commanded airspeed
		manual_airspeed_setpoint = _commanded_manual_airspeed_setpoint;
	}

	return manual_airspeed_setpoint;
}

void
FixedWingModeManager::landing_status_publish()
{
	position_controller_landing_status_s pos_ctrl_landing_status = {};

	pos_ctrl_landing_status.lateral_touchdown_offset = _lateral_touchdown_position_offset;
	pos_ctrl_landing_status.flaring = _flare_states.flaring;
	pos_ctrl_landing_status.abort_status = _landing_abort_status;
	pos_ctrl_landing_status.timestamp = hrt_absolute_time();

	_pos_ctrl_landing_status_pub.publish(pos_ctrl_landing_status);
}

void
FixedWingModeManager::updateLandingAbortStatus(const uint8_t new_abort_status)
{
	// prevent automatic aborts if already flaring, but allow manual aborts
	if (!_flare_states.flaring || new_abort_status == position_controller_landing_status_s::ABORTED_BY_OPERATOR) {

		// only announce changes
		if (new_abort_status > 0 && _landing_abort_status != new_abort_status) {

			switch (new_abort_status) {
			case (position_controller_landing_status_s::ABORTED_BY_OPERATOR): {
					events::send(events::ID("fixedwing_position_control_landing_abort_status_operator_abort"), events::Log::Critical,
						     "Landing aborted by operator");
					break;
				}

			case (position_controller_landing_status_s::TERRAIN_NOT_FOUND): {
					events::send(events::ID("fixedwing_position_control_landing_abort_status_terrain_not_found"), events::Log::Critical,
						     "Landing aborted: terrain measurement not found");
					break;
				}

			case (position_controller_landing_status_s::TERRAIN_TIMEOUT): {
					events::send(events::ID("fixedwing_position_control_landing_abort_status_terrain_timeout"), events::Log::Critical,
						     "Landing aborted: terrain estimate timed out");
					break;
				}

			default: {
					events::send(events::ID("fixedwing_position_control_landing_abort_status_unknown_criterion"), events::Log::Critical,
						     "Landing aborted: unknown criterion");
				}
			}
		}

		_landing_abort_status = (new_abort_status >= position_controller_landing_status_s::UNKNOWN_ABORT_CRITERION) ?
					position_controller_landing_status_s::UNKNOWN_ABORT_CRITERION : new_abort_status;
		landing_status_publish();
	}
}

float
FixedWingModeManager::getManualHeightRateSetpoint()
{
	float height_rate_setpoint = 0.f;

	if (_manual_control_setpoint_for_height_rate >= FLT_EPSILON) {
		height_rate_setpoint = math::interpolate<float>(math::deadzone(_manual_control_setpoint_for_height_rate,
				       kStickDeadBand), 0, 1.f, 0.f, -_param_sinkrate_target.get());

	} else {
		height_rate_setpoint = math::interpolate<float>(math::deadzone(_manual_control_setpoint_for_height_rate,
				       kStickDeadBand), -1., 0.f, _param_climbrate_target.get(), 0.f);
	}

	return height_rate_setpoint;
}

void
FixedWingModeManager::updateManualTakeoffStatus()
{
	if (!_completed_manual_takeoff) {
		const bool at_controllable_airspeed = _airspeed_eas > _param_fw_airspd_min.get()
						      || !_airspeed_valid;
		const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					 && _control_mode.flag_armed;
		_completed_manual_takeoff = (!_landed && at_controllable_airspeed) || is_hovering;
	}
}

void
FixedWingModeManager::set_control_mode_current(const hrt_abstime &now)
{
	/* only run position controller in fixed-wing mode and during transitions for VTOL */
	if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.in_transition_mode) {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;
		return; // do not publish the setpoint
	}

	const FW_POSCTRL_MODE previous_position_control_mode = _control_mode_current;

	_skipping_takeoff_detection = false;
	const bool doing_backtransition = _vehicle_status.in_transition_mode && !_vehicle_status.in_transition_to_fw;

	if (_control_mode.flag_control_offboard_enabled && _position_setpoint_current_valid
	    && _control_mode.flag_control_position_enabled) {
		if (PX4_ISFINITE(_pos_sp_triplet.current.vx) && PX4_ISFINITE(_pos_sp_triplet.current.vy)
		    && PX4_ISFINITE(_pos_sp_triplet.current.vz)) {
			// Offboard position with velocity setpoints
			_control_mode_current = FW_POSCTRL_MODE_AUTO_PATH;
			return;

		} else {
			// Offboard position setpoint only
			_control_mode_current = FW_POSCTRL_MODE_AUTO;
			return;
		}

	} else if ((_control_mode.flag_control_auto_enabled && _control_mode.flag_control_position_enabled)
		   && (_position_setpoint_current_valid
		       || _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE)) {

		// Enter this mode only if the current waypoint has valid 3D position setpoints or is of type IDLE.
		// A setpoint of type IDLE can be published by Navigator without a valid position, and is handled here in FW_POSCTRL_MODE_AUTO.

		if (doing_backtransition) {
			_control_mode_current = FW_POSCTRL_MODE_TRANSITION_TO_HOVER_LINE_FOLLOW;

		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {

			if (_vehicle_status.is_vtol && _vehicle_status.in_transition_mode) {
				_control_mode_current = FW_POSCTRL_MODE_AUTO;

				// in this case we want the waypoint handled as a position setpoint -- a submode in control_auto()
				_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

			} else {
				_control_mode_current = _local_pos.xy_valid ? FW_POSCTRL_MODE_AUTO_TAKEOFF : FW_POSCTRL_MODE_AUTO_TAKEOFF_NO_NAV;

				if (previous_position_control_mode != FW_POSCTRL_MODE_AUTO_TAKEOFF_NO_NAV
				    && previous_position_control_mode != FW_POSCTRL_MODE_AUTO_TAKEOFF && !_landed) {
					// skip takeoff detection when switching from any other mode, auto or manual,
					// while already in air.
					// TODO: find a better place for / way of doing this
					_skipping_takeoff_detection = true;
				}
			}

		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

			// Use _position_setpoint_previous_valid to determine if landing should be straight or circular.
			// Straight landings are currently only possible in Missions, and there the previous WP
			// is valid, and circular ones are used outside of Missions, as the land mode sets prev_valid=false.
			if (_position_setpoint_previous_valid) {
				_control_mode_current = FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT;

			} else {
				_control_mode_current = FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR;
			}

		} else {
			_control_mode_current = FW_POSCTRL_MODE_AUTO;
		}

	} else if (_control_mode.flag_control_auto_enabled
		   && _control_mode.flag_control_climb_rate_enabled
		   && _control_mode.flag_armed // only enter this modes if armed, as pure failsafe modes
		   && !_control_mode.flag_control_position_enabled) {

		// failsafe modes engaged if position estimate is invalidated

		if (previous_position_control_mode != FW_POSCTRL_MODE_AUTO_ALTITUDE
		    && previous_position_control_mode != FW_POSCTRL_MODE_AUTO_CLIMBRATE) {
			// reset timer the first time we switch into this mode
			_time_in_fixed_bank_loiter = now;
		}

		if (doing_backtransition) {
			// we handle loss of position control during backtransition as a special case
			_control_mode_current = FW_POSCTRL_MODE_TRANSITION_TO_HOVER_HEADING_HOLD;

		} else if (hrt_elapsed_time(&_time_in_fixed_bank_loiter) < (_param_nav_gpsf_lt.get() * 1_s)
			   && !_vehicle_status.in_transition_mode) {
			if (previous_position_control_mode != FW_POSCTRL_MODE_AUTO_ALTITUDE) {
				// Need to init because last loop iteration was in a different mode
				events::send(events::ID("fixedwing_position_control_fb_loiter"), events::Log::Critical,
					     "Start loiter with fixed bank angle");
			}

			_control_mode_current = FW_POSCTRL_MODE_AUTO_ALTITUDE;

		} else {
			if (previous_position_control_mode != FW_POSCTRL_MODE_AUTO_CLIMBRATE && !_vehicle_status.in_transition_mode) {
				events::send(events::ID("fixedwing_position_control_descend"), events::Log::Critical, "Start descending");
			}

			_control_mode_current = FW_POSCTRL_MODE_AUTO_CLIMBRATE;
		}


	} else if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_position_enabled) {
		if (previous_position_control_mode != FW_POSCTRL_MODE_MANUAL_POSITION) {
			/* Need to init because last loop iteration was in a different mode */
			_hdg_hold_yaw = _yaw; // yaw is not controlled, so set setpoint to current yaw
			_hdg_hold_enabled = false; // this makes sure the waypoints are reset below
			_yaw_lock_engaged = false;
		}

		_control_mode_current = FW_POSCTRL_MODE_MANUAL_POSITION;

	} else if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_altitude_enabled) {

		_control_mode_current = FW_POSCTRL_MODE_MANUAL_ALTITUDE;

	} else {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;
	}
}

void
FixedWingModeManager::update_in_air_states(const hrt_abstime now)
{
	/* reset flag when airplane landed */
	if (_landed) {
		_completed_manual_takeoff = false;
	}
}

void
FixedWingModeManager::move_position_setpoint_for_vtol_transition(position_setpoint_s &current_sp)
{
	// TODO: velocity, altitude, or just a heading hold position mode should be used for this, not position
	// shifting hacks

	if (_vehicle_status.in_transition_to_fw) {

		if (!PX4_ISFINITE(_transition_waypoint(0))) {
			double lat_transition, lon_transition;

			// Create a virtual waypoint HDG_HOLD_DIST_NEXT meters in front of the vehicle which the path navigation controller can track
			// during the transition. Use the current yaw setpoint to determine the transition heading, as that one in turn
			// is set to the transition heading by Navigator, or current yaw if setpoint is not valid.
			const float transition_heading = PX4_ISFINITE(current_sp.yaw) ? current_sp.yaw : _yaw;
			waypoint_from_heading_and_distance(_current_latitude, _current_longitude, transition_heading, HDG_HOLD_DIST_NEXT,
							   &lat_transition,
							   &lon_transition);

			_transition_waypoint(0) = lat_transition;
			_transition_waypoint(1) = lon_transition;
		}

		current_sp.lat = _transition_waypoint(0);
		current_sp.lon = _transition_waypoint(1);

	} else {
		/* reset transition waypoint, will be set upon entering front transition */
		_transition_waypoint(0) = static_cast<double>(NAN);
		_transition_waypoint(1) = static_cast<double>(NAN);
	}
}

void
FixedWingModeManager::control_auto(const float control_interval, const Vector2d &curr_pos,
				   const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr,
				   const position_setpoint_s &pos_sp_next)
{
	position_setpoint_s current_sp = pos_sp_curr;
	move_position_setpoint_for_vtol_transition(current_sp);

	const uint8_t position_sp_type = handle_setpoint_type(current_sp, pos_sp_next);

	_position_sp_type = position_sp_type;

	if (position_sp_type == position_setpoint_s::SETPOINT_TYPE_LOITER
	    || current_sp.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
#ifdef CONFIG_FIGURE_OF_EIGHT

		if (current_sp.loiter_pattern == position_setpoint_s::LOITER_TYPE_FIGUREEIGHT) {
			publishFigureEightStatus(current_sp);

		} else
#endif // CONFIG_FIGURE_OF_EIGHT
		{
			publishOrbitStatus(current_sp);
		}
	}

	switch (position_sp_type) {
	case position_setpoint_s::SETPOINT_TYPE_IDLE: {
			control_idle();
			break;
		}

	case position_setpoint_s::SETPOINT_TYPE_POSITION:
		control_auto_position(control_interval, curr_pos, ground_speed, pos_sp_prev, current_sp);
		break;

	case position_setpoint_s::SETPOINT_TYPE_VELOCITY:
		control_auto_velocity(control_interval, curr_pos, ground_speed, current_sp);
		break;

	case position_setpoint_s::SETPOINT_TYPE_LOITER:
#ifdef CONFIG_FIGURE_OF_EIGHT
		if (current_sp.loiter_pattern == position_setpoint_s::LOITER_TYPE_FIGUREEIGHT) {
			controlAutoFigureEight(control_interval, curr_pos, ground_speed, current_sp);

		} else
#endif // CONFIG_FIGURE_OF_EIGHT
		{
			control_auto_loiter(control_interval, curr_pos, ground_speed, current_sp, pos_sp_next);

		}

		break;
	}

#ifdef CONFIG_FIGURE_OF_EIGHT

	/* reset loiter state */
	if ((position_sp_type != position_setpoint_s::SETPOINT_TYPE_LOITER) ||
	    ((position_sp_type == position_setpoint_s::SETPOINT_TYPE_LOITER) &&
	     (current_sp.loiter_pattern != position_setpoint_s::LOITER_TYPE_FIGUREEIGHT))) {
		_figure_eight.resetPattern();
	}

#endif // CONFIG_FIGURE_OF_EIGHT

	if (!_vehicle_status.in_transition_to_fw) {
		publishLocalPositionSetpoint(current_sp);
	}
}

void FixedWingModeManager::control_idle()
{
	const hrt_abstime  now = hrt_absolute_time();
	fixed_wing_lateral_setpoint_s lateral_ctrl_sp {empty_lateral_control_setpoint};
	lateral_ctrl_sp.timestamp = now;
	lateral_ctrl_sp.lateral_acceleration = 0.0f;
	_lateral_ctrl_sp_pub.publish(lateral_ctrl_sp);

	fixed_wing_longitudinal_setpoint_s long_contrl_sp {empty_longitudinal_control_setpoint};
	long_contrl_sp.timestamp = now;
	long_contrl_sp.pitch_direct = 0.f;
	long_contrl_sp.throttle_direct = 0.0f;
	_longitudinal_ctrl_sp_pub.publish(long_contrl_sp);

	_ctrl_configuration_handler.setThrottleMax(0.0f);
	_ctrl_configuration_handler.setThrottleMin(0.0f);
}

void
FixedWingModeManager::control_auto_fixed_bank_alt_hold()
{
	const hrt_abstime now = hrt_absolute_time();
	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = now,
		.altitude = NAN,
		.height_rate = 0.f,
		.equivalent_airspeed = NAN,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	float throttle_max = _param_fw_thr_max.get();

	// Special case: if z or vz estimate is invalid we cannot control height anymore. To prevent a
	// "climb-away" we set the thrust to MIN in that case.
	if (_landed || !_local_pos.z_valid || !_local_pos.v_z_valid) {
		throttle_max = _param_fw_thr_min.get();
	}

	_ctrl_configuration_handler.setThrottleMax(throttle_max);

	fixed_wing_lateral_setpoint_s lateral_ctrl_sp = empty_lateral_control_setpoint;
	lateral_ctrl_sp.timestamp = hrt_absolute_time();
	const float roll_body = math::radians(_param_nav_gpsf_r.get()); // open loop loiter bank angle
	lateral_ctrl_sp.lateral_acceleration = rollAngleToLateralAccel(roll_body);
	_lateral_ctrl_sp_pub.publish(lateral_ctrl_sp);
}

void
FixedWingModeManager::control_auto_descend()
{
	// Hard-code descend rate to 0.5m/s. This is a compromise to give the system to recover,
	// but not letting it drift too far away.
	const float descend_rate = 0.5f;

	const hrt_abstime now = hrt_absolute_time();

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = now,
		.altitude = NAN,
		.height_rate = -descend_rate,
		.equivalent_airspeed = NAN,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	_ctrl_configuration_handler.setThrottleMax((_landed
			|| !_local_pos.v_z_valid) ? _param_fw_thr_min.get() : _param_fw_thr_max.get());

	fixed_wing_lateral_setpoint_s lateral_ctrl_sp = empty_lateral_control_setpoint;
	lateral_ctrl_sp.timestamp = now;
	const float roll_body = math::radians(_param_nav_gpsf_r.get()); // open loop loiter bank angle
	lateral_ctrl_sp.lateral_acceleration = rollAngleToLateralAccel(roll_body);
	_lateral_ctrl_sp_pub.publish(lateral_ctrl_sp);
}

uint8_t
FixedWingModeManager::handle_setpoint_type(const position_setpoint_s &pos_sp_curr,
		const position_setpoint_s &pos_sp_next)
{
	uint8_t position_sp_type = pos_sp_curr.type;

	if (!_control_mode.flag_control_position_enabled && _control_mode.flag_control_velocity_enabled) {
		return position_setpoint_s::SETPOINT_TYPE_VELOCITY;
	}

	Vector2d curr_wp{0, 0};

	/* current waypoint (the one currently heading for) */
	curr_wp = Vector2d(pos_sp_curr.lat, pos_sp_curr.lon);

	const float acc_rad = _directional_guidance.switchDistance(500.0f);

	const bool approaching_vtol_backtransition = _vehicle_status.is_vtol
			&& pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_POSITION && _position_setpoint_current_valid
			&& pos_sp_next.type == position_setpoint_s::SETPOINT_TYPE_LAND && _position_setpoint_next_valid;

	// check if we should switch to loiter but only if we are not expecting a backtransition to happen
	if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_POSITION && !approaching_vtol_backtransition) {

		float dist_xy = -1.f;
		float dist_z = -1.f;

		const float dist = get_distance_to_point_global_wgs84(
					   (double)curr_wp(0), (double)curr_wp(1), pos_sp_curr.alt,
					   _current_latitude, _current_longitude, _current_altitude,
					   &dist_xy, &dist_z);

		const float acc_rad_z = (PX4_ISFINITE(pos_sp_curr.alt_acceptance_radius)
					 && pos_sp_curr.alt_acceptance_radius > FLT_EPSILON) ? pos_sp_curr.alt_acceptance_radius :
					_param_nav_fw_alt_rad.get();

		// Achieve position setpoint altitude via loiter when laterally close to WP.
		// Detect if system has switchted into a Loiter before (check _position_sp_type), and in that
		// case remove the dist_xy check (not switch out of Loiter until altitude is reached).
		if ((!_vehicle_status.in_transition_mode) && (dist >= 0.f)
		    && (dist_z > acc_rad_z)
		    && (dist_xy < acc_rad || _position_sp_type == position_setpoint_s::SETPOINT_TYPE_LOITER)) {

			// SETPOINT_TYPE_POSITION -> SETPOINT_TYPE_LOITER
			position_sp_type = position_setpoint_s::SETPOINT_TYPE_LOITER;
		}
	}

	return position_sp_type;
}

void
FixedWingModeManager::control_auto_position(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
	const float acc_rad = _directional_guidance.switchDistance(500.0f);
	const float target_airspeed = pos_sp_curr.cruising_speed > FLT_EPSILON ? pos_sp_curr.cruising_speed : NAN;

	// waypoint is a plain navigation waypoint
	float position_sp_alt = pos_sp_curr.alt;

	// Altitude first order hold (FOH)
	if (_position_setpoint_previous_valid &&
	    ((pos_sp_prev.type == position_setpoint_s::SETPOINT_TYPE_POSITION) ||
	     (pos_sp_prev.type == position_setpoint_s::SETPOINT_TYPE_LOITER))
	   ) {
		const float d_curr_prev = get_distance_to_next_waypoint(pos_sp_curr.lat, pos_sp_curr.lon, pos_sp_prev.lat,
					  pos_sp_prev.lon);

		// Do not try to find a solution if the last waypoint is inside the acceptance radius of the current one
		if (d_curr_prev > math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius))) {
			// Calculate distance to current waypoint
			const float d_curr = get_distance_to_next_waypoint(pos_sp_curr.lat, pos_sp_curr.lon, _current_latitude,
					     _current_longitude);

			// Save distance to waypoint if it is the smallest ever achieved, however make sure that
			// _min_current_sp_distance_xy is never larger than the distance between the current and the previous wp
			_min_current_sp_distance_xy = math::min(d_curr, _min_current_sp_distance_xy, d_curr_prev);

			// if the minimal distance is smaller than the acceptance radius, we should be at waypoint alt
			// navigator will soon switch to the next waypoint item (if there is one) as soon as we reach this altitude
			if (_min_current_sp_distance_xy > math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius))) {
				// The setpoint is set linearly and such that the system reaches the current altitude at the acceptance
				// radius around the current waypoint
				const float delta_alt = (pos_sp_curr.alt - pos_sp_prev.alt);
				const float grad = -delta_alt / (d_curr_prev - math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius)));
				const float a = pos_sp_prev.alt - grad * d_curr_prev;

				position_sp_alt = a + grad * _min_current_sp_distance_xy;
			}
		}
	}

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = position_sp_alt,
		.height_rate = NAN,
		.equivalent_airspeed = target_airspeed,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	float throttle_min = NAN;
	float throttle_max = NAN;

	if (pos_sp_curr.gliding_enabled) {
		/* enable gliding with this waypoint */
		throttle_min = 0.0;
		throttle_max = 0.0;
		_ctrl_configuration_handler.setSpeedWeight(2.f);
	}

	_ctrl_configuration_handler.setThrottleMax(throttle_max);
	_ctrl_configuration_handler.setThrottleMin(throttle_min);

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	DirectionalGuidanceOutput sp{};

	if (_position_setpoint_previous_valid && pos_sp_prev.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
		Vector2f prev_wp_local = _global_local_proj_ref.project(pos_sp_prev.lat, pos_sp_prev.lon);
		sp = navigateWaypoints(prev_wp_local, curr_wp_local, curr_pos_local, ground_speed, _wind_vel);

	} else {
		sp = navigateWaypoint(curr_wp_local, curr_pos_local, ground_speed, _wind_vel);
	}

	fixed_wing_lateral_setpoint_s lateral_ctrl_sp{empty_lateral_control_setpoint};
	lateral_ctrl_sp.timestamp = hrt_absolute_time();
	lateral_ctrl_sp.course = sp.course_setpoint;
	lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
	_lateral_ctrl_sp_pub.publish(lateral_ctrl_sp);
}

void
FixedWingModeManager::control_auto_velocity(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	//Offboard velocity control
	Vector2f target_velocity{pos_sp_curr.vx, pos_sp_curr.vy};
	const float target_bearing = wrap_pi(atan2f(target_velocity(1), target_velocity(0)));

	const Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	const DirectionalGuidanceOutput sp = navigateBearing(curr_pos_local, target_bearing, ground_speed, _wind_vel);

	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.course = sp.course_setpoint;
	fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

	const float target_airspeed = pos_sp_curr.cruising_speed > FLT_EPSILON ? pos_sp_curr.cruising_speed : NAN;

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = pos_sp_curr.alt,
		.height_rate = pos_sp_curr.vz,
		.equivalent_airspeed = target_airspeed,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	if (pos_sp_curr.gliding_enabled) {
		_ctrl_configuration_handler.setThrottleMin(0.0f);
		_ctrl_configuration_handler.setThrottleMax(0.0f);
		_ctrl_configuration_handler.setSpeedWeight(2.0f);
	}
}

void
FixedWingModeManager::control_auto_loiter(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr,
		const position_setpoint_s &pos_sp_next)
{
	// current waypoint (the one currently heading for)
	const Vector2d curr_wp = Vector2d(pos_sp_curr.lat, pos_sp_curr.lon);

	float loiter_radius = fabsf(pos_sp_curr.loiter_radius);
	bool loiter_direction_ccw = pos_sp_curr.loiter_direction_counter_clockwise;

	if (loiter_radius < FLT_EPSILON) {
		loiter_radius = fabsf(_param_nav_loiter_rad.get());
		loiter_direction_ccw = _param_nav_loiter_rad.get() < -FLT_EPSILON;
	}

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local{_global_local_proj_ref.project(curr_wp(0), curr_wp(1))};
	Vector2f vehicle_to_loiter_center{curr_wp_local - curr_pos_local};

	const bool close_to_circle = vehicle_to_loiter_center.norm() < loiter_radius + _directional_guidance.switchDistance(
					     500);

	bool enforce_low_height{false};

	float target_airspeed = pos_sp_curr.cruising_speed > FLT_EPSILON ? pos_sp_curr.cruising_speed : NAN;

	if (pos_sp_next.type == position_setpoint_s::SETPOINT_TYPE_LAND && _position_setpoint_next_valid
	    && close_to_circle && _param_fw_lnd_earlycfg.get()) {
		// We're in a loiter directly before a landing WP. Enable our landing configuration (flaps,
		// landing airspeed and potentially tighter altitude control) already such that we don't
		// have to do this switch (which can cause significant altitude errors) close to the ground.
		enforce_low_height = true;

		if (_param_fw_lnd_airspd.get() > FLT_EPSILON) {
			target_airspeed = _param_fw_lnd_airspd.get();
		}

		_flaps_setpoint = _param_fw_flaps_lnd_scl.get();
		_spoilers_setpoint = _param_fw_spoilers_lnd.get();
		_new_landing_gear_position = landing_gear_s::GEAR_DOWN;
	}

	const DirectionalGuidanceOutput sp = navigateLoiter(curr_wp_local, curr_pos_local, loiter_radius,
					     loiter_direction_ccw,
					     ground_speed,
					     _wind_vel);

	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.course = sp.course_setpoint;
	fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;

	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

	if (_landing_abort_status) {
		if (pos_sp_curr.alt - _current_altitude  < kClearanceAltitudeBuffer) {
			// aborted landing complete, normal loiter over landing point
			updateLandingAbortStatus(position_controller_landing_status_s::NOT_ABORTED);

		} else {
			// continue straight until vehicle has sufficient altitude
			_ctrl_configuration_handler.setLateralAccelMax(0.0f);

			// keep flaps in landing configuration if the airspeed is below the min airspeed (keep deployed if airspeed not valid)
			if (!_airspeed_valid || _airspeed_eas < _param_fw_airspd_min.get()) {
				_flaps_setpoint = _param_fw_flaps_lnd_scl.get();

			} else {
				_flaps_setpoint = 0.f;
			}
		}

		enforce_low_height = true;
	}

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = pos_sp_curr.alt,
		.height_rate = NAN,
		.equivalent_airspeed = target_airspeed,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	if (pos_sp_curr.gliding_enabled) {
		_ctrl_configuration_handler.setThrottleMin(0.0f);
		_ctrl_configuration_handler.setThrottleMax(0.0f);
		_ctrl_configuration_handler.setSpeedWeight(2.0f);
	}

	_ctrl_configuration_handler.setEnforceLowHeightCondition(enforce_low_height);
}

#ifdef CONFIG_FIGURE_OF_EIGHT
void
FixedWingModeManager::controlAutoFigureEight(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	// airspeed settings
	const float target_airspeed = pos_sp_curr.cruising_speed > FLT_EPSILON ? pos_sp_curr.cruising_speed : NAN;

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};

	FigureEight::FigureEightPatternParameters params;
	params.center_pos_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);
	params.loiter_direction_counter_clockwise = pos_sp_curr.loiter_direction_counter_clockwise;
	params.loiter_minor_radius = pos_sp_curr.loiter_minor_radius;
	params.loiter_orientation = pos_sp_curr.loiter_orientation;
	params.loiter_radius = pos_sp_curr.loiter_radius;

	const DirectionalGuidanceOutput sp = _figure_eight.updateSetpoint(curr_pos_local, ground_speed, params);

	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.course = sp.course_setpoint;
	fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;

	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

	_closest_point_on_path = _figure_eight.getClosestPoint();

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = pos_sp_curr.alt,
		.height_rate = NAN,
		.equivalent_airspeed = target_airspeed,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	if (pos_sp_curr.gliding_enabled) {
		_ctrl_configuration_handler.setThrottleMin(0.0f);
		_ctrl_configuration_handler.setThrottleMax(0.0f);
		_ctrl_configuration_handler.setSpeedWeight(2.0f);
	}
}

void FixedWingModeManager::publishFigureEightStatus(const position_setpoint_s pos_sp)
{
	figure_eight_status_s figure_eight_status{};
	figure_eight_status.timestamp = hrt_absolute_time();
	figure_eight_status.major_radius = pos_sp.loiter_radius * (pos_sp.loiter_direction_counter_clockwise ? -1.f : 1.f);
	figure_eight_status.minor_radius = pos_sp.loiter_minor_radius;
	figure_eight_status.orientation = pos_sp.loiter_orientation;
	figure_eight_status.frame = 5; //MAV_FRAME_GLOBAL_INT
	figure_eight_status.x = static_cast<int32_t>(pos_sp.lat * 1e7);
	figure_eight_status.y = static_cast<int32_t>(pos_sp.lon * 1e7);
	figure_eight_status.z = pos_sp.alt;

	_figure_eight_status_pub.publish(figure_eight_status);
}
#endif // CONFIG_FIGURE_OF_EIGHT

void
FixedWingModeManager::control_auto_path(const float control_interval, const Vector2d &curr_pos,
					const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	const float target_airspeed = pos_sp_curr.cruising_speed > FLT_EPSILON ? pos_sp_curr.cruising_speed : NAN;

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	// Navigate directly on position setpoint and path tangent
	const matrix::Vector2f velocity_2d(pos_sp_curr.vx, pos_sp_curr.vy);
	const float curvature = PX4_ISFINITE(_pos_sp_triplet.current.loiter_radius) ? 1 /
				_pos_sp_triplet.current.loiter_radius :
				0.0f;
	const DirectionalGuidanceOutput sp = navigatePathTangent(curr_pos_local, curr_wp_local, velocity_2d.normalized(),
					     ground_speed, _wind_vel, curvature);

	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.course = sp.course_setpoint;
	fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = pos_sp_curr.alt,
		.height_rate = NAN,
		.equivalent_airspeed = target_airspeed,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	if (pos_sp_curr.gliding_enabled) {
		_ctrl_configuration_handler.setThrottleMin(0.0f);
		_ctrl_configuration_handler.setThrottleMax(0.0f);
		_ctrl_configuration_handler.setSpeedWeight(2.0f);
	}
}

void
FixedWingModeManager::control_auto_takeoff(const hrt_abstime &now, const float control_interval,
		const Vector2d &global_position, const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	if (!_control_mode.flag_armed) {
		reset_takeoff_state();
	}

	// for now taking current position setpoint altitude as clearance altitude. this is the altitude we need to
	// clear all occlusions in the takeoff path
	const float clearance_altitude_amsl = pos_sp_curr.alt;

	// set the altitude to something above the clearance altitude to ensure the vehicle climbs past the value
	// (navigator will accept the takeoff as complete once crossing the clearance altitude)
	const float altitude_setpoint_amsl = clearance_altitude_amsl + kClearanceAltitudeBuffer;

	const Vector2f local_2D_position{_local_pos.x, _local_pos.y};

	const float takeoff_airspeed = (_param_fw_tko_airspd.get() > FLT_EPSILON) ? _param_fw_tko_airspd.get() :
				       _param_fw_airspd_min.get();

	if (_runway_takeoff.runwayTakeoffEnabled()) {
		if (!_runway_takeoff.isInitialized()) {
			_runway_takeoff.init(now);
			_takeoff_init_position = global_position;
			_takeoff_ground_alt = _current_altitude;
			_launch_current_yaw = _yaw;
			// _airspeed_slew_rate_controller.setForcedValue(takeoff_airspeed); // TODO

			events::send(events::ID("fixedwing_position_control_takeoff"), events::Log::Info, "Takeoff on runway");
		}

		if (_skipping_takeoff_detection) {
			_runway_takeoff.forceSetFlyState();
		}

		_runway_takeoff.update(now, takeoff_airspeed, _airspeed_eas, _current_altitude - _takeoff_ground_alt,
				       clearance_altitude_amsl - _takeoff_ground_alt);

		const Vector2f start_pos_local = _global_local_proj_ref.project(_takeoff_init_position(0), _takeoff_init_position(1));
		const Vector2f takeoff_waypoint_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

		// by default set the takeoff bearing to the takeoff yaw, but override in a mission takeoff with bearing to takeoff WP
		float takeoff_bearing = _launch_current_yaw;

		if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
			// the bearing from runway start to the takeoff waypoint is followed until the clearance altitude is exceeded
			const Vector2f takeoff_bearing_vector = takeoff_waypoint_local - start_pos_local;

			if (takeoff_bearing_vector.norm() > FLT_EPSILON) {
				takeoff_bearing = atan2f(takeoff_bearing_vector(1), takeoff_bearing_vector(0));
			}
		}

		const DirectionalGuidanceOutput sp = navigateLine(start_pos_local, takeoff_bearing, local_2D_position, ground_speed,
						     _wind_vel);

		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = now;
		fw_lateral_ctrl_sp.course = sp.course_setpoint;
		fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;

		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

		const float roll_wingtip_strike = getMaxRollAngleNearGround(_current_altitude, _takeoff_ground_alt);
		_ctrl_configuration_handler.setLateralAccelMax(rollAngleToLateralAccel(roll_wingtip_strike));

		const float pitch_max = _runway_takeoff.getMaxPitch(math::radians(_param_fw_p_lim_max.get()));
		const float pitch_min = _runway_takeoff.getMinPitch(math::radians(_takeoff_pitch_min.get()),
					math::radians(_param_fw_p_lim_min.get()));

		const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
			.timestamp = now,
			.altitude = altitude_setpoint_amsl,
			.height_rate = NAN,
			.equivalent_airspeed = takeoff_airspeed,
			.pitch_direct = _runway_takeoff.getPitch(),
			.throttle_direct = _runway_takeoff.getThrottle(_param_fw_thr_idle.get())
		};

		_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

		_ctrl_configuration_handler.setPitchMin(pitch_min);
		_ctrl_configuration_handler.setPitchMax(pitch_max);
		_ctrl_configuration_handler.setClimbRateTarget(_param_fw_t_clmb_max.get());
		_ctrl_configuration_handler.setDisableUnderspeedProtection(true);

		if (_runway_takeoff.getState() == RunwayTakeoffState::CLIMBOUT) {
			// limits to release gradually once the climbout ends (apply_takeoff_exit_blend)
			_time_takeoff_climbout_last = now;
			_takeoff_exit_pitch_min = pitch_min;
			_takeoff_exit_throttle = _runway_takeoff.getThrottle(_param_fw_thr_idle.get());
		}

		_flaps_setpoint = _param_fw_flaps_to_scl.get();

		// retract ladning gear once passed the climbout state
		if (_runway_takeoff.getState() > RunwayTakeoffState::CLIMBOUT) {
			_new_landing_gear_position = landing_gear_s::GEAR_UP;
		}

		fixed_wing_runway_control_s fw_runway_control{};
		fw_runway_control.timestamp = now;
		fw_runway_control.wheel_steering_enabled = true;
		fw_runway_control.wheel_steering_nudging_rate = _param_rwto_nudge.get() ? _sticks.getYaw() : 0.f;

		_fixed_wing_runway_control_pub.publish(fw_runway_control);

	} else {
		/* Perform launch detection */
		if (!_skipping_takeoff_detection && _param_fw_laun_detcn_on.get() &&
		    _launchDetector.getLaunchDetected() < launch_detection_status_s::STATE_FLYING) {

			if (_control_mode.flag_armed) {
				/* Perform launch detection */

				/* Detect launch using body X (forward) acceleration */
				_launchDetector.update(control_interval, _body_acceleration_x);
			}

		} else	{
			/* no takeoff detection --> fly */
			_launchDetector.forceSetFlyState();
		}

		if (!_launch_detected && _launchDetector.getLaunchDetected() > launch_detection_status_s::STATE_WAITING_FOR_LAUNCH) {
			_launch_detected = true;
			_takeoff_init_position = global_position;
			_takeoff_ground_alt = _current_altitude;
			_launch_current_yaw = _yaw;
		}

		const Vector2f launch_local_position = _global_local_proj_ref.project(_takeoff_init_position(0),
						       _takeoff_init_position(1));

		// by default set the takeoff bearing to the takeoff yaw, but override in a mission takeoff with bearing to takeoff WP
		float takeoff_bearing = _launch_current_yaw;
		const float distance_to_takeoff_wp = get_distance_to_next_waypoint(_takeoff_init_position(0), _takeoff_init_position(1),
						     pos_sp_curr.lat, pos_sp_curr.lon);

		if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF && distance_to_takeoff_wp > 10.f) {
			// if a takeoff waypoint is set (and not straight above current location), use the bearing to the takeoff waypoint
			const Vector2f takeoff_waypoint_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);
			const Vector2f takeoff_bearing_vector = takeoff_waypoint_local - launch_local_position;

			if (takeoff_bearing_vector.norm() > FLT_EPSILON) {
				takeoff_bearing = atan2f(takeoff_bearing_vector(1), takeoff_bearing_vector(0));
			}
		}

		/* Set control values depending on the detection state */
		if (_launchDetector.getLaunchDetected() > launch_detection_status_s::STATE_WAITING_FOR_LAUNCH) {
			/* Launch has been detected, hence we have to control the plane. */

			const DirectionalGuidanceOutput sp = navigateLine(launch_local_position, takeoff_bearing, local_2D_position,
							     ground_speed,
							     _wind_vel);

			fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
			fw_lateral_ctrl_sp.timestamp = now;
			fw_lateral_ctrl_sp.course = sp.course_setpoint;
			fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;

			_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

			const float roll_wingtip_strike = getMaxRollAngleNearGround(_current_altitude, _takeoff_ground_alt);
			_ctrl_configuration_handler.setLateralAccelMax(rollAngleToLateralAccel(roll_wingtip_strike));

			const float max_takeoff_throttle = (_launchDetector.getLaunchDetected() < launch_detection_status_s::STATE_FLYING) ?
							   _param_fw_thr_idle.get() : NAN;
			const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
				.timestamp = now,
				.altitude = NAN,
				.height_rate = _param_fw_t_clmb_max.get(),
				.equivalent_airspeed = takeoff_airspeed,
				.pitch_direct = NAN,
				.throttle_direct = NAN
			};

			_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

			_ctrl_configuration_handler.setPitchMin(radians(_takeoff_pitch_min.get()));
			_ctrl_configuration_handler.setThrottleMax(max_takeoff_throttle);
			_ctrl_configuration_handler.setClimbRateTarget(_param_fw_t_clmb_max.get());
			_ctrl_configuration_handler.setDisableUnderspeedProtection(true);

			//float yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw

		} else {
			fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
			fw_lateral_ctrl_sp.timestamp = now;
			fw_lateral_ctrl_sp.lateral_acceleration = 0.f;
			/* Tell the attitude controller to stop integrating while we are waiting for the launch */
			_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

			fixed_wing_longitudinal_setpoint_s long_control_sp{empty_longitudinal_control_setpoint};
			long_control_sp.timestamp = now;
			long_control_sp.pitch_direct = radians(_takeoff_pitch_min.get());
			long_control_sp.throttle_direct = _param_fw_thr_idle.get();
			_longitudinal_ctrl_sp_pub.publish(long_control_sp);
		}

		launch_detection_status_s launch_detection_status;
		launch_detection_status.timestamp = now;
		launch_detection_status.launch_detection_state = _launchDetector.getLaunchDetected();
		_launch_detection_status_pub.publish(launch_detection_status);
	}

	_flaps_setpoint = _param_fw_flaps_to_scl.get();

	if (!_vehicle_status.in_transition_to_fw) {
		publishLocalPositionSetpoint(pos_sp_curr);
	}
}

void
FixedWingModeManager::control_auto_takeoff_no_nav(const hrt_abstime &now, const float control_interval,
		const float current_setpoint_altitude_amsl)
{
	if (!_control_mode.flag_armed) {
		reset_takeoff_state();
	}

	const float takeoff_airspeed = (_param_fw_tko_airspd.get() > FLT_EPSILON) ? _param_fw_tko_airspd.get() :
				       _param_fw_airspd_min.get();

	if (_runway_takeoff.runwayTakeoffEnabled()) {
		if (!_runway_takeoff.isInitialized()) {
			_runway_takeoff.init(now);
			_takeoff_ground_alt = _current_altitude;
			_launch_current_yaw = _yaw;
		}

		if (_skipping_takeoff_detection) {
			_runway_takeoff.forceSetFlyState();
		}

		const float clearance_altitude_amsl = _current_altitude + 20.f; // hard code it to 20m above takeoff point

		_runway_takeoff.update(now, takeoff_airspeed, _airspeed_eas, _current_altitude - _takeoff_ground_alt,
				       clearance_altitude_amsl - _takeoff_ground_alt);

		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = now;
		fw_lateral_ctrl_sp.lateral_acceleration = 0.f; // level wings

		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

		const float pitch_max = _runway_takeoff.getMaxPitch(math::radians(_param_fw_p_lim_max.get()));
		const float pitch_min = _runway_takeoff.getMinPitch(math::radians(_takeoff_pitch_min.get()),
					math::radians(_param_fw_p_lim_min.get()));

		const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
			.timestamp = now,
			.altitude = NAN,
			.height_rate = _param_fw_t_clmb_max.get(),
			.equivalent_airspeed = takeoff_airspeed,
			.pitch_direct = _runway_takeoff.getPitch(),
			.throttle_direct = _runway_takeoff.getThrottle(_param_fw_thr_idle.get())
		};

		_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

		_ctrl_configuration_handler.setPitchMin(pitch_min);
		_ctrl_configuration_handler.setPitchMax(pitch_max);
		_ctrl_configuration_handler.setDisableUnderspeedProtection(true);

		_flaps_setpoint = _param_fw_flaps_to_scl.get();

		// retract ladning gear once passed the climbout state
		if (_runway_takeoff.getState() > RunwayTakeoffState::CLIMBOUT) {
			_new_landing_gear_position = landing_gear_s::GEAR_UP;
		}

		fixed_wing_runway_control_s fw_runway_control{};
		fw_runway_control.timestamp = now;
		fw_runway_control.wheel_steering_enabled = true;
		fw_runway_control.wheel_steering_nudging_rate = _param_rwto_nudge.get() ? _sticks.getYaw() : 0.f;

		_fixed_wing_runway_control_pub.publish(fw_runway_control);

	} else {
		/* Perform launch detection */
		if (!_skipping_takeoff_detection && _param_fw_laun_detcn_on.get() &&
		    _launchDetector.getLaunchDetected() < launch_detection_status_s::STATE_FLYING) {

			if (_control_mode.flag_armed) {
				/* Detect launch using body X (forward) acceleration */
				_launchDetector.update(control_interval, _body_acceleration_x);
			}

		} else	{
			/* no takeoff detection --> fly */
			_launchDetector.forceSetFlyState();
		}

		if (!_launch_detected && _launchDetector.getLaunchDetected() > launch_detection_status_s::STATE_WAITING_FOR_LAUNCH) {
			_launch_detected = true;
			_takeoff_ground_alt = _current_altitude;
		}

		/* Launch has been detected, hence we have to control the plane. */

		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = now;
		fw_lateral_ctrl_sp.lateral_acceleration = 0.f; // level wings

		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

		const float max_takeoff_throttle = (_launchDetector.getLaunchDetected() < launch_detection_status_s::STATE_FLYING) ?
						   _param_fw_thr_idle.get() : NAN;
		const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
			.timestamp = now,
			.altitude = NAN,
			.height_rate = _param_fw_t_clmb_max.get(),
			.equivalent_airspeed = takeoff_airspeed,
			.pitch_direct = NAN,
			.throttle_direct = NAN
		};

		_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

		_ctrl_configuration_handler.setPitchMin(radians(_takeoff_pitch_min.get()));
		_ctrl_configuration_handler.setThrottleMax(max_takeoff_throttle);
		_ctrl_configuration_handler.setClimbRateTarget(_param_fw_t_clmb_max.get());
		_ctrl_configuration_handler.setDisableUnderspeedProtection(true);

		launch_detection_status_s launch_detection_status;
		launch_detection_status.timestamp = now;
		launch_detection_status.launch_detection_state = _launchDetector.getLaunchDetected();
		_launch_detection_status_pub.publish(launch_detection_status);
	}

	_flaps_setpoint = _param_fw_flaps_to_scl.get();
}

void
FixedWingModeManager::control_auto_landing_straight(const hrt_abstime &now, const float control_interval,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
	float airspeed_land = (_param_fw_lnd_airspd.get() > FLT_EPSILON) ? _param_fw_lnd_airspd.get() :
			      _param_fw_airspd_min.get();

	_ctrl_configuration_handler.setEnforceLowHeightCondition(true);

	// now handle position
	const Vector2f local_position{_local_pos.x, _local_pos.y};
	Vector2f local_land_point = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	initializeAutoLanding(now, pos_sp_prev, pos_sp_curr.alt, local_position, local_land_point);

	// touchdown may get nudged by manual inputs
	local_land_point = calculateTouchdownPosition(control_interval, local_land_point);
	const Vector2f landing_approach_vector = calculateLandingApproachVector();

	// calculate the altitude setpoint based on the landing glide slope
	const float along_track_dist_to_touchdown = -landing_approach_vector.unit_or_zero().dot(
				local_position - local_land_point);
	const float glide_slope = _landing_approach_entrance_rel_alt / _landing_approach_entrance_offset_vector.norm();

	// NOTE: this relative altitude can go below zero, this is intentional. in the case the vehicle is tracking the glide
	// slope at an offset above the track, making the altitude setpoint constant on intersection with terrain causes
	// an increase in throttle (to slow down and smoothly intersect the flattened altitude setpoint), which is undesirable
	// directly before the flare. instead, we keep the steady state behavior, and let the flare get triggered once at
	// the desired altitude
	const float glide_slope_rel_alt = math::min(along_track_dist_to_touchdown * glide_slope,
					  _landing_approach_entrance_rel_alt);

	const bool abort_on_terrain_measurement_timeout = checkLandingAbortBitMask(_param_fw_lnd_abort.get(),
			position_controller_landing_status_s::TERRAIN_NOT_FOUND);
	const bool abort_on_terrain_timeout = checkLandingAbortBitMask(_param_fw_lnd_abort.get(),
					      position_controller_landing_status_s::TERRAIN_TIMEOUT);
	const float terrain_alt = getLandingTerrainAltitudeEstimate(now, pos_sp_curr.alt, abort_on_terrain_measurement_timeout,
				  abort_on_terrain_timeout);
	const float glide_slope_reference_alt = (_param_fw_lnd_useter.get() ==
						TerrainEstimateUseOnLanding::kFollowTerrainRelativeLandingGlideSlope) ? terrain_alt : pos_sp_curr.alt;

	const int flare_law = _param_fw_lnd_fl_law.get();

	// flare law 6: approach airspeed and lower airspeed limit from the estimated stall speed instead of the weight
	if (flare_law == 6) {
		const float vs_est = updateStallSpeedEstimate(now, _current_altitude - terrain_alt);

		if (PX4_ISFINITE(vs_est)) {
			airspeed_land = _param_fw_lnd_vapp_k.get() * vs_est;
			_ctrl_configuration_handler.setEquivalentAirspeedMin(1.05f * vs_est);
		}
	}

	float altitude_setpoint;

	if (_current_altitude > glide_slope_reference_alt + glide_slope_rel_alt) {
		// descend to the glide slope
		altitude_setpoint = glide_slope_reference_alt + glide_slope_rel_alt;

	} else {
		// continue horizontally
		altitude_setpoint = _current_altitude;
	}

	// flare at the maximum of the altitude determined by the time before touchdown and a minimum flare altitude
	const float flare_rel_alt = math::max(_param_fw_lnd_fl_time.get() * _local_pos.vz, _param_fw_lnd_flalt.get());


	// the terrain estimate (if enabled) is always used to determine the flaring altitude
	if ((_current_altitude < terrain_alt + flare_rel_alt) || _flare_states.flaring) {
		// flare and land with minimal speed

		// flaring is a "point of no return"
		if (!_flare_states.flaring) {
			// every law starts from the same state; law 6 keeps the approach throttle (no step) as the centre of its
			// airspeed PI, law 5 starts from the feedforward throttle and law 4 from idle (its deceleration phase)
			flareStart(_flare_states, now, _current_altitude - terrain_alt);

			if (flare_law != 6) {
				_flare_states.throttle_cmd = (flare_law == 5) ? _param_fw_lnd_fl_tff.get() : _param_fw_thr_idle.get();
			}

			events::send(events::ID("fixedwing_position_control_landing_flaring"), events::Log::Info,
				     "Landing, flaring");
		}

		// ramp in flare limits and setpoints with the flare time, command a soft touchdown
		const float seconds_since_flare_start = hrt_elapsed_time(&_flare_states.start_time) * 1.e-6f;
		const float flare_ramp_interpolator = math::constrain(seconds_since_flare_start / _param_fw_lnd_fl_time.get(), 0.0f,
						      1.0f);

		/* lateral guidance first, because npfg will adjust the airspeed setpoint if necessary */
		const Vector2f local_approach_entrance = local_land_point - landing_approach_vector;

		const DirectionalGuidanceOutput sp = navigateLine(local_approach_entrance, local_land_point, local_position,
						     ground_speed,
						     _wind_vel);
		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = now;
		fw_lateral_ctrl_sp.course = sp.course_setpoint;
		fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

		const float roll_wingtip_strike = getMaxRollAngleNearGround(_current_altitude, _takeoff_ground_alt);
		_ctrl_configuration_handler.setLateralAccelMax(rollAngleToLateralAccel(roll_wingtip_strike));

		/* longitudinal guidance */

		const float flare_ramp_interpolator_sqrt = sqrtf(flare_ramp_interpolator);

		// Use separate ramp for the altitude setpoint that's starting only when the other is finished
		// to allow motor to be ramped down before height rate setpoint is adapted for flaring.
		const float flare_hieght_rate_interpolator = math::constrain((seconds_since_flare_start -
				_param_fw_lnd_fl_time.get()) / (_param_fw_lnd_fl_time.get()), 0.f, 1.f);
		const float flare_hieght_rate_interpolator_sqrt = sqrt(flare_hieght_rate_interpolator);

		float height_rate_setpoint = flare_hieght_rate_interpolator_sqrt * (-_param_fw_lnd_fl_sink.get()) +
					     (1.0f - flare_hieght_rate_interpolator_sqrt) * _flare_states.initial_height_rate_setpoint;

		// interpolator for the pitch limits: 0 at flare start, 1 at touchdown values
		float flare_limit_interpolator = flare_ramp_interpolator_sqrt;

		if (flare_law == 1 || flare_law == 2 || flare_law == 3) {
			// height based flare: sink rate, limits and throttle follow the height above the touchdown point
			// instead of the time since flare start (exponential flare / constant deceleration flare)
			const float sink_td = math::max(_param_fw_lnd_fl_sink.get(), 0.0f);
			const float sink_start = math::max(-_flare_states.initial_height_rate_setpoint, sink_td);
			const float hold_height = math::max(_param_fw_lnd_fl_hold.get(), 0.0f);
			const float flare_span = math::max(_flare_states.start_height - hold_height, 0.1f);
			// 1 at flare start, 0 at and below the hold-off height
			const float height_fraction = math::constrain((_current_altitude - terrain_alt - hold_height) / flare_span, 0.0f,
						      1.0f);

			float sink_rate;

			if (flare_law == 1 || flare_law == 3) {
				// sink rate linear in height -> height decays exponentially in time
				sink_rate = sink_td + (sink_start - sink_td) * height_fraction;

			} else {
				// constant vertical deceleration: sink^2 linear in height -> sink rate linear in time
				sink_rate = sqrtf(sink_td * sink_td + (sink_start * sink_start - sink_td * sink_td) * height_fraction);
			}

			height_rate_setpoint = -sink_rate;
			flare_limit_interpolator = 1.0f - height_fraction;

			// with idle throttle the airspeed falls below the landing airspeed during the flare; a reduced speed
			// weight lets pitch follow the sink rate instead of trying to regain airspeed (as ArduPlane LAND_SPDWGT)
			if (_param_fw_lnd_fl_spdw.get() >= 0.0f) {
				_ctrl_configuration_handler.setSpeedWeight(math::constrain(_param_fw_lnd_fl_spdw.get(), 0.0f, 2.0f));
			}
		}

		float pitch_min_rad = flare_limit_interpolator * radians(_param_fw_lnd_fl_pmin.get()) +
				      (1.0f - flare_limit_interpolator) * radians(_param_fw_p_lim_min.get());
		float pitch_max_rad = flare_limit_interpolator * radians(_param_fw_lnd_fl_pmax.get()) +
				      (1.0f - flare_limit_interpolator) * radians(_param_fw_p_lim_max.get());

		if (_param_fw_lnd_td_time.get() > FLT_EPSILON) {
			const float touchdown_time = math::max(_param_fw_lnd_td_time.get(), _param_fw_lnd_fl_time.get());

			const float touchdown_interpolator = math::constrain((seconds_since_flare_start - touchdown_time) /
							     POST_TOUCHDOWN_CLAMP_TIME, 0.0f,
							     1.0f);

			pitch_max_rad = touchdown_interpolator * math::radians(_param_rwto_psp.get()) + (1.0f - touchdown_interpolator) *
					pitch_max_rad;
			pitch_min_rad = touchdown_interpolator * math::radians(_param_rwto_psp.get()) + (1.0f - touchdown_interpolator) *
					pitch_min_rad;
		}

		// idle throttle may be >0 for internal combustion engines
		// normally set to zero for electric motors
		// throttle is retarded to idle over FW_LND_FL_TIME with every flare law, so airspeed bleeds off during the flare
		const float throttle_max = flare_ramp_interpolator_sqrt * _param_fw_thr_idle.get() +
					   (1.0f - flare_ramp_interpolator_sqrt) *
					   _param_fw_thr_max.get();

		float pitch_direct = NAN;
		float throttle_direct = NAN;

		if (flare_law == 6) {
			// same algorithm as the water scoop (law6Step), runway profile: retard and runway pitch after touchdown
			const Law6Profile runway_profile{_param_fw_lnd_vtd_k.get(), radians(_param_fw_lnd_fl_pmax.get()), false};
			const Law6Output out = law6Step(_flare_states, now, _current_altitude - terrain_alt, runway_profile);
			height_rate_setpoint = -out.sink_setpoint;
			pitch_direct = out.pitch;
			pitch_min_rad = out.pitch_min;
			pitch_max_rad = out.pitch_max;
			throttle_direct = out.throttle;
			airspeed_land = out.airspeed_setpoint;

			if (PX4_ISFINITE(_vs_est)) {
				_ctrl_configuration_handler.setEquivalentAirspeedMin(1.05f * _vs_est);
			}

		} else if (flare_law == 5) {
			// airliner style: pitch holds the sink rate, throttle holds the airspeed (slowly), retard near the ground.
			// Unlike law 3 / ArduPlane the throttle is not cut at flare start, so the touchdown airspeed and sink rate do
			// not depend on how long the flare lasts. Unlike law 4 the pitch is not fixed: the thrust line above the CG
			// pitches the aircraft with throttle, and the sink rate loop on pitch takes that out.
			const float dt = math::constrain(hrt_elapsed_time(&_flare_states.last_update) * 1e-6f, 0.001f, 0.1f);
			_flare_states.last_update = now;
			const float transition = math::constrain(seconds_since_flare_start / math::max(_param_fw_lnd_fl_tatt.get(), 0.1f),
						 0.0f, 1.0f);
			const float height = _current_altitude - terrain_alt;
			const float hold_height = _param_fw_lnd_fl_hold.get();
			const float sink_hold = math::max(_param_fw_lnd_fl_sink.get(), 0.0f);
			const float sink_upper = math::max(_param_fw_lnd_fl_snk1.get(), sink_hold);
			const float sink_target = (height > hold_height) ? sink_upper : sink_hold;
			const float sink_start = -_flare_states.initial_height_rate_setpoint;
			const float sink_setpoint = transition * sink_target + (1.0f - transition) * sink_start;
			height_rate_setpoint = -sink_setpoint;

			// pitch: PI on the sink rate error around the pitch at flare start (as law 3)
			const float sink_error = _local_pos.vz - sink_setpoint; // > 0: sinking faster than wanted
			const float pitch_lo = radians(_param_fw_p_lim_min.get());
			const float pitch_hi = radians(_param_fw_lnd_fl_pmax.get());
			const float pitch_unsat = _flare_states.start_pitch + radians(_param_fw_lnd_fl_kp.get()) * sink_error
						  + _flare_states.sink_rate_integral;

			pitch_direct = math::constrain(pitch_unsat, pitch_lo, pitch_hi);

			if (!((pitch_unsat >= pitch_hi && sink_error > 0.0f) || (pitch_unsat <= pitch_lo && sink_error < 0.0f))) {
				_flare_states.sink_rate_integral += radians(_param_fw_lnd_fl_ki.get()) * sink_error * dt;
			}

			pitch_min_rad = pitch_lo;
			pitch_max_rad = pitch_hi;

			// throttle: PI on the airspeed error to FW_LND_FL_VATT (weight scaled), slew limited; idle below the hold height
			const float speed_target = _param_fw_lnd_fl_vatt.get() * sqrtf(weightRatio());
			const bool airspeed_ok = hrt_elapsed_time(&_time_airspeed_sample) < 1_s;
			const float speed_error = airspeed_ok ? (speed_target - _airspeed_eas) : 0.0f; // > 0: too slow
			const float thr_min = _param_fw_thr_idle.get();
			const float thr_max = _param_fw_thr_max.get();
			const bool retard = height < hold_height || _landed;
			const float thr_unsat = _param_fw_lnd_fl_tff.get() + _param_fw_lnd_fl_tkp.get() * speed_error
						+ _flare_states.throttle_integral;
			const float thr_target = retard ? thr_min : math::constrain(thr_unsat, thr_min, thr_max);
			const float slew = _param_fw_lnd_fl_tslw.get();
			const float thr_prev = _flare_states.throttle_cmd;
			_flare_states.throttle_cmd = (slew > FLT_EPSILON)
						     ? thr_prev + math::constrain(thr_target - thr_prev, -slew * dt, slew * dt) : thr_target;
			const bool slew_limited = fabsf(_flare_states.throttle_cmd - thr_target) > 1e-4f;
			const bool held_up = thr_unsat >= thr_max || (slew_limited && thr_target > _flare_states.throttle_cmd);
			const bool held_down = thr_unsat <= thr_min || (slew_limited && thr_target < _flare_states.throttle_cmd);

			if (!retard && !((held_up && speed_error > 0.0f) || (held_down && speed_error < 0.0f))) {
				_flare_states.throttle_integral += _param_fw_lnd_fl_tki.get() * speed_error * dt;
			}

			throttle_direct = _landed ? thr_min : _flare_states.throttle_cmd;

			if (_landed) {
				pitch_direct = radians(_param_rwto_psp.get());
				pitch_min_rad = math::min(pitch_min_rad, pitch_direct);
			}

		} else if (flare_law == 4) {
			// power-on attitude landing: pitch held at the landing attitude, throttle holds the sink rate
			// (FAA-H-8083-23 ch. 6 glassy water landing). Angle of attack fixed and lift = weight, so the touchdown
			// airspeed depends on the attitude and the weight only, not on the pattern or the flare height.
			const float dt = math::constrain(hrt_elapsed_time(&_flare_states.last_update) * 1e-6f, 0.001f, 0.1f);
			_flare_states.last_update = now;
			const float transition = math::constrain(seconds_since_flare_start / math::max(_param_fw_lnd_fl_tatt.get(), 0.1f),
						 0.0f, 1.0f);
			const float height = _current_altitude - terrain_alt;
			const float sink_hold = math::max(_param_fw_lnd_fl_sink.get(), 0.0f);
			const float sink_upper = math::max(_param_fw_lnd_fl_snk1.get(), sink_hold);
			const float sink_target = (height > _param_fw_lnd_fl_hold.get()) ? sink_upper : sink_hold;
			const float sink_start = -_flare_states.initial_height_rate_setpoint;
			const float sink_setpoint = transition * sink_target + (1.0f - transition) * sink_start;
			height_rate_setpoint = -sink_setpoint;

			// weight scaling: airspeeds with sqrt(W / W_base), throttle (thrust ~ throttle^2, thrust ~ W) with W^(1/4)
			const float weight_ratio = weightRatio();

			const float speed_scale = sqrtf(weight_ratio);

			// airspeed guard: lower the attitude (more speed, less lift) while below the weight scaled minimum
			const float vmin = _param_fw_lnd_fl_vmin.get() * speed_scale;

			const bool airspeed_ok = hrt_elapsed_time(&_time_airspeed_sample) < 1_s;

			if (vmin > FLT_EPSILON && airspeed_ok && _airspeed_eas < vmin) {
				_flare_states.attitude_offset = math::max(_flare_states.attitude_offset - radians(1.0f) * dt, radians(-2.0f));

			} else {
				_flare_states.attitude_offset = math::min(_flare_states.attitude_offset + radians(0.5f) * dt, 0.0f);
			}

			const float attitude = radians(_param_fw_lnd_fl_att.get()) + _flare_states.attitude_offset;
			const float sink_error = _local_pos.vz - sink_setpoint; // > 0: sinking faster than wanted
			const float thr_min = _param_fw_thr_idle.get();
			const float thr_max = _param_fw_thr_max.get();

			if (!_flare_states.attitude_hold) {
				// 1) deceleration (FAA: slow to about Vs + 10 kt before setting the landing attitude): idle throttle,
				// pitch holds the sink rate (PI as law 3), so raising the nose cannot make the aircraft climb. As the
				// airspeed decays the pitch needed rises; at the landing attitude the airspeed is the equilibrium one.
				const float pitch_lo = radians(_param_fw_p_lim_min.get());
				const float pitch_unsat = _flare_states.start_pitch + radians(_param_fw_lnd_fl_kp.get()) * sink_error
							  + _flare_states.sink_rate_integral;

				if (!((pitch_unsat >= attitude && sink_error > 0.0f) || (pitch_unsat <= pitch_lo && sink_error < 0.0f))) {
					_flare_states.sink_rate_integral += radians(_param_fw_lnd_fl_ki.get()) * sink_error * dt;
				}

				pitch_direct = math::constrain(pitch_unsat, pitch_lo, attitude);
				throttle_direct = thr_min;
				_flare_states.throttle_cmd = thr_min;

				// the attitude hold airspeed is reached (or, without it, the attitude), or the hold height comes first:
				// hand the sink rate to the throttle. The pitch criterion alone switched too slow: sinking faster than the
				// setpoint, the angle of attack is higher and the airspeed lower at the same pitch (1200 kg stalled).
				const float vatt = _param_fw_lnd_fl_vatt.get() * speed_scale;
				const bool slow_enough = (vatt > FLT_EPSILON) ? (airspeed_ok && _airspeed_eas <= vatt) : (pitch_unsat >= attitude);

				if (slow_enough || height < _param_fw_lnd_fl_hold.get()) {
					_flare_states.attitude_hold = true;
					_flare_states.throttle_integral = 0.0f;
					_flare_states.hold_pitch = pitch_direct;
					// the attitude hold starts at the feedforward throttle (user: 20 %), then moves at the slew rate
					_flare_states.throttle_cmd = math::constrain(_param_fw_lnd_fl_tff.get(), thr_min, thr_max);
				}

				pitch_min_rad = pitch_lo;
				pitch_max_rad = attitude + radians(3.0f);

			} else {
				// 2) attitude hold: pitch to the landing attitude at 3 deg/s, then fixed; throttle PI on the sink rate error
				const float step = radians(3.0f) * dt;
				_flare_states.hold_pitch += math::constrain(attitude - _flare_states.hold_pitch, -step, step);
				pitch_direct = _flare_states.hold_pitch;
				pitch_min_rad = math::min(pitch_direct, attitude) - radians(3.0f);
				pitch_max_rad = math::max(pitch_direct, attitude) + radians(3.0f);
				const float thr_unsat = _param_fw_lnd_fl_tff.get() + _param_fw_lnd_fl_tkp.get() * sink_error
							+ _flare_states.throttle_integral;

				const float thr_target = math::constrain(thr_unsat, thr_min, thr_max);
				const float slew = _param_fw_lnd_fl_tslw.get();
				const float thr_prev = _flare_states.throttle_cmd;
				_flare_states.throttle_cmd = (slew > FLT_EPSILON)
							     ? thr_prev + math::constrain(thr_target - thr_prev, -slew * dt, slew * dt) : thr_target;
				const bool slew_limited = fabsf(_flare_states.throttle_cmd - thr_target) > 1e-4f;

				// anti-windup: no integration while the output is held at a limit or by the slew rate in the error's direction
				const bool held_up = (thr_unsat >= thr_max || (slew_limited && thr_target > _flare_states.throttle_cmd));
				const bool held_down = (thr_unsat <= thr_min || (slew_limited && thr_target < _flare_states.throttle_cmd));

				if (!((held_up && sink_error > 0.0f) || (held_down && sink_error < 0.0f))) {
					_flare_states.throttle_integral += _param_fw_lnd_fl_tki.get() * sink_error * dt;
				}

				throttle_direct = _flare_states.throttle_cmd;
			}

			// after touchdown: idle, nose to the runway pitch (as the touchdown clamp of the other laws)
			if (_landed) {
				pitch_direct = radians(_param_rwto_psp.get());
				pitch_min_rad = math::min(pitch_min_rad, pitch_direct);
				throttle_direct = thr_min;
			}

		} else if (flare_law == 3) {
			// autoland style flare: the sink rate error drives the pitch setpoint directly (PI around the pitch angle at
			// flare start), bypassing the TECS energy balance, so airspeed decay and weight do not change the pitch response.
			// Throttle stays with TECS under the idle ramp above (retard).
			const float sink_rate_error = height_rate_setpoint + _local_pos.vz; // > 0: sinking faster than wanted
			const float pitch_unsaturated = _flare_states.start_pitch + radians(_param_fw_lnd_fl_kp.get()) * sink_rate_error
							+ _flare_states.sink_rate_integral;

			// integrate only while the command is not held at a limit in the direction of the error (anti-windup)
			if (!((pitch_unsaturated >= pitch_max_rad && sink_rate_error > 0.0f)
			      || (pitch_unsaturated <= pitch_min_rad && sink_rate_error < 0.0f))) {
				_flare_states.sink_rate_integral += radians(_param_fw_lnd_fl_ki.get()) * sink_rate_error * control_interval;
			}

			pitch_direct = math::constrain(pitch_unsaturated, pitch_min_rad, pitch_max_rad);
		}

		const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
			.timestamp = now,
			.altitude = altitude_setpoint,
			.height_rate = height_rate_setpoint,
			.equivalent_airspeed = airspeed_land,
			.pitch_direct = pitch_direct,
			.throttle_direct = throttle_direct
		};

		_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

		_ctrl_configuration_handler.setPitchMin(pitch_min_rad);
		_ctrl_configuration_handler.setPitchMax(pitch_max_rad);
		// law 4 keeps power on until touchdown (the throttle command itself goes to idle when landed)
		_ctrl_configuration_handler.setThrottleMax((flare_law >= 4) ? _param_fw_thr_max.get() : throttle_max);
		_ctrl_configuration_handler.setThrottleMin(_param_fw_thr_idle.get());
		_ctrl_configuration_handler.setDisableUnderspeedProtection(true);

	} else {

		// follow the glide slope

		/* lateral guidance */

		const Vector2f local_approach_entrance = local_land_point - landing_approach_vector;

		const DirectionalGuidanceOutput sp = navigateLine(local_approach_entrance, local_land_point, local_position,
						     ground_speed,
						     _wind_vel);
		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
		fw_lateral_ctrl_sp.course = sp.course_setpoint;
		fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

		_ctrl_configuration_handler.setLateralAccelMax(rollAngleToLateralAccel(getMaxRollAngleNearGround(_current_altitude,
				terrain_alt)));

		/* longitudinal guidance */

		// Open the desired max sink rate to encompass the glide slope.
		// x/sqrt(x^2+1) = sin(arctan(x))
		const float glide_slope_sink_rate = airspeed_land * glide_slope / sqrtf(glide_slope * glide_slope + 1.0f);
		const float desired_max_sinkrate = math::max(glide_slope_sink_rate, _param_sinkrate_target.get());

		const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
			.timestamp = hrt_absolute_time(),
			.altitude = altitude_setpoint,
			.height_rate = NAN,
			.equivalent_airspeed = airspeed_land,
			.pitch_direct = NAN,
			.throttle_direct = NAN
		};

		_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

		_ctrl_configuration_handler.setThrottleMin(_param_fw_thr_idle.get());
		_ctrl_configuration_handler.setThrottleMax(_landed ? _param_fw_thr_idle.get() : NAN);
		_ctrl_configuration_handler.setSinkRateTarget(desired_max_sinkrate);
	}

	fixed_wing_runway_control_s fw_runway_control{};
	fw_runway_control.timestamp = now;
	fw_runway_control.wheel_steering_enabled = true;
	fw_runway_control.wheel_steering_nudging_rate = _param_fw_lnd_nudge.get() > LandingNudgingOption::kNudgingDisabled ?
			_sticks.getYaw() : 0.f;

	_fixed_wing_runway_control_pub.publish(fw_runway_control);

	_flaps_setpoint = _param_fw_flaps_lnd_scl.get();
	_spoilers_setpoint = _param_fw_spoilers_lnd.get();

	// deploy gear as soon as we're in land mode, if not already done before
	_new_landing_gear_position = landing_gear_s::GEAR_DOWN;

	if (!_vehicle_status.in_transition_to_fw) {
		publishLocalPositionSetpoint(pos_sp_curr);
	}

	landing_status_publish();
}

void
FixedWingModeManager::control_auto_landing_circular(const hrt_abstime &now, const float control_interval,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	const float airspeed_land = (_param_fw_lnd_airspd.get() > FLT_EPSILON) ? _param_fw_lnd_airspd.get() :
				    _param_fw_airspd_min.get();

	_ctrl_configuration_handler.setEnforceLowHeightCondition(true);


	const Vector2f local_position{_local_pos.x, _local_pos.y};
	Vector2f local_landing_orbit_center = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	if (_time_started_landing == 0) {
		// save time at which we started landing and reset landing abort status
		reset_landing_state();
		_time_started_landing = now;
	}

	const bool abort_on_terrain_timeout = checkLandingAbortBitMask(_param_fw_lnd_abort.get(),
					      position_controller_landing_status_s::TERRAIN_TIMEOUT);
	const float terrain_alt = getLandingTerrainAltitudeEstimate(now, pos_sp_curr.alt, false, abort_on_terrain_timeout);

	// flare at the maximum of the altitude determined by the time before touchdown and a minimum flare altitude
	const float flare_rel_alt = math::max(_param_fw_lnd_fl_time.get() * _local_pos.vz, _param_fw_lnd_flalt.get());

	float loiter_radius = fabsf(pos_sp_curr.loiter_radius);
	bool loiter_direction_ccw = pos_sp_curr.loiter_direction_counter_clockwise;

	if (loiter_radius < FLT_EPSILON) {
		loiter_radius = fabsf(_param_nav_loiter_rad.get());
		loiter_direction_ccw = _param_nav_loiter_rad.get() < -FLT_EPSILON;
	}

	if ((_current_altitude < terrain_alt + flare_rel_alt) || _flare_states.flaring) {
		// flare and land with minimal speed

		// flaring is a "point of no return"
		if (!_flare_states.flaring) {
			_flare_states.flaring = true;
			_flare_states.start_time = now;
			_flare_states.initial_height_rate_setpoint = -_local_pos.vz;
			events::send(events::ID("fixedwing_position_control_landing_circle_flaring"), events::Log::Info,
				     "Landing, flaring");
		}

		// ramp in flare limits and setpoints with the flare time, command a soft touchdown
		const float seconds_since_flare_start = hrt_elapsed_time(&_flare_states.start_time) * 1.e-6f;
		const float flare_ramp_interpolator = math::constrain(seconds_since_flare_start / _param_fw_lnd_fl_time.get(), 0.0f,
						      1.0f);

		/* lateral guidance first, because npfg will adjust the airspeed setpoint if necessary */
		const DirectionalGuidanceOutput sp = navigateLoiter(local_landing_orbit_center, local_position, loiter_radius,
						     loiter_direction_ccw,
						     ground_speed, _wind_vel);
		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = now;
		fw_lateral_ctrl_sp.course = sp.course_setpoint;
		fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;

		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);
		/* longitudinal guidance */

		const float flare_ramp_interpolator_sqrt = sqrtf(flare_ramp_interpolator);

		const float height_rate_setpoint = flare_ramp_interpolator_sqrt * (-_param_fw_lnd_fl_sink.get()) +
						   (1.0f - flare_ramp_interpolator_sqrt) * _flare_states.initial_height_rate_setpoint;

		float pitch_min_rad = flare_ramp_interpolator_sqrt * radians(_param_fw_lnd_fl_pmin.get()) +
				      (1.0f - flare_ramp_interpolator_sqrt) * radians(_param_fw_p_lim_min.get());
		float pitch_max_rad = flare_ramp_interpolator_sqrt * radians(_param_fw_lnd_fl_pmax.get()) +
				      (1.0f - flare_ramp_interpolator_sqrt) * radians(_param_fw_p_lim_max.get());

		if (_param_fw_lnd_td_time.get() > FLT_EPSILON) {
			const float touchdown_time = math::max(_param_fw_lnd_td_time.get(), _param_fw_lnd_fl_time.get());

			const float touchdown_interpolator = math::constrain((seconds_since_flare_start - touchdown_time) /
							     POST_TOUCHDOWN_CLAMP_TIME, 0.0f, 1.0f);

			pitch_max_rad = touchdown_interpolator * math::radians(_param_rwto_psp.get()) + (1.0f - touchdown_interpolator) *
					pitch_max_rad;
			pitch_min_rad = touchdown_interpolator * math::radians(_param_rwto_psp.get()) + (1.0f - touchdown_interpolator) *
					pitch_min_rad;
		}

		// idle throttle may be >0 for internal combustion engines
		// normally set to zero for electric motors
		const float throttle_max = flare_ramp_interpolator_sqrt * _param_fw_thr_idle.get() +
					   (1.0f - flare_ramp_interpolator_sqrt) *
					   _param_fw_thr_max.get();
		const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
			.timestamp = now,
			.altitude = NAN,
			.height_rate = height_rate_setpoint,
			.equivalent_airspeed = airspeed_land,
			.pitch_direct = NAN,
			.throttle_direct = NAN
		};

		_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

		_ctrl_configuration_handler.setPitchMin(pitch_min_rad);
		_ctrl_configuration_handler.setPitchMax(pitch_max_rad);
		_ctrl_configuration_handler.setThrottleMax(throttle_max);
		_ctrl_configuration_handler.setThrottleMin(_param_fw_thr_idle.get());
		_ctrl_configuration_handler.setDisableUnderspeedProtection(true);

	} else {

		// follow the glide slope
		const DirectionalGuidanceOutput sp = navigateLoiter(local_landing_orbit_center, local_position, loiter_radius,
						     loiter_direction_ccw,
						     ground_speed, _wind_vel);
		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = now;
		fw_lateral_ctrl_sp.course = sp.course_setpoint;
		fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;

		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

		/* longitudinal guidance */

		// Open the desired max sink rate to encompass the glide slope.
		// x/sqrt(x^2+1) = sin(arctan(x))
		const float glide_slope = math::radians(_param_fw_lnd_ang.get());
		const float glide_slope_sink_rate = airspeed_land * glide_slope / sqrtf(glide_slope * glide_slope + 1.0f);
		const float desired_max_sinkrate = math::max(glide_slope_sink_rate, _param_sinkrate_target.get());

		const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
			.timestamp = now,
			.altitude = NAN,
			.height_rate = -glide_slope_sink_rate,
			.equivalent_airspeed = airspeed_land,
			.pitch_direct = NAN,
			.throttle_direct = NAN
		};

		_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

		_ctrl_configuration_handler.setThrottleMin(_param_fw_thr_idle.get());
		_ctrl_configuration_handler.setThrottleMax(_landed ? _param_fw_thr_idle.get() : NAN);
		_ctrl_configuration_handler.setSinkRateTarget(desired_max_sinkrate);
	}

	fixed_wing_runway_control_s fw_runway_control{};
	fw_runway_control.timestamp = now;
	fw_runway_control.wheel_steering_enabled = true;
	fw_runway_control.wheel_steering_nudging_rate = _param_fw_lnd_nudge.get() > LandingNudgingOption::kNudgingDisabled ?
			_sticks.getYaw() : 0.f;

	_fixed_wing_runway_control_pub.publish(fw_runway_control);

	_ctrl_configuration_handler.setLateralAccelMax(rollAngleToLateralAccel(getMaxRollAngleNearGround(_current_altitude,
			terrain_alt)));

	_flaps_setpoint = _param_fw_flaps_lnd_scl.get();
	_spoilers_setpoint = _param_fw_spoilers_lnd.get();

	if (!_vehicle_status.in_transition_to_fw) {
		publishLocalPositionSetpoint(pos_sp_curr);
	}

	landing_status_publish();
	publishOrbitStatus(pos_sp_curr);
}

void
FixedWingModeManager::control_manual_altitude(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed)
{
	updateManualTakeoffStatus();

	const float height_rate_sp = getManualHeightRateSetpoint();

	// TECS may try to pitch down to gain airspeed if we underspeed, constrain the pitch when underspeeding if we are
	// just passed launch
	const float min_pitch = (_completed_manual_takeoff) ? NAN :
				MIN_PITCH_DURING_MANUAL_TAKEOFF;

	float throttle_max = NAN;

	// enable the operator to kill the throttle on ground
	if (_landed && (_manual_control_setpoint_for_airspeed < THROTTLE_THRESH)) {
		throttle_max = 0.0f;
	}

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = NAN,
		.height_rate = height_rate_sp,
		.equivalent_airspeed = get_manual_airspeed_setpoint(),
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	_ctrl_configuration_handler.setPitchMin(min_pitch);
	_ctrl_configuration_handler.setThrottleMax(throttle_max);

	const float roll_body = _sticks.getRoll() * radians(_param_fw_r_lim.get());
	const DirectionalGuidanceOutput sp = {.course_setpoint = NAN, .lateral_acceleration_feedforward = rollAngleToLateralAccel(roll_body)};
	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.course = sp.course_setpoint;
	fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);
}

void
FixedWingModeManager::control_manual_position(const hrt_abstime now, const float control_interval,
		const Vector2d &curr_pos,
		const Vector2f &ground_speed)
{
	updateManualTakeoffStatus();

	const float height_rate_sp = getManualHeightRateSetpoint();

	// TECS may try to pitch down to gain airspeed if we underspeed, constrain the pitch when underspeeding if we are
	// just passed launch
	const float min_pitch = (_completed_manual_takeoff) ? NAN :
				MIN_PITCH_DURING_MANUAL_TAKEOFF;

	float throttle_max = NAN;

	// enable the operator to kill the throttle on ground
	if (_landed && (_manual_control_setpoint_for_airspeed < THROTTLE_THRESH)) {
		throttle_max = 0.0f;
	}

	if (_local_pos.xy_reset_counter != _xy_reset_counter) {
		_time_last_xy_reset = now;
	}

	/* heading control */
	// TODO: either make it course hold (easier) or a real heading hold (minus all the complexity here)
	if (fabsf(_sticks.getRoll()) < HDG_HOLD_MAN_INPUT_THRESH &&
	    fabsf(_sticks.getYaw()) < HDG_HOLD_MAN_INPUT_THRESH) {

		/* heading / roll is zero, lock onto current heading */
		if (fabsf(_yawrate) < HDG_HOLD_YAWRATE_THRESH && !_yaw_lock_engaged) {
			// little yaw movement, lock to current heading
			_yaw_lock_engaged = true;
		}

		/* user tries to do a takeoff in heading hold mode, reset the yaw setpoint on every iteration
			to make sure the plane does not start rolling
		*/
		if (!_completed_manual_takeoff) {
			_hdg_hold_enabled = false;
			_yaw_lock_engaged = true;
		}

		if (_yaw_lock_engaged) {

			const Vector2f curr_pos_local{_local_pos.x, _local_pos.y};

			if (!_hdg_hold_enabled) {
				// just switched back from non heading-hold to heading hold
				_hdg_hold_enabled = true;
				_hdg_hold_yaw = _yaw;

				_hdg_hold_position = curr_pos_local;
			}

			// if there's a reset-by-fusion, the ekf needs some time to converge,
			// therefore we go into track holiding for 2 seconds
			if (now - _time_last_xy_reset < 2_s) {
				_hdg_hold_position = curr_pos_local;
			}

			const DirectionalGuidanceOutput sp = navigateLine(_hdg_hold_position, _hdg_hold_yaw, curr_pos_local, ground_speed,
							     _wind_vel);
			fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
			fw_lateral_ctrl_sp.timestamp = now;
			fw_lateral_ctrl_sp.course = sp.course_setpoint;
			fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;

			_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);
		}
	}

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = NAN,
		.height_rate = height_rate_sp,
		.equivalent_airspeed = get_manual_airspeed_setpoint(),
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	_ctrl_configuration_handler.setPitchMin(min_pitch);
	_ctrl_configuration_handler.setThrottleMax(throttle_max);

	if (!_yaw_lock_engaged || fabsf(_sticks.getRoll()) >= HDG_HOLD_MAN_INPUT_THRESH ||
	    fabsf(_sticks.getYaw()) >= HDG_HOLD_MAN_INPUT_THRESH) {

		_hdg_hold_enabled = false;
		_yaw_lock_engaged = false;

		const float roll_body = _sticks.getRoll() * radians(_param_fw_r_lim.get());
		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
		fw_lateral_ctrl_sp.lateral_acceleration = rollAngleToLateralAccel(roll_body);
		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);
	}
}

float FixedWingModeManager::rollAngleToLateralAccel(float roll_body) const
{
	return tanf(roll_body) * CONSTANTS_ONE_G;
}

void FixedWingModeManager::control_backtransition_heading_hold()
{
	if (!PX4_ISFINITE(_backtrans_heading)) {
		_backtrans_heading = _local_pos.heading;
	}

	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.airspeed_direction = _backtrans_heading;
	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);
}

void FixedWingModeManager::control_backtransition_line_follow(const Vector2f &ground_speed,
		const position_setpoint_s &pos_sp_curr)
{
	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	// Set the position where the backtransition started the first ime we pass through here.
	// Will get reset if not in transition anymore.
	if (!_lpos_where_backtrans_started.isAllFinite()) {
		_lpos_where_backtrans_started = curr_pos_local;
	}

	DirectionalGuidanceOutput sp = {.course_setpoint = NAN, .lateral_acceleration_feedforward = 0.f};

	if (_control_mode.flag_control_position_enabled) {
		sp = navigateLine(_lpos_where_backtrans_started, curr_wp_local, curr_pos_local, ground_speed, _wind_vel);
	}

	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.course = sp.course_setpoint;
	fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);
}

void
FixedWingModeManager::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	_vehicle_status_sub.update(&_vehicle_status);

	/* only run controller if position changed and we are not running an external mode*/

	const bool is_external_nav_state = (_vehicle_status.nav_state >= vehicle_status_s::NAVIGATION_STATE_EXTERNAL1)
					   && (_vehicle_status.nav_state <= vehicle_status_s::NAVIGATION_STATE_EXTERNAL8);

	if (is_external_nav_state) {
		// this will cause the configuration handler to publish immediately the next time an internal flight
		// mode is active
		_ctrl_configuration_handler.resetLastPublishTime();

	} else if (_local_pos_sub.update(&_local_pos)) {

		const hrt_abstime now = _local_pos.timestamp;

		const float control_interval = math::constrain((now - _last_time_position_control_called) * 1e-6f,
					       MIN_AUTO_TIMESTEP, MAX_AUTO_TIMESTEP);
		_last_time_position_control_called = now;

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			parameters_update();
		}

		vehicle_global_position_s gpos;

		if (_global_pos_sub.update(&gpos)) {
			_current_latitude = gpos.lat;
			_current_longitude = gpos.lon;
		}

		if (_local_pos.z_global && PX4_ISFINITE(_local_pos.ref_alt)) {
			_reference_altitude = _local_pos.ref_alt;

		} else {
			_reference_altitude = 0.f;
		}

		_current_altitude = -_local_pos.z + _reference_altitude; // Altitude AMSL in meters

		// handle estimator reset events. we only adjust setpoins for manual modes
		if (_control_mode.flag_control_manual_enabled) {
			// adjust navigation waypoints in position control mode
			if (_control_mode.flag_control_altitude_enabled && _control_mode.flag_control_velocity_enabled
			    && _local_pos.xy_reset_counter != _xy_reset_counter) {

				// reset heading hold flag, which will re-initialise position control
				_hdg_hold_enabled = false;
			}
		}

		// Convert Local setpoints to global setpoints
		if (!_global_local_proj_ref.isInitialized()
		    || (_global_local_proj_ref.getProjectionReferenceTimestamp() != _local_pos.ref_timestamp)
		    || (_local_pos.xy_reset_counter != _xy_reset_counter)) {

			double reference_latitude = 0.;
			double reference_longitude = 0.;

			if (_local_pos.xy_global && PX4_ISFINITE(_local_pos.ref_lat) && PX4_ISFINITE(_local_pos.ref_lon)) {
				reference_latitude = _local_pos.ref_lat;
				reference_longitude = _local_pos.ref_lon;
			}

			_global_local_proj_ref.initReference(reference_latitude, reference_longitude,
							     _local_pos.ref_timestamp);
		}

		if (_control_mode.flag_control_offboard_enabled) {
			trajectory_setpoint_s trajectory_setpoint;

			if (_trajectory_setpoint_sub.update(&trajectory_setpoint)) {
				bool valid_setpoint = false;
				_pos_sp_triplet = {}; // clear any existing
				_pos_sp_triplet.timestamp = trajectory_setpoint.timestamp;
				_pos_sp_triplet.current.timestamp = trajectory_setpoint.timestamp;
				_pos_sp_triplet.current.cruising_speed = NAN; // ignored
				_pos_sp_triplet.current.cruising_throttle = NAN; // ignored
				_pos_sp_triplet.current.vx = NAN;
				_pos_sp_triplet.current.vy = NAN;
				_pos_sp_triplet.current.vz = NAN;
				_pos_sp_triplet.current.lat = static_cast<double>(NAN);
				_pos_sp_triplet.current.lon = static_cast<double>(NAN);
				_pos_sp_triplet.current.alt = NAN;

				if (Vector3f(trajectory_setpoint.position).isAllFinite()) {
					if (_global_local_proj_ref.isInitialized()) {
						double lat;
						double lon;
						_global_local_proj_ref.reproject(trajectory_setpoint.position[0], trajectory_setpoint.position[1], lat, lon);
						valid_setpoint = true;
						_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
						_pos_sp_triplet.current.lat = lat;
						_pos_sp_triplet.current.lon = lon;
						_pos_sp_triplet.current.alt = _reference_altitude - trajectory_setpoint.position[2];
					}

				}

				if (Vector3f(trajectory_setpoint.velocity).isAllFinite()) {
					valid_setpoint = true;
					_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
					_pos_sp_triplet.current.vx = trajectory_setpoint.velocity[0];
					_pos_sp_triplet.current.vy = trajectory_setpoint.velocity[1];
					_pos_sp_triplet.current.vz = trajectory_setpoint.velocity[2];

					if (Vector3f(trajectory_setpoint.acceleration).isAllFinite()) {
						Vector2f velocity_sp_2d(trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1]);
						Vector2f normalized_velocity_sp_2d = velocity_sp_2d.normalized();
						Vector2f acceleration_sp_2d(trajectory_setpoint.acceleration[0], trajectory_setpoint.acceleration[1]);
						Vector2f acceleration_normal = acceleration_sp_2d - acceleration_sp_2d.dot(normalized_velocity_sp_2d) *
									       normalized_velocity_sp_2d;
						float direction = -normalized_velocity_sp_2d.cross(acceleration_normal.normalized());
						_pos_sp_triplet.current.loiter_radius = direction * velocity_sp_2d.norm() * velocity_sp_2d.norm() /
											acceleration_normal.norm();

					} else {
						_pos_sp_triplet.current.loiter_radius = NAN;
					}
				}

				_position_setpoint_current_valid = valid_setpoint;
			}

		} else {
			if (_pos_sp_triplet_sub.update(&_pos_sp_triplet)) {

				_position_setpoint_previous_valid = PX4_ISFINITE(_pos_sp_triplet.previous.lat)
								    && PX4_ISFINITE(_pos_sp_triplet.previous.lon)
								    && PX4_ISFINITE(_pos_sp_triplet.previous.alt);

				_position_setpoint_current_valid = PX4_ISFINITE(_pos_sp_triplet.current.lat)
								   && PX4_ISFINITE(_pos_sp_triplet.current.lon)
								   && PX4_ISFINITE(_pos_sp_triplet.current.alt);

				_position_setpoint_next_valid = PX4_ISFINITE(_pos_sp_triplet.next.lat)
								&& PX4_ISFINITE(_pos_sp_triplet.next.lon)
								&& PX4_ISFINITE(_pos_sp_triplet.next.alt);

				// reset the altitude foh (first order hold) logic
				_min_current_sp_distance_xy = FLT_MAX;
			}
		}

		airspeed_poll();
		manual_control_setpoint_poll();
		vehicle_attitude_poll();
		vehicle_command_poll();
		vehicle_control_mode_poll();
		wind_poll(now);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.update(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}

		if (!_vehicle_status.in_transition_mode) {
			// reset position of backtransition start if not in transition
			_lpos_where_backtrans_started = Vector2f(NAN, NAN);
			_backtrans_heading = NAN;
		}


		Vector2d curr_pos(_current_latitude, _current_longitude);
		Vector2f ground_speed(_local_pos.vx, _local_pos.vy);

		set_control_mode_current(now);

		update_in_air_states(now);

		// restore nominal TECS parameters in case changed intermittently (e.g. in landing handling)

		// restore lateral-directional guidance parameters (changed in takeoff mode)
		_directional_guidance.setPeriod(_param_npfg_period.get());

		// by default no flaps/spoilers, is overwritten below in certain modes
		_flaps_setpoint = 0.f;
		_spoilers_setpoint = 0.f;

		// by default set speed weight to the param value, can be overwritten inside the methods below
		_ctrl_configuration_handler.setSpeedWeight(_param_t_spdweight.get());

		if (_control_mode_current != FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT
		    && _control_mode_current != FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR) {
			reset_landing_state();
		}

		// a water scoop only runs in AUTO (mission); leaving it (mode change by the operator) ends it
		if (_control_mode_current != FW_POSCTRL_MODE_AUTO && _scoop.phase != ScoopPhase::OFF) {
			mavlink_log_info(&_mavlink_log_pub, "SU: IPTAL MOD\t");
			_scoop.phase = ScoopPhase::OFF;
			_scoop.armed = false;
		}

		if (_control_mode_current != FW_POSCTRL_MODE_AUTO_TAKEOFF
		    && _control_mode_current != FW_POSCTRL_MODE_AUTO_TAKEOFF_NO_NAV) {
			reset_takeoff_state();
		}

		int8_t old_landing_gear_position = _new_landing_gear_position;
		_new_landing_gear_position = landing_gear_s::GEAR_KEEP; // is overwritten in Takeoff and Land

		switch (_control_mode_current) {
		case FW_POSCTRL_MODE_AUTO: {
				// GASI water scoop: the water waypoints (and the climb out of them) are flown by control_auto_scoop
				if (scoopEngage(_pos_sp_triplet.previous, _pos_sp_triplet.current)) {
					control_auto_scoop(now, ground_speed, _pos_sp_triplet.current);

				} else {
					control_auto(control_interval, curr_pos, ground_speed, _pos_sp_triplet.previous, _pos_sp_triplet.current,
						     _pos_sp_triplet.next);
				}

				break;
			}

		case FW_POSCTRL_MODE_AUTO_ALTITUDE: {
				control_auto_fixed_bank_alt_hold();
				break;
			}

		case FW_POSCTRL_MODE_AUTO_CLIMBRATE: {
				control_auto_descend();
				break;
			}

		case FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT: {
				control_auto_landing_straight(now, control_interval, ground_speed, _pos_sp_triplet.previous,
							      _pos_sp_triplet.current);
				break;
			}

		case FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR: {
				control_auto_landing_circular(now, control_interval, ground_speed, _pos_sp_triplet.current);
				break;
			}

		case FW_POSCTRL_MODE_AUTO_PATH: {
				control_auto_path(control_interval, curr_pos, ground_speed, _pos_sp_triplet.current);
				break;
			}

		case FW_POSCTRL_MODE_AUTO_TAKEOFF: {
				control_auto_takeoff(now, control_interval, curr_pos, ground_speed, _pos_sp_triplet.current);
				break;
			}

		case FW_POSCTRL_MODE_AUTO_TAKEOFF_NO_NAV: {
				control_auto_takeoff_no_nav(_local_pos.timestamp, control_interval, _pos_sp_triplet.current.alt);
				break;
			}

		case FW_POSCTRL_MODE_MANUAL_POSITION: {
				control_manual_position(now, control_interval, curr_pos, ground_speed);
				break;
			}

		case FW_POSCTRL_MODE_MANUAL_ALTITUDE: {
				control_manual_altitude(control_interval, curr_pos, ground_speed);
				break;
			}

		case FW_POSCTRL_MODE_OTHER: {
				break;
			}

		case FW_POSCTRL_MODE_TRANSITION_TO_HOVER_LINE_FOLLOW: {
				control_backtransition_line_follow(ground_speed, _pos_sp_triplet.current);
				break;
			}

		case FW_POSCTRL_MODE_TRANSITION_TO_HOVER_HEADING_HOLD: {
				control_backtransition_heading_hold();
				break;
			}
		}

		if (_control_mode_current != FW_POSCTRL_MODE_OTHER) {
			apply_takeoff_exit_blend(now);
			_ctrl_configuration_handler.update(now);
		}

		// only publish status in full FW mode
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
		    || _vehicle_status.in_transition_mode) {
			publish_lateral_guidance_status(now);

		}

		// if there's any change in landing gear setpoint publish it
		if (_new_landing_gear_position != old_landing_gear_position
		    && _new_landing_gear_position != landing_gear_s::GEAR_KEEP) {

			landing_gear_s landing_gear = {};
			landing_gear.landing_gear = _new_landing_gear_position;
			landing_gear.timestamp = now;
			_landing_gear_pub.publish(landing_gear);
		}

		// In Manual modes flaps and spoilers are directly controlled in the Attitude controller and not published here
		if (_control_mode.flag_control_auto_enabled
		    && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			normalized_unsigned_setpoint_s flaps_setpoint;
			flaps_setpoint.normalized_setpoint = _flaps_setpoint;
			flaps_setpoint.timestamp = now;
			_flaps_setpoint_pub.publish(flaps_setpoint);

			normalized_unsigned_setpoint_s spoilers_setpoint;
			spoilers_setpoint.normalized_setpoint = _spoilers_setpoint;
			spoilers_setpoint.timestamp = now;
			_spoilers_setpoint_pub.publish(spoilers_setpoint);
		}

		_xy_reset_counter = _local_pos.xy_reset_counter;

		perf_end(_loop_perf);
	}
}

void
FixedWingModeManager::reset_takeoff_state()
{
	_runway_takeoff.reset();

	_launchDetector.reset();

	_launch_detected = false;

	_takeoff_ground_alt = _current_altitude;

	// _time_takeoff_climbout_last is kept: this runs every cycle outside the takeoff modes, where the exit blend runs.
	// apply_takeoff_exit_blend() clears it when the blend ends or the vehicle disarms.
}

void
FixedWingModeManager::apply_takeoff_exit_blend(const hrt_abstime now)
{
	const float blend_time = _param_fw_tko_blend_t.get();

	if (blend_time < FLT_EPSILON || _time_takeoff_climbout_last == 0 || now <= _time_takeoff_climbout_last) {
		// disabled, no runway climbout yet, or still in it
		return;
	}

	const float progress = (now - _time_takeoff_climbout_last) * 1e-6f / blend_time;

	if (progress >= 1.f || _landed || !_control_mode.flag_armed
	    || _control_mode_current == FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT
	    || _control_mode_current == FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR
	    || _control_mode_current == FW_POSCTRL_MODE_MANUAL_POSITION
	    || _control_mode_current == FW_POSCTRL_MODE_MANUAL_ALTITUDE
	    || _control_mode_current == FW_POSCTRL_MODE_OTHER) {
		_time_takeoff_climbout_last = 0;
		return;
	}

	// pitch floor: from the climbout floor down to FW_P_LIM_MIN
	const float pitch_min_blend = math::interpolate(progress, 0.f, 1.f, _takeoff_exit_pitch_min,
				      math::radians(_param_fw_p_lim_min.get()));
	const float pitch_min = _ctrl_configuration_handler.getPitchMin();
	_ctrl_configuration_handler.setPitchMin(PX4_ISFINITE(pitch_min) ? math::max(pitch_min, pitch_min_blend) : pitch_min_blend);

	// throttle range: opens from the takeoff throttle to FW_THR_MIN..FW_THR_MAX
	const float throttle_min_blend = math::interpolate(progress, 0.f, 1.f, _takeoff_exit_throttle, _param_fw_thr_min.get());
	const float throttle_max_blend = math::interpolate(progress, 0.f, 1.f, _takeoff_exit_throttle, _param_fw_thr_max.get());
	const float throttle_min = _ctrl_configuration_handler.getThrottleMin();
	const float throttle_max = _ctrl_configuration_handler.getThrottleMax();
	const float throttle_max_new = PX4_ISFINITE(throttle_max) ? math::min(throttle_max, throttle_max_blend) : throttle_max_blend;
	const float throttle_min_new = PX4_ISFINITE(throttle_min) ? math::max(throttle_min, throttle_min_blend) : throttle_min_blend;
	_ctrl_configuration_handler.setThrottleMin(math::min(throttle_min_new, throttle_max_new));
	_ctrl_configuration_handler.setThrottleMax(throttle_max_new);

	// flaps: retract from the takeoff setting instead of at once
	_flaps_setpoint = math::max(_flaps_setpoint, (1.f - progress) * _param_fw_flaps_to_scl.get());
}

void
FixedWingModeManager::reset_landing_state()
{
	_time_started_landing = 0;

	_flare_states = FlareStates{};

	_lateral_touchdown_position_offset = 0.0f;

	_last_time_terrain_alt_was_valid = 0;

	// reset abort land, unless loitering after an abort
	if ((_landing_abort_status && (_pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_LOITER)) ||
	    (_landing_abort_status && _param_fw_lnd_abort.get() == 0)) {

		updateLandingAbortStatus(position_controller_landing_status_s::NOT_ABORTED);
	}
}

float FixedWingModeManager::getMaxRollAngleNearGround(const float altitude, const float terrain_altitude) const
{
	// we want the wings level when at the wing height above ground
	const float height_above_ground = math::max(altitude - (terrain_altitude + _param_fw_wing_height.get()), 0.0f);

	// this is a conservative (linear) approximation of the roll angle that would cause wing tip strike
	// roll strike = arcsin( 2 * height / span )
	// d(roll strike)/d(height) = 2 / span / cos(2 * height / span)
	// d(roll strike)/d(height) (@height=0) = 2 / span
	// roll strike ~= 2 * height / span

	return  math::constrain(2.f * height_above_ground / _param_fw_wing_span.get(), 0.f,
				math::radians(_param_fw_r_lim.get()));
}


void
FixedWingModeManager::initializeAutoLanding(const hrt_abstime &now, const position_setpoint_s &pos_sp_prev,
		const float land_point_altitude, const Vector2f &local_position, const Vector2f &local_land_point)
{
	if (_time_started_landing == 0) {

		float height_above_land_point;
		Vector2f local_approach_entrance;

		// set the landing approach entrance location when we have just started the landing and store it
		// NOTE: the landing approach vector is relative to the land point. ekf resets may cause a local frame
		// jump, so we reference to the land point, which is globally referenced and will update
		if (_position_setpoint_previous_valid) {
			height_above_land_point = pos_sp_prev.alt - land_point_altitude;
			local_approach_entrance = _global_local_proj_ref.project(pos_sp_prev.lat, pos_sp_prev.lon);

		} else {
			// no valid previous waypoint, construct one from the glide slope and direction from current
			// position to land point

			// NOTE: this is not really a supported use case at the moment, this is just bandaiding any
			// ill-advised usage of the current implementation

			// TODO: proper handling of on-the-fly landing points would need to involve some more sophisticated
			// landing pattern generation and corresponding logic

			height_above_land_point = _current_altitude - land_point_altitude;
			local_approach_entrance = local_position;
		}

		_landing_approach_entrance_rel_alt = math::max(height_above_land_point, FLT_EPSILON);

		const Vector2f landing_approach_vector = local_land_point - local_approach_entrance;
		float landing_approach_distance = landing_approach_vector.norm();

		const float max_glide_slope = tanf(math::radians(_param_fw_lnd_ang.get()));
		const float glide_slope = _landing_approach_entrance_rel_alt / landing_approach_distance;

		if (glide_slope > max_glide_slope) {
			// rescale the landing distance - this will have the same effect as dropping down the approach
			// entrance altitude on the vehicle's behavior. if we reach here.. it means the navigator checks
			// didn't work, or something is using the control_auto_landing_straight() method inappropriately
			landing_approach_distance = _landing_approach_entrance_rel_alt / max_glide_slope;
		}

		if (landing_approach_vector.norm_squared() > FLT_EPSILON) {
			_landing_approach_entrance_offset_vector = -landing_approach_vector.unit_or_zero() * landing_approach_distance;

		} else {
			// land in direction of airframe
			_landing_approach_entrance_offset_vector = Vector2f({cosf(_yaw), sinf(_yaw)}) * landing_approach_distance;
		}

		// save time at which we started landing and reset landing abort status
		reset_landing_state();
		_time_started_landing = now;
	}
}

Vector2f
FixedWingModeManager::calculateTouchdownPosition(const float control_interval, const Vector2f &local_land_position)
{
	if (fabsf(_sticks.getYaw()) > MANUAL_TOUCHDOWN_NUDGE_INPUT_DEADZONE
	    && _param_fw_lnd_nudge.get() > LandingNudgingOption::kNudgingDisabled
	    && !_flare_states.flaring) {
		// laterally nudge touchdown location with yaw stick
		// positive is defined in the direction of a right hand turn starting from the approach vector direction
		const float signed_deadzone_threshold = MANUAL_TOUCHDOWN_NUDGE_INPUT_DEADZONE * math::signNoZero(
				_sticks.getYaw());
		_lateral_touchdown_position_offset += (_sticks.getYaw() - signed_deadzone_threshold) *
						      MAX_TOUCHDOWN_POSITION_NUDGE_RATE * control_interval;
		_lateral_touchdown_position_offset = math::constrain(_lateral_touchdown_position_offset, -_param_fw_lnd_td_off.get(),
						     _param_fw_lnd_td_off.get());
	}

	const Vector2f approach_unit_vector = -_landing_approach_entrance_offset_vector.unit_or_zero();
	const Vector2f approach_unit_normal_vector{-approach_unit_vector(1), approach_unit_vector(0)};

	return local_land_position + approach_unit_normal_vector * _lateral_touchdown_position_offset;
}

Vector2f
FixedWingModeManager::calculateLandingApproachVector() const
{
	Vector2f landing_approach_vector = -_landing_approach_entrance_offset_vector;
	const Vector2f approach_unit_vector = landing_approach_vector.unit_or_zero();
	const Vector2f approach_unit_normal_vector{-approach_unit_vector(1), approach_unit_vector(0)};

	if (_param_fw_lnd_nudge.get() == LandingNudgingOption::kNudgeApproachAngle) {
		// nudge the approach angle -- i.e. we adjust the approach vector to reach from the original approach
		// entrance position to the newly nudged touchdown point
		// NOTE: this lengthens the landing distance.. which will adjust the glideslope height slightly
		landing_approach_vector += approach_unit_normal_vector * _lateral_touchdown_position_offset;
	}

	// if _param_fw_lnd_nudge.get() == LandingNudgingOption::kNudgingDisabled, no nudging

	// if _param_fw_lnd_nudge.get() == LandingNudgingOption::kNudgeApproachPath, the full path (including approach
	// entrance point) is nudged with the touchdown point, which does not require any additions to the approach vector

	return landing_approach_vector;
}

float
FixedWingModeManager::updateStallSpeedEstimate(const hrt_abstime now, const float height)
{
	// frozen while flaring: ground effect and the flare manoeuvre would bias it
	if (_flare_states.flaring) {
		return (_vs_est_settled >= _param_fw_lnd_vs_tau.get()) ? _vs_est : NAN;
	}

	// a new landing approach (not called for a while): start over
	if (hrt_elapsed_time(&_vs_est_last_call) > 3_s) {
		_vs_est = NAN;
		_vs_est_settled = 0.0f;
		_vs_est_last_sample = 0;
		_vs_wings_level_since = 0;
	}

	_vs_est_last_call = now;

	const bool airspeed_ok = hrt_elapsed_time(&_time_airspeed_sample) < 1_s;

	// samples only after 2 s wings level (the turn onto final biased the first samples high), clear of ground effect
	if (fabsf(_roll) > radians(_param_fw_lnd_vs_roll.get()) || _vs_wings_level_since == 0) {
		_vs_wings_level_since = now;
	}

	if (airspeed_ok && _airspeed_eas > 10.0f && height > _param_fw_lnd_vs_hmin.get()
	    && hrt_elapsed_time(&_vs_wings_level_since) > (hrt_abstime)(_param_fw_lnd_vs_lvl.get() * 1e6f)
	    && _param_fw_lnd_vs_clmx.get() > FLT_EPSILON) {
		// synthetic angle of attack: pitch minus the flight path angle (still air, no vertical wind)
		const float flight_path_angle = asinf(math::constrain(-_local_pos.vz / _airspeed_eas, -0.5f, 0.5f));
		const float alpha = _pitch - flight_path_angle;
		const float cl = math::constrain(_param_fw_lnd_vs_cl0.get() + _param_fw_lnd_vs_cla.get() * alpha, 0.1f,
						 _param_fw_lnd_vs_clmx.get());
		// lift = weight cos(roll): 1 g stall speed V sqrt(CL cos(roll) / CLmax)
		const float vs_sample = _airspeed_eas * sqrtf(cl * cosf(_roll) / _param_fw_lnd_vs_clmx.get());
		const float dt = (_vs_est_last_sample > 0) ? math::constrain(hrt_elapsed_time(&_vs_est_last_sample) * 1e-6f, 0.0f,
				 0.5f) : 0.0f;
		_vs_est_last_sample = now;

		if (!PX4_ISFINITE(_vs_est)) {
			_vs_est = vs_sample;

		} else {
			// first order filter, FW_LND_VS_TAU time constant
			_vs_est += (vs_sample - _vs_est) * math::min(dt / math::max(_param_fw_lnd_vs_tau.get(), 0.1f), 1.0f);
			_vs_est_settled += dt;
		}
	}

	return (_vs_est_settled >= _param_fw_lnd_vs_tau.get()) ? _vs_est : NAN;
}


float
FixedWingModeManager::getLandingTerrainAltitudeEstimate(const hrt_abstime &now, const float land_point_altitude,
		const bool abort_on_terrain_measurement_timeout, const bool abort_on_terrain_timeout)
{
	if (_param_fw_lnd_useter.get() > TerrainEstimateUseOnLanding::kDisableTerrainEstimation) {

		if (_local_pos.dist_bottom_valid) {

			const float terrain_estimate = _local_pos.ref_alt + -_local_pos.z - _local_pos.dist_bottom;
			_last_valid_terrain_alt_estimate = terrain_estimate;
			_last_time_terrain_alt_was_valid = now;

			return terrain_estimate;
		}

		if (_last_time_terrain_alt_was_valid == 0) {

			const bool terrain_first_measurement_timed_out = (now - _time_started_landing) > TERRAIN_ALT_FIRST_MEASUREMENT_TIMEOUT;

			if (terrain_first_measurement_timed_out && abort_on_terrain_measurement_timeout) {
				updateLandingAbortStatus(position_controller_landing_status_s::TERRAIN_NOT_FOUND);
			}

			return land_point_altitude;
		}

		if (!_local_pos.dist_bottom_valid) {

			const bool terrain_timed_out = (now - _last_time_terrain_alt_was_valid) > TERRAIN_ALT_TIMEOUT;

			if (terrain_timed_out && abort_on_terrain_timeout) {
				updateLandingAbortStatus(position_controller_landing_status_s::TERRAIN_TIMEOUT);
			}

			return _last_valid_terrain_alt_estimate;
		}
	}

	return land_point_altitude;
}

bool FixedWingModeManager::checkLandingAbortBitMask(const uint8_t automatic_abort_criteria_bitmask,
		uint8_t landing_abort_criterion)
{
	// landing abort status contains a manual criterion at abort_status==1, need to subtract 2 to directly compare
	// to automatic criteria bits from the parameter FW_LND_ABORT
	if (landing_abort_criterion <= 1) {
		return false;
	}

	landing_abort_criterion -= 2;

	return ((1 << landing_abort_criterion) & automatic_abort_criteria_bitmask) == (1 << landing_abort_criterion);
}

void FixedWingModeManager::publishLocalPositionSetpoint(const position_setpoint_s &current_waypoint)
{
	vehicle_local_position_setpoint_s local_position_setpoint{};
	local_position_setpoint.timestamp = hrt_absolute_time();

	Vector2f current_setpoint;

	current_setpoint = _closest_point_on_path;

	local_position_setpoint.x = current_setpoint(0);
	local_position_setpoint.y = current_setpoint(1);
	local_position_setpoint.z = _reference_altitude - current_waypoint.alt;
	local_position_setpoint.yaw = NAN;
	local_position_setpoint.yawspeed = NAN;
	local_position_setpoint.vx = NAN;
	local_position_setpoint.vy = NAN;
	local_position_setpoint.vz = NAN;
	local_position_setpoint.acceleration[0] = NAN;
	local_position_setpoint.acceleration[1] = NAN;
	local_position_setpoint.acceleration[2] = NAN;
	_local_pos_sp_pub.publish(local_position_setpoint);
}

void FixedWingModeManager::publishOrbitStatus(const position_setpoint_s pos_sp)
{
	orbit_status_s orbit_status{};
	orbit_status.timestamp = hrt_absolute_time();
	float loiter_radius = pos_sp.loiter_radius * (pos_sp.loiter_direction_counter_clockwise ? -1.f : 1.f);

	if (fabsf(loiter_radius) < FLT_EPSILON) {
		loiter_radius = _param_nav_loiter_rad.get();
	}

	orbit_status.radius = loiter_radius;
	orbit_status.frame = 0; // MAV_FRAME::MAV_FRAME_GLOBAL
	orbit_status.x = static_cast<double>(pos_sp.lat);
	orbit_status.y = static_cast<double>(pos_sp.lon);
	orbit_status.z = pos_sp.alt;
	orbit_status.yaw_behaviour = orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE;
	_orbit_status_pub.publish(orbit_status);
}

DirectionalGuidanceOutput FixedWingModeManager::navigateWaypoints(const Vector2f &start_waypoint,
		const Vector2f &end_waypoint,
		const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const Vector2f start_waypoint_to_end_waypoint = end_waypoint - start_waypoint;
	const Vector2f start_waypoint_to_vehicle = vehicle_pos - start_waypoint;
	const Vector2f end_waypoint_to_vehicle = vehicle_pos - end_waypoint;

	if (start_waypoint_to_end_waypoint.norm() < FLT_EPSILON) {
		// degenerate case: the waypoints are on top of each other, this should only happen when someone uses this
		// method incorrectly. just as a safe guard, call the singular waypoint navigation method.
		return navigateWaypoint(end_waypoint, vehicle_pos, ground_vel, wind_vel);
	}

	if ((start_waypoint_to_end_waypoint.dot(start_waypoint_to_vehicle) < -FLT_EPSILON)
	    && (start_waypoint_to_vehicle.norm() > _directional_guidance.switchDistance(500.0f))) {
		// we are in front of the start waypoint, fly directly to it until we are within switch distance
		return navigateWaypoint(start_waypoint, vehicle_pos, ground_vel, wind_vel);
	}

	if (start_waypoint_to_end_waypoint.dot(end_waypoint_to_vehicle) > FLT_EPSILON) {
		// we are beyond the end waypoint, fly back to it
		// NOTE: this logic ideally never gets executed, as a waypoint switch should happen before passing the
		// end waypoint. however this included here as a safety precaution if any navigator (module) switch condition
		// is missed for any reason. in the future this logic should all be handled in one place in a dedicated
		// flight mode state machine.
		return navigateWaypoint(end_waypoint, vehicle_pos, ground_vel, wind_vel);
	}

	// follow the line segment between the start and end waypoints
	return navigateLine(start_waypoint, end_waypoint, vehicle_pos, ground_vel, wind_vel);
}

DirectionalGuidanceOutput FixedWingModeManager::navigateWaypoint(const Vector2f &waypoint_pos,
		const Vector2f &vehicle_pos,
		const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const Vector2f vehicle_to_waypoint = waypoint_pos - vehicle_pos;

	if (vehicle_to_waypoint.norm() < FLT_EPSILON) {
		// degenerate case: the vehicle is on top of the single waypoint. (can happen). maintain the last npfg command.
		return DirectionalGuidanceOutput{};
	}

	const Vector2f unit_path_tangent = vehicle_to_waypoint.normalized();
	_closest_point_on_path = waypoint_pos;

	const float path_curvature = 0.f;
	DirectionalGuidanceOutput sp = _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent,
				       _closest_point_on_path, path_curvature);

	return sp;
}

DirectionalGuidanceOutput FixedWingModeManager::navigateLine(const Vector2f &point_on_line_1,
		const Vector2f &point_on_line_2,
		const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const Vector2f line_segment = point_on_line_2 - point_on_line_1;

	if (line_segment.norm() <= FLT_EPSILON) {
		// degenerate case: line segment has zero length. maintain the last npfg command.
		return DirectionalGuidanceOutput{};
	}

	const Vector2f unit_path_tangent = line_segment.normalized();

	const Vector2f point_1_to_vehicle = vehicle_pos - point_on_line_1;
	_closest_point_on_path = point_on_line_1 + point_1_to_vehicle.dot(unit_path_tangent) * unit_path_tangent;

	const float path_curvature = 0.f;
	const DirectionalGuidanceOutput sp = _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel,
					     unit_path_tangent,
					     _closest_point_on_path, path_curvature);

	return sp;
}

DirectionalGuidanceOutput FixedWingModeManager::navigateLine(const Vector2f &point_on_line,
		const float line_bearing,
		const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const Vector2f unit_path_tangent{cosf(line_bearing), sinf(line_bearing)};

	const Vector2f point_on_line_to_vehicle = vehicle_pos - point_on_line;
	_closest_point_on_path = point_on_line + point_on_line_to_vehicle.dot(unit_path_tangent) * unit_path_tangent;

	const float path_curvature = 0.f;
	const DirectionalGuidanceOutput sp = _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel,
					     unit_path_tangent,
					     _closest_point_on_path, path_curvature);

	return sp;
}

DirectionalGuidanceOutput FixedWingModeManager::navigateLoiter(const Vector2f &loiter_center,
		const Vector2f &vehicle_pos,
		float radius, bool loiter_direction_counter_clockwise, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const float loiter_direction_multiplier = loiter_direction_counter_clockwise ? -1.f : 1.f;

	Vector2f vector_center_to_vehicle = vehicle_pos - loiter_center;
	const float dist_to_center = vector_center_to_vehicle.norm();

	// find the direction from the circle center to the closest point on its perimeter
	// from the vehicle position
	Vector2f unit_vec_center_to_closest_pt;

	if (dist_to_center < 0.1f) {
		// the logic breaks down at the circle center, employ some mitigation strategies
		// until we exit this region
		if (ground_vel.norm() < 0.1f) {
			// arbitrarily set the point in the northern top of the circle
			unit_vec_center_to_closest_pt = Vector2f{1.0f, 0.0f};

		} else {
			// set the point in the direction we are moving
			unit_vec_center_to_closest_pt = ground_vel.normalized();
		}

	} else {
		// set the point in the direction of the aircraft
		unit_vec_center_to_closest_pt = vector_center_to_vehicle.normalized();
	}

	// 90 deg clockwise rotation * loiter direction
	const Vector2f unit_path_tangent = loiter_direction_multiplier * Vector2f{-unit_vec_center_to_closest_pt(1), unit_vec_center_to_closest_pt(0)};

	const float path_curvature = loiter_direction_multiplier / radius;
	_closest_point_on_path = unit_vec_center_to_closest_pt * radius + loiter_center;
	return _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent,
			loiter_center + unit_vec_center_to_closest_pt * radius, path_curvature);
}

DirectionalGuidanceOutput FixedWingModeManager::navigatePathTangent(const matrix::Vector2f &vehicle_pos,
		const matrix::Vector2f &position_setpoint,
		const matrix::Vector2f &tangent_setpoint,
		const matrix::Vector2f &ground_vel, const matrix::Vector2f &wind_vel, const float &curvature)
{
	if (tangent_setpoint.norm() <= FLT_EPSILON) {
		// degenerate case: no direction. maintain the last npfg command.
		return DirectionalGuidanceOutput{};
	}

	const Vector2f unit_path_tangent{tangent_setpoint.normalized()};
	_closest_point_on_path = position_setpoint;
	return _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel, tangent_setpoint.normalized(),
			position_setpoint,
			curvature);
}

DirectionalGuidanceOutput FixedWingModeManager::navigateBearing(const matrix::Vector2f &vehicle_pos, float bearing,
		const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const Vector2f unit_path_tangent = Vector2f{cosf(bearing), sinf(bearing)};
	_closest_point_on_path = vehicle_pos;
	return _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent, vehicle_pos, 0.0f);
}

void FixedWingModeManager::publish_lateral_guidance_status(const hrt_abstime now)
{
	fixed_wing_lateral_guidance_status_s fixed_wing_lateral_guidance_status{};

	fixed_wing_lateral_guidance_status.timestamp = now;
	fixed_wing_lateral_guidance_status.course_setpoint = _directional_guidance.getCourseSetpoint();
	fixed_wing_lateral_guidance_status.lateral_acceleration_ff = _directional_guidance.getLateralAccelerationSetpoint();
	fixed_wing_lateral_guidance_status.bearing_feas = _directional_guidance.getBearingFeasibility();
	fixed_wing_lateral_guidance_status.bearing_feas_on_track = _directional_guidance.getBearingFeasibilityOnTrack();
	fixed_wing_lateral_guidance_status.signed_track_error = _directional_guidance.getSignedTrackError();
	fixed_wing_lateral_guidance_status.track_error_bound = _directional_guidance.getTrackErrorBound();
	fixed_wing_lateral_guidance_status.adapted_period = _directional_guidance.getAdaptedPeriod();
	fixed_wing_lateral_guidance_status.wind_est_valid = _wind_valid;

	_fixed_wing_lateral_guidance_status_pub.publish(fixed_wing_lateral_guidance_status);
}

int FixedWingModeManager::task_spawn(int argc, char *argv[])
{
	FixedWingModeManager *instance = new FixedWingModeManager();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FixedWingModeManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedWingModeManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the setpoint generation for all PX4-internal fixed-wing modes, height-rate control and higher.
It takes the current mode state of the vehicle as input and outputs setpoints consumed by the fixed-wing
lateral-longitudinal controller and and controllers below that (attitude, rate).

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_mode_manager", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_mode_manager_main(int argc, char *argv[])
{
	return FixedWingModeManager::main(argc, argv);
}


void
FixedWingModeManager::flareStart(FlareStates &fs, const hrt_abstime now, const float height)
{
	fs = FlareStates{};
	fs.flaring = true;
	fs.start_time = now;
	fs.initial_height_rate_setpoint = -_local_pos.vz;
	fs.start_height = height;
	fs.start_pitch = _pitch;
	fs.pitch_cmd = _pitch;
	vehicle_thrust_setpoint_s thrust_sp{};
	fs.throttle_start = (_vehicle_thrust_setpoint_sub.copy(&thrust_sp) && PX4_ISFINITE(thrust_sp.xyz[0]))
			    ? math::constrain(thrust_sp.xyz[0], _param_fw_thr_idle.get(), _param_fw_thr_max.get())
			    : _param_fw_lnd_fl_tff.get();
	fs.throttle_cmd = fs.throttle_start;
	fs.start_airspeed = _airspeed_eas;
	fs.last_update = now;
}

FixedWingModeManager::Law6Output
FixedWingModeManager::law6Step(FlareStates &fs, const hrt_abstime now, const float height, const Law6Profile &profile)
{
	// Flare law 6: pitch holds the sink rate (PI around the pitch at flare start, at most FW_LND_FL_PRT deg/s), the
	// throttle holds an airspeed falling with height to vtd_k x the estimated stall speed (PI around the flare start
	// throttle, at most FW_LND_FL_TSLW per s). Touchdown latched from the range finder and the sink rate. Runway: retard
	// (3 x slew) and runway pitch after touchdown. Water: the caller takes over at touchdown (scoop).
	Law6Output out{};
	const float dt = math::constrain(hrt_elapsed_time(&fs.last_update) * 1e-6f, 0.001f, 0.1f);
	fs.last_update = now;
	const float seconds_since_flare_start = hrt_elapsed_time(&fs.start_time) * 1e-6f;
	const float transition = math::constrain(seconds_since_flare_start / math::max(_param_fw_lnd_fl_tatt.get(), 0.1f),
				 0.0f, 1.0f);
	const float hold_height = _param_fw_lnd_fl_hold.get();
	const float sink_hold = math::max(_param_fw_lnd_fl_sink.get(), 0.0f);
	const float sink_upper = math::max(_param_fw_lnd_fl_snk1.get(), sink_hold);
	const float sink_target = (height > hold_height) ? sink_upper : sink_hold;
	const float sink_start = -fs.initial_height_rate_setpoint;
	const float sink_setpoint = transition * sink_target + (1.0f - transition) * sink_start;
	out.sink_setpoint = sink_setpoint;

	// pitch
	const float sink_error = _local_pos.vz - sink_setpoint; // > 0: sinking faster than wanted
	const float pitch_lo = radians(_param_fw_p_lim_min.get());
	const float pitch_hi = profile.pitch_max_rad;
	const float pitch_unsat = fs.start_pitch + radians(_param_fw_lnd_fl_kp.get()) * sink_error + fs.sink_rate_integral;
	float pitch = math::constrain(pitch_unsat, pitch_lo, pitch_hi);
	bool pitch_rate_up_held = false;
	bool pitch_rate_down_held = false;

	if (_param_fw_lnd_fl_prt.get() > FLT_EPSILON) {
		const float step = radians(_param_fw_lnd_fl_prt.get()) * dt;
		const float limited = fs.pitch_cmd + math::constrain(pitch - fs.pitch_cmd, -step, step);
		pitch_rate_up_held = pitch > limited + 1e-5f;
		pitch_rate_down_held = pitch < limited - 1e-5f;
		pitch = limited;
	}

	fs.pitch_cmd = pitch;

	if (!(((pitch_unsat >= pitch_hi || pitch_rate_up_held) && sink_error > 0.0f)
	      || ((pitch_unsat <= pitch_lo || pitch_rate_down_held) && sink_error < 0.0f))) {
		fs.sink_rate_integral += radians(_param_fw_lnd_fl_ki.get()) * sink_error * dt;
	}

	out.pitch_min = pitch_lo;
	out.pitch_max = pitch_hi;

	// airspeed target
	float speed_target;

	if (PX4_ISFINITE(_vs_est)) {
		const float speed_td = profile.vtd_k * _vs_est;
		const float speed_start = math::max(fs.start_airspeed, speed_td);
		const float span = math::max(fs.start_height - hold_height, 0.1f);
		const float fraction = math::constrain((height - hold_height) / span, 0.0f, 1.0f);
		speed_target = speed_td + (speed_start - speed_td) * fraction;

	} else {
		const float weight_ratio = weightRatio();

		speed_target = _param_fw_lnd_fl_vatt.get() * sqrtf(weight_ratio);
	}

	out.airspeed_setpoint = speed_target;

	// touchdown latch: range finder under 1.2 m and sink rate under 0.35 m/s for 0.5 s (the land detector needs a low
	// ground speed); without a range finder below the hold-off height
	if (!fs.touchdown) {
		const bool near_ground = _local_pos.dist_bottom_valid ? (_local_pos.dist_bottom < _param_fw_lnd_td_rng.get()) :
					 (height < hold_height);
		fs.touchdown_timer = (near_ground && _local_pos.vz < _param_fw_lnd_td_vz.get()) ? fs.touchdown_timer + dt : 0.0f;
		fs.touchdown = fs.touchdown_timer > _param_fw_lnd_td_t.get();

		// water: the hull sinks ~0.3 m and the radar (under the hull) goes under the water line. It then loses the
		// surface (invalid, below its minimum) or sees the sea bed (a jump of metres). Near the surface either one means
		// water contact (2026-11-11, first PX4 scoop: 0.53 m -> invalid -> 6.2 m sea bed)
		if (profile.water && PX4_ISFINITE(fs.last_range) && fs.last_range < _param_fw_lnd_td_rng.get() + 0.5f
		    && (!_local_pos.dist_bottom_valid || _local_pos.dist_bottom > fs.last_range + 1.5f)) {
			fs.touchdown = true;
		}
	}

	if (_local_pos.dist_bottom_valid && (!PX4_ISFINITE(fs.last_range) || _local_pos.dist_bottom < fs.last_range + 1.5f)) {
		fs.last_range = _local_pos.dist_bottom;
	}

	out.touched_down = _landed || fs.touchdown;

	// throttle
	const bool airspeed_ok = hrt_elapsed_time(&_time_airspeed_sample) < 1_s;
	const float speed_error = airspeed_ok ? (speed_target - _airspeed_eas) : 0.0f; // > 0: too slow
	const float thr_min = _param_fw_thr_idle.get();
	const float thr_max = _param_fw_thr_max.get();
	const bool retard = out.touched_down && !profile.water;
	const float thr_unsat = fs.throttle_start + _param_fw_lnd_fl_tkp.get() * speed_error + fs.throttle_integral;
	const float thr_target = retard ? thr_min : math::constrain(thr_unsat, thr_min, thr_max);
	const float slew = out.touched_down ? 3.0f * _param_fw_lnd_fl_tslw.get() : _param_fw_lnd_fl_tslw.get();
	const float thr_prev = fs.throttle_cmd;
	fs.throttle_cmd = (slew > FLT_EPSILON) ? thr_prev + math::constrain(thr_target - thr_prev, -slew * dt, slew * dt) : thr_target;
	const bool slew_limited = fabsf(fs.throttle_cmd - thr_target) > 1e-4f;
	const bool held_up = thr_unsat >= thr_max || (slew_limited && thr_target > fs.throttle_cmd);
	const bool held_down = thr_unsat <= thr_min || (slew_limited && thr_target < fs.throttle_cmd);

	if (!retard && !((held_up && speed_error > 0.0f) || (held_down && speed_error < 0.0f))) {
		fs.throttle_integral += _param_fw_lnd_fl_tki.get() * speed_error * dt;
	}

	out.throttle = fs.throttle_cmd;

	if (out.touched_down && !profile.water) {
		out.pitch = radians(_param_rwto_psp.get());
		out.pitch_min = math::min(out.pitch_min, out.pitch);

	} else {
		out.pitch = pitch;
	}

	return out;
}

const char *
FixedWingModeManager::scoopPhaseName(ScoopPhase phase)
{
	switch (phase) {
	case ScoopPhase::APPROACH: return "YAKLASMA";

	case ScoopPhase::GATE_PASSED: return "KAPI_GECTI";

	case ScoopPhase::FLARE: return "FLARE";

	case ScoopPhase::TOUCHDOWN: return "DEGME";

	case ScoopPhase::SCOOP: return "SUALMA";

	case ScoopPhase::TAKEOFF: return "KALKIS";

	case ScoopPhase::DONE: return "BITTI";

	case ScoopPhase::ABORT: return "IPTAL";

	default: return "KAPALI";
	}
}

void
FixedWingModeManager::setScoopPhase(const hrt_abstime now, ScoopPhase phase, const char *detail)
{
	_scoop.phase = phase;
	_scoop.phase_start = now;

	if (detail) {
		mavlink_log_info(&_mavlink_log_pub, "SU: %s %s\t", scoopPhaseName(phase), detail);

	} else {
		mavlink_log_info(&_mavlink_log_pub, "SU: %s\t", scoopPhaseName(phase));
	}

	publishScoopPhase(now);
}

void
FixedWingModeManager::publishScoopPhase(const hrt_abstime now)
{
	debug_key_value_s dbg{};
	strncpy(dbg.key, "SU_ASAMA", sizeof(dbg.key) - 1);
	dbg.value = static_cast<float>(_scoop.phase);
	dbg.timestamp = now;
	_debug_key_value_pub.publish(dbg);
	_scoop.last_status = now;
}

void
FixedWingModeManager::publishCommandAck(const vehicle_command_s &cmd, const uint8_t result)
{
	vehicle_command_ack_s ack{};
	ack.command = cmd.command;
	ack.result = result;
	ack.target_system = cmd.source_system;
	ack.target_component = cmd.source_component;
	ack.timestamp = hrt_absolute_time();
	_vehicle_command_ack_pub.publish(ack);
}

float
FixedWingModeManager::weightRatio() const
{
	if (_param_weight_base.get() > FLT_EPSILON && _param_weight_gross.get() > FLT_EPSILON) {
		return _param_weight_gross.get() / _param_weight_base.get();
	}

	return 1.0f;
}

void
FixedWingModeManager::scoopAbort(const hrt_abstime now, const char *reason)
{
	if (_scoop.phase == ScoopPhase::ABORT) {
		return;
	}

	setScoopPhase(now, ScoopPhase::ABORT, reason);
	_scoop.loiter_sent = false;
}

bool
FixedWingModeManager::scoopEngage(const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
	if (_scoop.phase != ScoopPhase::OFF) {
		return true;
	}

	if (!_scoop.armed || !PX4_ISFINITE(_scoop.water_amsl) || !pos_sp_curr.valid || !pos_sp_prev.valid
	    || pos_sp_curr.type != position_setpoint_s::SETPOINT_TYPE_POSITION
	    || fabsf(pos_sp_curr.alt - _scoop.water_amsl) > 2.f) {
		return false;
	}

	// the first water waypoint: freeze the approach line (previous waypoint -> touchdown point); the navigator may
	// move on to the second water waypoint and beyond while the scoop is flown
	_scoop.entrance = _global_local_proj_ref.project(pos_sp_prev.lat, pos_sp_prev.lon);
	_scoop.touchdown = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);
	_scoop.entrance_alt = pos_sp_prev.alt;
	_scoop.last_update = hrt_absolute_time();
	setScoopPhase(_scoop.last_update, ScoopPhase::APPROACH);
	return true;
}

void
FixedWingModeManager::control_auto_scoop(const hrt_abstime &now,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	const float dt = math::constrain((now - _scoop.last_update) * 1e-6f, 0.001f, 0.1f);
	_scoop.last_update = now;

	if (now - _scoop.last_status > 500_ms) {
		publishScoopPhase(now);
	}

	const Vector2f local_position{_local_pos.x, _local_pos.y};
	const Vector2f line = _scoop.touchdown - _scoop.entrance;
	const Vector2f dir = line.unit_or_zero();
	const Vector2f rel = local_position - _scoop.touchdown;
	const float along_to_touchdown = -dir.dot(rel);            // > 0 before the touchdown point
	const float cross = dir(0) * rel(1) - dir(1) * rel(0);     // > 0 right of the line
	// height over the water: the radar, unless it disagrees with the altitude over the water surface (item param7) by
	// more than 3 m once on the water (radar under the water line sees the sea bed) or is invalid
	const float height_baro = _current_altitude - _scoop.water_amsl;
	const bool radar = _local_pos.dist_bottom_valid
			   && (!_scoop.on_water || fabsf(_local_pos.dist_bottom - height_baro) < 3.f);
	const float height = radar ? _local_pos.dist_bottom : math::max(height_baro, 0.f);
	const float line_course = atan2f(dir(1), dir(0));
	const float ground_course = atan2f(ground_speed(1), ground_speed(0));
	const float course_error = math::degrees(fabsf(wrap_pi(ground_course - line_course)));
	const bool airspeed_ok = hrt_elapsed_time(&_time_airspeed_sample) < 1_s;

	// lateral: always the scoop line (beyond the touchdown point it continues straight)
	const DirectionalGuidanceOutput sp = navigateLine(_scoop.entrance, _scoop.touchdown, local_position, ground_speed,
					     _wind_vel);
	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = now;
	fw_lateral_ctrl_sp.course = sp.course_setpoint;
	fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

	if (_scoop.phase <= ScoopPhase::TAKEOFF) {
		_ctrl_configuration_handler.setLateralAccelMax(rollAngleToLateralAccel(getMaxRollAngleNearGround(_current_altitude,
				_scoop.water_amsl)));
	}

	fixed_wing_longitudinal_setpoint_s lon_sp{};
	lon_sp.timestamp = now;
	lon_sp.altitude = NAN;
	lon_sp.height_rate = NAN;
	lon_sp.equivalent_airspeed = NAN;
	lon_sp.pitch_direct = NAN;
	lon_sp.throttle_direct = NAN;

	switch (_scoop.phase) {
	case ScoopPhase::APPROACH:
	case ScoopPhase::GATE_PASSED: {
			// glide slope (FW_LND_ANG) to the touchdown point on the water, approach airspeed from the stall speed estimate
			_flaps_setpoint = _param_fw_flaps_lnd_scl.get();
			_ctrl_configuration_handler.setEnforceLowHeightCondition(true);
			const float vs_est = updateStallSpeedEstimate(now, height);
			float airspeed = _param_fw_lnd_airspd.get();

			if (PX4_ISFINITE(vs_est)) {
				airspeed = _param_fw_lnd_vapp_k.get() * vs_est;
				_ctrl_configuration_handler.setEquivalentAirspeedMin(1.05f * vs_est);
			}

			const float glide = tanf(radians(_param_fw_lnd_ang.get()));
			const float glide_alt = _scoop.water_amsl + math::min(math::max(along_to_touchdown, 0.f) * glide,
						_scoop.entrance_alt - _scoop.water_amsl);
			lon_sp.altitude = math::min(_current_altitude, glide_alt);
			lon_sp.equivalent_airspeed = airspeed;
			_ctrl_configuration_handler.setSinkRateTarget(math::max(airspeed * glide, _param_sinkrate_target.get()));
			_ctrl_configuration_handler.setThrottleMin(_param_fw_thr_idle.get());

			if (height < _param_fw_lnd_flalt.get()) {
				// gate (as hizalanma.gate_ok): on the line, wings level, at the approach airspeed
				char why[64] {};

				if (fabsf(cross) > _param_fw_sc_g_xtrk.get()) {
					snprintf(why, sizeof(why), "KAPI yan %.1f m", (double)cross);

				} else if (course_error > _param_fw_sc_g_crs.get()) {
					snprintf(why, sizeof(why), "KAPI yon %.1f deg", (double)course_error);

				} else if (fabsf(math::degrees(_roll)) > _param_fw_sc_g_roll.get()) {
					snprintf(why, sizeof(why), "KAPI yatis %.1f deg", (double)math::degrees(_roll));

				} else if (airspeed_ok && fabsf(_airspeed_eas - airspeed) > _param_fw_sc_g_spd.get()) {
					snprintf(why, sizeof(why), "KAPI hiz %.1f (hedef %.1f)", (double)_airspeed_eas, (double)airspeed);
				}

				if (why[0] != '\0') {
					scoopAbort(now, why);

				} else {
					setScoopPhase(now, ScoopPhase::GATE_PASSED);
					flareStart(_scoop.flare, now, height);
					setScoopPhase(now, ScoopPhase::FLARE);
				}
			}

			break;
		}

	case ScoopPhase::FLARE: {
			_flaps_setpoint = _param_fw_flaps_lnd_scl.get();
			const Law6Profile water_profile{_param_fw_sc_vtd_k.get(), radians(_param_fw_sc_pmax.get()), true};
			const Law6Output out = law6Step(_scoop.flare, now, height, water_profile);
			lon_sp.height_rate = -out.sink_setpoint;
			lon_sp.pitch_direct = out.pitch;
			lon_sp.throttle_direct = out.throttle;
			lon_sp.equivalent_airspeed = out.airspeed_setpoint;
			_ctrl_configuration_handler.setPitchMin(out.pitch_min);
			_ctrl_configuration_handler.setPitchMax(out.pitch_max);

			if (PX4_ISFINITE(_vs_est)) {
				_ctrl_configuration_handler.setEquivalentAirspeedMin(1.05f * _vs_est);
			}

			// own go-around checks above 1 m (below it the touchdown comes first)
			if (height > _param_fw_sc_a_hmin.get()) {
				char why[64] {};

				if (airspeed_ok && PX4_ISFINITE(_vs_est) && _airspeed_eas < _vs_est) {
					snprintf(why, sizeof(why), "HIZ_DUSUK %.1f < %.1f", (double)_airspeed_eas, (double)_vs_est);

				} else if (fabsf(math::degrees(_roll)) > _param_fw_sc_a_roll.get()) {
					snprintf(why, sizeof(why), "YATIS %.1f deg", (double)math::degrees(_roll));

				} else if (fabsf(cross) > _param_fw_sc_a_xtrk.get()) {
					snprintf(why, sizeof(why), "YAN_SAPMA %.1f m", (double)cross);

				} else if (!radar && height > _param_fw_lnd_td_rng.get() + 1.f && height < _param_fw_lnd_flalt.get()) {
					snprintf(why, sizeof(why), "RADAR");
				}

				if (why[0] != '\0') {
					scoopAbort(now, why);
					break;
				}
			}

			if (out.touched_down) {
				_scoop.on_water = true;
				char detail[96] {};
				snprintf(detail, sizeof(detail), "v=%.1f p=%.1f vz=%.2f", (double)_airspeed_eas, (double)math::degrees(_pitch),
					 (double)_local_pos.vz);
				setScoopPhase(now, ScoopPhase::TOUCHDOWN, detail);
				_scoop.pitch_cmd = _scoop.flare.pitch_cmd;
				_scoop.throttle_cmd = _scoop.flare.throttle_cmd;
				_scoop.throttle_integral = 0.f;
				setScoopPhase(now, ScoopPhase::SCOOP);
			}

			break;
		}

	case ScoopPhase::TOUCHDOWN:
	case ScoopPhase::SCOOP: {
			// on the water with power: pitch to FW_SC_PITCH, throttle holds the scoop airspeed (PI around the touchdown
			// throttle, 3 x FW_LND_FL_TSLW); the tank fills while the airspeed is in the band
			_flaps_setpoint = _param_fw_flaps_lnd_scl.get();

			// predicted airspeed (FW_SC_LOOK s ahead): the throttle opens while still decelerating towards the band
			// (7b, 1200 kg: gaz ~0 until under 25 m/s, then the airspeed fell to 21 m/s)
			if (airspeed_ok) {
				const float accel = (_scoop.last_airspeed > 0.f) ? (_airspeed_eas - _scoop.last_airspeed) / dt : 0.f;
				_scoop.accel_filt += (math::constrain(accel, -10.f, 10.f) - _scoop.accel_filt) * math::min(dt / 0.5f, 1.f);
				_scoop.last_airspeed = _airspeed_eas;
			}

			const float predicted = _airspeed_eas + _scoop.accel_filt * _param_fw_sc_look.get();
			const float speed_error = airspeed_ok ? (_scoop.target_speed - predicted) : 0.f;
			const float thr_min = _param_fw_thr_idle.get();
			const float thr_max = _param_fw_thr_max.get();

			// feedforward (water_leg: sqrt(base^2 + scoop momentum drag / max thrust)), the base scaled with the weight
			// from the stall speed estimate (FW_SC_THR_FF at FW_SC_FF_VS)
			const float weight_scale = (PX4_ISFINITE(_vs_est) && _param_fw_sc_ff_vs.get() > FLT_EPSILON) ?
						   math::sq(_vs_est / _param_fw_sc_ff_vs.get()) : 1.f;
			const float filling = (_scoop.scoop_started && _scoop.scoop_time < _scoop.duration) ? 1.f : 0.f;
			const float thr_ff = sqrtf(math::sq(_param_fw_sc_thr_ff.get()) * weight_scale
						   + filling * _param_fw_sc_fill.get() * math::max(_airspeed_eas, 0.f) / math::max(_param_fw_sc_tmax.get(), 1.f));

			// thrust line above the CG: more throttle pitches the nose down; pitch feedforward with the throttle
			const float pitch_target = radians(_param_fw_sc_pitch.get() + _param_fw_sc_p_thr.get() * (_scoop.throttle_cmd - thr_ff));
			const float step = radians(math::max(_param_fw_lnd_fl_prt.get(), 0.5f)) * dt;
			_scoop.pitch_cmd += math::constrain(pitch_target - _scoop.pitch_cmd, -step, step);
			const float thr_unsat = thr_ff + _param_fw_sc_tkp.get() * speed_error + _scoop.throttle_integral;
			// low speed guard: predicted airspeed within FW_SC_V_MARG of the band floor -> full throttle (PX4's airspeed
			// reads ~0.2 m/s under the simulator's, 7b round 5)
			const bool low_guard = airspeed_ok && predicted < _scoop.speed_min + _param_fw_sc_v_marg.get();
			const float thr_target = low_guard ? thr_max : math::constrain(thr_unsat, thr_min, thr_max);
			const float slew = 3.f * _param_fw_lnd_fl_tslw.get() * dt;
			const float prev = _scoop.throttle_cmd;
			_scoop.throttle_cmd = prev + math::constrain(thr_target - prev, -slew, slew);
			const bool slew_limited = fabsf(_scoop.throttle_cmd - thr_target) > 1e-4f;
			const bool held_up = thr_unsat >= thr_max || (slew_limited && thr_target > _scoop.throttle_cmd);
			const bool held_down = thr_unsat <= thr_min || (slew_limited && thr_target < _scoop.throttle_cmd);

			if (!((held_up && speed_error > 0.f) || (held_down && speed_error < 0.f))) {
				_scoop.throttle_integral += _param_fw_sc_tki.get() * speed_error * dt;
			}

			lon_sp.pitch_direct = _scoop.pitch_cmd;
			lon_sp.throttle_direct = _scoop.throttle_cmd;
			lon_sp.height_rate = 0.f;
			lon_sp.equivalent_airspeed = _scoop.target_speed;
			_ctrl_configuration_handler.setPitchMin(math::min(_scoop.pitch_cmd, radians(_param_fw_sc_pitch.get())) - radians(2.f));
			_ctrl_configuration_handler.setPitchMax(math::max(_scoop.pitch_cmd, radians(_param_fw_sc_pitch.get())) + radians(2.f));
			_ctrl_configuration_handler.setEquivalentAirspeedMin(10.f);

			const bool in_band = airspeed_ok && _airspeed_eas >= _scoop.speed_min && _airspeed_eas <= _param_fw_sc_vmax.get();

			// control band (item param4 .. FW_SC_VMAX): the throttle aims inside it, the tank fills in it.
			// Abort band (FW_SC_A_VMIN .. FW_SC_A_VMAX, wider): only leaving it longer than FW_SC_OOB_T aborts (user)
			if (in_band) {
				_scoop.scoop_started = true;
				_scoop.scoop_time += dt;

			} else if (_scoop.scoop_started) {
				_scoop.out_of_band += dt;
			}

			if (_scoop.scoop_started && airspeed_ok
			    && (_airspeed_eas < _param_fw_sc_a_vmin.get() || _airspeed_eas > _param_fw_sc_a_vmax.get())) {
				_scoop.out_of_abort_band += dt;
			}

			if (_scoop.scoop_started && airspeed_ok) {
				_scoop.speed_lo = math::min(_scoop.speed_lo, _airspeed_eas);
				_scoop.speed_hi = math::max(_scoop.speed_hi, _airspeed_eas);
			}

			const float in_phase = (now - _scoop.phase_start) * 1e-6f;

			if (_scoop.scoop_time >= _scoop.duration) {
				char detail[96] {};
				snprintf(detail, sizeof(detail), "t=%.1f v=%.1f-%.1f d=%.1f a=%.1f k=%d", (double)_scoop.scoop_time,
					 (double)_scoop.speed_lo, (double)_scoop.speed_hi, (double)_scoop.out_of_band, (double)_scoop.out_of_abort_band,
					 _scoop.out_of_abort_band < 0.05f ? 1 : 0);
				setScoopPhase(now, ScoopPhase::TAKEOFF, detail);

			} else if (_scoop.out_of_abort_band > _param_fw_sc_oob_t.get() || in_phase > _scoop.duration + _param_fw_sc_t_marg.get()) {
				char why[96] {};
				snprintf(why, sizeof(why), "BANT t=%.1f v=%.1f-%.1f a=%.1f", (double)_scoop.scoop_time,
					 (double)_scoop.speed_lo, (double)_scoop.speed_hi, (double)_scoop.out_of_abort_band);
				scoopAbort(now, why);
			}

			break;
		}

	case ScoopPhase::TAKEOFF: {
			// on the water: throttle up (0.5 /s), pitch to FW_SC_TO_PITCH with the same throttle feedforward as the scoop
			// (thrust line above the CG). Off the water (radar/altitude over FW_SC_LIFT_H): TECS climb at the trim airspeed
			// to FW_SC_DONE_H, so there is no handover step at the end (7b round 3: 64 m/s, then +15 deg at the handover).
			_flaps_setpoint = _param_fw_flaps_to_scl.get();
			_ctrl_configuration_handler.setEquivalentAirspeedMin(10.f);

			if (height < _param_fw_sc_lift_h.get()) {
				const float weight_scale = (PX4_ISFINITE(_vs_est) && _param_fw_sc_ff_vs.get() > FLT_EPSILON) ?
							   math::sq(_vs_est / _param_fw_sc_ff_vs.get()) : 1.f;
				const float thr_ff = _param_fw_sc_thr_ff.get() * sqrtf(weight_scale);
				_scoop.throttle_cmd = math::min(_scoop.throttle_cmd + 0.5f * dt, _param_fw_thr_max.get());
				const float pitch_target = radians(_param_fw_sc_to_pitch.get()
								   + _param_fw_sc_p_thr.get() * math::max(_scoop.throttle_cmd - thr_ff, 0.f));
				const float step = radians(math::max(_param_fw_lnd_fl_prt.get(), 0.5f)) * dt;
				_scoop.pitch_cmd += math::constrain(pitch_target - _scoop.pitch_cmd, -step, step);
				lon_sp.pitch_direct = _scoop.pitch_cmd;
				lon_sp.throttle_direct = _scoop.throttle_cmd;
				lon_sp.height_rate = 1.f;
				_ctrl_configuration_handler.setPitchMax(radians(_param_fw_p_lim_max.get()));

			} else {
				// airborne: TECS climb; the airspeed setpoint starts at the liftoff airspeed and comes down at
				// FW_SC_CLB_DEC m/s^2, so the excess speed is not turned into a zoom (7b round 4: +14 deg, throttle 6 %)
				if (_scoop.liftoff_airspeed <= 0.f) {
					_scoop.liftoff_airspeed = _airspeed_eas;
					_scoop.liftoff_time = now;
				}

				const float since = (now - _scoop.liftoff_time) * 1e-6f;
				const float trim = _param_fw_airspd_trim.get();
				lon_sp.altitude = _scoop.water_amsl + _param_fw_sc_done_h.get() + 10.f;
				lon_sp.equivalent_airspeed = math::max(trim, _scoop.liftoff_airspeed - _param_fw_sc_clb_dec.get() * since);
				_ctrl_configuration_handler.setPitchMax(radians(_param_fw_sc_to_pitch.get() + 2.f));
			}

			if (height > _param_fw_sc_done_h.get()) {
				char detail[96] {};
				snprintf(detail, sizeof(detail), "t=%.1f v=%.1f-%.1f d=%.1f a=%.1f k=%d", (double)_scoop.scoop_time,
					 (double)_scoop.speed_lo, (double)_scoop.speed_hi, (double)_scoop.out_of_band, (double)_scoop.out_of_abort_band,
					 _scoop.out_of_abort_band < 0.05f ? 1 : 0);
				setScoopPhase(now, ScoopPhase::DONE, detail);
			}

			break;
		}

	case ScoopPhase::DONE: {
			// TECS already flies (climb at trim): stay on the line climbing while the navigator is still on a water
			// waypoint, then hand back to the mission
			if (pos_sp_curr.valid && fabsf(pos_sp_curr.alt - _scoop.water_amsl) < 2.f) {
				lon_sp.altitude = math::max(_current_altitude, _scoop.water_amsl + _param_fw_sc_clb_h.get());
				_ctrl_configuration_handler.setPitchMax(radians(_param_fw_sc_to_pitch.get() + 2.f));

			} else {
				_scoop.phase = ScoopPhase::OFF;
				_scoop.armed = false;
				return;
			}

			break;
		}

	case ScoopPhase::ABORT: {
			// go-around: wings level on the line, full power climb to FW_SC_GA_ALT over the water, then AUTO.LOITER
			// (the planner decides what comes next). On the water: takeoff pitch first.
			const float ga_alt = _scoop.water_amsl + _param_fw_sc_ga_alt.get();

			if (height < _param_fw_sc_ga_h.get()) {
				_flaps_setpoint = _param_fw_flaps_to_scl.get();
				const float step = radians(math::max(_param_fw_lnd_fl_prt.get(), 0.5f)) * dt * 2.f;
				_scoop.pitch_cmd += math::constrain(radians(_param_fw_sc_to_pitch.get() + 2.f) - _scoop.pitch_cmd, -step, step);
				_scoop.throttle_cmd = math::min(math::max(_scoop.throttle_cmd, 0.5f) + 0.5f * dt, _param_fw_thr_max.get());
				lon_sp.pitch_direct = _scoop.pitch_cmd;
				lon_sp.throttle_direct = _scoop.throttle_cmd;
				lon_sp.height_rate = 2.f;
				_ctrl_configuration_handler.setPitchMax(radians(_param_fw_p_lim_max.get()));

			} else {
				lon_sp.altitude = ga_alt;
				_scoop.pitch_cmd = _pitch;
				_scoop.throttle_cmd = _param_fw_thr_max.get();
			}

			_ctrl_configuration_handler.setEquivalentAirspeedMin(10.f);

			if (_current_altitude > ga_alt - 5.f && !_scoop.loiter_sent) {
				vehicle_command_s cmd{};
				cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
				cmd.param1 = 1.f;   // custom mode
				cmd.param2 = 4.f;   // PX4_CUSTOM_MAIN_MODE_AUTO
				cmd.param3 = 3.f;   // PX4_CUSTOM_SUB_MODE_AUTO_LOITER
				cmd.target_system = _vehicle_status.system_id;
				cmd.target_component = _vehicle_status.component_id;
				cmd.source_system = _vehicle_status.system_id;
				cmd.source_component = _vehicle_status.component_id;
				cmd.from_external = false;
				cmd.timestamp = now;
				_vehicle_command_pub.publish(cmd);
				_scoop.loiter_sent = true;
				mavlink_log_info(&_mavlink_log_pub, "SU: IPTAL tamam, bekleme (AUTO.LOITER)\t");
				_scoop.phase = ScoopPhase::OFF;
				_scoop.armed = false;
			}

			break;
		}

	default:
		return;
	}

	_longitudinal_ctrl_sp_pub.publish(lon_sp);
	_ctrl_configuration_handler.setThrottleMax(_param_fw_thr_max.get());
}
