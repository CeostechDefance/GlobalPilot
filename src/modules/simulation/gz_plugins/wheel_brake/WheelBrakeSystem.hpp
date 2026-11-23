/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

#include <atomic>
#include <limits>
#include <string>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace custom
{
/**
 * Equal wheel brake torque on the two main wheels during the landing roll.
 *
 * Brakes work only with the gear down and locked, the engine command low and the main
 * wheels turning, so they are off in the air, on the water and during the takeoff run.
 * An airborne wheel is not turning, so the brakes cannot lock it before touchdown. The
 * torque starts low at touchdown and reaches full value once the nose wheel turns too.
 * It fades out at low speed, so the brakes slow the wheels but never hold them still.
 * When a main wheel falls behind the ground speed, because it skids or a bounce has lifted it,
 * both wheels are released together until they have caught up.
 *
 * Topics under /model/<name>/brake:
 *   enable (gz.msgs.Double): above 0.5 enables the brakes, otherwise releases them.
 *   state  (gz.msgs.Double_V, 10 Hz): torque, left, right and nose wheel speed, motor command, brake level, engaged,
 *                                     reference wheel speed, skidding.
 */
class WheelBrakeSystem:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &event_manager) final;

	void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) final;

private:
	struct Joint {
		std::string name;
		gz::sim::Entity entity{gz::sim::kNullEntity};
	};

	bool resolve(Joint &joint, gz::sim::EntityComponentManager &ecm);
	double speed(const Joint &joint, const gz::sim::EntityComponentManager &ecm) const;
	double position(const Joint &joint, const gz::sim::EntityComponentManager &ecm) const;
	void reset(gz::sim::EntityComponentManager &ecm);

	gz::sim::Entity _model{gz::sim::kNullEntity};
	gz::transport::Node _node;
	gz::transport::Node::Publisher _state_pub;

	Joint _left_wheel, _right_wheel, _nose_wheel, _left_gear, _right_gear;

	double _max_torque{0.};     // N m per wheel
	double _touch_fraction{0.}; // part of the torque allowed until the nose wheel is down
	double _ramp_s{1.};         // time for the brake level to go from 0 to 1
	double _spin_rad_s{1.};     // a wheel above this speed is rolling on the ground
	double _fade_rad_s{1.};     // the torque fades to zero below this wheel speed
	double _confirm_s{0.2};     // a main wheel must roll this long to count
	double _nose_confirm_s{1.}; // the nose wheel must roll this long to count
	double _motor_off{0.};      // the brakes release above this motor command
	double _wheel_radius{1.};   // m
	double _slip_release{0.15}; // the brakes release when a wheel falls this far below the ground speed
	double _slip_resume{0.05};  // and come back when both wheels are this close to it
	double _release_s{0.05};    // time for the brake level to go from 1 to 0 while releasing

	std::atomic<bool> _enabled{true};
	std::atomic<bool> _gear_up_commanded{false};
	// No motor command seen means the engine state is unknown, and that must not apply the brakes.
	std::atomic<double> _motor_command{std::numeric_limits<double>::infinity()};

	bool _joints_resolved{false};
	bool _engaged{false};
	bool _nose_down{false};
	bool _skidding{false};
	gz::sim::Entity _body{gz::sim::kNullEntity};
	double _main_rolling_s{0.};
	double _nose_rolling_s{0.};
	double _level{0.};
	double _next_state_s{0.};
};
} // end namespace custom
