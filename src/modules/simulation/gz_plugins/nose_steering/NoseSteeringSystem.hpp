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
 * Turns the nose wheel from the PX4 Landing_Gear_Wheel servo output.
 *
 * It follows the command only with the engine command low, the gear down and locked and the nose wheel
 * rolling or the aircraft standing still, so it stays centred during the takeoff run, in the air and while
 * the gear moves. The angle is limited by the ground speed, so a full command at speed gives a small
 * correction and not a sharp turn.
 */
class NoseSteeringSystem:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &event_manager) final;

	void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) final;

private:
	gz::sim::Entity jointEntity(const std::string &name, gz::sim::EntityComponentManager &ecm);

	gz::sim::Entity _model{gz::sim::kNullEntity};
	gz::sim::Entity _body{gz::sim::kNullEntity};
	gz::sim::Entity _steer{gz::sim::kNullEntity};
	gz::sim::Entity _wheel{gz::sim::kNullEntity};
	gz::sim::Entity _gear{gz::sim::kNullEntity};
	gz::transport::Node _node;
	gz::transport::Node::Publisher _state_pub;

	std::string _steer_name, _wheel_name, _gear_name;
	double _full_angle{0.};   // rad, joint limit and the servo output at full command
	double _wheelbase{1.};    // m
	double _lateral_accel{1.}; // m/s2 allowed by the speed limit
	double _rate{1.};         // rad/s
	double _spin{1.};         // rad/s: a nose wheel above this is rolling on the ground
	double _standing{1.};     // m/s: below this the aircraft is on the ground, not rolling
	double _gain{1.};         // 1/s
	double _motor_off{0.};    // the wheel centres above this motor command
	double _fade_start{10.};  // m/s: the angle limit starts to fall to zero here
	double _cutoff{15.};      // m/s: and is zero from here on
	double _liftoff_height{10.}; // m above the start height that counts as having flown
	double _start_height{std::numeric_limits<double>::quiet_NaN()};
	bool _flown{false};       // the wheel stays centred on the takeoff run, until the aircraft has flown
	double _sign{1.};
	double _next_state_s{0.};
	bool _resolved{false};

	std::atomic<double> _command{0.};
	std::atomic<bool> _gear_up_commanded{false};
	// No motor command seen means the engine state is unknown, and that must not steer.
	std::atomic<double> _motor_command{std::numeric_limits<double>::infinity()};
};
} // end namespace custom
