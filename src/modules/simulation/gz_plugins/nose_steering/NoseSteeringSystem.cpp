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


#include "NoseSteeringSystem.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <gz/common/Console.hh>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>

using namespace custom;

GZ_ADD_PLUGIN(
	NoseSteeringSystem,
	gz::sim::System,
	NoseSteeringSystem::ISystemConfigure,
	NoseSteeringSystem::ISystemPreUpdate
)

namespace
{
constexpr double GEAR_DOWN_RAD = 0.01;
constexpr double STATE_PERIOD_S = 0.1;
}

void NoseSteeringSystem::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
				   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &)
{
	_model = entity;
	const std::string name = gz::sim::Model(entity).Name(ecm);

	_steer_name = sdf->Get<std::string>("steer_joint");
	_wheel_name = sdf->Get<std::string>("wheel_joint");
	_gear_name = sdf->Get<std::string>("gear_joint");
	_full_angle = sdf->Get<double>("full_angle");
	_wheelbase = sdf->Get<double>("wheelbase");
	_lateral_accel = sdf->Get<double>("lateral_accel");
	_rate = sdf->Get<double>("rate");
	_spin = sdf->Get<double>("spin_rad_s");
	_standing = sdf->Get<double>("standing_m_s");
	_gain = sdf->Get<double>("gain");
	_sign = sdf->Get<double>("sign");
	_motor_off = sdf->Get<double>("motor_off");
	// Defaults for a model.sdf written before these existed.
	_fade_start = sdf->Get<double>("steer_fade_start_m_s", 10.).first;
	_cutoff = sdf->Get<double>("steer_cutoff_m_s", 15.).first;
	_liftoff_height = sdf->Get<double>("liftoff_height_m", 10.).first;

	const std::function<void(const gz::msgs::Double &)> on_command = [this](const gz::msgs::Double & msg) {
		_command = msg.data();
	};
	const std::function<void(const gz::msgs::Double &)> on_gear = [this](const gz::msgs::Double & msg) {
		_gear_up_commanded = msg.data() > 0.1; // the same reading as GearSequencerSystem
	};

	const std::function<void(const gz::msgs::Actuators &)> on_motor = [this](const gz::msgs::Actuators & msg) {
		if (msg.velocity_size() > 0) {
			_motor_command = msg.velocity(0);
		}
	};

	if (!_node.Subscribe("/" + name + "/" + sdf->Get<std::string>("motor_topic"), on_motor)
	    || !_node.Subscribe("/model/" + name + "/" + sdf->Get<std::string>("command_topic"), on_command)
	    || !_node.Subscribe("/model/" + name + "/" + sdf->Get<std::string>("gear_topic"), on_gear)) {
		gzerr << "NoseSteeringSystem: failed to subscribe" << std::endl;
	}

	_state_pub = _node.Advertise<gz::msgs::Double_V>("/model/" + name + "/steering/state");
}

gz::sim::Entity NoseSteeringSystem::jointEntity(const std::string &name, gz::sim::EntityComponentManager &ecm)
{
	const gz::sim::Entity joint = gz::sim::Model(_model).JointByName(ecm, name);

	if (joint == gz::sim::kNullEntity) {
		gzerr << "NoseSteeringSystem: joint " << name << " not found" << std::endl;
		return joint;
	}

	ecm.CreateComponent(joint, gz::sim::components::JointPosition());
	ecm.CreateComponent(joint, gz::sim::components::JointVelocity());
	return joint;
}

void NoseSteeringSystem::PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm)
{
	if (!_resolved) {
		_steer = jointEntity(_steer_name, ecm);
		_wheel = jointEntity(_wheel_name, ecm);
		_gear = jointEntity(_gear_name, ecm);
		_resolved = _steer != gz::sim::kNullEntity && _wheel != gz::sim::kNullEntity && _gear != gz::sim::kNullEntity;

		if (!_resolved) {
			return;
		}

		ecm.CreateComponent(_steer, gz::sim::components::JointVelocityCmd({0.}));
		_body = gz::sim::Model(_model).CanonicalLink(ecm);
		gz::sim::Link(_body).EnableVelocityChecks(ecm);
	}

	if (info.paused) {
		return;
	}

	const auto value = [&ecm](gz::sim::Entity joint) {
		const auto *position = ecm.Component<gz::sim::components::JointPosition>(joint);
		return position && !position->Data().empty() ? position->Data()[0] : 0.;
	};
	const auto *wheel_velocity = ecm.Component<gz::sim::components::JointVelocity>(_wheel);
	const double wheel_speed = wheel_velocity && !wheel_velocity->Data().empty() ? std::abs(wheel_velocity->Data()[0]) : 0.;
	const auto *body_velocity = ecm.Component<gz::sim::components::WorldLinearVelocity>(_body);
	const double speed = body_velocity ? std::hypot(body_velocity->Data().X(), body_velocity->Data().Y()) : 0.;

	const bool gear_down = !_gear_up_commanded && std::abs(value(_gear)) < GEAR_DOWN_RAD;
	const bool on_ground = wheel_speed > _spin || speed < _standing;
	const double motor = _motor_command;

	// Flown: the gear up command, or the body this far above where it started. The height is the check that always
	// works: PX4 does not always send the gear up command, and a nose wheel keeps spinning for a while after liftoff.
	// Standing still again with the engine back at idle arms the lock for the next takeoff run.
	const double height = gz::sim::worldPose(_body, ecm).Pos().Z();

	if (!std::isfinite(_start_height)) {
		_start_height = height;
	}

	if (_gear_up_commanded || height > _start_height + _liftoff_height) {
		_flown = true;

	} else if (_flown && speed < _standing && motor <= _motor_off) {
		_flown = false;
		_start_height = height;
	}

	const bool active = _flown && gear_down && on_ground && motor <= _motor_off;

	// The turn this angle gives at this speed stays below the allowed lateral acceleration, and above the
	// cutoff speed the wheel stays straight, with a linear fade in between.
	const double turn_limit = speed > 0.1 ? std::min(_full_angle, std::atan(_wheelbase * _lateral_accel / (speed * speed))) : _full_angle;
	const double fade = std::clamp((_cutoff - speed) / std::max(_cutoff - _fade_start, 1e-3), 0., 1.);
	const double limit = turn_limit * fade;
	const double command = std::clamp(double(_command), -_full_angle, _full_angle) / _full_angle;
	const double target = active ? _sign * command * limit : 0.;

	const double angle = value(_steer);
	ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(_steer, {std::clamp(_gain * (target - angle), -_rate, _rate)});

	const double now = std::chrono::duration<double>(info.simTime).count();

	if (now >= _next_state_s) {
		_next_state_s = now + STATE_PERIOD_S;
		gz::msgs::Double_V state;

		for (const double item : {command, target, angle, limit, speed, wheel_speed, active ? 1. : 0., motor, _flown ? 1. : 0.}) {
			state.add_data(item);
		}

		_state_pub.Publish(state);
	}
}
