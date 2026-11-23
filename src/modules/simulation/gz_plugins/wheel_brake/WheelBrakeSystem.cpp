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

#include "WheelBrakeSystem.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <gz/common/Console.hh>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>

using namespace custom;

GZ_ADD_PLUGIN(
	WheelBrakeSystem,
	gz::sim::System,
	WheelBrakeSystem::ISystemConfigure,
	WheelBrakeSystem::ISystemPreUpdate
)

namespace
{
constexpr double GEAR_DOWN_RAD = 0.01;
constexpr double STATE_PERIOD_S = 0.1;
}

void WheelBrakeSystem::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
				 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &)
{
	_model = entity;
	const std::string name = gz::sim::Model(entity).Name(ecm);

	_left_wheel.name = sdf->Get<std::string>("left_wheel_joint");
	_right_wheel.name = sdf->Get<std::string>("right_wheel_joint");
	_nose_wheel.name = sdf->Get<std::string>("nose_wheel_joint");
	_left_gear.name = sdf->Get<std::string>("left_gear_joint");
	_right_gear.name = sdf->Get<std::string>("right_gear_joint");

	_max_torque = sdf->Get<double>("max_torque");
	_touch_fraction = sdf->Get<double>("touch_fraction");
	_ramp_s = sdf->Get<double>("ramp_s");
	_spin_rad_s = sdf->Get<double>("spin_rad_s");
	_fade_rad_s = sdf->Get<double>("fade_rad_s");
	_confirm_s = sdf->Get<double>("confirm_s");
	_nose_confirm_s = sdf->Get<double>("nose_confirm_s");
	_motor_off = sdf->Get<double>("motor_off");
	_wheel_radius = sdf->Get<double>("wheel_radius");
	_slip_release = sdf->Get<double>("slip_release");
	_slip_resume = sdf->Get<double>("slip_resume");
	_release_s = sdf->Get<double>("release_s");
	_enabled = sdf->Get<bool>("enabled", true).first;

	const std::function<void(const gz::msgs::Double &)> on_enable = [this](const gz::msgs::Double & msg) {
		_enabled = msg.data() > 0.5;
	};
	const std::function<void(const gz::msgs::Double &)> on_gear = [this](const gz::msgs::Double & msg) {
		// The same reading as GearSequencerSystem: only a command above zero is gear up.
		_gear_up_commanded = msg.data() > 0.1;
	};
	const std::function<void(const gz::msgs::Actuators &)> on_motor = [this](const gz::msgs::Actuators & msg) {
		if (msg.velocity_size() > 0) {
			_motor_command = msg.velocity(0);
		}
	};

	const std::string motor_topic = "/" + name + "/" + sdf->Get<std::string>("motor_topic");
	const std::string gear_topic = "/model/" + name + "/" + sdf->Get<std::string>("gear_topic");

	if (!_node.Subscribe("/model/" + name + "/brake/enable", on_enable) || !_node.Subscribe(gear_topic, on_gear)
	    || !_node.Subscribe(motor_topic, on_motor)) {
		gzerr << "WheelBrakeSystem: failed to subscribe" << std::endl;
	}

	_state_pub = _node.Advertise<gz::msgs::Double_V>("/model/" + name + "/brake/state");
}

bool WheelBrakeSystem::resolve(Joint &joint, gz::sim::EntityComponentManager &ecm)
{
	joint.entity = gz::sim::Model(_model).JointByName(ecm, joint.name);

	if (joint.entity == gz::sim::kNullEntity) {
		gzerr << "WheelBrakeSystem: joint " << joint.name << " not found" << std::endl;
		return false;
	}

	ecm.CreateComponent(joint.entity, gz::sim::components::JointPosition());
	ecm.CreateComponent(joint.entity, gz::sim::components::JointVelocity());
	return true;
}

double WheelBrakeSystem::speed(const Joint &joint, const gz::sim::EntityComponentManager &ecm) const
{
	const auto *velocity = ecm.Component<gz::sim::components::JointVelocity>(joint.entity);
	return velocity && !velocity->Data().empty() ? velocity->Data()[0] : 0.;
}

double WheelBrakeSystem::position(const Joint &joint, const gz::sim::EntityComponentManager &ecm) const
{
	const auto *value = ecm.Component<gz::sim::components::JointPosition>(joint.entity);
	return value && !value->Data().empty() ? value->Data()[0] : 0.;
}

void WheelBrakeSystem::reset(gz::sim::EntityComponentManager &)
{
	_engaged = false;
	_nose_down = false;
	_main_rolling_s = 0.;
	_nose_rolling_s = 0.;
	_level = 0.;
	_skidding = false;
}

void WheelBrakeSystem::PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm)
{
	if (!_joints_resolved) {
		_joints_resolved = true;

		for (auto *joint : {&_left_wheel, &_right_wheel, &_nose_wheel, &_left_gear, &_right_gear}) {
			_joints_resolved = resolve(*joint, ecm) && _joints_resolved;
		}

		if (!_joints_resolved) {
			return;
		}

		// Wheels only. Gazebo ignores a joint's velocity command while it has a force command, even a zero
		// one, so a force command on a gear joint would stop GearSequencerSystem holding the gear in place.
		ecm.CreateComponent(_left_wheel.entity, gz::sim::components::JointForceCmd({0.}));
		ecm.CreateComponent(_right_wheel.entity, gz::sim::components::JointForceCmd({0.}));

		_body = gz::sim::Model(_model).CanonicalLink(ecm);
		gz::sim::Link(_body).EnableVelocityChecks(ecm);
	}

	if (info.paused) {
		return;
	}

	const double dt = std::chrono::duration<double>(info.dt).count();
	const double left = speed(_left_wheel, ecm);
	const double right = speed(_right_wheel, ecm);
	const double nose = speed(_nose_wheel, ecm);
	const double main_speed = (std::abs(left) + std::abs(right)) / 2.;
	const double motor = _motor_command;

	const bool gear_down = !_gear_up_commanded && std::abs(position(_left_gear, ecm)) < GEAR_DOWN_RAD
			       && std::abs(position(_right_gear, ecm)) < GEAR_DOWN_RAD;

	if (!_enabled || !gear_down || motor > _motor_off) {
		reset(ecm);

	} else {
		_main_rolling_s = main_speed > _spin_rad_s ? _main_rolling_s + dt : 0.;
		_nose_rolling_s = std::abs(nose) > _spin_rad_s ? _nose_rolling_s + dt : 0.;
		// Both stay set once reached: the end of the roll fades the torque out through the wheel speed.
		_engaged = _engaged || _main_rolling_s >= _confirm_s;
		// Longer than the main wheels: a bounce right after touchdown also spins the nose wheel for a moment.
		_nose_down = _nose_down || _nose_rolling_s >= _nose_confirm_s;
	}

	// The wheel speed the ground speed calls for. A main wheel well below it is skidding or, after a bounce, in the
	// air. The nose wheel has no brake and the body speed needs no contact, so together they hold in both cases.
	const auto *velocity = ecm.Component<gz::sim::components::WorldLinearVelocity>(_body);
	const double ground_speed = velocity ? std::hypot(velocity->Data().X(), velocity->Data().Y()) : 0.;
	const double reference = std::max(std::abs(nose), ground_speed / _wheel_radius);
	const double slower_wheel = std::min(std::abs(left), std::abs(right));
	const double slip = reference > _spin_rad_s ? 1. - slower_wheel / reference : 0.;

	if (slip > _slip_release) {
		_skidding = true;

	} else if (slip < _slip_resume) {
		_skidding = false;
	}

	// One value for both wheels, so the release cannot cause a yaw moment either.
	const double target = !_engaged || _skidding ? 0. : (_nose_down ? 1. : _touch_fraction);

	if (_skidding) {
		_level = std::max(0., _level - dt / _release_s);

	} else {
		const double step = dt / _ramp_s;
		_level += std::clamp(target - _level, -step, step);
	}

	// Fades with the ground speed rather than the wheel speed: a stopped wheel is not a sign that the aircraft has stopped.
	const double torque = _level * _max_torque * std::min(1., reference / _fade_rad_s);

	const auto command = [&ecm, torque](const Joint & joint, double wheel_speed) {
		const double value = wheel_speed > 0. ? -torque : (wheel_speed < 0. ? torque : 0.);
		ecm.SetComponentData<gz::sim::components::JointForceCmd>(joint.entity, {value});
	};
	command(_left_wheel, left);
	command(_right_wheel, right);

	const double now = std::chrono::duration<double>(info.simTime).count();

	if (now >= _next_state_s) {
		_next_state_s = now + STATE_PERIOD_S;
		gz::msgs::Double_V state;

		for (const double value : {torque, left, right, nose, motor, _level, _engaged ? 1. : 0., reference, _skidding ? 1. : 0.}) {
			state.add_data(value);
		}

		_state_pub.Publish(state);
	}
}
