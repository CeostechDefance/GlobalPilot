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

#include "GearSequencerSystem.hpp"

#include <algorithm>
#include <cmath>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

using namespace custom;

GZ_ADD_PLUGIN(
	GearSequencerSystem,
	gz::sim::System,
	GearSequencerSystem::ISystemConfigure,
	GearSequencerSystem::ISystemPreUpdate
)

namespace
{
constexpr double ARRIVED_RAD = 0.01;
constexpr double POSITION_GAIN = 5.; // 1/s, slows a joint down over the last part of its travel
}

void GearSequencerSystem::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
				    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &)
{
	_model = entity;
	const gz::sim::Model model(entity);

	const auto read_joints = [&sdf](const char *tag, std::vector<Joint> &joints) {
		auto element = sdf->FindElement(tag);

		while (element) {
			Joint joint;
			joint.name = element->Get<std::string>("name");
			joint.travel = element->Get<double>("travel");
			joint.rate = element->Get<double>("rate");
			joints.push_back(joint);
			element = element->GetNextElement(tag);
		}
	};

	read_joints("gear_joint", _gear);
	read_joints("door_joint", _doors);

	const std::string topic = "/model/" + model.Name(ecm) + "/" + sdf->Get<std::string>("command_topic");

	if (!_node.Subscribe(topic, &GearSequencerSystem::commandCallback, this)) {
		gzerr << "GearSequencerSystem: failed to subscribe to " << topic << std::endl;
	}
}

void GearSequencerSystem::commandCallback(const gz::msgs::Double &msg)
{
	_gear_up_commanded = msg.data() > 0.1;
}

bool GearSequencerSystem::resolveJoint(Joint &joint, gz::sim::EntityComponentManager &ecm)
{
	joint.entity = gz::sim::Model(_model).JointByName(ecm, joint.name);

	if (joint.entity == gz::sim::kNullEntity) {
		gzerr << "GearSequencerSystem: joint " << joint.name << " not found" << std::endl;
		return false;
	}

	ecm.CreateComponent(joint.entity, gz::sim::components::JointPosition());
	ecm.CreateComponent(joint.entity, gz::sim::components::JointVelocityCmd({0.}));
	return true;
}

void GearSequencerSystem::driveJoint(Joint &joint, gz::sim::EntityComponentManager &ecm)
{
	const auto *position = ecm.Component<gz::sim::components::JointPosition>(joint.entity);

	if (position && !position->Data().empty()) {
		joint.position = position->Data()[0];
	}

	const double velocity = std::clamp(POSITION_GAIN * (joint.target - joint.position), -joint.rate, joint.rate);
	ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint.entity, {velocity});
}

void GearSequencerSystem::PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm)
{
	if (!_joints_resolved) {
		_joints_resolved = true;

		for (auto *joints : {&_gear, &_doors}) {
			for (auto &joint : *joints) {
				_joints_resolved = resolveJoint(joint, ecm) && _joints_resolved;
			}
		}

		if (!_joints_resolved) {
			return;
		}
	}

	if (info.paused) {
		return;
	}

	const bool gear_up = _gear_up_commanded;

	const auto arrived = [](const std::vector<Joint> &joints, bool at_travel) {
		return std::all_of(joints.begin(), joints.end(), [at_travel](const Joint & joint) {
			return std::abs(joint.position - (at_travel ? joint.travel : 0.)) < ARRIVED_RAD;
		});
	};

	const bool gear_in_position = _gear_target_up == gear_up && arrived(_gear, gear_up);

	// Doors stay open until the gear has arrived; the gear target only changes with the doors open.
	for (auto &door : _doors) {
		door.target = gear_in_position ? 0. : door.travel;
	}

	if (_gear_target_up != gear_up && arrived(_doors, true)) {
		_gear_target_up = gear_up;

		for (auto &joint : _gear) {
			joint.target = gear_up ? joint.travel : 0.;
		}
	}

	for (auto *joints : {&_gear, &_doors}) {
		for (auto &joint : *joints) {
			driveJoint(joint, ecm);
		}
	}
}
