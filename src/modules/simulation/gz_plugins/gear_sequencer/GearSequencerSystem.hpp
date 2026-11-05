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
#include <string>
#include <vector>

#include <gz/msgs/double.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace custom
{
/**
 * Drives retractable landing gear and its doors from one command topic.
 *
 * The doors open first, the gear moves only once every door is open, and the doors
 * close when the gear has arrived. A command above zero means gear up; anything else,
 * including the centered value PX4 sends while disarmed, means gear down.
 */
class GearSequencerSystem:
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
		double travel{0.};  // joint angle at gear up (gear) or door open (door), rad
		double rate{0.};    // rad/s
		double target{0.};
		double position{0.};
	};

	bool resolveJoint(Joint &joint, gz::sim::EntityComponentManager &ecm);
	void driveJoint(Joint &joint, gz::sim::EntityComponentManager &ecm);
	void commandCallback(const gz::msgs::Double &msg);

	gz::sim::Entity _model{gz::sim::kNullEntity};
	gz::transport::Node _node;
	std::vector<Joint> _gear;
	std::vector<Joint> _doors;
	std::atomic<bool> _gear_up_commanded{false};
	bool _gear_target_up{false};
	bool _joints_resolved{false};
};
} // end namespace custom
