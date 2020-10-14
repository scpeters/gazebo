/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>

#include <plugins/JointTrajectoryPlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
JointTrajectoryPlugin::JointTrajectoryPlugin()
{
}

/////////////////////////////////////////////////
JointTrajectoryPlugin::~JointTrajectoryPlugin()
{
  this->updateConnection.reset();
}

/////////////////////////////////////////////////
void JointTrajectoryPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr /*_sdf*/)
{
  // Get the world name.
  this->model = _parent;
  this->world = this->model->GetWorld();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&JointTrajectoryPlugin::UpdateStates, this,
      std::placeholders::_1));
}

/////////////////////////////////////////////////
void JointTrajectoryPlugin::UpdateStates(const common::UpdateInfo & /*_info*/)
{
  common::Time cur_time = this->world->SimTime();

  auto controller = this->model->GetJointController();

  const double sinusoid = sin(0.5 * cur_time.Double());
  const double squareWave= sinusoid > 0 ? 1 : 0;
  const double velocityTarget = 1.25 * squareWave;

  controller->SetVelocityTarget(this->model->GetName() + "::wheel_rear_left_spin", velocityTarget);
  controller->SetVelocityTarget(this->model->GetName() + "::wheel_rear_right_spin", velocityTarget);

  // auto velocities = controller->GetVelocities();
  // for (auto mapIter : velocities)
  // {
  //   std::cerr << mapIter.first << ", " << mapIter.second << std::endl;
  // }
}

GZ_REGISTER_MODEL_PLUGIN(JointTrajectoryPlugin)
}
