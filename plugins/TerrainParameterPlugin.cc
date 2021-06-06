/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gazebo/common/Assert.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include "plugins/TerrainParameterPlugin.hh"

namespace gazebo
{
  namespace physics
  {
    typedef boost::weak_ptr<physics::Model> ModelWeakPtr;
  }

  class TerrainParameterPluginPrivate
  {
    /// \brief Model pointer.
    public: physics::ModelWeakPtr model;

    /// \brief Pointer to the update event connection
    public: event::ConnectionPtr updateConnection;
  };
}

using namespace gazebo;

// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(TerrainParameterPlugin)

/////////////////////////////////////////////////
TerrainParameterPlugin::TerrainParameterPlugin()
  : dataPtr(new TerrainParameterPluginPrivate)
{
}

/////////////////////////////////////////////////
TerrainParameterPlugin::~TerrainParameterPlugin()
{
}

/////////////////////////////////////////////////
void TerrainParameterPlugin::Fini()
{
  this->dataPtr->updateConnection.reset();
}

/////////////////////////////////////////////////
void TerrainParameterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "TerrainParameterPlugin model pointer is NULL");
  GZ_ASSERT(_sdf, "TerrainParameterPlugin sdf pointer is NULL");

  this->dataPtr->model = _model;
  auto world = _model->GetWorld();
  GZ_ASSERT(world, "world pointer is NULL");

  if (!_sdf->HasElement("wheel"))
  {
    gzerr << "No wheel tags specified, plugin is disabled" << std::endl;
    return;
  }

  // Read each wheel element
  for (auto wheelElem = _sdf->GetElement("wheel"); wheelElem;
      wheelElem = wheelElem->GetNextElement("wheel"))
  {
    if (!wheelElem->HasAttribute("link_name"))
    {
      gzerr << "wheel element missing link_name attribute" << std::endl;
      continue;
    }

    // Get link name
    auto linkName = wheelElem->Get<std::string>("link_name");

    auto link = _model->GetLink(linkName);
    if (link == nullptr)
    {
      gzerr << "Could not find link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }
  }

  // Connect to the update event
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&TerrainParameterPlugin::Update, this));
}

/////////////////////////////////////////////////
physics::ModelPtr TerrainParameterPlugin::GetParentModel() const
{
  return this->dataPtr->model.lock();
}

/////////////////////////////////////////////////
void TerrainParameterPlugin::Update()
{
  // Get slip data so it can be published later
  std::map<std::string, ignition::math::Vector3d> slips;
  this->GetSlips(slips);

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  for (const auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    const auto &params = linkSurface.second;

    // get user-defined normal force constant
    double force = params.wheelNormalForce;

    // get link angular velocity parallel to joint axis
    ignition::math::Vector3d wheelAngularVelocity;
    auto link = linkSurface.first.lock();
    if (link)
      wheelAngularVelocity = link->WorldAngularVel();

    ignition::math::Vector3d jointAxis;
    auto joint = params.joint.lock();
    if (joint)
      jointAxis = joint->GlobalAxis(0);

    double spinAngularVelocity = wheelAngularVelocity.Dot(jointAxis);

    auto surface = params.surface.lock();
    if (surface)
    {
      // As discussed in TerrainParameterPlugin.hh, the ODE slip1 and slip2
      // parameters have units of inverse viscous damping:
      // [linear velocity / force] or [m / s / N].
      // Since the slip compliance parameters supplied to the plugin
      // are unitless, they must be scaled by a linear speed and force
      // magnitude before being passed to ODE.
      // The force is taken from a user-defined constant that should roughly
      // match the steady-state normal force at the wheel.
      // The linear speed is computed dynamically at each time step as
      // radius * spin angular velocity.
      // This choice of linear speed corresponds to the denominator of
      // the slip ratio during acceleration (see equation (1) in
      // Yoshida, Hamano 2002 DOI 10.1109/ROBOT.2002.1013712
      // "Motion dynamics of a rover with slip-based traction model").
      // The acceleration form is more well-behaved numerically at low-speed
      // and when the vehicle is at rest than the braking form,
      // so it is used for both slip directions.
      double speed = params.wheelRadius * std::abs(spinAngularVelocity);
      surface->slip1 = speed / force * params.slipComplianceLateral;
      surface->slip2 = speed / force * params.slipComplianceLongitudinal;
      auto frictionPyramid = surface->FrictionPyramid();
      if (frictionPyramid != nullptr)
      {
        if (params.frictionLateral >= 0)
        {
          frictionPyramid->SetMuPrimary(params.frictionLateral);
        }
        if (params.frictionLongitudinal >= 0)
        {
          frictionPyramid->SetMuSecondary(params.frictionLongitudinal);
        }
      }
    }

    // Try to publish slip data for this wheel
    if (link)
    {
      msgs::Vector3d msg;
      auto name = link->GetName();
      msg = msgs::Convert(slips[name]);
      if (params.slipPub)
        params.slipPub->Publish(msg);
    }
  }
}
