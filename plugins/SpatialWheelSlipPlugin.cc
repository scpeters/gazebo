/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "plugins/SpatialWheelSlipPlugin.hh"

namespace gazebo
{
  namespace physics
  {
    typedef boost::weak_ptr<physics::Link> LinkWeakPtr;
    typedef boost::weak_ptr<physics::Model> ModelWeakPtr;
  }

  class SpatialWheelSlipPluginPrivate
  {
    /// \brief Model pointer.
    public: physics::ModelWeakPtr model;

    /// \brief Link names and pointers.
    public: std::map<std::string, physics::LinkWeakPtr> mapNameToLink;
  };
}

using namespace gazebo;

// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(SpatialWheelSlipPlugin)

/////////////////////////////////////////////////
SpatialWheelSlipPlugin::SpatialWheelSlipPlugin()
  : dataPtr(new SpatialWheelSlipPluginPrivate)
{
}

/////////////////////////////////////////////////
SpatialWheelSlipPlugin::~SpatialWheelSlipPlugin()
{
}

/////////////////////////////////////////////////
void SpatialWheelSlipPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "WheelSlipPlugin model pointer is NULL");
  GZ_ASSERT(_sdf, "WheelSlipPlugin sdf pointer is NULL");

  this->dataPtr->model = _model;

  if (!_sdf->HasElement("wheel"))
  {
    gzerr << "No wheel tags specified, plugin is disabled" << std::endl;
    return;
  }

  WheelSlipPlugin::Load(_model, _sdf);

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
    this->dataPtr->mapNameToLink[linkName] = link;
  }
}

/////////////////////////////////////////////////
void SpatialWheelSlipPlugin::Update()
{
  // Get current position of each wheel and update parameters before calling
  // WheelSlipPlugin::Update.
  for (const auto &nameToLink : this->dataPtr->mapNameToLink)
  {
    auto link = nameToLink.second.lock();
    if (!link)
    {
      continue;
    }

    // Current wheel position
    ignition::math::Vector3d wheelPos = link->WorldPose().Pos();

    // This is just an example of setting parameters based on spatial position.
    // TODO: implement a more general approach like a lookup table.

    // set longitudinal slip equal to X coordinate
    // replace negative values with 0
    double slip = (wheelPos.X() < 0) ? 0 : wheelPos.X();
    this->SetSlipComplianceLongitudinal(slip, nameToLink.first);

    // set longitudinal friction equal to Y coordinate
    // replace negative values with 0
    double friction = (wheelPos.Y() < 0) ? 0 : wheelPos.Y();
    this->SetFrictionLongitudinal(friction, nameToLink.first);
  }

  WheelSlipPlugin::Update();
}
