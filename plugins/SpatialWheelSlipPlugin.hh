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

#ifndef GAZEBO_PLUGINS_SPATIALWHEELSLIPPLUGIN_HH_
#define GAZEBO_PLUGINS_SPATIALWHEELSLIPPLUGIN_HH_

#include <memory>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"

#include "WheelSlipPlugin.hh"

namespace gazebo
{
  // Forward declare private data class
  class SpatialWheelSlipPluginPrivate;

  /// \brief Extension of WheelSlipPlugin that uses spatial maps of surface
  /// parameters to update wheel friction and slip parameters based on their
  /// position in the environment.
  /// compliance parameters
  class GZ_PLUGIN_VISIBLE SpatialWheelSlipPlugin : public WheelSlipPlugin
  {
    /// \brief Constructor.
    public: SpatialWheelSlipPlugin();

    /// \brief Destructor.
    public: ~SpatialWheelSlipPlugin() override;

    // Documentation inherited
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    // Documentation inherited
    public: virtual void Update() override;

    /// \brief Private data pointer.
    private: std::unique_ptr<SpatialWheelSlipPluginPrivate> dataPtr;
  };
}
#endif
