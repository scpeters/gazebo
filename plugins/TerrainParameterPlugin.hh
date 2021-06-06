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

#ifndef GAZEBO_PLUGINS_TERRAIN_PARAMETER_PLUGIN_HH_
#define GAZEBO_PLUGINS_TERRAIN_PARAMETER_PLUGIN_HH_

#include <memory>

#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief An base class plugin for custom force torque sensor processing.
  class GZ_PLUGIN_VISIBLE TerrainParameterPlugin : public ModelPlugin
  {
    // Forward declarations.
    class TerrainParameterPluginPrivate;

    /// \brief Constructor
    public: TerrainParameterPlugin();

    /// \brief Destructor
    public: virtual ~TerrainParameterPlugin();

    /// \brief Load the plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf SDF element for the plugin.
    public: virtual void Load(sensors::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Private data pointer.
    private: std::unique_ptr<TerrainParameterPluginPrivate> dataPtr;
  };
}
#endif
