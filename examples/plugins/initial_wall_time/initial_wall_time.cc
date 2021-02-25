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
#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>

namespace gazebo
{
  class InitialWallTime : public WorldPlugin
  {
    public: InitialWallTime() : WorldPlugin()
            {
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
              auto wallTime = common::Time::GetWallTime();
              _world->SetSimTime(wallTime);
              gzmsg << "Initializing simulation time to " << wallTime
                    << std::endl;
            }
  };
  GZ_REGISTER_WORLD_PLUGIN(InitialWallTime)
}
