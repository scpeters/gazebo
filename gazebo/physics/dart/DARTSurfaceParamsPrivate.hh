/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_DARTSURFACEPARAMS_PRIVATE_HH_
#define _GAZEBO_DARTSURFACEPARAMS_PRIVATE_HH_

#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for DARTSurfaceParams
    class DARTSurfaceParamsPrivate
    {
      /// \brief Constructor
      public: DARTSurfaceParamsPrivate()
        : frictionPyramid(new FrictionPyramid())
      {
      }

      /// \brief Default destructor
      public: ~DARTSurfaceParamsPrivate()
      {
      }

      /// \brief Friction pyramid parameters (mu1, mu2).
      public: FrictionPyramidPtr frictionPyramid;

      /// \brief Artificial contact slip in the primary friction direction.
      /// \sa    See dContactSlip1 in
      ///        http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
      public: double slip1;

      /// \brief Artificial contact slip in the secondary friction dirction.
      /// \sa    See dContactSlip2 in
      ///        http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
      public: double slip2;
    };
  }
}
#endif
