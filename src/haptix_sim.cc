/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "haptix/comm/haptix.h"
#include "haptix/comm/haptix_sim.h"

extern "C" {
  //////////////////////////////////////////////////
  hxResult hxs_siminfo(hxSimInfo * /*_siminfo*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_camera(hxCamera * /*_camera*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_camera_transform(hxTransform /*_transform*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_contacts(hxContact * /*_contact*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_jacobian(const hxLink * /*_link*/, const float * /*_point*/,
                        float * /*_jacobian*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_state(const hxModel * /*_model*/, const hxJoint * /*_joint*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxModel *hxs_add_model(const char * /*_urdf*/, float /*_x*/, float /*_y*/,
                         float /*_z*/, float /*_roll*/, float /*_pitch*/,
                         float /*_yaw*/)
  {
    return 0;
  }

  //////////////////////////////////////////////////
  hxResult hxs_remove_model_id(int /*_id*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_remove_model(const hxModel * /*_model*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_model_transform(int /*_id*/, hxTransform /*_transform*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_linear_velocity(int /*_id*/, hxVector3 /*_velocity*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_angular_velocity(int /*_id*/, hxVector3 /*_velocity*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_linear_accel(int /*_id*/, hxVector3 /*_accel*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_angular_accel(int /*_id*/, hxVector3 /*_accel*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_force(const hxLink * /*_link*/, hxVector3 /*_force*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_torque(const hxLink * /*_link*/, hxVector3 /*_torque*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_reset(int /*_resetLimbPose*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_reset_timer()
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_start_timer()
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_stop_timer()
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_start_logging(const char * /*_filename*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  int hxs_is_logging()
  {
    return 0;
  }

  //////////////////////////////////////////////////
  hxResult hxs_stop_logging()
  {
    return hxOK;
  }
}
