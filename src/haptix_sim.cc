/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
  hxResult hxs_getdeviceinfo(int /*_target*/, hxsDeviceInfo* /*_deviceinfo*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_getbody(int /*_target*/, hxsBody* /*_body*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_getjoint(int /*_target*/, hxsJoint* /*_joint*/)
  {
    return hxOK;
  }

  /////////////////////////////////////////////////
  hxResult hxs_getcontact(int /*_target*/, hxsContact* /*_contact*/)
  {
    return hxOK;
  }

  /////////////////////////////////////////////////
  hxResult hxs_getjacobian(int /*_target*/, int /*_link*/,
                           const float* /*_point*/, float* /*_jacobian*/)
  {
    return hxOK;
  }

  /////////////////////////////////////////////////
  hxResult hxs_setstate(int /*_target*/, const hxsBody* /*_body*/,
                        const hxsJoint* /*_joint*/)
  {
    return hxOK;
  }

  /////////////////////////////////////////////////
  hxResult hxs_updatedirect(int /*_target*/, const float* /*_torque*/,
                            hxSensor* /*_sensor*/, int /*_flg_sleep*/)
  {
    return hxOK;
  }
}
