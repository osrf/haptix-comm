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

#include <cstdlib>
#include "gtest/gtest.h"
#include "haptix/comm/config.hh"
#include "haptix/comm/haptix.h"
#include "msg/hxCommand.pb.h"
#include "msg/hxDevice.pb.h"
#include "msg/hxSensor.pb.h"

std::string deviceInfoTopic = "/haptix/gazebo/GetDeviceInfo";
std::string updateTopic = "/haptix/gazebo/Update";

//////////////////////////////////////////////////
/// \brief Three different nodes running in two different processes. In the
/// subscriber processs there are two nodes. Both should receive the message.
/// After some time one of them unsubscribe. After that check that only one
/// node receives the message.
TEST(twoProcesses, SrvTwoProcs)
{
  // Launch an ignition transport node that will advertise services.
  std::string command = BUILD_DIR + std::string("/example/responser 1000&");
  ASSERT_EQ(std::system(command.c_str()), 0);

  hxDeviceInfo deviceInfo;
  hxCommand cmd;
  hxSensor sensor;

  EXPECT_EQ(hx_connect(hxGAZEBO), hxOK);

  // Request the device information.
  ASSERT_EQ(hx_getdeviceinfo(hxGAZEBO, &deviceInfo), hxOK);

  // Fill the joint command.
  for (int i = 0; i < deviceInfo.nmotor; ++i)
  {
    cmd.ref_pos[i] = i;
    cmd.ref_vel[i] = i + 1;
    cmd.gain_pos[i] = i + 2;
    cmd.gain_vel[i] = i + 3;
  }

  // Request an update.
  EXPECT_EQ(hx_update(hxGAZEBO, &cmd, &sensor), hxOK);

  // Check the response.
  for (int i = 0; i < deviceInfo.nmotor; ++i)
  {
    EXPECT_FLOAT_EQ(sensor.motor_pos[i], i);
    EXPECT_FLOAT_EQ(sensor.motor_vel[i], i + 1);
    EXPECT_FLOAT_EQ(sensor.motor_torque[i], i + 2);
  }

  for (int i = 0; i < deviceInfo.njoint; ++i)
  {
    EXPECT_FLOAT_EQ(sensor.joint_pos[i], i);
    EXPECT_FLOAT_EQ(sensor.joint_vel[i], i + 1);
  }

  for (int i = 0; i < deviceInfo.ncontactsensor; ++i)
    EXPECT_FLOAT_EQ(sensor.contact[i], i);

  for (int i = 0; i < deviceInfo.nIMU; ++i)
  {
    EXPECT_FLOAT_EQ(sensor.IMU_linacc[i][0], i);
    EXPECT_FLOAT_EQ(sensor.IMU_linacc[i][1], i + 1);
    EXPECT_FLOAT_EQ(sensor.IMU_linacc[i][2], i + 2);
    EXPECT_FLOAT_EQ(sensor.IMU_angvel[i][0], i + 3);
    EXPECT_FLOAT_EQ(sensor.IMU_angvel[i][1], i + 4);
    EXPECT_FLOAT_EQ(sensor.IMU_angvel[i][2], i + 5);
  }

  EXPECT_EQ(hx_close(hxGAZEBO), hxOK);
}
