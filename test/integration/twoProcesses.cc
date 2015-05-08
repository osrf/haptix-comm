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

#include <cstdlib>
#include "gtest/gtest.h"
#include "haptix/comm/haptix.h"
#include "msg/hxCommand.pb.h"
#include "msg/hxRobot.pb.h"
#include "msg/hxSensor.pb.h"
#include "test_config.h"

#ifdef _WIN32
#define HX_CMD_PREFIX "start "
#else
#define HX_CMD_PREFIX ""
#endif

std::string partition;
std::string robotInfoTopic = "/haptix/gazebo/GetRobotInfo";
std::string updateTopic = "/haptix/gazebo/Update";

//////////////////////////////////////////////////
/// \brief Three different nodes running in two different processes. In the
/// subscriber processs there are two nodes. Both should receive the message.
/// After some time one of them unsubscribe. After that check that only one
/// node receives the message.
TEST(twoProcesses, SrvTwoProcs)
{
  // Launch an ignition transport node that will advertise services.
  std::string responserPath = testing::portablePathUnion(
    PROJECT_BINARY_PATH, "test/integration/hx_responser_test");

  testing::forkHandlerType pi = testing::forkAndRun(responserPath.c_str(),
    partition.c_str());
  /*std::string command = std::string(HX_CMD_PREFIX) + BUILD_DIR +
    std::string("/test/integration/hx_responser_test 10000&");
  ASSERT_EQ(std::system(command.c_str()), 0);*/

  hxRobotInfo robotInfo;
  hxCommand cmd;
  hxSensor sensor;

  EXPECT_EQ(hx_connect(NULL, 0), hxOK);

  // Request the robot information.
  ASSERT_EQ(hx_robot_info(&robotInfo), hxOK);

  EXPECT_EQ(robotInfo.motor_count, 4);
  EXPECT_EQ(robotInfo.joint_count, 5);
  EXPECT_EQ(robotInfo.contact_sensor_count, 6);
  EXPECT_EQ(robotInfo.imu_count, 7);
  EXPECT_FLOAT_EQ(robotInfo.update_rate, 8.0);

  for (int i = 0; i < robotInfo.motor_count; ++i)
  {
    EXPECT_FLOAT_EQ(robotInfo.motor_limit[i][0], -i);
    EXPECT_FLOAT_EQ(robotInfo.motor_limit[i][1], i);
  }
  for (int i = 0; i < robotInfo.joint_count; ++i)
  {
    EXPECT_FLOAT_EQ(robotInfo.joint_limit[i][0], -i);
    EXPECT_FLOAT_EQ(robotInfo.joint_limit[i][1], i);
  }

  // Fill the joint command.
  for (int i = 0; i < robotInfo.motor_count; ++i)
  {
    cmd.ref_pos[i] = i;
    cmd.ref_vel_max[i] = i + 1;
    cmd.gain_pos[i] = i + 2;
    cmd.gain_vel[i] = i + 3;
  }

  // Request an update.
  EXPECT_EQ(hx_update(&cmd, &sensor), hxOK);

  // Check the response.
  for (int i = 0; i < robotInfo.motor_count; ++i)
  {
    EXPECT_FLOAT_EQ(sensor.motor_pos[i], i);
    EXPECT_FLOAT_EQ(sensor.motor_vel[i], i + 1);
    EXPECT_FLOAT_EQ(sensor.motor_torque[i], i + 2);
  }

  for (int i = 0; i < robotInfo.joint_count; ++i)
  {
    EXPECT_FLOAT_EQ(sensor.joint_pos[i], i);
    EXPECT_FLOAT_EQ(sensor.joint_vel[i], i + 1);
  }

  for (int i = 0; i < robotInfo.contact_sensor_count; ++i)
    EXPECT_FLOAT_EQ(sensor.contact[i], i);

  for (int i = 0; i < robotInfo.imu_count; ++i)
  {
    EXPECT_FLOAT_EQ(sensor.imu_linear_acc[i][0], i);
    EXPECT_FLOAT_EQ(sensor.imu_linear_acc[i][1], i + 1);
    EXPECT_FLOAT_EQ(sensor.imu_linear_acc[i][2], i + 2);
    EXPECT_FLOAT_EQ(sensor.imu_angular_vel[i][0], i + 3);
    EXPECT_FLOAT_EQ(sensor.imu_angular_vel[i][1], i + 4);
    EXPECT_FLOAT_EQ(sensor.imu_angular_vel[i][2], i + 5);
  }

  EXPECT_EQ(sensor.time_stamp.sec, 9);
  EXPECT_EQ(sensor.time_stamp.nsec, 10);

  EXPECT_EQ(hx_close(), hxOK);

  // Need to kill the responser node running on an external process.
  testing::killFork(pi);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Get a random partition name.
  partition = testing::getRandomNumber();

  // Set the partition name for this process.
  setenv("IGN_PARTITION", partition.c_str(), 1);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
