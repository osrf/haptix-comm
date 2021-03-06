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

#include <ignition/transport.hh>
#include "gtest/gtest.h"
#include "haptix/comm/haptix.h"
#include "msg/hxCommand.pb.h"
#include "msg/hxRobot.pb.h"
#include "msg/hxSensor.pb.h"
#include "test_config.h"

const std::string kRobotInfoTopic = "/haptix/gazebo/GetRobotInfo";
const std::string kUpdateTopic    = "/haptix/gazebo/Update";
const std::string kReadTopic      = "/haptix/gazebo/Read";

const int kNumMotors         = 4;
const int kNumJoints         = 5;
const int kNumContactSensors = 6;
const int kNumIMUs           = 7;

std::string partition;

//////////////////////////////////////////////////
/// \brief Provide a "GetRobotInfo" service.
void onGetRobotInfo(const haptix::comm::msgs::hxRobot &/*_req*/,
  haptix::comm::msgs::hxRobot &_rep,
  bool &_result)
{
  _rep.Clear();

  // Create some dummy response.
  _rep.set_motor_count(kNumMotors);
  _rep.set_joint_count(kNumJoints);
  _rep.set_contact_sensor_count(kNumContactSensors);
  _rep.set_imu_count(kNumIMUs);

  for (int i = 0; i < kNumJoints; ++i)
  {
    haptix::comm::msgs::hxRobot::hxLimit *joint = _rep.add_joint_limit();
    joint->set_minimum(-i);
    joint->set_maximum(i);
  }

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide an "Update" service.
void onUpdate(const haptix::comm::msgs::hxCommand &_req,
  haptix::comm::msgs::hxSensor &_rep,
  bool &_result)
{
  _rep.Clear();

  // Read the request parameters.
  ASSERT_EQ(_req.ref_pos_size(), hxMAXMOTOR);
  for (int i = 0; i < kNumMotors; ++i)
  {
    EXPECT_FLOAT_EQ(_req.ref_pos(i), i);
    EXPECT_FLOAT_EQ(_req.ref_vel(i), i + 1);
    EXPECT_FLOAT_EQ(_req.ref_vel_max(i), i + 1);
    EXPECT_FLOAT_EQ(_req.gain_pos(i), i + 2);
    EXPECT_FLOAT_EQ(_req.gain_vel(i), i + 3);
  }

  // Create some dummy response.
  for (int i = 0; i < kNumMotors; ++i)
  {
    _rep.add_motor_pos(i);
    _rep.add_motor_vel(i + 1);
    _rep.add_motor_torque(i + 2);
  }

  for (int i = 0; i < kNumJoints; ++i)
  {
    _rep.add_joint_pos(i);
    _rep.add_joint_vel(i + 1);
  }

  for (int i = 0; i < kNumContactSensors; ++i)
    _rep.add_contact(i);

  for (int i = 0; i < kNumIMUs; ++i)
  {
    haptix::comm::msgs::imu *linear_acc = _rep.add_imu_linear_acc();
    linear_acc->set_x(i);
    linear_acc->set_y(i + 1);
    linear_acc->set_z(i + 2);
    haptix::comm::msgs::imu *angular_vel = _rep.add_imu_angular_vel();
    angular_vel->set_x(i + 3);
    angular_vel->set_y(i + 4);
    angular_vel->set_z(i + 5);
  }

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "Read" service.
void onRead(const haptix::comm::msgs::hxSensor &/*_req*/,
  haptix::comm::msgs::hxSensor &_rep,
  bool &_result)
{
  _rep.Clear();

  // Create some dummy response.
  for (int i = 0; i < kNumMotors; ++i)
  {
    _rep.add_motor_pos(i);
    _rep.add_motor_vel(i + 1);
    _rep.add_motor_torque(i + 2);
  }

  for (int i = 0; i < kNumJoints; ++i)
  {
    _rep.add_joint_pos(i);
    _rep.add_joint_vel(i + 1);
  }

  for (int i = 0; i < kNumContactSensors; ++i)
    _rep.add_contact(i);

  for (int i = 0; i < kNumIMUs; ++i)
  {
    haptix::comm::msgs::imu *linear_acc = _rep.add_imu_linear_acc();
    linear_acc->set_x(i);
    linear_acc->set_y(i + 1);
    linear_acc->set_z(i + 2);
    haptix::comm::msgs::imu *angular_vel = _rep.add_imu_angular_vel();
    angular_vel->set_x(i + 3);
    angular_vel->set_y(i + 4);
    angular_vel->set_z(i + 5);
  }

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Check that we can use the C-wrapper.
TEST(CommTest, BasicUsage)
{
  ignition::transport::Node node;

  // Advertise the "getdeviceinfo" service.
  node.Advertise(kRobotInfoTopic, onGetRobotInfo);

  // Advertise the "update" service.
  node.Advertise(kUpdateTopic, onUpdate);

  // Advertise the "read" service.
  node.Advertise(kReadTopic, onRead);

  EXPECT_EQ(hx_connect(NULL, 0), hxOK);

  hxRobotInfo robotInfo;
  ASSERT_EQ(hx_robot_info(&robotInfo), hxOK);

  ASSERT_EQ(robotInfo.motor_count, kNumMotors);
  ASSERT_EQ(robotInfo.joint_count, kNumJoints);
  ASSERT_EQ(robotInfo.contact_sensor_count, kNumContactSensors);
  ASSERT_EQ(robotInfo.imu_count, kNumIMUs);

  hxCommand cmd;
  hxSensor sensor;

  // Fill the joint command.
  for (int i = 0; i < robotInfo.motor_count; ++i)
  {
    cmd.ref_pos[i] = i;
    cmd.ref_vel[i] = i + 1;
    cmd.ref_vel_max[i] = i + 1;
    cmd.gain_pos[i] = i + 2;
    cmd.gain_vel[i] = i + 3;
  }

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

  // Test hx_read_sensors.
  EXPECT_EQ(hx_read_sensors(&sensor), hxOK);

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

  EXPECT_EQ(hx_close(), hxOK);
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
