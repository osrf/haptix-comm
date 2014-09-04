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

#include <ignition/transport.hh>
#include "gtest/gtest.h"
#include "haptix/comm/comm.h"
#include "haptix/comm/types.h"
#include "msg/hxCommand.pb.h"
#include "msg/hxDevice.pb.h"
#include "msg/hxSensor.pb.h"

int numMotors = 4;
int numJoints = 5;
int numContactSensors = 6;
int numIMUs = 7;

//////////////////////////////////////////////////
/// \brief Provide a "GetDeviceInfo" service.
void onGetDeviceInfo(const std::string &_service,
  const haptix::comm::msgs::hxDevice &/*_req*/,
  haptix::comm::msgs::hxDevice &_rep, bool &_result)
{
  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/GetDeviceInfo");

  // Create some dummy response.
  _rep.set_nmotor(numMotors);
  _rep.set_njoint(numJoints);
  _rep.set_ncontactsensor(numContactSensors);
  _rep.set_nimu(numIMUs);

  for (int i = 0; i < numJoints; ++i)
  {
    haptix::comm::msgs::hxJointAngle *joint = _rep.add_limit();
    joint->set_min(-i);
    joint->set_max(i);
  }

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide an "Update" service.
void onUpdate(const std::string &_service,
  const haptix::comm::msgs::hxCommand &_req, haptix::comm::msgs::hxSensor &_rep,
  bool &_result)
{
  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/Update");

  // Read the request parameters.
  ASSERT_EQ(_req.ref_pos_size(), hxMAXMOTOR);
  for (int i = 0; i < numMotors; ++i)
  {
    EXPECT_FLOAT_EQ(_req.ref_pos(i), i);
    EXPECT_FLOAT_EQ(_req.ref_vel(i), i + 1);
    EXPECT_FLOAT_EQ(_req.gain_pos(i), i + 2);
    EXPECT_FLOAT_EQ(_req.gain_vel(i), i + 3);
  }

  // Create some dummy response.
  for (int i = 0; i < numMotors; ++i)
  {
    _rep.add_motor_pos(i);
    _rep.add_motor_vel(i + 1);
    _rep.add_motor_torque(i + 2);
  }

  for (int i = 0; i < numJoints; ++i)
  {
    _rep.add_joint_pos(i);
    _rep.add_joint_vel(i + 1);
  }

  for (int i = 0; i < numContactSensors; ++i)
    _rep.add_contact(i);

  for (int i = 0; i < numIMUs; ++i)
  {
    haptix::comm::msgs::imu *linacc = _rep.add_imu_linacc();
    linacc->set_x(i);
    linacc->set_y(i + 1);
    linacc->set_z(i + 2);
    haptix::comm::msgs::imu *angvel = _rep.add_imu_angvel();
    angvel->set_x(i + 3);
    angvel->set_y(i + 4);
    angvel->set_z(i + 5);
  }

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Check that we can use the C-wrapper.
TEST(CommTest, BasicUsage)
{
  ignition::transport::Node node;

  // Advertise the "getdeviceinfo" service.
  node.Advertise(std::string("/haptix/gazebo/GetDeviceInfo"), onGetDeviceInfo);

  // Advertise the "update" service.
  node.Advertise(std::string("/haptix/gazebo/Update"), onUpdate);

  EXPECT_EQ(hx_connect(hxGAZEBO), hxOK);

  hxDeviceInfo deviceInfo;
  ASSERT_EQ(hx_getdeviceinfo(hxGAZEBO, &deviceInfo), hxOK);

  ASSERT_EQ(deviceInfo.nmotor, numMotors);
  ASSERT_EQ(deviceInfo.njoint, numJoints);
  ASSERT_EQ(deviceInfo.ncontactsensor, numContactSensors);
  ASSERT_EQ(deviceInfo.nIMU, numIMUs);

  hxCommand cmd;
  hxSensor sensor;
  // Fill the joint command.

  for (int i = 0; i < deviceInfo.nmotor; ++i)
  {
    cmd.ref_pos[i] = i;
    cmd.ref_vel[i] = i + 1;
    cmd.gain_pos[i] = i + 2;
    cmd.gain_vel[i] = i + 3;
  }

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

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
