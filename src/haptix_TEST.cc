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
#include "haptix/comm/haptix.h"
#include "msg/hxCommand.pb.h"
#include "msg/hxDevice.pb.h"
#include "msg/hxSensor.pb.h"

static const std::string DeviceInfoTopic = "/haptix/gazebo/GetDeviceInfo";
static const std::string UpdateTopic     = "/haptix/gazebo/Update";
static const std::string ReadTopic       = "/haptix/gazebo/Read";

static const int NumMotors         = 4;
static const int NumJoints         = 5;
static const int NumContactSensors = 6;
static const int NumIMUs           = 7;

//////////////////////////////////////////////////
/// \brief Provide a "GetDeviceInfo" service.
void onGetDeviceInfo(const std::string &_service,
  const haptix::comm::msgs::hxDevice &/*_req*/,
  haptix::comm::msgs::hxDevice &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, DeviceInfoTopic);

  // Create some dummy response.
  _rep.set_nmotor(NumMotors);
  _rep.set_njoint(NumJoints);
  _rep.set_ncontactsensor(NumContactSensors);
  _rep.set_nimu(NumIMUs);

  for (int i = 0; i < NumJoints; ++i)
  {
    haptix::comm::msgs::hxJointAngle *joint = _rep.add_limit();
    joint->set_minimum(-i);
    joint->set_maximum(i);
  }

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide an "Update" service.
void onUpdate(const std::string &_service,
  const haptix::comm::msgs::hxCommand &_req,
  haptix::comm::msgs::hxSensor &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, UpdateTopic);

  // Read the request parameters.
  ASSERT_EQ(_req.ref_pos_size(), hxMAXMOTOR);
  for (int i = 0; i < NumMotors; ++i)
  {
    EXPECT_FLOAT_EQ(_req.ref_pos(i), i);
    EXPECT_FLOAT_EQ(_req.ref_vel(i), i + 1);
    EXPECT_FLOAT_EQ(_req.gain_pos(i), i + 2);
    EXPECT_FLOAT_EQ(_req.gain_vel(i), i + 3);
  }

  // Create some dummy response.
  for (int i = 0; i < NumMotors; ++i)
  {
    _rep.add_motor_pos(i);
    _rep.add_motor_vel(i + 1);
    _rep.add_motor_torque(i + 2);
  }

  for (int i = 0; i < NumJoints; ++i)
  {
    _rep.add_joint_pos(i);
    _rep.add_joint_vel(i + 1);
  }

  for (int i = 0; i < NumContactSensors; ++i)
    _rep.add_contact(i);

  for (int i = 0; i < NumIMUs; ++i)
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
/// \brief Provide a "Read" service.
void onRead(const std::string &_service,
  const haptix::comm::msgs::hxSensor &/*_req*/,
  haptix::comm::msgs::hxSensor &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, ReadTopic);

  // Create some dummy response.
  for (int i = 0; i < NumMotors; ++i)
  {
    _rep.add_motor_pos(i);
    _rep.add_motor_vel(i + 1);
    _rep.add_motor_torque(i + 2);
  }

  for (int i = 0; i < NumJoints; ++i)
  {
    _rep.add_joint_pos(i);
    _rep.add_joint_vel(i + 1);
  }

  for (int i = 0; i < NumContactSensors; ++i)
    _rep.add_contact(i);

  for (int i = 0; i < NumIMUs; ++i)
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
  node.Advertise(DeviceInfoTopic, onGetDeviceInfo);

  // Advertise the "update" service.
  node.Advertise(UpdateTopic, onUpdate);

  // Advertise the "read" service.
  node.Advertise(ReadTopic, onRead);

  EXPECT_EQ(hx_connect(hxGAZEBO), hxOK);

  hxDeviceInfo deviceInfo;
  ASSERT_EQ(hx_getdeviceinfo(hxGAZEBO, &deviceInfo), hxOK);

  ASSERT_EQ(deviceInfo.nmotor, NumMotors);
  ASSERT_EQ(deviceInfo.njoint, NumJoints);
  ASSERT_EQ(deviceInfo.ncontactsensor, NumContactSensors);
  ASSERT_EQ(deviceInfo.nIMU, NumIMUs);

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

  // Test hx_readsensors.
  EXPECT_EQ(hx_readsensors(hxGAZEBO, &sensor), hxOK);

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
