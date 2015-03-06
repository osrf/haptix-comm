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
#include "haptix/comm/haptix_sim.h"
#include "msg/hxCamera.pb.h"
#include "msg/hxContact.pb.h"
#include "msg/hxContact_V.pb.h"
#include "msg/hxEmpty.pb.h"
#include "msg/hxParam.pb.h"
#include "msg/hxSimInfo.pb.h"
#include "msg/hxTransform.pb.h"

/// \brief Global constants.
const int kNumModels         = 5;
const int kNumLinksPerModel  = 40;
const int kNumJointsPerModel = 20;
const int kNumContacts       = 30;

/// \brief Global variables.
haptix::comm::msgs::hxSimInfo simState;
haptix::comm::msgs::hxContact_V simContactsState;

void setup()
{
  simState.Clear();
  simContactsState.Clear();

  // Create a fake simulation state.
  // Models.
  for (int i = 0; i < kNumModels; ++i)
  {
    haptix::comm::msgs::hxModel *model = simState.add_models();
    model->mutable_transform()->mutable_pos()->set_x(i);
    model->mutable_transform()->mutable_pos()->set_y(i + 0.1);
    model->mutable_transform()->mutable_pos()->set_z(i + 0.2);
    model->mutable_transform()->mutable_orient()->set_w(i + 0.3);
    model->mutable_transform()->mutable_orient()->set_x(i + 0.4);
    model->mutable_transform()->mutable_orient()->set_y(i + 0.5);
    model->mutable_transform()->mutable_orient()->set_z(i + 0.6);
    model->set_is_static(1);
    model->set_id(i);
    // Links.
    for (int j = 0; j < kNumLinksPerModel; ++j)
    {
      float v = 10 * i + j;
      haptix::comm::msgs::hxLink *link = model->add_links();
      link->mutable_transform()->mutable_pos()->set_x(v);
      link->mutable_transform()->mutable_pos()->set_y(v + 0.1);
      link->mutable_transform()->mutable_pos()->set_z(v + 0.2);
      link->mutable_transform()->mutable_orient()->set_w(v + 0.3);
      link->mutable_transform()->mutable_orient()->set_x(v + 0.4);
      link->mutable_transform()->mutable_orient()->set_y(v + 0.5);
      link->mutable_transform()->mutable_orient()->set_z(v + 0.6);
      link->mutable_linvel()->set_x(v + 0.7);
      link->mutable_linvel()->set_y(v + 0.8);
      link->mutable_linvel()->set_z(v + 0.9);
      link->mutable_angvel()->set_x(v + 1);
      link->mutable_angvel()->set_y(v + 1.1);
      link->mutable_angvel()->set_z(v + 1.2);
      link->mutable_linacc()->set_x(v + 1.3);
      link->mutable_linacc()->set_y(v + 1.4);
      link->mutable_linacc()->set_z(v + 1.5);
      link->mutable_angacc()->set_x(v + 1.6);
      link->mutable_angacc()->set_y(v + 1.7);
      link->mutable_angacc()->set_z(v + 1.8);
    }
    // Joints.
    for (int j = 0; j < kNumJointsPerModel; ++j)
    {
      float v = 20 * i + j;
      haptix::comm::msgs::hxJoint *joint = model->add_joints();
      joint->set_pos(v);
      joint->set_vel(v + 0.1);
      joint->set_acc(v + 0.2);
      joint->set_torque_motor(v + 0.3);
      joint->set_torque_passive(v + 0.4);
    }
  }

  // Camera.
  simState.mutable_camera()->mutable_transform()->mutable_pos()->set_x(30.0);
  simState.mutable_camera()->mutable_transform()->mutable_pos()->set_y(30.1);
  simState.mutable_camera()->mutable_transform()->mutable_pos()->set_z(30.2);
  simState.mutable_camera()->mutable_transform()->mutable_orient()->set_w(30.3);
  simState.mutable_camera()->mutable_transform()->mutable_orient()->set_x(30.4);
  simState.mutable_camera()->mutable_transform()->mutable_orient()->set_y(30.5);
  simState.mutable_camera()->mutable_transform()->mutable_orient()->set_z(30.6);

  // Create some contacts.
  for (int i = 0; i < kNumContacts; ++i)
  {
    haptix::comm::msgs::hxContact *contact = simContactsState.add_contacts();
    contact->set_body1(i);
    contact->set_body2(i + 1);
    contact->mutable_point()->set_x(i + 0.2);
    contact->mutable_point()->set_y(i + 0.3);
    contact->mutable_point()->set_z(i + 0.4);
    contact->mutable_normal()->set_x(i + 0.5);
    contact->mutable_normal()->set_y(i + 0.6);
    contact->mutable_normal()->set_z(i + 0.7);
    contact->mutable_tangent1()->set_x(i + 0.8);
    contact->mutable_tangent1()->set_y(i + 0.9);
    contact->mutable_tangent1()->set_z(i + 1);
    contact->mutable_tangent2()->set_x(i + 1.1);
    contact->mutable_tangent2()->set_y(i + 1.2);
    contact->mutable_tangent2()->set_z(i + 1.3);
    contact->set_distance(i + 1.4);
    contact->mutable_velocity()->set_x(i + 1.5);
    contact->mutable_velocity()->set_y(i + 1.6);
    contact->mutable_velocity()->set_z(i + 1.7);
    contact->mutable_force()->set_x(i + 1.8);
    contact->mutable_force()->set_y(i + 1.9);
    contact->mutable_force()->set_z(i + 2);
  }
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_siminfo" service.
void onHxsSimInfo(const std::string &_service,
  const haptix::comm::msgs::hxEmpty &/*_req*/,
  haptix::comm::msgs::hxSimInfo &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_siminfo");

  // Create some dummy response.
  _rep = simState;

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_camera" service.
void onHxsCamera(const std::string &_service,
  const haptix::comm::msgs::hxEmpty &/*_req*/,
  haptix::comm::msgs::hxCamera &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_camera");

  // Create some dummy response.
  _rep = simState.camera();

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_camera_transform" service.
void onHxsCameraTransform(const std::string &_service,
  const haptix::comm::msgs::hxTransform &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_camera_transform");

  // Apply the camera transform.
  simState.mutable_camera()->mutable_transform()->mutable_pos()->set_x(
    _req.pos().x());
  simState.mutable_camera()->mutable_transform()->mutable_pos()->set_y(
    _req.pos().y());
  simState.mutable_camera()->mutable_transform()->mutable_pos()->set_z(
    _req.pos().z());
  simState.mutable_camera()->mutable_transform()->mutable_orient()->set_w(
    _req.orient().w());
  simState.mutable_camera()->mutable_transform()->mutable_orient()->set_x(
    _req.orient().x());
  simState.mutable_camera()->mutable_transform()->mutable_orient()->set_y(
    _req.orient().y());
  simState.mutable_camera()->mutable_transform()->mutable_orient()->set_z(
    _req.orient().z());

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_contacts" service.
void onHxsContacts(const std::string &_service,
  const haptix::comm::msgs::hxEmpty &/*_req*/,
  haptix::comm::msgs::hxContact_V &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_contacts");

  // Create some dummy response.
  _rep = simContactsState;

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_state" service.
void onHxsState(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_state");

  // Sanity check: The message should contain a model.
  if (!_req.has_model())
  {
    std::cerr << "onHxsState() error: Missing model in request" << std::endl;
    return;
  }

  // Sanity check: The message should contain a joint.
  if (!_req.has_joint())
  {
    std::cerr << "onHxsState() error: Missing joint in request" << std::endl;
    return;
  }

  // Apply the new simulation state.
  // ToDo.

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Check hxs_simInfo.
TEST(hxsTest, hxs_simInfo)
{
  setup();

  ignition::transport::Node node;
  hxSimInfo simInfo;

  // Advertise the "hxs_siminfo" service.
  node.Advertise("/haptix/gazebo/hxs_siminfo", onHxsSimInfo);

  // Request simulation information.
  ASSERT_EQ(hxs_siminfo(&simInfo), hxOK);

  // Check the model information
  ASSERT_EQ(simInfo.modelCount, kNumModels);
  for (int i = 0; i < simInfo.modelCount; ++i)
  {
    EXPECT_FLOAT_EQ(simInfo.models[i].transform.pos.x, i);
    EXPECT_FLOAT_EQ(simInfo.models[i].transform.pos.y, i + 0.1);
    EXPECT_FLOAT_EQ(simInfo.models[i].transform.pos.z, i + 0.2);
    EXPECT_FLOAT_EQ(simInfo.models[i].transform.orient.w, i + 0.3);
    EXPECT_FLOAT_EQ(simInfo.models[i].transform.orient.x, i + 0.4);
    EXPECT_FLOAT_EQ(simInfo.models[i].transform.orient.y, i + 0.5);
    EXPECT_FLOAT_EQ(simInfo.models[i].transform.orient.z, i + 0.6);
    EXPECT_FLOAT_EQ(simInfo.models[i].is_static, 1);
    EXPECT_FLOAT_EQ(simInfo.models[i].id, i);
    ASSERT_EQ(simInfo.models[i].link_count, kNumLinksPerModel);
    for (int j = 0; j < simInfo.models[i].link_count; ++j)
    {
      float v = 10 * i + j;
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].transform.pos.x, v);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].transform.pos.y, v + 0.1);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].transform.pos.z, v + 0.2);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].transform.orient.w, v + 0.3);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].transform.orient.x, v + 0.4);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].transform.orient.y, v + 0.5);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].transform.orient.z, v + 0.6);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].linvel.x, v + 0.7);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].linvel.y, v + 0.8);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].linvel.z, v + 0.9);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].angvel.x, v + 1);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].angvel.y, v + 1.1);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].angvel.z, v + 1.2);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].linacc.x, v + 1.3);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].linacc.y, v + 1.4);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].linacc.z, v + 1.5);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].angacc.x, v + 1.6);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].angacc.y, v + 1.7);
      EXPECT_FLOAT_EQ(simInfo.models[i].links[j].angacc.z, v + 1.8);
    }
    ASSERT_EQ(simInfo.models[i].joint_count, kNumJointsPerModel);
    for (int j = 0; j < simInfo.models[i].joint_count; ++j)
    {
      float v = 20 * i + j;
      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].pos, v);
      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].vel, v + 0.1);
      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].acc, v + 0.2);
      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].torque_motor, v + 0.3);
      EXPECT_FLOAT_EQ(simInfo.models[i].joints[j].torque_passive, v + 0.4);
    }
  }

  // Check the camera information.
  EXPECT_FLOAT_EQ(simInfo.camera.transform.pos.x, 30.0);
  EXPECT_FLOAT_EQ(simInfo.camera.transform.pos.y, 30.1);
  EXPECT_FLOAT_EQ(simInfo.camera.transform.pos.z, 30.2);
  EXPECT_FLOAT_EQ(simInfo.camera.transform.orient.w, 30.3);
  EXPECT_FLOAT_EQ(simInfo.camera.transform.orient.x, 30.4);
  EXPECT_FLOAT_EQ(simInfo.camera.transform.orient.y, 30.5);
  EXPECT_FLOAT_EQ(simInfo.camera.transform.orient.z, 30.6);
}

//////////////////////////////////////////////////
/// \brief Check hxs_camera.
TEST(hxsTest, hxs_camera)
{
  setup();

  ignition::transport::Node node;
  hxCamera camInfo;

  // Advertise the "hxs_camera" service.
  node.Advertise("/haptix/gazebo/hxs_camera", onHxsCamera);

  // Request camera information.
  ASSERT_EQ(hxs_camera(&camInfo), hxOK);

  // Check the camera information.
  EXPECT_FLOAT_EQ(camInfo.transform.pos.x, 30.0);
  EXPECT_FLOAT_EQ(camInfo.transform.pos.y, 30.1);
  EXPECT_FLOAT_EQ(camInfo.transform.pos.z, 30.2);
  EXPECT_FLOAT_EQ(camInfo.transform.orient.w, 30.3);
  EXPECT_FLOAT_EQ(camInfo.transform.orient.x, 30.4);
  EXPECT_FLOAT_EQ(camInfo.transform.orient.y, 30.5);
  EXPECT_FLOAT_EQ(camInfo.transform.orient.z, 30.6);
}

//////////////////////////////////////////////////
/// \brief Check hxs_camera_transform.
TEST(hxsTest, hxs_camera_transform)
{
  setup();

  ignition::transport::Node node;
  hxTransform newTransform;

  // Advertise the "hxs_camera_transform" service.
  node.Advertise("/haptix/gazebo/hxs_camera_transform", onHxsCameraTransform);

  // Set a camera transformation.
  newTransform.pos.x = 40.0;
  newTransform.pos.y = 40.1;
  newTransform.pos.z = 40.2;
  newTransform.orient.w = 40.3;
  newTransform.orient.x = 40.4;
  newTransform.orient.y = 40.5;
  newTransform.orient.z = 40.6;
  ASSERT_EQ(hxs_camera_transform(newTransform), hxOK);

  // Verify that the camera transformation has been applied.
  EXPECT_FLOAT_EQ(simState.camera().transform().pos().x(), 40.0);
  EXPECT_FLOAT_EQ(simState.camera().transform().pos().y(), 40.1);
  EXPECT_FLOAT_EQ(simState.camera().transform().pos().z(), 40.2);
  EXPECT_FLOAT_EQ(simState.camera().transform().orient().w(), 40.3);
  EXPECT_FLOAT_EQ(simState.camera().transform().orient().x(), 40.4);
  EXPECT_FLOAT_EQ(simState.camera().transform().orient().y(), 40.5);
  EXPECT_FLOAT_EQ(simState.camera().transform().orient().z(), 40.6);
}

//////////////////////////////////////////////////
/// \brief Check hxs_contacts.
TEST(hxsTest, hxs_contacts)
{
  setup();

  ignition::transport::Node node;
  hxContact contactsInfo;

  // Advertise the "hxs_contacts" service.
  node.Advertise("/haptix/gazebo/hxs_contacts", onHxsContacts);

  ASSERT_EQ(hxs_contacts(&contactsInfo), hxOK);

  // Check the contacts information.
  ASSERT_EQ(contactsInfo.contactCount, kNumContacts);
  for (int i = 0; i < contactsInfo.contactCount; ++i)
  {
    EXPECT_EQ(contactsInfo.body1[i], i);
    EXPECT_EQ(contactsInfo.body2[i], i + 1);
    EXPECT_FLOAT_EQ(contactsInfo.point[i].x, i + 0.2);
    EXPECT_FLOAT_EQ(contactsInfo.point[i].y, i + 0.3);
    EXPECT_FLOAT_EQ(contactsInfo.point[i].z, i + 0.4);
    EXPECT_FLOAT_EQ(contactsInfo.normal[i].x, i + 0.5);
    EXPECT_FLOAT_EQ(contactsInfo.normal[i].y, i + 0.6);
    EXPECT_FLOAT_EQ(contactsInfo.normal[i].z, i + 0.7);
    EXPECT_FLOAT_EQ(contactsInfo.tangent1[i].x, i + 0.8);
    EXPECT_FLOAT_EQ(contactsInfo.tangent1[i].y, i + 0.9);
    EXPECT_FLOAT_EQ(contactsInfo.tangent1[i].z, i + 1);
    EXPECT_FLOAT_EQ(contactsInfo.tangent2[i].x, i + 1.1);
    EXPECT_FLOAT_EQ(contactsInfo.tangent2[i].y, i + 1.2);
    EXPECT_FLOAT_EQ(contactsInfo.tangent2[i].z, i + 1.3);
    EXPECT_FLOAT_EQ(contactsInfo.distance[i], i + 1.4);
    EXPECT_FLOAT_EQ(contactsInfo.velocity[i].x, i + 1.5);
    EXPECT_FLOAT_EQ(contactsInfo.velocity[i].y, i + 1.6);
    EXPECT_FLOAT_EQ(contactsInfo.velocity[i].z, i + 1.7);
    EXPECT_FLOAT_EQ(contactsInfo.force[i].x, i + 1.8);
    EXPECT_FLOAT_EQ(contactsInfo.force[i].y, i + 1.9);
    EXPECT_FLOAT_EQ(contactsInfo.force[i].z, i + 2);
  }
}

//////////////////////////////////////////////////
/// \brief Check hxs_state.
TEST(hxsTest, hxs_state)
{
  setup();

  ignition::transport::Node node;
  hxModel model;
  hxJoint joint;

  // Advertise the "hxs_camera_state" service.
  node.Advertise("/haptix/gazebo/hxs_state", onHxsState);

  // Set a new state.
  // ToDo.
  ASSERT_EQ(hxs_state(&model, &joint), hxOK);

  // Verify that the new state has been set.
  // ToDo.
}

//////////////////////////////////////////////////
/// \brief Check that we can use the C-wrapper.
/*TEST(CommTest, BasicUsage)
{
  ignition::transport::Node node;

  // Advertise the "getdeviceinfo" service.
  node.Advertise(RobotInfoTopic, onGetRobotInfo);

  // Advertise the "update" service.
  node.Advertise(UpdateTopic, onUpdate);

  // Advertise the "read" service.
  node.Advertise(ReadTopic, onRead);

  EXPECT_EQ(hx_connect(NULL, 0), hxOK);

  hxRobotInfo robotInfo;
  ASSERT_EQ(hx_robot_info(&robotInfo), hxOK);

  ASSERT_EQ(robotInfo.motor_count, NumMotors);
  ASSERT_EQ(robotInfo.joint_count, NumJoints);
  ASSERT_EQ(robotInfo.contact_sensor_count, NumContactSensors);
  ASSERT_EQ(robotInfo.imu_count, NumIMUs);

  hxCommand cmd;
  hxSensor sensor;

  // Fill the joint command.
  for (int i = 0; i < robotInfo.motor_count; ++i)
  {
    cmd.ref_pos[i] = i;
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
}*/
