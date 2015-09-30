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

#include <chrono>
#include <string>
#include <stdio.h>
#include <ignition/transport.hh>
#include "gtest/gtest.h"
#include "haptix/comm/haptix.h"
#include "haptix/comm/haptix_sim.h"
#include "msg/hxCommand.pb.h"
#include "msg/hxContactPoint.pb.h"
#include "msg/hxContactPoint_V.pb.h"
#include "msg/hxEmpty.pb.h"
#include "msg/hxInt.pb.h"
#include "msg/hxParam.pb.h"
#include "msg/hxRobot.pb.h"
#include "msg/hxSensor.pb.h"
#include "msg/hxSimInfo.pb.h"
#include "msg/hxString.pb.h"
#include "msg/hxTime.pb.h"
#include "msg/hxTransform.pb.h"
#include "test_config.h"

/// \brief Global constants.
const int kNumModels          = 5;
const int kNumLinksPerModel   = 40;
const int kNumJointsPerModel  = 20;
const int kNumContactPoints   = 30;
const int kNumMotors          = 4;
const int kNumJoints          = 5;
const int kNumContactSensors  = 6;
const int kNumIMUs            = 7;
const float kUpdateRate       = 8.0f;
const unsigned int kTime_sec  = 9u;
const unsigned int kTime_nsec = 10u;

const std::string kDeviceInfoTopic = "/haptix/gazebo/GetRobotInfo";
const std::string kUpdateTopic     = "/haptix/gazebo/Update";

/// \brief Global variables.
haptix::comm::msgs::hxSimInfo simState;
haptix::comm::msgs::hxContactPoint_V simContactPointsState;

//////////////////////////////////////////////////
/// \brief Populate a simulation state and a simulation contact points variables
void setup()
{
  simState.Clear();
  simContactPointsState.Clear();

  // Create a fake simulation state.
  // Models.
  for (int i = 0; i < kNumModels; ++i)
  {
    haptix::comm::msgs::hxModel *model = simState.add_models();
    model->set_name("model " + std::to_string(i));
    model->mutable_transform()->mutable_pos()->set_x(i);
    model->mutable_transform()->mutable_pos()->set_y(i + 0.1f);
    model->mutable_transform()->mutable_pos()->set_z(i + 0.2f);
    model->mutable_transform()->mutable_orient()->set_w(i + 0.3f);
    model->mutable_transform()->mutable_orient()->set_x(i + 0.4f);
    model->mutable_transform()->mutable_orient()->set_y(i + 0.5f);
    model->mutable_transform()->mutable_orient()->set_z(i + 0.6f);
    // Links.
    for (int j = 0; j < kNumLinksPerModel; ++j)
    {
      float v = 10 * i + j;
      haptix::comm::msgs::hxLink *link = model->add_links();
      link->set_name("link " + std::to_string(j));
      link->mutable_transform()->mutable_pos()->set_x(v);
      link->mutable_transform()->mutable_pos()->set_y(v + 0.1f);
      link->mutable_transform()->mutable_pos()->set_z(v + 0.2f);
      link->mutable_transform()->mutable_orient()->set_w(v + 0.3f);
      link->mutable_transform()->mutable_orient()->set_x(v + 0.4f);
      link->mutable_transform()->mutable_orient()->set_y(v + 0.5f);
      link->mutable_transform()->mutable_orient()->set_z(v + 0.6f);
      link->mutable_lin_vel()->set_x(v + 0.7f);
      link->mutable_lin_vel()->set_y(v + 0.8f);
      link->mutable_lin_vel()->set_z(v + 0.9f);
      link->mutable_ang_vel()->set_x(v + 1.0f);
      link->mutable_ang_vel()->set_y(v + 1.1f);
      link->mutable_ang_vel()->set_z(v + 1.2f);
      link->mutable_lin_acc()->set_x(v + 1.3f);
      link->mutable_lin_acc()->set_y(v + 1.4f);
      link->mutable_lin_acc()->set_z(v + 1.5f);
      link->mutable_ang_acc()->set_x(v + 1.6f);
      link->mutable_ang_acc()->set_y(v + 1.7f);
      link->mutable_ang_acc()->set_z(v + 1.8f);
    }
    // Joints.
    for (int j = 0; j < kNumJointsPerModel; ++j)
    {
      float v = 20 * i + j;
      haptix::comm::msgs::hxJoint *joint = model->add_joints();
      joint->set_name("joint " + std::to_string(j));
      joint->set_pos(v);
      joint->set_vel(v + 0.1f);
      joint->set_torque_motor(v + 0.3f);
      joint->mutable_wrench_reactive()->mutable_force()->set_x(v+0.4f);
      joint->mutable_wrench_reactive()->mutable_force()->set_y(v+0.5f);
      joint->mutable_wrench_reactive()->mutable_force()->set_z(v+0.6f);
      joint->mutable_wrench_reactive()->mutable_torque()->set_x(v+0.7f);
      joint->mutable_wrench_reactive()->mutable_torque()->set_y(v+0.8f);
      joint->mutable_wrench_reactive()->mutable_torque()->set_z(v+0.9f);
    }

    model->set_gravity_mode(true);
  }

  // Camera.
  simState.mutable_camera_transform()->mutable_pos()->set_x(30.0f);
  simState.mutable_camera_transform()->mutable_pos()->set_y(30.1f);
  simState.mutable_camera_transform()->mutable_pos()->set_z(30.2f);
  simState.mutable_camera_transform()->mutable_orient()->set_w(30.3f);
  simState.mutable_camera_transform()->mutable_orient()->set_x(30.4f);
  simState.mutable_camera_transform()->mutable_orient()->set_y(30.5f);
  simState.mutable_camera_transform()->mutable_orient()->set_z(30.6f);

  // Create some contacts.
  for (int i = 0; i < kNumContactPoints; ++i)
  {
    haptix::comm::msgs::hxContactPoint *contact =
      simContactPointsState.add_contacts();
    contact->set_link1(("link " + std::to_string(i)).c_str());
    contact->set_link2(("link " + std::to_string(i + 1)).c_str());
    contact->mutable_point()->set_x(i + 0.2f);
    contact->mutable_point()->set_y(i + 0.3f);
    contact->mutable_point()->set_z(i + 0.4f);
    contact->mutable_normal()->set_x(i + 0.5f);
    contact->mutable_normal()->set_y(i + 0.6f);
    contact->mutable_normal()->set_z(i + 0.7f);
    contact->set_distance(i + 1.4f);
    contact->mutable_wrench()->mutable_force()->set_x(i + 1.8f);
    contact->mutable_wrench()->mutable_force()->set_y(i + 1.9f);
    contact->mutable_wrench()->mutable_force()->set_z(i + 2.0f);
    contact->mutable_wrench()->mutable_torque()->set_x(i + 2.1f);
    contact->mutable_wrench()->mutable_torque()->set_y(i + 2.2f);
    contact->mutable_wrench()->mutable_torque()->set_z(i + 2.3f);
  }
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_sim_info" service.
void onHxsSimInfo(const std::string &_service,
  const haptix::comm::msgs::hxEmpty &/*_req*/,
  haptix::comm::msgs::hxSimInfo &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_sim_info");

  // Create some dummy response.
  _rep = simState;

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_camera_transform" service.
void onHxsCamera(const std::string &_service,
  const haptix::comm::msgs::hxEmpty &/*_req*/,
  haptix::comm::msgs::hxTransform &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_camera_transform");

  // Create some dummy response.
  _rep = simState.camera_transform();

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_set_camera_transform" service.
void onHxsCameraTransform(const std::string &_service,
  const haptix::comm::msgs::hxTransform &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_set_camera_transform");

  std::string msg1, msg2;

  _req.SerializeToString(&msg1);
  simState.camera_transform().SerializeToString(&msg2);
  EXPECT_EQ(msg1, msg2);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_contacts" service.
void onHxsContactPoints(const std::string &_service,
  const haptix::comm::msgs::hxString &/*_req*/,
  haptix::comm::msgs::hxContactPoint_V &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_contacts");

  // Create some dummy response.
  _rep = simContactPointsState;

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_model_joint_state" service.
void onHxsJointState(const std::string &_service,
  const haptix::comm::msgs::hxString &_req,
  haptix::comm::msgs::hxModel &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_model_joint_state");

  // Verify the request.
  EXPECT_EQ(_req.data(), "model 0");

  // Create a response.
  _rep.set_name(_req.data());

  _rep.add_joints();
  _rep.mutable_joints(0)->set_pos(3.0);
  _rep.mutable_joints(0)->set_vel(4.0);

  // set required fields
  _rep.mutable_joints(0)->set_name("joint 1");
  _rep.mutable_joints(0)->set_torque_motor(5.0);
  // w.mutable_force()->set_force(v);
  // w.mutable_force()->set_torque(v);
  haptix::comm::msgs::hxWrench *w =
    _rep.mutable_joints(0)->mutable_wrench_reactive();
  haptix::comm::msgs::hxVector3 *f = w->mutable_force();
  haptix::comm::msgs::hxVector3 *t = w->mutable_torque();
  f->set_x(11);
  f->set_y(21);
  f->set_z(31);
  t->set_x(12);
  t->set_y(22);
  t->set_z(32);
  haptix::comm::msgs::hxTransform *tf =
    _rep.mutable_transform();
  haptix::comm::msgs::hxVector3 *tfv = tf->mutable_pos();
  haptix::comm::msgs::hxQuaternion *tfq = tf->mutable_orient();
  tfv->set_x(13);
  tfv->set_y(23);
  tfv->set_z(33);
  tfq->set_w(14);
  tfq->set_x(24);
  tfq->set_y(34);
  tfq->set_z(44);
  _rep.set_gravity_mode(0);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_set_model_joint_state" service.
void onHxsSetJointState(const std::string &_service,
  const haptix::comm::msgs::hxModel &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_set_model_joint_state");

  // Verify the request.
  EXPECT_EQ(_req.name(), "model 0");

  EXPECT_EQ(_req.joints_size(), 1);

  EXPECT_EQ(_req.joints(0).name(), "joint 1");
  EXPECT_FLOAT_EQ(_req.joints(0).pos(), 1.0f);
  EXPECT_FLOAT_EQ(_req.joints(0).vel(), 2.0f);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_add_model" service.
void onHxsAddModel(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxModel &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_add_model");

  // Sanity check: The message should contain a string with the urdf.
  if (!_req.has_string_value())
  {
    std::cerr << "onHxsAddModel() error: Missing urdf in request" << std::endl;
    return;
  }

  // Sanity check: The message should contain a string with the model name.
  if (!_req.has_name())
  {
    std::cerr << "onHxsAddModel() error: Missing model name in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a position.
  if (!_req.has_vector3())
  {
    std::cerr << "onHxsAddModel() error: Missing vector3 in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain an orientation.
  if (!_req.has_orientation())
  {
    std::cerr << "onHxsAddModel() error: Missing orientation in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a gravity_mode field.
  if (!_req.has_gravity_mode())
  {
    std::cerr << "onHxsAddModel() error: Missing gravity_mode in request"
              << std::endl;
    return;
  }

  // Verify the request.
  EXPECT_EQ(_req.string_value(), "fake URDF");
  EXPECT_EQ(_req.name(), "model 1");
  EXPECT_FLOAT_EQ(_req.vector3().x(), 1.0f);
  EXPECT_FLOAT_EQ(_req.vector3().y(), 2.0f);
  EXPECT_FLOAT_EQ(_req.vector3().z(), 3.0f);
  EXPECT_FLOAT_EQ(_req.orientation().roll(), 4.0f);
  EXPECT_FLOAT_EQ(_req.orientation().pitch(), 5.0f);
  EXPECT_FLOAT_EQ(_req.orientation().yaw(), 6.0f);
  EXPECT_TRUE(_req.gravity_mode());

  // Return the first model of simState as an answer.
  _rep = simState.models(0);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_remove_model" service.
void onHxsRemoveModel(const std::string &_service,
  const haptix::comm::msgs::hxString &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_remove_model");

  // Verify the request.
  EXPECT_EQ(_req.data(), "model 1");

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_set_model_transform" service.
void onHxsSetModelTransform(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_set_model_transform");

  // Sanity check: The message should contain a name.
  if (!_req.has_name())
  {
    std::cerr << "onHxsSetModelTransform() error: Missing name in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a transform.
  if (!_req.has_transform())
  {
    std::cerr << "onHxsSetModelTransform() error: Missing transform in request"
              << std::endl;
    return;
  }

  // Verify the name.
  EXPECT_EQ(_req.name(), "model 1");

  // Verify the transform.
  std::string msg1, msg2;
  _req.transform().SerializeToString(&msg1);
  simState.models(1).transform().SerializeToString(&msg2);
  EXPECT_EQ(msg1, msg2);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_model_transform" service.
void onHxsModelTransform(const std::string &_service,
  const haptix::comm::msgs::hxString &_req,
  haptix::comm::msgs::hxTransform &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_model_transform");

  // Verify the name.
  EXPECT_EQ(_req.data(), "model 1");

  // Return the transform from the second model.
  _rep = simState.models(1).transform();

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_linear_velocity" service.
void onHxsLinearVelocity(const std::string &_service,
  const haptix::comm::msgs::hxString &_req,
  haptix::comm::msgs::hxVector3 &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_linear_velocity");

  // Verify the name.
  EXPECT_EQ(_req.data(), "model 1");

  _rep.set_x(1.0f);
  _rep.set_y(1.1f);
  _rep.set_z(1.2f);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_set_linear_velocity" service.
void onHxsSetLinearVelocity(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_set_linear_velocity");

  // Sanity check: The message should contain a name.
  if (!_req.has_name())
  {
    std::cerr << "onHxsLinearVelocity() error: Missing name in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a vector3.
  if (!_req.has_vector3())
  {
    std::cerr << "onHxsLinearVelocity() error: Missing lin_vel in request"
              << std::endl;
    return;
  }

  // Verify the name.
  EXPECT_EQ(_req.name(), "model 1");

  // Verify the lin_vel.
  EXPECT_FLOAT_EQ(_req.vector3().x(), 1.0f);
  EXPECT_FLOAT_EQ(_req.vector3().y(), 1.1f);
  EXPECT_FLOAT_EQ(_req.vector3().z(), 1.2f);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_angular_velocity" service.
void onHxsAngularVelocity(const std::string &_service,
  const haptix::comm::msgs::hxString &_req,
  haptix::comm::msgs::hxVector3 &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_angular_velocity");

  // Verify the name.
  EXPECT_EQ(_req.data(), "model 1");

  _rep.set_x(2.0f);
  _rep.set_y(2.1f);
  _rep.set_z(2.2f);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_set_angular_velocity" service.
void onHxsSetAngularVelocity(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_set_angular_velocity");

  // Sanity check: The message should contain a name.
  if (!_req.has_name())
  {
    std::cerr << "onHxsSetAngularVelocity() error: Missing name in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a vector3.
  if (!_req.has_vector3())
  {
    std::cerr << "onHxsSetAngularVelocity() error: Missing ang_vel in request"
              << std::endl;
    return;
  }

  // Verify the name.
  EXPECT_EQ(_req.name(), "model 1");

  // Verify the ang_vel.
  EXPECT_FLOAT_EQ(_req.vector3().x(), 2.0f);
  EXPECT_FLOAT_EQ(_req.vector3().y(), 2.1f);
  EXPECT_FLOAT_EQ(_req.vector3().z(), 2.2f);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_apply_force" service.
void onHxsApplyForce(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_apply_force");

  // Sanity check: The message should contain a model name.
  if (!_req.has_name())
  {
    std::cerr << "onHxsApplyForce() error: Missing name in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a link name.
  if (!_req.has_string_value())
  {
    std::cerr << "onHxsApplyForce() error: Missing link name in request"
              << std::endl;
    return;
  }

  if (!_req.has_float_value())
  {
    std::cerr << "onHxsApplyForce() error: Missing duration in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a vector3.
  if (!_req.has_vector3())
  {
    std::cerr << "onHxsApplyForce() error: Missing force in request"
              << std::endl;
    return;
  }

  // Verify the link received.
  EXPECT_EQ(std::string(simState.models(0).name()), _req.name());
  EXPECT_EQ(std::string(simState.models(0).links(0).name()),
    _req.string_value());

  // Verify the duration received.
  EXPECT_FLOAT_EQ(_req.float_value(), 0.1f);

  // Verify the force vector received.
  EXPECT_FLOAT_EQ(_req.vector3().x(), 5.1f);
  EXPECT_FLOAT_EQ(_req.vector3().y(), 5.2f);
  EXPECT_FLOAT_EQ(_req.vector3().z(), 5.3f);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_apply_torque" service.
void onHxsApplyTorque(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_apply_torque");

  // Sanity check: The message should contain a model name.
  if (!_req.has_name())
  {
    std::cerr << "onHxsApplyTorque() error: Missing name in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a link name.
  if (!_req.has_string_value())
  {
    std::cerr << "onHxsApplyTorque() error: Missing link name in request"
              << std::endl;
    return;
  }

  if (!_req.has_float_value())
  {
    std::cerr << "onHxsApplyTorque() error: Missing duration in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a vector3.
  if (!_req.has_vector3())
  {
    std::cerr << "onHxsApplyTorque() error: Missing torque in request"
              << std::endl;
    return;
  }

  // Verify the link received.
  EXPECT_EQ(std::string(simState.models(0).name()), _req.name());
  EXPECT_EQ(std::string(simState.models(0).links(0).name()),
    _req.string_value());

  // Verify the duration received.
  EXPECT_FLOAT_EQ(_req.float_value(), 0.1f);

  // Verify the torque vector received.
  EXPECT_FLOAT_EQ(_req.vector3().x(), 6.1f);
  EXPECT_FLOAT_EQ(_req.vector3().y(), 6.2f);
  EXPECT_FLOAT_EQ(_req.vector3().z(), 6.3f);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_apply_wrench" service.
void onHxsApplyWrench(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_apply_wrench");

  // Sanity check: The message should contain a model name.
  if (!_req.has_name())
  {
    std::cerr << "onHxsApplyWrench() error: Missing name in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a link name.
  if (!_req.has_string_value())
  {
    std::cerr << "onHxsApplyWrench() error: Missing link name in request"
              << std::endl;
    return;
  }

  if (!_req.has_float_value())
  {
    std::cerr << "onHxsApplyWrench() error: Missing duration in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a wrench.
  if (!_req.has_wrench())
  {
    std::cerr << "onHxsApplyWrench() error: Missing wrench in request"
              << std::endl;
    return;
  }

  // Verify the link received.
  EXPECT_EQ(std::string(simState.models(0).name()), _req.name());
  EXPECT_EQ(std::string(simState.models(0).links(0).name()),
    _req.string_value());

  // Verify the duration received.
  EXPECT_FLOAT_EQ(_req.float_value(), 0.1f);

  // Verify the torque vector received.
  EXPECT_FLOAT_EQ(_req.wrench().force().x(), 7.1f);
  EXPECT_FLOAT_EQ(_req.wrench().force().y(), 7.2f);
  EXPECT_FLOAT_EQ(_req.wrench().force().z(), 7.3f);
  EXPECT_FLOAT_EQ(_req.wrench().torque().x(), 7.4f);
  EXPECT_FLOAT_EQ(_req.wrench().torque().y(), 7.5f);
  EXPECT_FLOAT_EQ(_req.wrench().torque().z(), 7.6f);

  _result = true;
}


//////////////////////////////////////////////////
/// \brief Provide a "hxs_reset" service.
void onHxsReset(const std::string &_service,
  const haptix::comm::msgs::hxInt &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_reset");

  // Verify the value received.
  EXPECT_EQ(_req.data(), 1);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_start_logging" service.
void onHxsStartLogging(const std::string &_service,
  const haptix::comm::msgs::hxString &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_start_logging");

  // Check the filename received.
  EXPECT_EQ(_req.data(), "a filename");

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_is_logging" service.
void onHxsIsLogging(const std::string &_service,
  const haptix::comm::msgs::hxEmpty &/*_req*/,
  haptix::comm::msgs::hxInt &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_is_logging");

  // Set the response.
  _rep.set_data(1);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_stop_logging" service.
void onHxsStopLogging(const std::string &_service,
  const haptix::comm::msgs::hxEmpty &/*_req*/,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_stop_logging");

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_model_gravity_mode" service.
void onHxsModelGravity(const std::string &_service,
  const haptix::comm::msgs::hxString &_req,
  haptix::comm::msgs::hxInt &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_model_gravity_mode");

  EXPECT_EQ(_req.data(), "model_1");
  _rep.set_data(1);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_set_gravity_mode" service.
void onHxsSetModelGravity(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &/*_rep*/,
  bool &_result)
{
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_set_model_gravity_mode");

  // Sanity check: The message should contain a model name.
  if (!_req.has_name())
  {
    std::cerr << "onHxsSetModelGravity() error: Missing name in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a gravity_mode flag.
  if (!_req.has_gravity_mode())
  {
    std::cerr << "onHxsSetModelGravity() error: Missing gravity_mode in request"
              << std::endl;
    return;
  }

  EXPECT_TRUE(_req.has_name());
  EXPECT_TRUE(_req.has_gravity_mode());
  EXPECT_EQ(_req.name(), "model_1");
  EXPECT_EQ(_req.gravity_mode(), 1u);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_set_model_color" service.
void onHxsSetModelColor(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &/*_rep*/,
  bool &_result)
{
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_set_model_color");

  // Sanity check: The message should contain a model name.
  if (!_req.has_name())
  {
    std::cerr << "onHxsSetModelColor() error: Missing name in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a color.
  if (!_req.has_color())
  {
    std::cerr << "onHxsSetModelColor() error: Missing color in request"
              << std::endl;
    return;
  }

  EXPECT_EQ(_req.name(), "model_1");
  EXPECT_FLOAT_EQ(_req.color().r(), 0.5f);
  EXPECT_FLOAT_EQ(_req.color().g(), 0.6f);
  EXPECT_FLOAT_EQ(_req.color().b(), 0.7f);
  EXPECT_FLOAT_EQ(_req.color().alpha(), 0.8f);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_model_color" service.
void onHxsModelColor(const std::string &_service,
  const haptix::comm::msgs::hxString &_req,
  haptix::comm::msgs::hxColor &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_model_color");

  EXPECT_EQ(_req.data(), "model_1");

  _rep.set_r(0.1f);
  _rep.set_g(0.2f);
  _rep.set_b(0.3f);
  _rep.set_alpha(0.4f);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_set_model_collide_mode" service.
void onHxsSetModelCollideMode(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &/*_rep*/,
  bool &_result)
{
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_set_model_collide_mode");

  // Sanity check: The message should contain a model name.
  if (!_req.has_name())
  {
    std::cerr << "onHxsSetModelCollideMode() error: Missing name in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a collide mode.
  if (!_req.has_collide_mode())
  {
    std::cerr << "onHxsSetModelCollideMode() error: Missing collide mode in "
              << "request" << std::endl;
    return;
  }

  EXPECT_EQ(_req.name(), "model_1");
  EXPECT_EQ(_req.collide_mode().mode(),
    haptix::comm::msgs::hxCollideMode::hxsCOLLIDE);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_model_collide_mode" service.
void onHxsModelCollideMode(const std::string &_service,
  const haptix::comm::msgs::hxString &_req,
  haptix::comm::msgs::hxCollideMode &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_model_collide_mode");

  EXPECT_EQ(_req.data(), "model_1");

  _rep.set_mode(haptix::comm::msgs::hxCollideMode::hxsDETECTIONONLY);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_add_constraint" service.
void onHxsAddConstraint(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &/*_rep*/,
  bool &_result)
{
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_add_constraint");

  // Sanity check: The message should contain a string with the sdf.
  if (!_req.has_string_value())
  {
    std::cerr << "onHxsAddConstraint() error: Missing "
              << " constraint sdf in request" << std::endl;
    return;
  }

  // Sanity check: The message should contain a string with the model name.
  if (!_req.has_name())
  {
    std::cerr << "onHxsAddConstraint() error: Missing model name in request"
              << std::endl;
    return;
  }

  std::string constraint_sdf =
    "<sdf version=\"1.5\">"
    "  <joint name=\"cricket_ball_constraint\" type=\"revolute\">"
    "    <parent>world</parent>"
    "    <child>green_cricket_ball</child>"
    "    <axis>"
    "      <xyz>0 1 0</xyz>"
    "    </axis>"
    "  </joint>"
    "</sdf>";

  // Verify the request.
  EXPECT_EQ(_req.string_value(), constraint_sdf);
  EXPECT_EQ(_req.name(), "model 1");

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_remove_constraint" service.
void onHxsRemoveConstraint(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &/*_rep*/,
  bool &_result)
{
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_remove_constraint");

  // Sanity check: The message should contain a constraint/joint name.
  if (!_req.has_string_value())
  {
    std::cerr << "onHxsAddConstraint() error: Missing "
              << " constraint name in request" << std::endl;
    return;
  }

  // Sanity check: The message should contain a string with the model name.
  if (!_req.has_name())
  {
    std::cerr << "onHxsAddConstraint() error: Missing model name in request"
              << std::endl;
    return;
  }

  // Verify the request.
  EXPECT_EQ(_req.string_value(), "cricket_ball_constraint");
  EXPECT_EQ(_req.name(), "model 1");

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a service.
void onGetRobotInfo(const std::string &_service,
  const haptix::comm::msgs::hxRobot &/*_req*/,
  haptix::comm::msgs::hxRobot &_rep, bool &_result)
{
  _result = true;

  if (_service != "/haptix/gazebo/GetRobotInfo")
    _result = false;

  _rep.set_motor_count(kNumMotors);
  _rep.set_joint_count(kNumJoints);
  _rep.set_contact_sensor_count(kNumContactSensors);
  _rep.set_imu_count(kNumIMUs);
  _rep.set_update_rate(kUpdateRate);

  for (int i = 0; i < kNumJoints; ++i)
  {
    haptix::comm::msgs::hxRobot::hxLimit *joint = _rep.add_joint_limit();
    joint->set_minimum(-i);
    joint->set_maximum(i);
  }

  for (int i = 0; i < kNumMotors; ++i)
  {
    haptix::comm::msgs::hxRobot::hxLimit *motor = _rep.add_motor_limit();
    motor->set_minimum(-i);
    motor->set_maximum(i);
  }
}

//////////////////////////////////////////////////
/// \brief Provide an "Update" service.
void onUpdate(const std::string &_service,
  const haptix::comm::msgs::hxCommand &/*_req*/,
  haptix::comm::msgs::hxSensor &_rep, bool &_result)
{
  _result = true;

  if (_service != "/haptix/gazebo/Update")
    _result = false;

  // Read the request parameters.
  // Debug output.
  /*std::cout << "Received a new motor command:" << std::endl;
  for (int i = 0; i < numMotors; ++i)
  {
    std::cout << "\tMotor " << i << ":" << std::endl;
    std::cout << "\t\t" << _req.ref_pos(i) << std::endl;
    std::cout << "\t\t" << _req.ref_vel(i) << std::endl;
    std::cout << "\t\t" << _req.gain_pos(i) << std::endl;
    std::cout << "\t\t" << _req.gain_vel(i) << std::endl;
  }*/

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
    haptix::comm::msgs::quaternion *orientation = _rep.add_imu_orientation();
    orientation->set_x(i + 6);
    orientation->set_y(i + 7);
    orientation->set_z(i + 8);
    orientation->set_w(i + 9);
  }

  _rep.mutable_time_stamp()->set_sec(kTime_sec);
  _rep.mutable_time_stamp()->set_nsec(kTime_nsec);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Sanity check: Make sure that a partition name is passed in as an argument.
  if (argc != 2)
  {
    std::cerr << "Partition name has not be passed as argument" << std::endl;
    return -1;
  }

  // Set the partition name for this test.
  setenv("IGN_PARTITION", argv[1], 1);
  // Max lifetime of the program.
  int time = 30000;
  // Initialize the test by creating an initial state.
  setup();

  // ---------- HAPTIX API ----------
  ignition::transport::Node node;

  // Advertise the "getdeviceinfo" service.
  std::string service = "/haptix/gazebo/GetRobotInfo";
  if (!node.Advertise(service, onGetRobotInfo))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "update" service.
  service = "/haptix/gazebo/Update";
  if (!node.Advertise(service, onUpdate))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // ---------- HAPTIX SIM API ----------
  hxsSimInfo *simInfo = new hxsSimInfo();

  // Advertise the "hxs_sim_info" service.
  service = "/haptix/gazebo/hxs_sim_info";
  if (!node.Advertise(service, onHxsSimInfo))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_camera_transform" service.
  service = "/haptix/gazebo/hxs_camera_transform";
  if (!node.Advertise(service, onHxsCamera))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_set_camera_transform" service.
  service = "/haptix/gazebo/hxs_set_camera_transform";
  if (!node.Advertise(service, onHxsCameraTransform))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_contacts" service.
  service = "/haptix/gazebo/hxs_contacts";
  if (!node.Advertise(service, onHxsContactPoints))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_model_joint_state" service.
  service = "/haptix/gazebo/hxs_model_joint_state";
  if (!node.Advertise(service, onHxsJointState))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_set_model_joint_state" service.
  service = "/haptix/gazebo/hxs_set_model_joint_state";
  if (!node.Advertise(service, onHxsSetJointState))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_add_model" service.
  service = "/haptix/gazebo/hxs_add_model";
  if (!node.Advertise(service, onHxsAddModel))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_remove_model" service.
  service = "/haptix/gazebo/hxs_remove_model";
  if (!node.Advertise(service, onHxsRemoveModel))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_set_model_transform" service.
  service = "/haptix/gazebo/hxs_set_model_transform";
  if (!node.Advertise(service, onHxsSetModelTransform))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_model_transform" service.
  service = "/haptix/gazebo/hxs_model_transform";
  if (!node.Advertise(service, onHxsModelTransform))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_model_gravity_mode" service.
  service = "/haptix/gazebo/hxs_model_gravity_mode";
  if (!node.Advertise(service, onHxsModelGravity))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_set_model_gravity_mode" service.
  service = "/haptix/gazebo/hxs_set_model_gravity_mode";
  if (!node.Advertise(service, onHxsSetModelGravity))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_linear_velocity" service.
  service = "/haptix/gazebo/hxs_linear_velocity";
  if (!node.Advertise(service, onHxsLinearVelocity))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_set_linear_velocity" service.
  service = "/haptix/gazebo/hxs_set_linear_velocity";
  if (!node.Advertise(service, onHxsSetLinearVelocity))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_angular_velocity" service.
  service = "/haptix/gazebo/hxs_angular_velocity";
  if (!node.Advertise(service, onHxsAngularVelocity))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_set_angular_velocity" service.
  service = "/haptix/gazebo/hxs_set_angular_velocity";
  if (!node.Advertise(service, onHxsSetAngularVelocity))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_apply_force" service.
  service = "/haptix/gazebo/hxs_apply_force";
  if (!node.Advertise(service, onHxsApplyForce))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_apply_torque" service.
  service = "/haptix/gazebo/hxs_apply_torque";
  if (!node.Advertise(service, onHxsApplyTorque))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_apply_wrench" service.
  service = "/haptix/gazebo/hxs_apply_wrench";
  if (!node.Advertise(service, onHxsApplyWrench))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_reset" service.
  service = "/haptix/gazebo/hxs_reset";
  if (!node.Advertise(service, onHxsReset))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_start_logging" service.
  service = "/haptix/gazebo/hxs_start_logging";
  if (!node.Advertise(service, onHxsStartLogging))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_is_logging" service.
  service = "/haptix/gazebo/hxs_is_logging";
  if (!node.Advertise(service, onHxsIsLogging))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_stop_logging" service.
  service = "/haptix/gazebo/hxs_stop_logging";
  if (!node.Advertise(service, onHxsStopLogging))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_set_model_color" service.
  service = "/haptix/gazebo/hxs_set_model_color";
  if (!node.Advertise(service, onHxsSetModelColor))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_model_color" service.
  service = "/haptix/gazebo/hxs_model_color";
  if (!node.Advertise(service, onHxsModelColor))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_set_model_collide_mode" service.
  service = "/haptix/gazebo/hxs_set_model_collide_mode";
  if (!node.Advertise(service, onHxsSetModelCollideMode))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_model_collide_mode" service.
  service = "/haptix/gazebo/hxs_model_collide_mode";
  if (!node.Advertise(service, onHxsModelCollideMode))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_add_constraint" service.
  service = "/haptix/gazebo/hxs_add_constraint";
  if (!node.Advertise(service, onHxsAddConstraint))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Advertise the "hxs_remove_constraint" service.
  service = "/haptix/gazebo/hxs_remove_constraint";
  if (!node.Advertise(service, onHxsRemoveConstraint))
    std::cerr << "Error advertising service [" << service << "]." << std::endl;

  // Zzzz.
  std::this_thread::sleep_for(std::chrono::milliseconds(time));

  delete simInfo;
}
