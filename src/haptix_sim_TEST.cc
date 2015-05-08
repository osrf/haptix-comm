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
#include "msg/hxContactPoint.pb.h"
#include "msg/hxContactPoint_V.pb.h"
#include "msg/hxEmpty.pb.h"
#include "msg/hxInt.pb.h"
#include "msg/hxParam.pb.h"
#include "msg/hxSimInfo.pb.h"
#include "msg/hxString.pb.h"
#include "msg/hxTime.pb.h"
#include "msg/hxTransform.pb.h"
#include "test_config.h"

/// \brief Global constants.
const int kNumModels         = 5;
const int kNumLinksPerModel  = 40;
const int kNumJointsPerModel = 20;
const int kNumContactPoints  = 30;

/// \brief Global variables.
std::string partition;
haptix::comm::msgs::hxSimInfo simState;
haptix::comm::msgs::hxContactPoint_V simContactPointsState;

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
    model->set_id(i);
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
    contact->mutable_velocity()->set_x(i + 1.5f);
    contact->mutable_velocity()->set_y(i + 1.6f);
    contact->mutable_velocity()->set_z(i + 1.7f);
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
/// \brief Provide a "hxs_set_model_joint_state" service.
void onHxsJointState(const std::string &_service,
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
/// \brief Check hxs_sim_info.
TEST(hxsTest, hxs_sim_info)
{
  setup();

  ignition::transport::Node node;
  hxsSimInfo *simInfo = new hxsSimInfo();

  // Advertise the "hxs_sim_info" service.
  node.Advertise("/haptix/gazebo/hxs_sim_info", onHxsSimInfo);

  // Request simulation information.
  ASSERT_EQ(hxs_sim_info(simInfo), hxOK);

  // Check the model information
  ASSERT_EQ(simInfo->model_count, kNumModels);
  for (int i = 0; i < simInfo->model_count; ++i)
  {
    EXPECT_FLOAT_EQ(simInfo->models[i].transform.pos.x, i);
    EXPECT_FLOAT_EQ(simInfo->models[i].transform.pos.y, i + 0.1f);
    EXPECT_FLOAT_EQ(simInfo->models[i].transform.pos.z, i + 0.2f);
    EXPECT_FLOAT_EQ(simInfo->models[i].transform.orient.w, i + 0.3f);
    EXPECT_FLOAT_EQ(simInfo->models[i].transform.orient.x, i + 0.4f);
    EXPECT_FLOAT_EQ(simInfo->models[i].transform.orient.y, i + 0.5f);
    EXPECT_FLOAT_EQ(simInfo->models[i].transform.orient.z, i + 0.6f);
    EXPECT_FLOAT_EQ(simInfo->models[i].id, i);
    ASSERT_EQ(simInfo->models[i].link_count, kNumLinksPerModel);
    for (int j = 0; j < simInfo->models[i].link_count; ++j)
    {
      float v = 10 * i + j;
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].transform.pos.x, v);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].transform.pos.y, v + 0.1f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].transform.pos.z, v + 0.2f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].transform.orient.w, v + 0.3f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].transform.orient.x, v + 0.4f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].transform.orient.y, v + 0.5f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].transform.orient.z, v + 0.6f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].lin_vel.x, v + 0.7f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].lin_vel.y, v + 0.8f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].lin_vel.z, v + 0.9f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].ang_vel.x, v + 1.0f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].ang_vel.y, v + 1.1f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].ang_vel.z, v + 1.2f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].lin_acc.x, v + 1.3f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].lin_acc.y, v + 1.4f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].lin_acc.z, v + 1.5f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].ang_acc.x, v + 1.6f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].ang_acc.y, v + 1.7f);
      EXPECT_FLOAT_EQ(simInfo->models[i].links[j].ang_acc.z, v + 1.8f);
    }
    ASSERT_EQ(simInfo->models[i].joint_count, kNumJointsPerModel);
    for (int j = 0; j < simInfo->models[i].joint_count; ++j)
    {
      float v = 20 * i + j;
      EXPECT_FLOAT_EQ(simInfo->models[i].joints[j].pos, v);
      EXPECT_FLOAT_EQ(simInfo->models[i].joints[j].vel, v + 0.1f);
      EXPECT_FLOAT_EQ(simInfo->models[i].joints[j].torque_motor, v + 0.3f);
      EXPECT_FLOAT_EQ(simInfo->models[i].joints[j].wrench_reactive.force.x,
        v + 0.4f);
      EXPECT_FLOAT_EQ(simInfo->models[i].joints[j].wrench_reactive.force.y,
        v + 0.5f);
      EXPECT_FLOAT_EQ(simInfo->models[i].joints[j].wrench_reactive.force.z,
        v + 0.6f);
      EXPECT_FLOAT_EQ(simInfo->models[i].joints[j].wrench_reactive.torque.x,
        v + 0.7f);
      EXPECT_FLOAT_EQ(simInfo->models[i].joints[j].wrench_reactive.torque.y,
        v + 0.8f);
      EXPECT_FLOAT_EQ(simInfo->models[i].joints[j].wrench_reactive.torque.z,
        v + 0.9f);
    }
    EXPECT_TRUE(simInfo->models[i].gravity_mode);
  }

  // Check the camera information.
  EXPECT_FLOAT_EQ(simInfo->camera_transform.pos.x, 30.0f);
  EXPECT_FLOAT_EQ(simInfo->camera_transform.pos.y, 30.1f);
  EXPECT_FLOAT_EQ(simInfo->camera_transform.pos.z, 30.2f);
  EXPECT_FLOAT_EQ(simInfo->camera_transform.orient.w, 30.3f);
  EXPECT_FLOAT_EQ(simInfo->camera_transform.orient.x, 30.4f);
  EXPECT_FLOAT_EQ(simInfo->camera_transform.orient.y, 30.5f);
  EXPECT_FLOAT_EQ(simInfo->camera_transform.orient.z, 30.6f);

  delete simInfo;
}

//////////////////////////////////////////////////
/// \brief Check hxs_camera_transform.
TEST(hxsTest, hxs_camera_transform)
{
  setup();

  ignition::transport::Node node;
  hxsTransform camInfo;

  // Advertise the "hxs_camera" service.
  node.Advertise("/haptix/gazebo/hxs_camera_transform", onHxsCamera);

  // Request camera information.
  ASSERT_EQ(hxs_camera_transform(&camInfo), hxOK);

  // Check the camera information.
  EXPECT_FLOAT_EQ(camInfo.pos.x, 30.0f);
  EXPECT_FLOAT_EQ(camInfo.pos.y, 30.1f);
  EXPECT_FLOAT_EQ(camInfo.pos.z, 30.2f);
  EXPECT_FLOAT_EQ(camInfo.orient.w, 30.3f);
  EXPECT_FLOAT_EQ(camInfo.orient.x, 30.4f);
  EXPECT_FLOAT_EQ(camInfo.orient.y, 30.5f);
  EXPECT_FLOAT_EQ(camInfo.orient.z, 30.6f);
}

//////////////////////////////////////////////////
/// \brief Check hxs_set_camera_transform.
TEST(hxsTest, hxs_set_camera_transform)
{
  setup();

  ignition::transport::Node node;
  hxsSimInfo *simInfo = new hxsSimInfo();

  // Advertise the "hxs_sim_info" service.
  node.Advertise("/haptix/gazebo/hxs_sim_info", onHxsSimInfo);

  // Request simulation information.
  ASSERT_EQ(hxs_sim_info(simInfo), hxOK);

  // Advertise the "hxs_camera_transform" service.
  node.Advertise("/haptix/gazebo/hxs_set_camera_transform",
    onHxsCameraTransform);

  // Set a camera transformation similar to the camera in simState.
  ASSERT_EQ(hxs_set_camera_transform(&(simInfo->camera_transform)), hxOK);

  delete simInfo;
}

//////////////////////////////////////////////////
/// \brief Check hxs_contacts.
TEST(hxsTest, hxs_contacts)
{
  setup();

  ignition::transport::Node node;
  hxsContactPoints contactsInfo;

  // Advertise the "hxs_contacts" service.
  node.Advertise("/haptix/gazebo/hxs_contacts", onHxsContactPoints);

  // Get the list of contacts for model "model 0".
  ASSERT_EQ(hxs_contacts("model 0", &contactsInfo), hxOK);

  // Check the contacts information.
  ASSERT_EQ(contactsInfo.contact_count, kNumContactPoints);
  for (int i = 0; i < contactsInfo.contact_count; ++i)
  {
    EXPECT_EQ(std::string(contactsInfo.contacts[i].link1),
      "link " + std::to_string(i));
    EXPECT_EQ(std::string(contactsInfo.contacts[i].link2),
      "link " + std::to_string(i + 1));
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].point.x, i + 0.2f);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].point.y, i + 0.3f);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].point.z, i + 0.4f);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].normal.x, i + 0.5f);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].normal.y, i + 0.6f);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].normal.z, i + 0.7f);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].distance, i + 1.4f);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].wrench.force.x, i + 1.8f);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].wrench.force.y, i + 1.9f);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].wrench.force.z, i + 2.0f);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].wrench.torque.x, i + 2.1f);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].wrench.torque.y, i + 2.2f);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].wrench.torque.z, i + 2.3f);
  }
}

//////////////////////////////////////////////////
/// \brief Check hxs_set_model_joint_state.
TEST(hxsTest, hxs_set_model_joint_state)
{
  setup();

  ignition::transport::Node node;
  hxsSimInfo *simInfo = new hxsSimInfo();

  // Advertise the "hxs_sim_info" service.
  node.Advertise("/haptix/gazebo/hxs_sim_info", onHxsSimInfo);

  // Advertise the "hxs_state" service.
  node.Advertise("/haptix/gazebo/hxs_set_model_joint_state", onHxsJointState);

  // Request simulation information.
  ASSERT_EQ(hxs_sim_info(simInfo), hxOK);

  EXPECT_EQ(hxs_set_model_joint_state("model 0", "joint 1", 1.0f, 2.0f), hxOK);

  delete simInfo;
}

//////////////////////////////////////////////////
/// \brief Check hxs_add_model.
TEST(hxsTest, hxs_add_model)
{
  setup();

  ignition::transport::Node node;
  std::string urdf = "fake URDF";
  std::string name = "model 1";
  hxsModel model;
  float x = 1.0f;
  float y = 2.0f;
  float z = 3.0f;
  float roll = 4.0f;
  float pitch = 5.0f;
  float yaw = 6.0f;

  // Advertise the "hxs_add_model" service.
  node.Advertise("/haptix/gazebo/hxs_add_model", onHxsAddModel);

  // Create a new model.
  ASSERT_EQ(hxs_add_model(urdf.c_str(), name.c_str(), x, y, z, roll, pitch, yaw,
    true, &model), hxOK);

  // Verify that the new model matches the first model in simState.
  EXPECT_FLOAT_EQ(model.transform.pos.x, 0.0f);
  EXPECT_FLOAT_EQ(model.transform.pos.y, 0.1f);
  EXPECT_FLOAT_EQ(model.transform.pos.z, 0.2f);
  EXPECT_FLOAT_EQ(model.transform.orient.w, 0.3f);
  EXPECT_FLOAT_EQ(model.transform.orient.x, 0.4f);
  EXPECT_FLOAT_EQ(model.transform.orient.y, 0.5f);
  EXPECT_FLOAT_EQ(model.transform.orient.z, 0.6f);
  EXPECT_FLOAT_EQ(model.id, 0);
  ASSERT_EQ(model.link_count, kNumLinksPerModel);
  for (int i = 0; i < model.link_count; ++i)
  {
    float v = i;
    EXPECT_FLOAT_EQ(model.links[i].transform.pos.x, v);
    EXPECT_FLOAT_EQ(model.links[i].transform.pos.y, v + 0.1f);
    EXPECT_FLOAT_EQ(model.links[i].transform.pos.z, v + 0.2f);
    EXPECT_FLOAT_EQ(model.links[i].transform.orient.w, v + 0.3f);
    EXPECT_FLOAT_EQ(model.links[i].transform.orient.x, v + 0.4f);
    EXPECT_FLOAT_EQ(model.links[i].transform.orient.y, v + 0.5f);
    EXPECT_FLOAT_EQ(model.links[i].transform.orient.z, v + 0.6f);
    EXPECT_FLOAT_EQ(model.links[i].lin_vel.x, v + 0.7f);
    EXPECT_FLOAT_EQ(model.links[i].lin_vel.y, v + 0.8f);
    EXPECT_FLOAT_EQ(model.links[i].lin_vel.z, v + 0.9f);
    EXPECT_FLOAT_EQ(model.links[i].ang_vel.x, v + 1.0f);
    EXPECT_FLOAT_EQ(model.links[i].ang_vel.y, v + 1.1f);
    EXPECT_FLOAT_EQ(model.links[i].ang_vel.z, v + 1.2f);
    EXPECT_FLOAT_EQ(model.links[i].lin_acc.x, v + 1.3f);
    EXPECT_FLOAT_EQ(model.links[i].lin_acc.y, v + 1.4f);
    EXPECT_FLOAT_EQ(model.links[i].lin_acc.z, v + 1.5f);
    EXPECT_FLOAT_EQ(model.links[i].ang_acc.x, v + 1.6f);
    EXPECT_FLOAT_EQ(model.links[i].ang_acc.y, v + 1.7f);
    EXPECT_FLOAT_EQ(model.links[i].ang_acc.z, v + 1.8f);
  }
  ASSERT_EQ(model.joint_count, kNumJointsPerModel);
  for (int i = 0; i < model.joint_count; ++i)
  {
    float v = i;
    EXPECT_FLOAT_EQ(model.joints[i].pos, v);
    EXPECT_FLOAT_EQ(model.joints[i].vel, v + 0.1f);
    EXPECT_FLOAT_EQ(model.joints[i].torque_motor, v + 0.3f);
    EXPECT_FLOAT_EQ(model.joints[i].wrench_reactive.force.x, v + 0.4f);
    EXPECT_FLOAT_EQ(model.joints[i].wrench_reactive.force.y, v + 0.5f);
    EXPECT_FLOAT_EQ(model.joints[i].wrench_reactive.force.z, v + 0.6f);
    EXPECT_FLOAT_EQ(model.joints[i].wrench_reactive.torque.x, v + 0.7f);
    EXPECT_FLOAT_EQ(model.joints[i].wrench_reactive.torque.y, v + 0.8f);
    EXPECT_FLOAT_EQ(model.joints[i].wrench_reactive.torque.z, v + 0.9f);
  }
  EXPECT_TRUE(model.gravity_mode);
}

//////////////////////////////////////////////////
/// \brief Check hxs_remove_model.
TEST(hxsTest, hxs_remove_model)
{
  setup();

  ignition::transport::Node node;

  // Advertise the "hxs_remove_model" service.
  node.Advertise("/haptix/gazebo/hxs_remove_model", onHxsRemoveModel);

  // Remove "model 1" model.
  ASSERT_EQ(hxs_remove_model("model 1"), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_set_model_transform.
TEST(hxsTest, hxs_set_model_transform)
{
  setup();

  ignition::transport::Node node;
  hxsSimInfo *simInfo = new hxsSimInfo();

  // Advertise the "hxs_sim_info" service.
  node.Advertise("/haptix/gazebo/hxs_sim_info", onHxsSimInfo);

  // Advertise the "hxs_set_model_transform" service.
  node.Advertise("/haptix/gazebo/hxs_set_model_transform",
    onHxsSetModelTransform);

  // Request simulation information.
  ASSERT_EQ(hxs_sim_info(simInfo), hxOK);

  // Let's use the transform from the second model.
  ASSERT_EQ(hxs_set_model_transform("model 1", &(simInfo->models[1].transform)),
    hxOK);

  delete simInfo;
}

//////////////////////////////////////////////////
/// \brief Check hxs_model_transform.
TEST(hxsTest, hxs_model_transform)
{
  setup();

  ignition::transport::Node node;
  hxsTransform transform;

  // Advertise the "hxs_model_transform" service.
  node.Advertise("/haptix/gazebo/hxs_model_transform", onHxsModelTransform);

  // Request transform for model 1.
  ASSERT_EQ(hxs_model_transform("model 1", &transform), hxOK);

  // Verify the transform.
  EXPECT_FLOAT_EQ(transform.pos.x, 1.0f);
  EXPECT_FLOAT_EQ(transform.pos.y, 1.1f);
  EXPECT_FLOAT_EQ(transform.pos.z, 1.2f);
  EXPECT_FLOAT_EQ(transform.orient.w, 1.3f);
  EXPECT_FLOAT_EQ(transform.orient.x, 1.4f);
  EXPECT_FLOAT_EQ(transform.orient.y, 1.5f);
  EXPECT_FLOAT_EQ(transform.orient.z, 1.6f);
}

//////////////////////////////////////////////////
/// \brief Check hxs_model_gravity_mode.
TEST(hxsTest, hxs_model_gravity_mode)
{
  setup();

  ignition::transport::Node node;

  // Advertise the "hxs_model_gravity_mode" service.
  node.Advertise("/haptix/gazebo/hxs_model_gravity_mode", onHxsModelGravity);

  int gravity_mode = 0;
  ASSERT_EQ(hxs_model_gravity_mode("model_1", &gravity_mode), hxOK);
  EXPECT_EQ(gravity_mode, 1);
}

//////////////////////////////////////////////////
/// \brief Check hxs_set_model_gravity_mode.
TEST(hxsTest, hxs_set_model_gravity_mode)
{
  setup();

  ignition::transport::Node node;

  // Advertise the "hxs_set_model_gravity_mode" service.
  node.Advertise("/haptix/gazebo/hxs_set_model_gravity_mode",
    onHxsSetModelGravity);

  ASSERT_EQ(hxs_set_model_gravity_mode("model_1", 1), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_linear_velocity.
TEST(hxsTest, hxs_linear_velocity)
{
  setup();

  ignition::transport::Node node;
  hxsVector3 lin_vel;

  // Advertise the "hxs_linear_velocity" service.
  node.Advertise("/haptix/gazebo/hxs_linear_velocity",
    onHxsLinearVelocity);

  ASSERT_EQ(hxs_linear_velocity("model 1", &lin_vel), hxOK);

  // Verify linear velocity.
  EXPECT_FLOAT_EQ(lin_vel.x, 1.0f);
  EXPECT_FLOAT_EQ(lin_vel.y, 1.1f);
  EXPECT_FLOAT_EQ(lin_vel.z, 1.2f);
}

//////////////////////////////////////////////////
/// \brief Check hxs_set_linear_velocity.
TEST(hxsTest, hxs_set_linear_velocity)
{
  setup();

  ignition::transport::Node node;
  hxsVector3 lin_vel;

  // Advertise the "hxs_set_linear_velocity" service.
  node.Advertise("/haptix/gazebo/hxs_set_linear_velocity",
    onHxsSetLinearVelocity);

  lin_vel.x = 1.0f;
  lin_vel.y = 1.1f;
  lin_vel.z = 1.2f;

  ASSERT_EQ(hxs_set_linear_velocity("model 1", &lin_vel), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_angular_velocity.
TEST(hxsTest, hxs_angular_velocity)
{
  setup();

  ignition::transport::Node node;
  hxsVector3 ang_vel;

  // Advertise the "hxs_angular_velocity" service.
  node.Advertise("/haptix/gazebo/hxs_angular_velocity", onHxsAngularVelocity);

  ASSERT_EQ(hxs_angular_velocity("model 1", &ang_vel), hxOK);

  // Verify the angular velocity.
  EXPECT_FLOAT_EQ(ang_vel.x, 2.0f);
  EXPECT_FLOAT_EQ(ang_vel.y, 2.1f);
  EXPECT_FLOAT_EQ(ang_vel.z, 2.2f);
}

//////////////////////////////////////////////////
/// \brief Check hxs_set_angular_velocity.
TEST(hxsTest, hxs_set_angular_velocity)
{
  setup();

  ignition::transport::Node node;
  hxsVector3 ang_vel;

  // Advertise the "hxs_set_angular_velocity" service.
  node.Advertise("/haptix/gazebo/hxs_set_angular_velocity",
    onHxsSetAngularVelocity);

  ang_vel.x = 2.0f;
  ang_vel.y = 2.1f;
  ang_vel.z = 2.2f;

  ASSERT_EQ(hxs_set_angular_velocity("model 1", &ang_vel), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_apply_force.
TEST(hxsTest, hxs_apply_force)
{
  setup();

  ignition::transport::Node node;
  hxsVector3 force;
  hxsSimInfo *simInfo = new hxsSimInfo();

  // Advertise the "hxs_sim_info" service.
  node.Advertise("/haptix/gazebo/hxs_sim_info", onHxsSimInfo);

  // Advertise the "hxs_apply_force" service.
  node.Advertise("/haptix/gazebo/hxs_apply_force", onHxsApplyForce);

  // Request simulation information.
  ASSERT_EQ(hxs_sim_info(simInfo), hxOK);

  // Set some force.
  force.x = 5.1f;
  force.y = 5.2f;
  force.z = 5.3f;

  // Use the first link of the first model in simState.
  ASSERT_EQ(hxs_apply_force(simInfo->models[0].name,
      simInfo->models[0].links[0].name, &force, 0.1), hxOK);

  delete simInfo;
}

//////////////////////////////////////////////////
/// \brief Check hxs_apply_torque.
TEST(hxsTest, hxs_apply_torque)
{
  setup();

  ignition::transport::Node node;
  hxsVector3 torque;
  hxsSimInfo *simInfo = new hxsSimInfo();

  // Advertise the "hxs_sim_info" service.
  node.Advertise("/haptix/gazebo/hxs_sim_info", onHxsSimInfo);

  // Advertise the "hxs_apply_torque" service.
  node.Advertise("/haptix/gazebo/hxs_apply_torque", onHxsApplyTorque);

  // Request simulation information.
  ASSERT_EQ(hxs_sim_info(simInfo), hxOK);

  // Set some force.
  torque.x = 6.1f;
  torque.y = 6.2f;
  torque.z = 6.3f;

  // Use the first link of the first model in simState.
  ASSERT_EQ(hxs_apply_torque(simInfo->models[0].name,
      simInfo->models[0].links[0].name, &torque, 0.1), hxOK);

  delete simInfo;
}

//////////////////////////////////////////////////
/// \brief Check hxs_apply_wrench.
TEST(hxsTest, hxs_apply_wrench)
{
  setup();

  ignition::transport::Node node;
  hxsWrench wrench;
  hxsSimInfo *simInfo = new hxsSimInfo();

  // Advertise the "hxs_sim_info" service.
  node.Advertise("/haptix/gazebo/hxs_sim_info", onHxsSimInfo);

  // Advertise the "hxs_apply_wrench" service.
  node.Advertise("/haptix/gazebo/hxs_apply_wrench", onHxsApplyWrench);

  // Request simulation information.
  ASSERT_EQ(hxs_sim_info(simInfo), hxOK);

  // Set some torque/force.
  wrench.force.x = 7.1f;
  wrench.force.y = 7.2f;
  wrench.force.z = 7.3f;
  wrench.torque.x = 7.4f;
  wrench.torque.y = 7.5f;
  wrench.torque.z = 7.6f;

  // Use the first link of the first model in simState.
  ASSERT_EQ(hxs_apply_wrench(simInfo->models[0].name,
      simInfo->models[0].links[0].name, &wrench, 0.1f), hxOK);

  delete simInfo;
}

//////////////////////////////////////////////////
/// \brief Check hxs_reset.
TEST(hxsTest, hxs_reset)
{
  setup();

  ignition::transport::Node node;
  int resetLimbPose = 1;

  // Advertise the "hxs_reset" service.
  node.Advertise("/haptix/gazebo/hxs_reset", onHxsReset);

  ASSERT_EQ(hxs_reset(resetLimbPose), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_start_logging.
TEST(hxsTest, hxs_start_logging)
{
  setup();

  ignition::transport::Node node;
  std::string filename = "a filename";

  // Advertise the "hxs_start_logging" service.
  node.Advertise("/haptix/gazebo/hxs_start_logging", onHxsStartLogging);

  ASSERT_EQ(hxs_start_logging(filename.c_str()), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_is_logging.
TEST(hxsTest, hxs_is_logging)
{
  setup();

  ignition::transport::Node node;
  int isLogging;

  // Advertise the "hxs_is_logging" service.
  node.Advertise("/haptix/gazebo/hxs_is_logging", onHxsIsLogging);

  ASSERT_EQ(hxs_is_logging(&isLogging), hxOK);

  // Verify the result.
  EXPECT_EQ(isLogging, 1);
}

//////////////////////////////////////////////////
/// \brief Check hxs_stop_logging.
TEST(hxsTest, hxs_stop_logging)
{
  setup();

  ignition::transport::Node node;

  // Advertise the "hxs_stop_logging" service.
  node.Advertise("/haptix/gazebo/hxs_stop_logging", onHxsStopLogging);

  ASSERT_EQ(hxs_stop_logging(), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_set_model_color.
TEST(hxsTest, hxs_set_model_color)
{
  setup();

  ignition::transport::Node node;
  hxsColor color;

  // Advertise the "hxs_set_model_color" service.
  node.Advertise("/haptix/gazebo/hxs_set_model_color", onHxsSetModelColor);

  color.r = 0.5f;
  color.g = 0.6f;
  color.b = 0.7f;
  color.alpha = 0.8f;

  ASSERT_EQ(hxs_set_model_color("model_1", &color), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_model_color.
TEST(hxsTest, hxs_model_color)
{
  setup();

  ignition::transport::Node node;
  hxsColor color;

  // Advertise the "hxs_model_color" service.
  node.Advertise("/haptix/gazebo/hxs_model_color", onHxsModelColor);

  ASSERT_EQ(hxs_model_color("model_1", &color), hxOK);

  // Check the color received.
  EXPECT_FLOAT_EQ(color.r, 0.1f);
  EXPECT_FLOAT_EQ(color.g, 0.2f);
  EXPECT_FLOAT_EQ(color.b, 0.3f);
  EXPECT_FLOAT_EQ(color.alpha, 0.4f);
}

//////////////////////////////////////////////////
/// \brief Check hxs_set_model_collide_mode.
TEST(hxsTest, hxs_set_model_collide_mode)
{
  setup();

  ignition::transport::Node node;
  hxsCollideMode collideMode;

  // Advertise the "hxs_set_model_collide_mode" service.
  node.Advertise("/haptix/gazebo/hxs_set_model_collide_mode",
    onHxsSetModelCollideMode);

  collideMode = hxsCOLLIDE;

  ASSERT_EQ(hxs_set_model_collide_mode("model_1", &collideMode), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_model_collide_mode.
TEST(hxsTest, hxs_model_collide_mode)
{
  setup();

  ignition::transport::Node node;
  hxsCollideMode collideMode;

  // Advertise the "hxs_model_collide_mode" service.
  node.Advertise("/haptix/gazebo/hxs_model_collide_mode",
    onHxsModelCollideMode);

  ASSERT_EQ(hxs_model_collide_mode("model_1", &collideMode), hxOK);

  // Check the collide mode received.
  EXPECT_EQ(collideMode, hxsDETECTIONONLY);
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
