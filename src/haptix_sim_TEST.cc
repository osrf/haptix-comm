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
#include "msg/hxInt.pb.h"
#include "msg/hxJacobian.pb.h"
#include "msg/hxParam.pb.h"
#include "msg/hxSimInfo.pb.h"
#include "msg/hxString.pb.h"
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
    model->set_name("model " + std::to_string(i));
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
      link->set_name("link " + std::to_string(j));
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
      joint->set_name("joint " + std::to_string(j));
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

  std::string msg1, msg2;

  _req.SerializeToString(&msg1);
  simState.camera().transform().SerializeToString(&msg2);
  EXPECT_EQ(msg1, msg2);

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
/// \brief Provide a "hxs_jacobian" service.
void onHxsJacobian(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxJacobian &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_jacobian");

  // Sanity check: The message should contain a link.
  if (!_req.has_link())
  {
    std::cerr << "onHxsJacobian() error: Missing link in request" << std::endl;
    return;
  }

  // Sanity check: The message should contain a point.
  if (!_req.has_vector3())
  {
    std::cerr << "onHxsState() error: Missing point in request" << std::endl;
    return;
  }

  // Check that the link has the expected values.
  std::string msg1;
  _req.link().SerializeToString(&msg1);
  std::string msg2;
  simState.models(0).links(0).SerializeToString(&msg2);
  EXPECT_EQ(msg1, msg2);

  // Check the point.
  EXPECT_FLOAT_EQ(_req.vector3().x(), 1.1);
  EXPECT_FLOAT_EQ(_req.vector3().y(), 1.2);
  EXPECT_FLOAT_EQ(_req.vector3().z(), 1.3);

  // Create some dummy Jacobian matrix.
  for (int i = 0; i < kNumJointsPerModel; ++i)
  {
    haptix::comm::msgs::hxVector3 *column = _rep.add_columns();
    column->set_x(i);
    column->set_y(i + 0.1);
    column->set_z(i + 0.2);
  }

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

  // Verify the request. The model should be the first model in simState and
  // the link should be the second link of the first model.
  std::string msg1;
  _req.model().SerializeToString(&msg1);
  std::string msg2;
  simState.models(0).SerializeToString(&msg2);
  EXPECT_EQ(msg1, msg2);

  // Verify that the joint is the second joint of model #1 in simState.
  _req.joint().SerializeToString(&msg1);
  simState.models(0).joints(1).SerializeToString(&msg2);
  EXPECT_EQ(msg1, msg2);

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
  if (!_req.has_pos())
  {
    std::cerr << "onHxsAddModel() error: Missing pos in request" << std::endl;
    return;
  }

  // Sanity check: The message should contain an orientation.
  if (!_req.has_orientation())
  {
    std::cerr << "onHxsAddModel() error: Missing orientation in request"
              << std::endl;
    return;
  }

  // Verify the request.
  EXPECT_EQ(_req.string_value(), "fake URDF");
  EXPECT_EQ(_req.name(), "model 1");
  EXPECT_FLOAT_EQ(_req.pos().x(), 1.0);
  EXPECT_FLOAT_EQ(_req.pos().y(), 2.0);
  EXPECT_FLOAT_EQ(_req.pos().z(), 3.0);
  EXPECT_FLOAT_EQ(_req.orientation().roll(), 4.0);
  EXPECT_FLOAT_EQ(_req.orientation().pitch(), 5.0);
  EXPECT_FLOAT_EQ(_req.orientation().yaw(), 6.0);

  // Return the first model of simState as an answer.
  _rep = simState.models(0);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_remove_model_id" service.
void onHxsRemoveModelId(const std::string &_service,
  const haptix::comm::msgs::hxInt &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_remove_model_id");

  // Verify the request.
  EXPECT_EQ(_req.data(), 1);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_model_transform" service.
void onHxsModelTransform(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_model_transform");

  // Sanity check: The message should contain an ID.
  if (!_req.id())
  {
    std::cerr << "onHxsModelTransform() error: Missing ID in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a transform.
  if (!_req.has_transform())
  {
    std::cerr << "onHxsModelTransform() error: Missing transform in request"
              << std::endl;
    return;
  }

  // Verify the ID.
  EXPECT_EQ(_req.id(), 1);

  // Verify the transform.
  std::string msg1, msg2;
  _req.transform().SerializeToString(&msg1);
  simState.models(1).transform().SerializeToString(&msg2);
  EXPECT_EQ(msg1, msg2);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_linear_velocity" service.
void onHxsLinearVelocity(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_linear_velocity");

  // Sanity check: The message should contain an ID.
  if (!_req.id())
  {
    std::cerr << "onHxsLinearVelocity() error: Missing ID in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a vector3.
  if (!_req.has_vector3())
  {
    std::cerr << "onHxsLinearVelocity() error: Missing linvel in request"
              << std::endl;
    return;
  }

  // Verify the ID.
  EXPECT_EQ(_req.id(), 1);

  // Verify the linvel.
  EXPECT_FLOAT_EQ(_req.vector3().x(), 1.0);
  EXPECT_FLOAT_EQ(_req.vector3().y(), 1.1);
  EXPECT_FLOAT_EQ(_req.vector3().z(), 1.2);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_angular_velocity" service.
void onHxsAngularVelocity(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_angular_velocity");

  // Sanity check: The message should contain an ID.
  if (!_req.id())
  {
    std::cerr << "onHxsAngularVelocity() error: Missing ID in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a vector3.
  if (!_req.has_vector3())
  {
    std::cerr << "onHxsAngularVelocity() error: Missing angvel in request"
              << std::endl;
    return;
  }

  // Verify the ID.
  EXPECT_EQ(_req.id(), 1);

  // Verify the angvel.
  EXPECT_FLOAT_EQ(_req.vector3().x(), 2.0);
  EXPECT_FLOAT_EQ(_req.vector3().y(), 2.1);
  EXPECT_FLOAT_EQ(_req.vector3().z(), 2.2);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_linear_accel" service.
void onHxsLinearAccel(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_linear_accel");

  // Sanity check: The message should contain an ID.
  if (!_req.id())
  {
    std::cerr << "onHxsLinearAccel() error: Missing ID in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a vector3.
  if (!_req.has_vector3())
  {
    std::cerr << "onHxsLinearAccel() error: Missing linaccel in request"
              << std::endl;
    return;
  }

  // Verify the ID.
  EXPECT_EQ(_req.id(), 1);

  // Verify the linaccel.
  EXPECT_FLOAT_EQ(_req.vector3().x(), 3.0);
  EXPECT_FLOAT_EQ(_req.vector3().y(), 3.1);
  EXPECT_FLOAT_EQ(_req.vector3().z(), 3.2);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_angular_accel" service.
void onHxsAngularAccel(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_angular_accel");

  // Sanity check: The message should contain an ID.
  if (!_req.id())
  {
    std::cerr << "onHxsAngularAccel() error: Missing ID in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a vector3.
  if (!_req.has_vector3())
  {
    std::cerr << "onHxsAngularAccel() error: Missing angaccel in request"
              << std::endl;
    return;
  }

  // Verify the ID.
  EXPECT_EQ(_req.id(), 1);

  // Verify the angaccel.
  EXPECT_FLOAT_EQ(_req.vector3().x(), 4.0);
  EXPECT_FLOAT_EQ(_req.vector3().y(), 4.1);
  EXPECT_FLOAT_EQ(_req.vector3().z(), 4.2);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_force" service.
void onHxsForce(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_force");

  // Sanity check: The message should contain a link.
  if (!_req.has_link())
  {
    std::cerr << "onHxsForce() error: Missing link in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a vector3.
  if (!_req.has_vector3())
  {
    std::cerr << "onHxsForce() error: Missing force in request"
              << std::endl;
    return;
  }

  // Verify the link received.
  std::string msg1, msg2;
  _req.link().SerializeToString(&msg1);
  simState.models(0).links(0).SerializeToString(&msg2);
  EXPECT_EQ(msg1, msg2);

  // Verify the force vector received.
  EXPECT_FLOAT_EQ(_req.vector3().x(), 5.1);
  EXPECT_FLOAT_EQ(_req.vector3().y(), 5.2);
  EXPECT_FLOAT_EQ(_req.vector3().z(), 5.3);

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_torque" service.
void onHxsTorque(const std::string &_service,
  const haptix::comm::msgs::hxParam &_req,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();
  _result = false;

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_torque");

  // Sanity check: The message should contain a link.
  if (!_req.has_link())
  {
    std::cerr << "onHxsTorque() error: Missing link in request"
              << std::endl;
    return;
  }

  // Sanity check: The message should contain a vector3.
  if (!_req.has_vector3())
  {
    std::cerr << "onHxsTorque() error: Missing torque in request"
              << std::endl;
    return;
  }

  // Verify the link received.
  std::string msg1, msg2;
  _req.link().SerializeToString(&msg1);
  simState.models(0).links(0).SerializeToString(&msg2);
  EXPECT_EQ(msg1, msg2);

  // Verify the torque vector received.
  EXPECT_FLOAT_EQ(_req.vector3().x(), 6.1);
  EXPECT_FLOAT_EQ(_req.vector3().y(), 6.2);
  EXPECT_FLOAT_EQ(_req.vector3().z(), 6.3);

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
/// \brief Provide a "hxs_reset_timer" service.
void onHxsResetTimer(const std::string &_service,
  const haptix::comm::msgs::hxEmpty &/*_req*/,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_reset_timer");

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_start_timer" service.
void onHxsStartTimer(const std::string &_service,
  const haptix::comm::msgs::hxEmpty &/*_req*/,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_start_timer");

  _result = true;
}

//////////////////////////////////////////////////
/// \brief Provide a "hxs_stop_timer" service.
void onHxsStopTimer(const std::string &_service,
  const haptix::comm::msgs::hxEmpty &/*_req*/,
  haptix::comm::msgs::hxEmpty &_rep,
  bool &_result)
{
  _rep.Clear();

  // Check the name of the service received.
  EXPECT_EQ(_service, "/haptix/gazebo/hxs_stop_timer");

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
  hxSimInfo simInfo;

  // Advertise the "hxs_siminfo" service.
  node.Advertise("/haptix/gazebo/hxs_siminfo", onHxsSimInfo);

  // Request simulation information.
  ASSERT_EQ(hxs_siminfo(&simInfo), hxOK);

  // Advertise the "hxs_camera_transform" service.
  node.Advertise("/haptix/gazebo/hxs_camera_transform", onHxsCameraTransform);

  // Set a camera transformation similar to the camera in simState.
  ASSERT_EQ(hxs_camera_transform(&simInfo.camera.transform), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_contacts.
TEST(hxsTest, hxs_contacts)
{
  setup();

  ignition::transport::Node node;
  hxContacts contactsInfo;

  // Advertise the "hxs_contacts" service.
  node.Advertise("/haptix/gazebo/hxs_contacts", onHxsContacts);

  ASSERT_EQ(hxs_contacts(&contactsInfo), hxOK);

  // Check the contacts information.
  ASSERT_EQ(contactsInfo.contactCount, kNumContacts);
  for (int i = 0; i < contactsInfo.contactCount; ++i)
  {
    EXPECT_EQ(contactsInfo.contacts[i].body1, i);
    EXPECT_EQ(contactsInfo.contacts[i].body2, i + 1);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].point.x, i + 0.2);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].point.y, i + 0.3);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].point.z, i + 0.4);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].normal.x, i + 0.5);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].normal.y, i + 0.6);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].normal.z, i + 0.7);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].tangent1.x, i + 0.8);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].tangent1.y, i + 0.9);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].tangent1.z, i + 1);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].tangent2.x, i + 1.1);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].tangent2.y, i + 1.2);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].tangent2.z, i + 1.3);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].distance, i + 1.4);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].velocity.x, i + 1.5);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].velocity.y, i + 1.6);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].velocity.z, i + 1.7);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].force.x, i + 1.8);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].force.y, i + 1.9);
    EXPECT_FLOAT_EQ(contactsInfo.contacts[i].force.z, i + 2);
  }
}

//////////////////////////////////////////////////
/// \brief Check hxs_jacobian.
TEST(hxsTest, hxs_jacobian)
{
  setup();

  ignition::transport::Node node;
  hxVector3 point;
  hxJacobian jacobian;
  hxSimInfo simInfo;

  // Advertise the "hxs_siminfo" service.
  node.Advertise("/haptix/gazebo/hxs_siminfo", onHxsSimInfo);

  // Advertise the "hxs_jacobian" service.
  node.Advertise("/haptix/gazebo/hxs_jacobian", onHxsJacobian);

  // Request simulation information.
  ASSERT_EQ(hxs_siminfo(&simInfo), hxOK);

  // Create a random point.
  point.x = 1.1;
  point.y = 1.2;
  point.z = 1.3;

  // Use the first link of the first model as an example.
  ASSERT_EQ(hxs_jacobian(&simInfo.models[0].links[0], &point, &jacobian), hxOK);

  // Check the Jacobian matrix.
  EXPECT_EQ(jacobian.jointCount, kNumJointsPerModel);
  for (int i = 0; i < jacobian.jointCount; ++i)
  {
    EXPECT_FLOAT_EQ(jacobian.mat[0][i], i);
    EXPECT_FLOAT_EQ(jacobian.mat[1][i], i + 0.1);
    EXPECT_FLOAT_EQ(jacobian.mat[2][i], i + 0.2);
  }
}

//////////////////////////////////////////////////
/// \brief Check hxs_state.
TEST(hxsTest, hxs_state)
{
  setup();

  ignition::transport::Node node;
  hxSimInfo simInfo;

  // Advertise the "hxs_siminfo" service.
  node.Advertise("/haptix/gazebo/hxs_siminfo", onHxsSimInfo);

  // Advertise the "hxs_state" service.
  node.Advertise("/haptix/gazebo/hxs_state", onHxsState);

  // Request simulation information.
  ASSERT_EQ(hxs_siminfo(&simInfo), hxOK);

  // I'll use the first model stored in simState and the second joint.
  EXPECT_EQ(hxs_state(&simInfo.models[0], &simInfo.models[0].joints[1]), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_add_model.
TEST(hxsTest, hxs_add_model)
{
  setup();

  ignition::transport::Node node;
  std::string urdf = "fake URDF";
  std::string name = "model 1";
  hxModel model;
  float x = 1.0;
  float y = 2.0;
  float z = 3.0;
  float roll = 4.0;
  float pitch = 5.0;
  float yaw = 6.0;

  // Advertise the "hxs_add_model" service.
  node.Advertise("/haptix/gazebo/hxs_add_model", onHxsAddModel);

  // Create a new model.
  ASSERT_EQ(hxs_add_model(urdf.c_str(), name.c_str(), x, y, z, roll, pitch, yaw,
    &model), hxOK);

  // Verify that the new model matches the first model in simState.
  EXPECT_FLOAT_EQ(model.transform.pos.x, 0);
  EXPECT_FLOAT_EQ(model.transform.pos.y, 0.1);
  EXPECT_FLOAT_EQ(model.transform.pos.z, 0.2);
  EXPECT_FLOAT_EQ(model.transform.orient.w, 0.3);
  EXPECT_FLOAT_EQ(model.transform.orient.x, 0.4);
  EXPECT_FLOAT_EQ(model.transform.orient.y, 0.5);
  EXPECT_FLOAT_EQ(model.transform.orient.z, 0.6);
  EXPECT_FLOAT_EQ(model.is_static, 1);
  EXPECT_FLOAT_EQ(model.id, 0);
  ASSERT_EQ(model.link_count, kNumLinksPerModel);
  for (int i = 0; i < model.link_count; ++i)
  {
    float v = i;
    EXPECT_FLOAT_EQ(model.links[i].transform.pos.x, v);
    EXPECT_FLOAT_EQ(model.links[i].transform.pos.y, v + 0.1);
    EXPECT_FLOAT_EQ(model.links[i].transform.pos.z, v + 0.2);
    EXPECT_FLOAT_EQ(model.links[i].transform.orient.w, v + 0.3);
    EXPECT_FLOAT_EQ(model.links[i].transform.orient.x, v + 0.4);
    EXPECT_FLOAT_EQ(model.links[i].transform.orient.y, v + 0.5);
    EXPECT_FLOAT_EQ(model.links[i].transform.orient.z, v + 0.6);
    EXPECT_FLOAT_EQ(model.links[i].linvel.x, v + 0.7);
    EXPECT_FLOAT_EQ(model.links[i].linvel.y, v + 0.8);
    EXPECT_FLOAT_EQ(model.links[i].linvel.z, v + 0.9);
    EXPECT_FLOAT_EQ(model.links[i].angvel.x, v + 1);
    EXPECT_FLOAT_EQ(model.links[i].angvel.y, v + 1.1);
    EXPECT_FLOAT_EQ(model.links[i].angvel.z, v + 1.2);
    EXPECT_FLOAT_EQ(model.links[i].linacc.x, v + 1.3);
    EXPECT_FLOAT_EQ(model.links[i].linacc.y, v + 1.4);
    EXPECT_FLOAT_EQ(model.links[i].linacc.z, v + 1.5);
    EXPECT_FLOAT_EQ(model.links[i].angacc.x, v + 1.6);
    EXPECT_FLOAT_EQ(model.links[i].angacc.y, v + 1.7);
    EXPECT_FLOAT_EQ(model.links[i].angacc.z, v + 1.8);
  }
  ASSERT_EQ(model.joint_count, kNumJointsPerModel);
  for (int i = 0; i < model.joint_count; ++i)
  {
    float v = i;
    EXPECT_FLOAT_EQ(model.joints[i].pos, v);
    EXPECT_FLOAT_EQ(model.joints[i].vel, v + 0.1);
    EXPECT_FLOAT_EQ(model.joints[i].acc, v + 0.2);
    EXPECT_FLOAT_EQ(model.joints[i].torque_motor, v + 0.3);
    EXPECT_FLOAT_EQ(model.joints[i].torque_passive, v + 0.4);
  }
}

//////////////////////////////////////////////////
/// \brief Check hxs_remove_model_id.
TEST(hxsTest, hxs_remove_model_id)
{
  setup();

  ignition::transport::Node node;
  int id = 1;

  // Advertise the "hxs_remove_model_id" service.
  node.Advertise("/haptix/gazebo/hxs_remove_model_id", onHxsRemoveModelId);

  // Remove a model with ID = 1.
  ASSERT_EQ(hxs_remove_model_id(id), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_model_transform.
TEST(hxsTest, hxs_model_transform)
{
  setup();

  ignition::transport::Node node;
  hxSimInfo simInfo;
  int id = 1;

  // Advertise the "hxs_siminfo" service.
  node.Advertise("/haptix/gazebo/hxs_siminfo", onHxsSimInfo);

  // Advertise the "hxs_model_transform" service.
  node.Advertise("/haptix/gazebo/hxs_model_transform", onHxsModelTransform);

  // Request simulation information.
  ASSERT_EQ(hxs_siminfo(&simInfo), hxOK);

  // Let's use the transform from the second model.
  ASSERT_EQ(hxs_model_transform(id, &simInfo.models[1].transform), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_linear_velocity.
TEST(hxsTest, hxs_linear_velocity)
{
  setup();

  ignition::transport::Node node;
  hxVector3 linvel;
  int id = 1;

  // Advertise the "hxs_linear_velocity" service.
  node.Advertise("/haptix/gazebo/hxs_linear_velocity", onHxsLinearVelocity);

  linvel.x = 1.0;
  linvel.y = 1.1;
  linvel.z = 1.2;

  ASSERT_EQ(hxs_linear_velocity(id, &linvel), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_angular_velocity.
TEST(hxsTest, hxs_angular_velocity)
{
  setup();

  ignition::transport::Node node;
  hxVector3 angvel;
  int id = 1;

  // Advertise the "hxs_angular_velocity" service.
  node.Advertise("/haptix/gazebo/hxs_angular_velocity", onHxsAngularVelocity);

  angvel.x = 2.0;
  angvel.y = 2.1;
  angvel.z = 2.2;

  ASSERT_EQ(hxs_angular_velocity(id, &angvel), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_linear_accel.
TEST(hxsTest, hxs_linear_accel)
{
  setup();

  ignition::transport::Node node;
  hxVector3 linaccel;
  int id = 1;

  // Advertise the "hxs_linear_accel" service.
  node.Advertise("/haptix/gazebo/hxs_linear_accel", onHxsLinearAccel);

  linaccel.x = 3.0;
  linaccel.y = 3.1;
  linaccel.z = 3.2;

  ASSERT_EQ(hxs_linear_accel(id, &linaccel), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_angular_accel.
TEST(hxsTest, hxs_angular_accel)
{
  setup();

  ignition::transport::Node node;
  hxVector3 angaccel;
  int id = 1;

  // Advertise the "hxs_angular_accel" service.
  node.Advertise("/haptix/gazebo/hxs_angular_accel", onHxsAngularAccel);

  angaccel.x = 4.0;
  angaccel.y = 4.1;
  angaccel.z = 4.2;

  ASSERT_EQ(hxs_angular_accel(id, &angaccel), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_force.
TEST(hxsTest, hxs_force)
{
  setup();

  ignition::transport::Node node;
  hxVector3 force;
  hxSimInfo simInfo;

  // Advertise the "hxs_siminfo" service.
  node.Advertise("/haptix/gazebo/hxs_siminfo", onHxsSimInfo);

  // Advertise the "hxs_force" service.
  node.Advertise("/haptix/gazebo/hxs_force", onHxsForce);

  // Set some force.
  force.x = 5.1;
  force.y = 5.2;
  force.z = 5.3;

  // Use the first link of the first model in simState.
  ASSERT_EQ(hxs_force(&simInfo.models[0].links[0], &force), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_torque.
TEST(hxsTest, hxs_torque)
{
  setup();

  ignition::transport::Node node;
  hxVector3 torque;
  hxSimInfo simInfo;

  // Advertise the "hxs_siminfo" service.
  node.Advertise("/haptix/gazebo/hxs_siminfo", onHxsSimInfo);

  // Advertise the "hxs_torque" service.
  node.Advertise("/haptix/gazebo/hxs_torque", onHxsTorque);

  // Set some force.
  torque.x = 6.1;
  torque.y = 6.2;
  torque.z = 6.3;

  // Use the first link of the first model in simState.
  ASSERT_EQ(hxs_torque(&simInfo.models[0].links[0], &torque), hxOK);
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
/// \brief Check hxs_reset_timer.
TEST(hxsTest, hxs_reset_timer)
{
  setup();

  ignition::transport::Node node;

  // Advertise the "hxs_reset_timer" service.
  node.Advertise("/haptix/gazebo/hxs_reset_timer", onHxsResetTimer);

  ASSERT_EQ(hxs_reset_timer(), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_start_timer.
TEST(hxsTest, hxs_start_timer)
{
  setup();

  ignition::transport::Node node;

  // Advertise the "hxs_start_timer" service.
  node.Advertise("/haptix/gazebo/hxs_start_timer", onHxsStartTimer);

  ASSERT_EQ(hxs_start_timer(), hxOK);
}

//////////////////////////////////////////////////
/// \brief Check hxs_stop_timer.
TEST(hxsTest, hxs_stop_timer)
{
  setup();

  ignition::transport::Node node;

  // Advertise the "hxs_stop_timer" service.
  node.Advertise("/haptix/gazebo/hxs_stop_timer", onHxsStopTimer);

  ASSERT_EQ(hxs_stop_timer(), hxOK);
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
