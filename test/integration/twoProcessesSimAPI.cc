/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gtest/gtest.h"
#include "haptix/comm/haptix.h"
#include "haptix/comm/haptix_sim.h"
#include "msg/hxCommand.pb.h"
#include "msg/hxContactPoint.pb.h"
#include "msg/hxContactPoint_V.pb.h"
#include "msg/hxJoint.pb.h"
#include "msg/hxLink.pb.h"
#include "msg/hxModel.pb.h"
#include "msg/hxRobot.pb.h"
#include "msg/hxSensor.pb.h"
#include "msg/hxSimInfo.pb.h"
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
/// \brief Two nodes running on two different processes. The node in this test
/// will exercise the HAPTIX simulation API, whereas the node in the external
/// process will simulate the HAPTIX Gazebo plugin providing the HAPTIX services
TEST(twoProcessesSimAPI, hxs_sim_info)
{
  // Initialize test.
  setup();

  // Launch an ignition transport node that will advertise services.
  std::string responserPath = testing::portablePathUnion(
    PROJECT_BINARY_PATH, "test/integration/INTEGRATION_hx_responser");

  testing::forkHandlerType pi = testing::forkAndRun(responserPath.c_str(),
    partition.c_str());

  hxsSimInfo *simInfo = new hxsSimInfo();

  // ---------- hxs_sim_info ----------
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

  // ---------- hxs_camera_transform ----------
  hxsTransform camInfo;

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

  // ---------- hxs_set_camera_transform ----------
  ASSERT_EQ(hxs_set_camera_transform(&(simInfo->camera_transform)), hxOK);

  // ---------- hxs_contacts ----------
  hxsContactPoints contactsInfo;

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

  // ---------- hxs_set_model_joint_state ----------
  hxsModel model;

  EXPECT_EQ(hxs_set_model_joint_state("model 0", "joint 1", 1.0f, 2.0f), hxOK);

  // ---------- hxs_model_joint_state ----------
  hxsModel jointState;
  EXPECT_EQ(hxs_model_joint_state("model 0", &jointState), hxOK);
  EXPECT_EQ(jointState.joint_count, 1);
  for (int i = 0; i < jointState.joint_count; ++i)
  {
    EXPECT_FLOAT_EQ(jointState.joints[i].pos, 3.0f);
    EXPECT_FLOAT_EQ(jointState.joints[i].vel, 4.0f);
  }

  // ---------- hxs_add_model ----------
  std::string urdf = "fake URDF";
  std::string name = "model 1";
  float x = 1.0f;
  float y = 2.0f;
  float z = 3.0f;
  float roll = 4.0f;
  float pitch = 5.0f;
  float yaw = 6.0f;

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

  // ---------- hxs_remove_model ----------
  ASSERT_EQ(hxs_remove_model("model 1"), hxOK);

  // ---------- hxs_set_model_transform ----------
  ASSERT_EQ(hxs_set_model_transform("model 1", &(simInfo->models[1].transform)),
    hxOK);

  // ---------- hxs_model_transform ----------
  hxsTransform transform;

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

  // ---------- hxs_model_gravity_mode ----------
  int gravity_mode = 0;
  ASSERT_EQ(hxs_model_gravity_mode("model_1", &gravity_mode), hxOK);
  EXPECT_EQ(gravity_mode, 1);

  // ---------- hxs_set_model_gravity_mode ----------
  ASSERT_EQ(hxs_set_model_gravity_mode("model_1", 1), hxOK);

  // ---------- hxs_linear_velocity ----------
  hxsVector3 lin_vel;
  ASSERT_EQ(hxs_linear_velocity("model 1", &lin_vel), hxOK);

  // Verify linear velocity.
  EXPECT_FLOAT_EQ(lin_vel.x, 1.0f);
  EXPECT_FLOAT_EQ(lin_vel.y, 1.1f);
  EXPECT_FLOAT_EQ(lin_vel.z, 1.2f);

  // ---------- hxs_set_linear_velocity ----------
  lin_vel.x = 1.0f;
  lin_vel.y = 1.1f;
  lin_vel.z = 1.2f;

  ASSERT_EQ(hxs_set_linear_velocity("model 1", &lin_vel), hxOK);

  // ---------- hxs_angular_velocity ----------
  hxsVector3 ang_vel;
  ASSERT_EQ(hxs_angular_velocity("model 1", &ang_vel), hxOK);

  // Verify the angular velocity.
  EXPECT_FLOAT_EQ(ang_vel.x, 2.0f);
  EXPECT_FLOAT_EQ(ang_vel.y, 2.1f);
  EXPECT_FLOAT_EQ(ang_vel.z, 2.2f);

  // ---------- hxs_set_angular_velocity ----------
  ang_vel = {0.0f, 0.0f, 0.0f};

  ang_vel.x = 2.0f;
  ang_vel.y = 2.1f;
  ang_vel.z = 2.2f;

  ASSERT_EQ(hxs_set_angular_velocity("model 1", &ang_vel), hxOK);

  // ---------- hxs_apply_force ----------
  hxsVector3 force;

  // Set some force.
  force.x = 5.1f;
  force.y = 5.2f;
  force.z = 5.3f;

  // Use the first link of the first model in simState.
  ASSERT_EQ(hxs_apply_force(simInfo->models[0].name,
      simInfo->models[0].links[0].name, &force, 0.1), hxOK);

  // ---------- hxs_apply_torque ----------
  hxsVector3 torque;

  // Set some torque.
  torque.x = 6.1f;
  torque.y = 6.2f;
  torque.z = 6.3f;

  // Use the first link of the first model in simState.
  ASSERT_EQ(hxs_apply_torque(simInfo->models[0].name,
      simInfo->models[0].links[0].name, &torque, 0.1), hxOK);

  // ---------- hxs_apply_wrench ----------
  hxsWrench wrench;

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

  // ---------- hxs_reset ----------
  int resetLimbPose = 1;
  ASSERT_EQ(hxs_reset(resetLimbPose), hxOK);

  // ---------- hxs_start_logging ----------
  std::string filename = "a filename";
  ASSERT_EQ(hxs_start_logging(filename.c_str()), hxOK);

  // ---------- hxs_is_logging ----------
  int isLogging;

  ASSERT_EQ(hxs_is_logging(&isLogging), hxOK);

  // Verify the result.
  EXPECT_EQ(isLogging, 1);

  // ---------- hxs_stop_logging ----------
  ASSERT_EQ(hxs_stop_logging(), hxOK);

  // ---------- hxs_set_model_color ----------
  hxsColor color;

  color.r = 0.5f;
  color.g = 0.6f;
  color.b = 0.7f;
  color.alpha = 0.8f;

  ASSERT_EQ(hxs_set_model_color("model_1", &color), hxOK);

  // ---------- hxs_model_color ----------
  ASSERT_EQ(hxs_model_color("model_1", &color), hxOK);

  // Check the color received.
  EXPECT_FLOAT_EQ(color.r, 0.1f);
  EXPECT_FLOAT_EQ(color.g, 0.2f);
  EXPECT_FLOAT_EQ(color.b, 0.3f);
  EXPECT_FLOAT_EQ(color.alpha, 0.4f);

  // ---------- hxs_set_model_collide_mode ----------
  hxsCollideMode collideMode = hxsCOLLIDE;
  ASSERT_EQ(hxs_set_model_collide_mode("model_1", &collideMode), hxOK);

  // ---------- hxs_model_collide_mode ----------
  ASSERT_EQ(hxs_model_collide_mode("model_1", &collideMode), hxOK);

  // Check the collide mode received.
  EXPECT_EQ(collideMode, hxsDETECTIONONLY);

  // ---------- hxs_add_constraint ----------
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
  std::string model_name = "model 1";

  // Create a new constraint.
  ASSERT_EQ(hxs_add_constraint(constraint_sdf.c_str(), model_name.c_str()), hxOK);

  /// \TODO: Verify that the new constraint

  // ---------- hxs_remove_model ----------
  ASSERT_EQ(hxs_remove_constraint("cricket_ball_constraint", "model 1"), hxOK);

  // Teardown.
  delete simInfo;
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
