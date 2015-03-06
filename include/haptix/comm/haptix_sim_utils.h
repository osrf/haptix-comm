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

/// \file haptix_sim_utils.h
/// \brief This file contains auxiliary functions for haptix_sim.

#ifndef __HAPTIX_SIM_UTILS_API_HAPTIX_H__
#define __HAPTIX_SIM_UTILS_API_HAPTIX_H__

#include <iostream>
#include <mutex>
#include <ignition/transport.hh>
#include "haptix/comm/haptix.h"
#include "msg/hxCamera.pb.h"
#include "msg/hxContact.pb.h"
#include "msg/hxContact_V.pb.h"
#include "msg/hxEmpty.pb.h"
#include "msg/hxJoint.pb.h"
#include "msg/hxLink.pb.h"
#include "msg/hxModel.pb.h"
#include "msg/hxQuaternion.pb.h"
#include "msg/hxSimInfo.pb.h"
#include "msg/hxTransform.pb.h"
#include "msg/hxVector3.pb.h"

/// \brief Timeout used for the service requests (ms.).
unsigned int kTimeout = 1000;

/// \brief Error string, to be retrieved by hx_last_result()
static std::string lastResult;
static std::mutex lastResultLock;

/// \brief ignition transport node.
static ignition::transport::Node *haptixSimUtilsNode = NULL;

//////////////////////////////////////////////////
/// \brief Private function that creates an Ignition Transport node
/// or return a pointer to it if has been already created.
static ignition::transport::Node *getHxNode()
{
  if (!haptixSimUtilsNode)
    haptixSimUtilsNode = new ignition::transport::Node();

  return haptixSimUtilsNode;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf message to a scalar.
template <typename T, typename T2> void hxs_convertScalar(T _in, T2 *_out)
{
  *_out = _in.data();
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxVector3 message to a
/// C struct hxVector3.
static void hxs_convertVector3(const haptix::comm::msgs::hxVector3 _in,
  hxVector3 *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxVector3));

  _out->x = _in.x();
  _out->y = _in.y();
  _out->z = _in.z();
}

//////////////////////////////////////////////////
/// \brief Private function that converts a C struct hxVector3. to a
/// protobuf hxVector3 message.
static void hxs_convertVector3(const hxVector3 *_in,
  haptix::comm::msgs::hxVector3 *_out)
{
  // Initialize the protobuf message.
  _out->Clear();

  _out->set_x(_in->x);
  _out->set_y(_in->y);
  _out->set_z(_in->z);

}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxQuaternion message to a
/// C struct hxQuaternion.
static void hxs_convertQuaternion(const haptix::comm::msgs::hxQuaternion _in,
  hxQuaternion *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxQuaternion));

  _out->w = _in.w();
  _out->x = _in.x();
  _out->y = _in.y();
  _out->z = _in.z();
}

//////////////////////////////////////////////////
/// \brief Private function that converts a C struct hxQuaternion to a
/// protobuf hxQuaternion message.
static void hxs_convertQuaternion(const hxQuaternion *_in,
  haptix::comm::msgs::hxQuaternion *_out)
{
  // Initialize the protobuf message.
  _out->Clear();

  _out->set_w(_in->w);
  _out->set_x(_in->x);
  _out->set_y(_in->y);
  _out->set_z(_in->z);
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxQuaternion message to a
/// C struct hxQuaternion.
static void hxs_convertTransform(const haptix::comm::msgs::hxTransform _in,
  hxTransform *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxTransform));

  hxs_convertVector3(_in.pos(), &(_out->pos));
  hxs_convertQuaternion(_in.orient(), &(_out->orient));
}

//////////////////////////////////////////////////
/// \brief Private function that converts a C struct hxQuaternion to a
/// protobuf hxQuaternion message.
static void hxs_convertTransform(const hxTransform *_in,
  haptix::comm::msgs::hxTransform *_out)
{
  // Initialize the message.
  _out->Clear();

  hxs_convertVector3(&(_in->pos), _out->mutable_pos());
  hxs_convertQuaternion(&(_in->orient), _out->mutable_orient());
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxJoint message to a
/// C struct hxJoint.
static void hxs_convertJoint(const haptix::comm::msgs::hxJoint _in,
  hxJoint *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxJoint));

  _out->pos = _in.pos();
  _out->vel = _in.vel();
  _out->acc = _in.acc();
  _out->torque_motor = _in.torque_motor();
  _out->torque_passive = _in.torque_passive();
}

//////////////////////////////////////////////////
/// \brief Private function that converts a C struct hxJoint to a
/// protobuf hxJoint message.
static void hxs_convertJoint(const hxJoint *_in,
  haptix::comm::msgs::hxJoint *_out)
{
  // Initialize the message.
  _out->Clear();

  _out->set_pos(_in->pos);
  _out->set_vel(_in->vel);
  _out->set_acc(_in->acc);
  _out->set_torque_motor(_in->torque_motor);
  _out->set_torque_passive(_in->torque_passive);
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxLink message to a
/// C struct hxLink.
static void hxs_convertLink(const haptix::comm::msgs::hxLink _in, hxLink *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxLink));

  hxs_convertTransform(_in.transform(), &(_out->transform));
  hxs_convertVector3(_in.linvel(), &(_out->linvel));
  hxs_convertVector3(_in.angvel(), &(_out->angvel));
  hxs_convertVector3(_in.linacc(), &(_out->linacc));
  hxs_convertVector3(_in.angacc(), &(_out->angacc));
}

//////////////////////////////////////////////////
/// \brief Private function that converts a C struct hxLink to a
/// protobuf hxLink message.
static void hxs_convertLink(const hxLink *_in, haptix::comm::msgs::hxLink *_out)
{
  // Initialize the message.
  _out->Clear();

  hxs_convertTransform(&(_in->transform), _out->mutable_transform());
  hxs_convertVector3(&(_in->linvel), _out->mutable_linvel());
  hxs_convertVector3(&(_in->angvel), _out->mutable_angvel());
  hxs_convertVector3(&(_in->linacc), _out->mutable_linacc());
  hxs_convertVector3(&(_in->angacc), _out->mutable_angacc());
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxModel message to a
/// C struct hxModel.
static void hxs_convertModel(const haptix::comm::msgs::hxModel _in,
  hxModel *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxModel));

  hxs_convertTransform(_in.transform(), &(_out->transform));
  _out->is_static = _in.is_static();
  _out->id = _in.id();
  _out->link_count = _in.links_size();
  _out->joint_count = _in.joints_size();

  // Fill the links.
  for (int i = 0; i < _out->link_count; ++i)
    hxs_convertLink(_in.links(i), &(_out->links[i]));

  // Fill the joints.
  for (int i = 0; i < _out->joint_count; ++i)
    hxs_convertJoint(_in.joints(i), &(_out->joints[i]));
}

//////////////////////////////////////////////////
/// \brief Private function that converts a C struct hxModel to a
/// protobuf hxModel message.
/*static void hxs_convertModel(const hxModel *_in,
  haptix::comm::msgs::hxModel *_out)
{
  // Initialize the message.
  _out->Clear();

  hxs_convertTransform(&(_in->transform), _out->mutable_transform());
  _out->set_is_static(_in->is_static);
  _out->set_id(_in->id);

  // Create the links.
  for (int i = 0; i < _in->link_count; ++i)
  {
    haptix::comm::msgs::hxLink *link = _out->add_links();
    hxs_convertLink(&(_in->links[i]), link);
  }

  // Create the joints.
  for (int i = 0; i < _in->joint_count; ++i)
  {
    haptix::comm::msgs::hxJoint *joint = _out->add_joints();
    hxs_convertJoint(&(_in->joints[i]), joint);
  }
}*/

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxContact_V message to a
/// C struct hxContact.
static void hxs_convertContact(const haptix::comm::msgs::hxContact_V _in,
  hxContact *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxContact));

  _out->contactCount = _in.contacts_size();

  for (int i = 0; i < _out->contactCount; ++i)
  {
    _out->body1[i] = _in.contacts(i).body1();
    _out->body2[i] = _in.contacts(i).body2();
    hxs_convertVector3(_in.contacts(i).point(), &(_out->point[i]));
    hxs_convertVector3(_in.contacts(i).normal(), &(_out->normal[i]));
    hxs_convertVector3(_in.contacts(i).tangent1(), &(_out->tangent1[i]));
    hxs_convertVector3(_in.contacts(i).tangent2(), &(_out->tangent2[i]));
    _out->distance[i] = _in.contacts(i).distance();
    hxs_convertVector3(_in.contacts(i).velocity(), &(_out->velocity[i]));
    hxs_convertVector3(_in.contacts(i).force(), &(_out->force[i]));
  }
}

//////////////////////////////////////////////////
/// \brief Private function that converts a C struct hxContact to a
/// protobuf hxContact message.
/*static void hxs_convertContact(const hxContact *_in,
  haptix::comm::msgs::hxContact *_out)
{
  // Initialize the message.
  _out->Clear();

  for (int i = 0; i < _in->contactCount; ++i)
  {
    _out->add_body1(_in->body1[i]);
    _out->add_body2(_in->body2[i]);

    haptix::comm::msgs::hxVector3 *point = _out->add_point();
    hxs_convertVector3(&(_in->point[i]), point);

    haptix::comm::msgs::hxVector3 *normal = _out->add_normal();
    hxs_convertVector3(&(_in->normal[i]), normal);

    haptix::comm::msgs::hxVector3 *tangent1 = _out->add_tangent1();
    hxs_convertVector3(&(_in->tangent1[i]), tangent1);

    haptix::comm::msgs::hxVector3 *tangent2 = _out->add_tangent2();
    hxs_convertVector3(&(_in->tangent2[i]), tangent2);

    _out->add_distance(_in->distance[i]);

    haptix::comm::msgs::hxVector3 *velocity = _out->add_velocity();
    hxs_convertVector3(&(_in->velocity[i]), velocity);

    haptix::comm::msgs::hxVector3 *force = _out->add_force();
    hxs_convertVector3(&(_in->force[i]), force);
  }
}*/

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxCamera message to a
/// C struct hxCamera.
static void hxs_convertCamera(const haptix::comm::msgs::hxCamera _in,
  hxCamera *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxCamera));

  hxs_convertTransform(_in.transform(), &(_out->transform));
}

//////////////////////////////////////////////////
/// \brief Private function that converts a  C struct hxCamera to a
/// protobuf hxCamera message.
/*static void hxs_convertCamera(const hxCamera *_in,
  haptix::comm::msgs::hxCamera *_out)
{
  // Initialize the protobuf message.
  _out->Clear();

  hxs_convertTransform(&(_in->transform), _out->mutable_transform());
}*/

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxSimInfo message to a
/// C struct hxSimInfo.
static void hxs_convertSimInfo(const haptix::comm::msgs::hxSimInfo _in,
  hxSimInfo *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxSimInfo));

  _out->modelCount = _in.models_size();

  // Fill the models.
  for (int i = 0; i < _out->modelCount; ++i)
    hxs_convertModel(_in.models(i), &(_out->models[i]));

  // Fill the camera.
  hxs_convertCamera(_in.camera(), &(_out->camera));
}

//////////////////////////////////////////////////
/// \brief Private function that converts a C struct hxSimInfo to a
/// protobuf hxSimInfo message.
/*static void hxs_convertSimInfo(const hxSimInfo *_in,
  haptix::comm::msgs::hxSimInfo *_out)
{
  // Initialize the protobuf message.
  _out->Clear();

  // Create the models.
  for (int i = 0; i < _in->modelCount; ++i)
  {
    haptix::comm::msgs::hxModel *model = _out->add_models();
    hxs_convertModel(&(_in->models[i]), model);
  }

  // Fill the camera.
  hxs_convertCamera(&(_in->camera), _out->mutable_camera());
}*/

//////////////////////////////////////////////////
/// \brief .
template<typename REQ, typename REP, typename T>
hxResult hxs_call(const std::string &_service,
                  const std::string &_funcName,
                  REQ _req,
                  REP _rep,
                  T _dst,
                  void(*_f)(const REP _rep, T _dst))
{
  bool result;
  ignition::transport::Node *hxNode = getHxNode();

  // Request the service.
  bool executed = hxNode->Request(_service, _req, kTimeout, _rep, result);

  if (executed)
  {
    if (result)
    {
      // Fill the struct with the response.
      _f(_rep, _dst);
      return hxOK;
    }
    else
    {
      std::lock_guard<std::mutex> lock(lastResultLock);
      lastResult = _funcName + "() Service call failed.";
      std::cerr << lastResult << std::endl;
    }
  }
  else
  {
    std::lock_guard<std::mutex> lock(lastResultLock);
    lastResult = _funcName + "() Service call timed out.";
    std::cerr << lastResult << std::endl;
  }

  return hxERROR;
}

//////////////////////////////////////////////////
/// \brief .
template<typename REQ, typename REP>
hxResult hxs_call(const std::string &_service,
                  const std::string &_funcName,
                  REQ _req,
                  REP _rep)
{
  bool result;
  ignition::transport::Node *hxNode = getHxNode();

  // Request the service.
  bool executed = hxNode->Request(_service, _req, kTimeout, _rep, result);

  if (executed)
  {
    if (result)
      return hxOK;
    else
    {
      std::lock_guard<std::mutex> lock(lastResultLock);
      lastResult = _funcName + "() Service call failed.";
      std::cerr << lastResult << std::endl;
    }
  }
  else
  {
    std::lock_guard<std::mutex> lock(lastResultLock);
    lastResult = _funcName + "() Service call timed out.";
    std::cerr << lastResult << std::endl;
  }

  return hxERROR;
}

#endif
