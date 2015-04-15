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
#include <string>
#include <mutex>
#include <ignition/transport.hh>
#include "haptix/comm/haptix.h"
#include "msg/hxContactPoint_V.pb.h"
#include "msg/hxEmpty.pb.h"
#include "msg/hxJoint.pb.h"
#include "msg/hxLink.pb.h"
#include "msg/hxModel.pb.h"
#include "msg/hxQuaternion.pb.h"
#include "msg/hxSimInfo.pb.h"
#include "msg/hxTime.pb.h"
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
/// \return Pointer to the Ignition Transport node.
static ignition::transport::Node *getHxNode()
{
  if (!haptixSimUtilsNode)
    haptixSimUtilsNode = new ignition::transport::Node();

  return haptixSimUtilsNode;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf message to a scalar.
/// \param[_in] Protobuf message with a [data] field.
/// \param[_out] Output value.
/// \return True if the function succeed or false otherwise.
template <typename T, typename T2> bool hxs_convertScalar(T _in, T2 *_out)
{
  if (!_in.has_data())
  {
    std::cerr << "hxs_convertScalar() error: No [data] in msg" << std::endl;
    return false;
  }
  *_out = _in.data();
  return true;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxTime message to a
/// C struct hxTime.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertTime(const haptix::comm::msgs::hxTime _in, hxTime *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxTime));

  _out->sec = _in.sec();
  _out->nsec = _in.nsec();

  return true;
}
//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxVector3 message to a
/// C struct hxVector3.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertVector3(const haptix::comm::msgs::hxVector3 _in,
  hxVector3 *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxVector3));

  _out->x = _in.x();
  _out->y = _in.y();
  _out->z = _in.z();

  return true;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a C struct hxVector3. to a
/// protobuf hxVector3 message.
/// \param[in] _in C-struct.
/// \param[out] _out Protobuf message.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertVector3(const hxVector3 *_in,
  haptix::comm::msgs::hxVector3 *_out)
{
  if (!_in)
  {
    std::cerr << "hxs_convertVector3() error: NULL input" << std::endl;
    return false;
  }

  // Initialize the protobuf message.
  _out->Clear();

  _out->set_x(_in->x);
  _out->set_y(_in->y);
  _out->set_z(_in->z);

  return true;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxQuaternion message to a
/// C struct hxQuaternion.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertQuaternion(const haptix::comm::msgs::hxQuaternion _in,
  hxQuaternion *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxQuaternion));

  _out->w = _in.w();
  _out->x = _in.x();
  _out->y = _in.y();
  _out->z = _in.z();

  return true;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a C struct hxQuaternion to a
/// protobuf hxQuaternion message.
/// \param[in] _in C-struct.
/// \param[out] _out Protobuf message.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertQuaternion(const hxQuaternion *_in,
  haptix::comm::msgs::hxQuaternion *_out)
{
  if (!_in)
  {
    std::cerr << "hxs_convertQuaternion() error: NULL input" << std::endl;
    return false;
  }

  // Initialize the protobuf message.
  _out->Clear();

  _out->set_w(_in->w);
  _out->set_x(_in->x);
  _out->set_y(_in->y);
  _out->set_z(_in->z);

  return true;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxQuaternion message to a
/// C struct hxQuaternion.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertTransform(const haptix::comm::msgs::hxTransform _in,
  hxTransform *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxTransform));

  hxs_convertVector3(_in.pos(), &_out->pos);
  hxs_convertQuaternion(_in.orient(), &_out->orient);

  return true;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a C struct hxQuaternion to a
/// protobuf hxQuaternion message.
/// \param[in] _in C-struct.
/// \param[out] _out Protobuf message.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertTransform(const hxTransform *_in,
  haptix::comm::msgs::hxTransform *_out)
{
  if (!_in)
  {
    std::cerr << "hxs_convertTransform() error: NULL input" << std::endl;
    return false;
  }

  // Initialize the message.
  _out->Clear();

  hxs_convertVector3(&(_in->pos), _out->mutable_pos());
  hxs_convertQuaternion(&(_in->orient), _out->mutable_orient());

  return true;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxJoint message to a
/// C struct hxJoint.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertJoint(const haptix::comm::msgs::hxJoint _in,
  hxJoint *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxJoint));

  if (_in.name().size() > hxMAXNAMESIZE - 1)
  {
    std::cerr << "hxs_convertJoint() error: The name of the joint ["
              << _in.name() << "] exceeds the maximum size allowed ("
              << hxMAXNAMESIZE << " chars)." << std::endl;
    return false;
  }

  strncpy(_out->name, _in.name().c_str(), strlen(_in.name().c_str()));
  _out->name[strlen(_in.name().c_str())] = '\0';
  _out->pos = _in.pos();
  _out->vel = _in.vel();
  _out->torque_motor = _in.torque_motor();
  _out->torque_passive = _in.torque_passive();

  return true;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a C struct hxJoint to a
/// protobuf hxJoint message.
/// \param[in] _in C-struct.
/// \param[out] _out Protobuf message.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertJoint(const hxJoint *_in,
  haptix::comm::msgs::hxJoint *_out)
{
  if (!_in)
  {
    std::cerr << "hxs_convertJoint() error: NULL input" << std::endl;
    return false;
  }

  // Initialize the message.
  _out->Clear();

  _out->set_name(std::string(_in->name));
  _out->set_pos(_in->pos);
  _out->set_vel(_in->vel);
  _out->set_torque_motor(_in->torque_motor);
  _out->set_torque_passive(_in->torque_passive);

  return true;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxLink message to a
/// C struct hxLink.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertLink(const haptix::comm::msgs::hxLink _in, hxLink *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxLink));

  if (_in.name().size() > hxMAXNAMESIZE - 1)
  {
    std::cerr << "hxs_convertLink() error: The name of the link ["
              << _in.name() << "] exceeds the maximum size allowed ("
              << hxMAXNAMESIZE << " chars)." << std::endl;
    return false;
  }

  strncpy(_out->name, _in.name().c_str(), strlen(_in.name().c_str()));
  _out->name[strlen(_in.name().c_str())] = '\0';
  hxs_convertTransform(_in.transform(), &_out->transform);
  hxs_convertVector3(_in.linvel(), &_out->linvel);
  hxs_convertVector3(_in.angvel(), &_out->angvel);
  hxs_convertVector3(_in.linacc(), &_out->linacc);
  hxs_convertVector3(_in.angacc(), &_out->angacc);

  return true;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a C struct hxLink to a
/// protobuf hxLink message.
/// \param[in] _in C-struct.
/// \param[out] _out Protobuf message.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertLink(const hxLink *_in, haptix::comm::msgs::hxLink *_out)
{
  if (!_in)
  {
    std::cerr << "hxs_convertLink() error: NULL input" << std::endl;
    return false;
  }

  // Initialize the message.
  _out->Clear();

  _out->set_name(std::string(_in->name));
  hxs_convertTransform(&_in->transform, _out->mutable_transform());
  hxs_convertVector3(&_in->linvel, _out->mutable_linvel());
  hxs_convertVector3(&_in->angvel, _out->mutable_angvel());
  hxs_convertVector3(&_in->linacc, _out->mutable_linacc());
  hxs_convertVector3(&_in->angacc, _out->mutable_angacc());

  return true;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxModel message to a
/// C struct hxModel.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertModel(const haptix::comm::msgs::hxModel _in,
  hxModel *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxModel));

  if (_in.name().size() > hxMAXNAMESIZE - 1)
  {
    std::cerr << "hxs_convertModel() error: The name of the model ["
              << _in.name() << "] exceeds the maximum size allowed ("
              << hxMAXNAMESIZE << " chars)." << std::endl;
    return false;
  }

  strncpy(_out->name, _in.name().c_str(), strlen(_in.name().c_str()));
  _out->name[strlen(_in.name().c_str())] = '\0';

  hxs_convertTransform(_in.transform(), &(_out->transform));
  _out->id = _in.id();
  _out->link_count = _in.links_size();
  _out->joint_count = _in.joints_size();

  // Fill the links.
  for (int i = 0; i < _out->link_count; ++i)
    hxs_convertLink(_in.links(i), &_out->links[i]);

  // Fill the joints.
  for (int i = 0; i < _out->joint_count; ++i)
    hxs_convertJoint(_in.joints(i), &_out->joints[i]);

  _out->gravity = _in.gravity();

  return true;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a C struct hxModel to a
/// protobuf hxModel message.
/// \param[in] _in C-struct.
/// \param[out] _out Protobuf message.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertModel(const hxModel *_in,
  haptix::comm::msgs::hxModel *_out)
{
  if (!_in)
  {
    std::cerr << "hxs_convertModel() error: NULL input" << std::endl;
    return false;
  }

  // Initialize the message.
  _out->Clear();

  _out->set_name(_in->name);

  hxs_convertTransform(&(_in->transform), _out->mutable_transform());
  _out->set_id(_in->id);

  // Create the links.
  for (int i = 0; i < _in->link_count; ++i)
  {
    haptix::comm::msgs::hxLink *link = _out->add_links();
    hxs_convertLink(&_in->links[i], link);
  }

  // Create the joints.
  for (int i = 0; i < _in->joint_count; ++i)
  {
    haptix::comm::msgs::hxJoint *joint = _out->add_joints();
    hxs_convertJoint(&_in->joints[i], joint);
  }

  _out->set_gravity(_in->gravity);

  return true;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxContactPoint_V message to
/// a C struct hxContactPoints.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertContactPoints(
  const haptix::comm::msgs::hxContactPoint_V _in, hxContactPoints *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxContactPoints));

  _out->contactCount = _in.contacts_size();

  for (int i = 0; i < _out->contactCount; ++i)
  {

    if (_in.contacts(i).link1().size() > hxMAXNAMESIZE - 1)
    {
      std::cerr << "hxs_convertContactPoints() error: The name of the link1 ["
                << _in.contacts(i).link1() << "] exceeds the maximum size "
                << "allowed (" << hxMAXNAMESIZE << " chars)." << std::endl;
      return false;
    }

    if (_in.contacts(i).link2().size() > hxMAXNAMESIZE - 1)
    {
      std::cerr << "hxs_convertContactPoints() error: The name of the link2 ["
                << _in.contacts(i).link2() << "] exceeds the maximum size "
                << "allowed (" << hxMAXNAMESIZE << " chars)." << std::endl;
      return false;
    }

    strncpy(_out->contacts[i].link1, _in.contacts(i).link1().c_str(),
      strlen(_in.contacts(i).link1().c_str()));
    _out->contacts[i].link1[strlen(_in.contacts(i).link1().c_str())] = '\0';

    strncpy(_out->contacts[i].link2, _in.contacts(i).link2().c_str(),
      strlen(_in.contacts(i).link2().c_str()));
    _out->contacts[i].link2[strlen(_in.contacts(i).link2().c_str())] = '\0';

    hxs_convertVector3(_in.contacts(i).point(), &_out->contacts[i].point);
    hxs_convertVector3(_in.contacts(i).normal(), &_out->contacts[i].normal);
    hxs_convertVector3(_in.contacts(i).tangent1(), &_out->contacts[i].tangent1);
    hxs_convertVector3(_in.contacts(i).tangent2(), &_out->contacts[i].tangent2);
    _out->contacts[i].distance = _in.contacts(i).distance();
    hxs_convertVector3(_in.contacts(i).force(), &_out->contacts[i].force);
  }

  return true;
}

//////////////////////////////////////////////////
/// \brief Private function that converts a protobuf hxSimInfo message to a
/// C struct hxSimInfo.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertSimInfo(const haptix::comm::msgs::hxSimInfo _in,
  hxSimInfo *_out)
{
  // Initialize the C struct.
  memset(_out, 0, sizeof(hxSimInfo));

  _out->modelCount = _in.models_size();

  // Fill the models.
  for (int i = 0; i < _out->modelCount; ++i)
    hxs_convertModel(_in.models(i), &_out->models[i]);

  // Fill the camera.
  hxs_convertTransform(_in.camera_transform(), &_out->camera_transform);

  return true;
}

//////////////////////////////////////////////////
/// \brief Function that requests a given service and converts the protobuf
/// response to a C-struct. The conversion from Protobuf to C-struct is done
/// via a function that is also passed as a parameter.
/// \param[in] _service Service name to request.
/// \param[in] _funcName Name of the caller function.
/// \param[in] _req Protobuf message used in the service request.
/// \param[out] _rep Protobuf message used in the service response.
/// \param[out] _dst Target C-Struct that will be converted from _rep.
/// \param[in] _f Function that will be able to convert the _rep Protobuf
/// message into the C-struct _dst.
template<typename REQ, typename REP, typename T>
hxResult hxs_call(const std::string &_service,
                  const std::string &_funcName,
                  const REQ _req,
                  REP _rep,
                  T _dst,
                  bool(*_f)(const REP _rep, T _dst))
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
      if (_f(_rep, _dst))
        return hxOK;
      else
        return hxERROR;
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
/// \brief Function that requests a given service.
/// \param[in] _service Service name to request.
/// \param[in] _funcName Name of the caller function.
/// \param[in] _req Protobuf message used in the service request.
/// \param[out] _rep Protobuf message used in the service response.
template<typename REQ, typename REP>
hxResult hxs_call(const std::string &_service,
                  const std::string &_funcName,
                  const REQ _req,
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
