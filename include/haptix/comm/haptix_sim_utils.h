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

/// \file haptix_sim_utils.h
/// \brief This file contains auxiliary functions for haptix_sim.

#ifndef __HAPTIX_SIM_UTILS_API_HAPTIX_H__
#define __HAPTIX_SIM_UTILS_API_HAPTIX_H__

#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <ignition/transport.hh>
#include "haptix/comm/haptix.h"
#include "msg/hxCollideMode.pb.h"
#include "msg/hxColor.pb.h"
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
/// \internal Private function that creates an Ignition Transport node
/// or return a pointer to it if has been already created.
/// \return Pointer to the Ignition Transport node.
static ignition::transport::Node *getHxNode()
{
  if (!haptixSimUtilsNode)
    haptixSimUtilsNode = new ignition::transport::Node();

  return haptixSimUtilsNode;
}

//////////////////////////////////////////////////
/// \internal Private function that converts a protobuf message to a scalar.
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

  if (!_out)
  {
    std::cerr << "hxs_convertScalar() error: NULL output" << std::endl;
    return false;
  }

  *_out = _in.data();
  return true;
}

//////////////////////////////////////////////////
/// \internal Private function that converts a protobuf hxVector3 message to a
/// C struct hxsVector3.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertVector3(const haptix::comm::msgs::hxVector3 _in,
  hxsVector3 *_out)
{
  if (!_out)
  {
    std::cerr << "hxs_convertVector3() error: NULL output" << std::endl;
    return false;
  }

  // Initialize the C struct.
  memset(_out, 0, sizeof(hxsVector3));

  _out->x = _in.x();
  _out->y = _in.y();
  _out->z = _in.z();

  return true;
}

//////////////////////////////////////////////////
/// \internal Private function that converts a C struct hxsVector3. to a
/// protobuf hxVector3 message.
/// \param[in] _in C-struct.
/// \param[out] _out Protobuf message.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertVector3(const hxsVector3 *_in,
  haptix::comm::msgs::hxVector3 *_out)
{
  if (!_in)
  {
    std::cerr << "hxs_convertVector3() error: NULL input" << std::endl;
    return false;
  }

  if (!_out)
  {
    std::cerr << "hxs_convertVector3() error: NULL output" << std::endl;
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
/// \internal Private function that converts a protobuf hxQuaternion message to
/// a C struct hxsQuaternion.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertQuaternion(const haptix::comm::msgs::hxQuaternion _in,
  hxsQuaternion *_out)
{
  if (!_out)
  {
    std::cerr << "hxs_convertQuaternion() error: NULL output" << std::endl;
    return false;
  }

  // Initialize the C struct.
  memset(_out, 0, sizeof(hxsQuaternion));

  _out->w = _in.w();
  _out->x = _in.x();
  _out->y = _in.y();
  _out->z = _in.z();

  return true;
}

//////////////////////////////////////////////////
/// \internal Private function that converts a C struct hxsQuaternion to a
/// protobuf hxQuaternion message.
/// \param[in] _in C-struct.
/// \param[out] _out Protobuf message.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertQuaternion(const hxsQuaternion *_in,
  haptix::comm::msgs::hxQuaternion *_out)
{
  if (!_in)
  {
    std::cerr << "hxs_convertQuaternion() error: NULL input" << std::endl;
    return false;
  }

  if (!_out)
  {
    std::cerr << "hxs_convertQuaternion() error: NULL output" << std::endl;
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
/// \internal Private function that converts a protobuf hxColor message to
/// a C struct hxsColor.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertColor(const haptix::comm::msgs::hxColor _in,
  hxsColor *_out)
{
  if (!_out)
  {
    std::cerr << "hxs_convertColor() error: NULL output" << std::endl;
    return false;
  }

  // Initialize the C struct.
  memset(_out, 0, sizeof(hxsColor));

  _out->r = _in.r();
  _out->g = _in.g();
  _out->b = _in.b();
  _out->alpha = _in.alpha();

  return true;
}

//////////////////////////////////////////////////
/// \internal Private function that converts a protobuf hxCollideMode message
/// to a C struct hxsCollideMode.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertCollisionMode(
  const haptix::comm::msgs::hxCollideMode _in, hxsCollideMode *_out)
{
  if (!_out)
  {
    std::cerr << "hxs_convertCollisionMode() error: NULL output" << std::endl;
    return false;
  }

  // Initialize the C struct.
  memset(_out, 0, sizeof(hxsCollideMode));

  switch (_in.mode())
  {
    case haptix::comm::msgs::hxCollideMode::hxsNOCOLLIDE:
      *_out = hxsNOCOLLIDE;
      break;
    case haptix::comm::msgs::hxCollideMode::hxsDETECTIONONLY:
      *_out = hxsDETECTIONONLY;
      break;
    case haptix::comm::msgs::hxCollideMode::hxsCOLLIDE:
      *_out = hxsCOLLIDE;
      break;
    default:
      std::cerr << "hxs_convertCollisionMode() error: Unkown collision mode ["
                << _in.mode() << "]" << std::endl;
      return false;
  }

  return true;
}

//////////////////////////////////////////////////
/// \internal Private function that converts a protobuf hxQuaternion message to
/// a C struct hxsQuaternion.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertTransform(const haptix::comm::msgs::hxTransform _in,
  hxsTransform *_out)
{
  if (!_out)
  {
    std::cerr << "hxs_convertTransform() error: NULL output" << std::endl;
    return false;
  }

  // Initialize the C struct.
  memset(_out, 0, sizeof(hxsTransform));

  bool result = true;
  result &= hxs_convertVector3(_in.pos(), &_out->pos);
  result &= hxs_convertQuaternion(_in.orient(), &_out->orient);

  return result;
}

//////////////////////////////////////////////////
/// \internal Private function that converts a C struct hxsQuaternion to a
/// protobuf hxQuaternion message.
/// \param[in] _in C-struct.
/// \param[out] _out Protobuf message.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertTransform(const hxsTransform *_in,
  haptix::comm::msgs::hxTransform *_out)
{
  if (!_in)
  {
    std::cerr << "hxs_convertTransform() error: NULL input" << std::endl;
    return false;
  }

  if (!_out)
  {
    std::cerr << "hxs_convertTransform() error: NULL output" << std::endl;
    return false;
  }

  // Initialize the message.
  _out->Clear();

  bool result = true;
  result &= hxs_convertVector3(&(_in->pos), _out->mutable_pos());
  result &= hxs_convertQuaternion(&(_in->orient), _out->mutable_orient());

  return result;
}

//////////////////////////////////////////////////
/// \internal Private function that converts a protobuf hxJoint message to a
/// C struct hxsJoint.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertJoint(const haptix::comm::msgs::hxJoint _in,
  hxsJoint *_out)
{
  if (!_out)
  {
    std::cerr << "hxs_convertJoint() error: NULL output" << std::endl;
    return false;
  }

  // Initialize the C struct.
  memset(_out, 0, sizeof(hxsJoint));

  if (_in.name().size() > hxsMAXNAMESIZE - 1)
  {
    std::cerr << "hxs_convertJoint() error: The name of the joint ["
              << _in.name() << "] exceeds the maximum size allowed ("
              << hxsMAXNAMESIZE << " chars)." << std::endl;
    return false;
  }

#ifdef _MSC_VER
  strncpy_s(_out->name, hxsMAXNAMESIZE,
      _in.name().c_str(), strlen(_in.name().c_str()));
#else
  strncpy(_out->name, _in.name().c_str(), strlen(_in.name().c_str()));
#endif
  _out->name[strlen(_in.name().c_str())] = '\0';
  _out->pos = _in.pos();
  _out->vel = _in.vel();
  _out->torque_motor = _in.torque_motor();

  bool result = true;
  result &= hxs_convertVector3(_in.wrench_reactive().force(),
    &_out->wrench_reactive.force);

  result &= hxs_convertVector3(_in.wrench_reactive().torque(),
    &_out->wrench_reactive.torque);

  return result;
}

//////////////////////////////////////////////////
/// \internal Private function that converts a C struct hxsWrench to a
/// protobuf hxWrench message.
/// \param[in] _in C-struct.
/// \param[out] _out Protobuf message.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertWrench(const hxsWrench *_in,
  haptix::comm::msgs::hxWrench *_out)
{
  if (!_in)
  {
    std::cerr << "hxs_convertWrench() error: NULL input" << std::endl;
    return false;
  }

  if (!_out)
  {
    std::cerr << "hxs_convertWrench() error: NULL output" << std::endl;
    return false;
  }

  bool result = true;
  result &= hxs_convertVector3(&_in->force, _out->mutable_force());
  result &= hxs_convertVector3(&_in->torque, _out->mutable_torque());
  return result;
}

//////////////////////////////////////////////////
/// \internal Private function that converts a protobuf hxLink message to a
/// C struct hxsLink.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertLink(const haptix::comm::msgs::hxLink _in, hxsLink *_out)
{
  if (!_out)
  {
    std::cerr << "hxs_convertLink() error: NULL output" << std::endl;
    return false;
  }

  // Initialize the C struct.
  memset(_out, 0, sizeof(hxsLink));

  if (_in.name().size() > hxsMAXNAMESIZE - 1)
  {
    std::cerr << "hxs_convertLink() error: The name of the link ["
              << _in.name() << "] exceeds the maximum size allowed ("
              << hxsMAXNAMESIZE << " chars)." << std::endl;
    return false;
  }

#ifdef _MSC_VER
  strncpy_s(_out->name, hxsMAXNAMESIZE,
      _in.name().c_str(), strlen(_in.name().c_str()));
#else
  strncpy(_out->name, _in.name().c_str(), strlen(_in.name().c_str()));
#endif
  _out->name[strlen(_in.name().c_str())] = '\0';

  bool result = true;
  result &= hxs_convertTransform(_in.transform(), &_out->transform);
  result &= hxs_convertVector3(_in.lin_vel(), &_out->lin_vel);
  result &= hxs_convertVector3(_in.ang_vel(), &_out->ang_vel);
  result &= hxs_convertVector3(_in.lin_acc(), &_out->lin_acc);
  result &= hxs_convertVector3(_in.ang_acc(), &_out->ang_acc);

  return result;
}

//////////////////////////////////////////////////
/// \internal Private function that converts a protobuf hxModel message to a
/// C struct hxsModel.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertModel(const haptix::comm::msgs::hxModel _in,
  hxsModel *_out)
{
  if (_in.name().size() > hxsMAXNAMESIZE - 1)
  {
    std::cerr << "hxs_convertModel() error: The name of the model ["
              << _in.name() << "] exceeds the maximum size allowed ("
              << hxsMAXNAMESIZE << " chars)." << std::endl;
    return false;
  }

  if (!_out)
  {
    std::cerr << "hxs_convertModel() error: NULL output" << std::endl;
    return false;
  }

  // Initialize the C struct.
  memset(_out, 0, sizeof(hxsModel));

#ifdef _MSC_VER
  strncpy_s(_out->name, hxsMAXNAMESIZE,
      _in.name().c_str(), strlen(_in.name().c_str()));
#else
  strncpy(_out->name, _in.name().c_str(), strlen(_in.name().c_str()));
#endif
  _out->name[strlen(_in.name().c_str())] = '\0';

  bool result = true;
  result &= hxs_convertTransform(_in.transform(), &(_out->transform));
  _out->link_count = _in.links_size();
  _out->joint_count = _in.joints_size();

  // Fill the links.
  for (int i = 0; i < _out->link_count; ++i)
    result &= hxs_convertLink(_in.links(i), &_out->links[i]);

  // Fill the joints.
  for (int i = 0; i < _out->joint_count; ++i)
    result &= hxs_convertJoint(_in.joints(i), &_out->joints[i]);

  _out->gravity_mode = _in.gravity_mode();

  return result;
}

//////////////////////////////////////////////////
/// \internal Private function that converts a protobuf hxContactPoint_V message
/// to a C struct hxsContactPoints.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertContactPoints(
  const haptix::comm::msgs::hxContactPoint_V _in, hxsContactPoints *_out)
{
  if (!_out)
  {
    std::cerr << "hxs_convertContactPoints() error: NULL output" << std::endl;
    return false;
  }

  // Initialize the C struct.
  memset(_out, 0, sizeof(hxsContactPoints));

  _out->contact_count = _in.contacts_size();

  bool result = true;

  for (int i = 0; i < _out->contact_count; ++i)
  {
    if (_in.contacts(i).link1().size() > hxsMAXNAMESIZE - 1)
    {
      std::cerr << "hxs_convertContactPoints() error: The name of the link1 ["
                << _in.contacts(i).link1() << "] exceeds the maximum size "
                << "allowed (" << hxsMAXNAMESIZE << " chars)." << std::endl;
      return false;
    }

    if (_in.contacts(i).link2().size() > hxsMAXNAMESIZE - 1)
    {
      std::cerr << "hxs_convertContactPoints() error: The name of the link2 ["
                << _in.contacts(i).link2() << "] exceeds the maximum size "
                << "allowed (" << hxsMAXNAMESIZE << " chars)." << std::endl;
      return false;
    }

#ifdef _MSC_VER
    strncpy_s(_out->contacts[i].link1, hxsMAXNAMESIZE,
      _in.contacts(i).link1().c_str(), strlen(_in.contacts(i).link1().c_str()));
#else
    strncpy(_out->contacts[i].link1,
      _in.contacts(i).link1().c_str(), strlen(_in.contacts(i).link1().c_str()));
#endif
    _out->contacts[i].link1[strlen(_in.contacts(i).link1().c_str())] = '\0';

#ifdef _MSC_VER
    strncpy_s(_out->contacts[i].link2, hxsMAXNAMESIZE,
      _in.contacts(i).link2().c_str(), strlen(_in.contacts(i).link2().c_str()));
#else
    strncpy(_out->contacts[i].link2,
      _in.contacts(i).link2().c_str(), strlen(_in.contacts(i).link2().c_str()));
#endif
    _out->contacts[i].link2[strlen(_in.contacts(i).link2().c_str())] = '\0';

    result &= hxs_convertVector3(_in.contacts(i).point(),
      &_out->contacts[i].point);
    result &= hxs_convertVector3(_in.contacts(i).normal(),
      &_out->contacts[i].normal);
    _out->contacts[i].distance = _in.contacts(i).distance();
    result &= hxs_convertVector3(_in.contacts(i).wrench().force(),
      &_out->contacts[i].wrench.force);
    result &= hxs_convertVector3(_in.contacts(i).wrench().torque(),
      &_out->contacts[i].wrench.torque);
  }

  return result;
}

//////////////////////////////////////////////////
/// \internal Private function that converts a protobuf hxSimInfo message to a
/// C struct hxsSimInfo.
/// \param[in] _in Protobuf message.
/// \param[out] _out C-struct.
/// \return True if the function succeed or false otherwise.
static bool hxs_convertSimInfo(const haptix::comm::msgs::hxSimInfo _in,
  hxsSimInfo *_out)
{
  if (!_out)
  {
    std::cerr << "hxs_convertSimInfo() error: NULL output" << std::endl;
    return false;
  }

  // Initialize the C struct.
  memset(_out, 0, sizeof(hxsSimInfo));

  _out->model_count = _in.models_size();

  bool result = true;

  // Fill the models.
  for (int i = 0; i < _out->model_count; ++i)
    result &= hxs_convertModel(_in.models(i), &_out->models[i]);

  // Fill the camera.
  result &= hxs_convertTransform(_in.camera_transform(),
    &_out->camera_transform);

  return result;
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
                  bool (*_f)(const REP _rep, T _dst))
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
