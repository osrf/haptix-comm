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

#include <iostream>
#include <ignition/transport.hh>
#include "haptix/comm/Api.h"
#include "haptix/comm/Arm.pb.h"
#include "haptix/comm/Comm.h"

extern "C" {

  void (*cb)(const char *_service, Arm_t _req, Arm_t *_rep, int *_result);

  //////////////////////////////////////////////////
  void internalCallback(const std::string &_service,
    const haptix::comm::Arm &_req, haptix::comm::Arm &_rep, bool &_result)
  {
    Arm_t req, rep;
    int result;

    // Call the user callback.
    cb(_service.c_str(), req, &rep, &result);

    // Set the response's content.
    _rep.set_pos(_req.pos());
    _rep.set_vel(_req.vel());

    // The response succeed.
    _result = true;
  }

  //////////////////////////////////////////////////
  NodePtr newNode()
  {
    return reinterpret_cast<void*>(new ignition::transport::Node());
  }

  //////////////////////////////////////////////////
  int nodeAdvertise(NodePtr _node, const char *_service,
    void (*_cb)(const char *_service, Arm_t _req, Arm_t *_rep, int *_result))
  {
    // Store the user callback.
    cb = _cb;

    // Advertise a service call.
    ignition::transport::Node *node =
      reinterpret_cast<ignition::transport::Node*>(_node);

    // Advertise a service call.
    bool res = node->Advertise(_service, internalCallback);

    if (res)
      return 0;
    else
      return -1;
  }

  //////////////////////////////////////////////////
  int nodeRequest(NodePtr _node, const char *_service, Arm_t _req, int _timeout,
    Arm_t *_rep, int *_result)
  {
    // Prepare the input parameters.
    haptix::comm::Arm req;
    req.set_pos(_req.pos);
    req.set_vel(_req.vel);

    haptix::comm::Arm rep;
    bool result;

    ignition::transport::Node *node =
      reinterpret_cast<ignition::transport::Node*>(_node);

    // Request the service.
    bool executed = node->Request(_service, req, _timeout, rep, result);

    if (executed)
    {
      if (result)
      {
        _rep->pos = rep.pos();
        _rep->vel = rep.vel();
        *_result = 0;
      }
      else
      {
        std::cerr << "Service call failed" << std::endl;
        *_result = -1;
      }
      return 0;
    }
    else
      std::cerr << "Service call timed out" << std::endl;

    return -1;
  }

  //////////////////////////////////////////////////
  void deleteNode(NodePtr /*_node*/)
  {
  }
}
