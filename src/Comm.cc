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
#include "haptix/comm/AplControlInterface.h"
#include "haptix/comm/Comm.h"
#include "msg/AplControl.pb.h"

extern "C" {
  void (*cb)(const char *_service, struct AplRobotCommand _req,
             struct AplRobotState *_rep, int *_result);

  //////////////////////////////////////////////////
  void internalCallback(const std::string &_service,
    const haptix::comm::msgs::AplRobotCommand &_req,
    haptix::comm::msgs::AplRobotState &_rep, bool &_result)
  {
    struct AplRobotCommand req;
    struct AplRobotState rep;
    int result;

    // Fill the request.
    for (int i = 0; i < _req.command_size(); ++i)
    {
      req.command[i].position = _req.command(i).position();
      req.command[i].velocity = _req.command(i).velocity();
      req.command[i].effort = _req.command(i).effort();
      req.command[i].kp_position = _req.command(i).kp_position();
      req.command[i].ki_position = _req.command(i).ki_position();
      req.command[i].kp_velocity = _req.command(i).kp_velocity();
      req.command[i].force = _req.command(i).force();
    }

    // Call the user callback.
    cb(_service.c_str(), req, &rep, &result);

    // Fill the response.
    for (int i = 0; i < num_joints; ++i)
    {
      haptix::comm::msgs::JointState *state = _rep.add_state();
      state->set_position(rep.state[i].position);
      state->set_velocity(rep.state[i].velocity);
      state->set_effort(rep.state[i].effort);
    }

    // The response succeed.
    _result = true;
  }

  //////////////////////////////////////////////////
  HaptixNodePtr HaptixNewNode()
  {
    return reinterpret_cast<void*>(new ignition::transport::Node());
  }

  //////////////////////////////////////////////////
  int HaptixAdvertise(HaptixNodePtr _node, const char *_service,
    void (*_cb)(const char *_service, struct AplRobotCommand _req,
                struct AplRobotState *_rep, int *_result))
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
  int HaptixRequest(HaptixNodePtr _node, const char *_service,
    struct AplRobotCommand _req, int _timeout, struct AplRobotState *_rep,
    int *_result)
  {
    // Prepare the robot command.
    haptix::comm::msgs::AplRobotCommand req;

    for (int i = 0; i < num_joints; ++i)
    {
      haptix::comm::msgs::JointCommand *cmd = req.add_command();
      cmd->set_position(_req.command[i].position);
      cmd->set_velocity(_req.command[i].velocity);
      cmd->set_effort(_req.command[i].effort);
      cmd->set_kp_position(_req.command[i].kp_position);
      cmd->set_ki_position(_req.command[i].ki_position);
      cmd->set_kp_velocity(_req.command[i].kp_velocity);
      cmd->set_force(_req.command[i].force);
    }

    haptix::comm::msgs::AplRobotState rep;
    bool result;

    ignition::transport::Node *node =
      reinterpret_cast<ignition::transport::Node*>(_node);

    // Request the service.
    bool executed = node->Request(_service, req, _timeout, rep, result);

    if (executed)
    {
      if (result)
      {
        // Fill the response.
        for (int i = 0; i < rep.state_size(); ++i)
        {
          _rep->state[i].position = rep.state(i).position();
          _rep->state[i].velocity =rep.state(i).velocity();
          _rep->state[i].effort = rep.state(i).effort();
        }
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
  void HaptixDeleteNode(HaptixNodePtr /*_node*/)
  {
  }
}
