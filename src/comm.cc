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
#include "haptix/comm/comm.h"
#include "haptix/comm/types.h"
#include "msg/AplControl.pb.h"
#include "msg/hxCommand.pb.h"
#include "msg/hxDevice.pb.h"
#include "msg/hxSensor.pb.h"

extern "C" {

  std::string ProjectTopic = "haptix";
  std::string DekaTopic = "deka";
  std::string MPLTopic = "mpl";
  std::string GazeboTopic = "gazebo";
  std::string MujocoTopic = "mujoco";
  std::array<std::string, 4> DeviceTopics =
      {DekaTopic, MPLTopic, GazeboTopic, MujocoTopic};

  ignition::transport::Node hxNode(ProjectTopic);
  int Timeout = 100; // ms.


  /// \brief Return true if the target is supported or false otherwise.
  //////////////////////////////////////////////////
  bool checkTarget(int _target)
  {
    if (_target < hxDEKA || _target > hxMUJOCO)
    {
      std::cerr << "Unsupported target [" << _target << "]." << std::endl;
      return false;
    }
    return true;
  }

  //////////////////////////////////////////////////
  hxResult hx_connect(int _target)
  {
    // Sanity check.
    if (checkTarget(_target))
      return hxOK;
    else
      return hxBAD;
  }

  //////////////////////////////////////////////////
  hxResult hx_close(int _target)
  {
    // Sanity check.
    if (checkTarget(_target))
      return hxOK;
    else
      return hxBAD;
  }

  //////////////////////////////////////////////////
  hxResult hx_getdeviceinfo(int _target, hxDeviceInfo* _deviceinfo)
  {
    // Sanity check.
    if (!checkTarget(_target))
      return hxBAD;

    haptix::comm::msgs::hxDevice req;
    haptix::comm::msgs::hxDevice rep;
    bool result;

    // Request the service.
    std::string service = "/" + ProjectTopic + "/" + DeviceTopics[_target] +
        "/GetDeviceInfo";
    bool executed = hxNode.Request(service, req, Timeout, rep, result);

    if (executed)
    {
      if (result)
      {
        // Fill the struct with the response.
        _deviceinfo->nmotor = rep.nmotor();
        _deviceinfo->njoint = rep.njoint();
        _deviceinfo->ncontactsensor = rep.ncontactsensor();
        _deviceinfo->nIMU = rep.nimu();

        // Fill the limit field.
        for (int i = 0; i < rep.limit_size(); ++i)
        {
          _deviceinfo->limit[i][0] = rep.limit(i).min();
          _deviceinfo->limit[i][1] = rep.limit(i).max();
        }

        return hxOK;
      }
      else
        std::cerr << "hx_getdevicefo() Service call failed." << std::endl;
    }
    else
      std::cerr << "hx_getdevicefo() Service call timed out." << std::endl;

    return hxBAD;
  }

  //////////////////////////////////////////////////
  hxResult hx_update(int _target, const hxCommand* _command, hxSensor* _sensor)
  {
    // Sanity check.
    if (!checkTarget(_target))
      return hxBAD;

    haptix::comm::msgs::hxCommand req;
    haptix::comm::msgs::hxSensor rep;
    bool result;

    // Fill the message with the request.
    for (int i = 0; i < hxMAXMOTOR; ++i)
    {
      req.add_ref_pos(_command->ref_pos[i]);
      req.add_ref_vel(_command->ref_vel[i]);
      req.add_gain_pos(_command->gain_pos[i]);
      req.add_gain_vel(_command->gain_vel[i]);
    }

    // Request the service.
    std::string service = "/" + ProjectTopic + "/" + DeviceTopics[_target] +
        "/Update";
    bool executed = hxNode.Request(service, req, Timeout, rep, result);

    if (executed)
    {
      if (result)
      {
        // Fill the struct with the response.
        for (int i = 0; i < rep.motor_pos_size(); ++i)
        {
          _sensor->motor_pos[i] = rep.motor_pos(i);
          _sensor->motor_vel[i] = rep.motor_vel(i);
          _sensor->motor_torque[i] = rep.motor_torque(i);
        }

        for (int i = 0; i < rep.joint_pos_size(); ++i)
        {
          _sensor->joint_pos[i] = rep.joint_pos(i);
          _sensor->joint_vel[i] = rep.joint_vel(i);
        }

        for (int i = 0; i < rep.contact_size(); ++i)
          _sensor->contact[i] = rep.contact(i);

        for (int i = 0; i < rep.imu_linacc_size(); ++i)
        {
          _sensor->IMU_linacc[i][0] = rep.imu_linacc(i).x();
          _sensor->IMU_linacc[i][1] = rep.imu_linacc(i).y();
          _sensor->IMU_linacc[i][2] = rep.imu_linacc(i).z();
          _sensor->IMU_angvel[i][0] = rep.imu_angvel(i).x();
          _sensor->IMU_angvel[i][1] = rep.imu_angvel(i).y();
          _sensor->IMU_angvel[i][2] = rep.imu_angvel(i).z();
        }

        return hxOK;
      }
      else
        std::cerr << "hx_update() Service call failed." << std::endl;
    }
    else
      std::cerr << "hx_update() Service call timed out." << std::endl;

    return hxBAD;
  }

  //////////////////////////////////////////////////
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
  HaptixNodePtr HaptixNewNodeNamespace(const char *_ns)
  {
    return reinterpret_cast<void*>(new ignition::transport::Node(_ns));
  }

  //////////////////////////////////////////////////
  int HaptixAdvertise(HaptixNodePtr _node, const char *_service,
    void (*_cb)(const char *_service, struct AplRobotCommand _req,
                struct AplRobotState *_rep, int *_result))
  {
    // Sanity check.
    if (!_node)
    {
      std::cerr << "HaptixAdvertise() Error: NULL node." << std::endl;
      return -1;
    }

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
    // Sanity check.
    if (!_node)
    {
      std::cerr << "HaptixRequest() Error: NULL node." << std::endl;
      return -1;
    }

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
  void HaptixDeleteNode(HaptixNodePtr _node)
  {
    if (_node != NULL)
    {
      ignition::transport::Node *node =
        reinterpret_cast<ignition::transport::Node*>(_node);

      delete node;
      _node = NULL;
    }
  }
}
