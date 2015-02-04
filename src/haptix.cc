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

#include <cstring>
#include <iostream>
#include <array>
#include <ignition/transport.hh>
#include "haptix/comm/haptix.h"
#include "msg/hxCommand.pb.h"
#include "msg/hxDevice.pb.h"
#include "msg/hxSensor.pb.h"

extern "C" {
  /// \brief Different device/simulators supported.
  std::string ProjectTopic = "haptix";
  std::string DekaTopic    = "deka";
  std::string MPLTopic     = "mpl";
  std::string GazeboTopic  = "gazebo";
  std::string MujocoTopic  = "mujoco";

  std::array<std::string, 4> DeviceTopics =
      {DekaTopic, MPLTopic, GazeboTopic, MujocoTopic};

  /// \brief ignition transport node.
  ignition::transport::Node *haptixNode = NULL;

  /// \brief Timeout used for the service requests (ms.).
  unsigned int Timeout = 1000;

  //////////////////////////////////////////////////
  /// \brief Private function that converts a protobuf hxSensor message to a
  /// C struct hxSensor.
  static void convert(const haptix::comm::msgs::hxSensor _in, hxSensor *_out)
  {
    // Initialize the C struct.
    memset(_out, 0, sizeof(hxSensor));

    for (int i = 0; i < _in.motor_pos_size(); ++i)
    {
      _out->motor_pos[i] = _in.motor_pos(i);
      _out->motor_vel[i] = _in.motor_vel(i);
      _out->motor_torque[i] = _in.motor_torque(i);
    }

    for (int i = 0; i < _in.joint_pos_size(); ++i)
    {
      _out->joint_pos[i] = _in.joint_pos(i);
      _out->joint_vel[i] = _in.joint_vel(i);
    }

    for (int i = 0; i < _in.contact_size(); ++i)
      _out->contact[i] = _in.contact(i);

    for (int i = 0; i < _in.imu_linacc_size(); ++i)
    {
      _out->IMU_linacc[i][0] = _in.imu_linacc(i).x();
      _out->IMU_linacc[i][1] = _in.imu_linacc(i).y();
      _out->IMU_linacc[i][2] = _in.imu_linacc(i).z();
      _out->IMU_angvel[i][0] = _in.imu_angvel(i).x();
      _out->IMU_angvel[i][1] = _in.imu_angvel(i).y();
      _out->IMU_angvel[i][2] = _in.imu_angvel(i).z();
    }
  }

  //////////////////////////////////////////////////
  /// \brief Private function that creates an Ignition Transport node
  /// or return a pointer to it if has been already created.
  static ignition::transport::Node *getHxNodeInstance()
  {
    if (!haptixNode)
      haptixNode = new ignition::transport::Node();

    return haptixNode;
  }

  //////////////////////////////////////////////////
  /// \brief Private function that returns true if the target is supported
  /// or false otherwise.
  static bool checkTarget(int _target)
  {
    if (_target < hxDEKA || _target > hxMUJOCO)
    {
      std::cerr << "Unsupported target [" << _target << "]." << std::endl;
      return false;
    }
    return true;
  }

  //////////////////////////////////////////////////
  hxResult hx_connect(int _target, const char *_host, int _port)
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
    // Initialize the C struct.
    memset(_deviceinfo, 0, sizeof(hxDeviceInfo));

    // Sanity check.
    if (!checkTarget(_target))
      return hxBAD;

    haptix::comm::msgs::hxDevice req;
    haptix::comm::msgs::hxDevice rep;
    bool result;
    ignition::transport::Node *hxNode = getHxNodeInstance();

    req.set_nmotor(0.0);
    req.set_njoint(0.0);
    req.set_ncontactsensor(0.0);
    req.set_nimu(0.0);
    haptix::comm::msgs::hxJointAngle *limit = req.add_limit();
    limit->set_minimum(0.0);
    limit->set_maximum(0.0);

    // Request the service.
    std::string service = "/" + ProjectTopic + "/" + DeviceTopics[_target] +
        "/GetDeviceInfo";
    bool executed = hxNode->Request(service, req, Timeout, rep, result);

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
          _deviceinfo->limit[i][0] = rep.limit(i).minimum();
          _deviceinfo->limit[i][1] = rep.limit(i).maximum();
        }

        return hxOK;
      }
      else
        std::cerr << "hx_getdeviceinfo() Service call failed." << std::endl;
    }
    else
      std::cerr << "hx_getdeviceinfo() Service call timed out." << std::endl;

    return hxBAD;
  }

  //////////////////////////////////////////////////
  hxResult hx_update(int _target, const hxCommand* _command, hxSensor* _sensor,
      hxTime *_timestamp)
  {
    // Initialize the C struct.
    memset(_sensor, 0, sizeof(hxSensor));

    // Sanity check.
    if (!checkTarget(_target))
      return hxBAD;

    haptix::comm::msgs::hxCommand req;
    haptix::comm::msgs::hxSensor rep;
    bool result;
    ignition::transport::Node *hxNode = getHxNodeInstance();

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
    bool executed = hxNode->Request(service, req, Timeout, rep, result);

    if (executed)
    {
      if (result)
      {
        // Fill the struct with the response.
        convert(rep, _sensor);

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
  hxResult hx_readsensors(int _target, hxSensor *_sensor, hxTime *_timestamp)
  {
    // Sanity check.
    if (!checkTarget(_target) || !_sensor)
      return hxBAD;

    haptix::comm::msgs::hxSensor req;
    haptix::comm::msgs::hxSensor rep;
    bool result;
    ignition::transport::Node *hxNode = getHxNodeInstance();

    // Request the service.
    std::string service = "/" + ProjectTopic + "/" + DeviceTopics[_target] +
        "/Read";
    bool executed = hxNode->Request(service, req, Timeout, rep, result);

    if (!executed)
    {
      std::cerr << "hx_readsensors() Service call timed out." << std::endl;
      return hxBAD;
    }

    if (!result)
    {
      std::cerr << "hx_readsensors() Service call failed." << std::endl;
      return hxBAD;
    }

    // Fill the struct with the response.
    convert(rep, _sensor);

    return hxOK;
  }

// end extern "C"
}
