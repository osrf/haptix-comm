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
#include <array>
// #include <ignition/transport.hh>
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
  // ignition::transport::Node hxNode(ProjectTopic);

  /// \brief Timeout used for the service requests (ms.).
  unsigned int Timeout = 1000;

  //////////////////////////////////////////////////
  /// \brief Return true if the target is supported or false otherwise.
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
  int hx_connect(int /*_target*/)
  {
    // Test some protobuf stuff.
    haptix::comm::msgs::hxDevice req;

    req.set_nmotor(0);
    req.set_njoint(0);
    req.set_ncontactsensor(0);
    req.set_nimu(0);
    haptix::comm::msgs::hxJointAngle *limit = req.add_limit();
    limit->set_minimum(0.0);
    limit->set_maximum(0.0);

    printf("Hello haptix_comm 0.1\n");
    printf("%s\n",req.DebugString().c_str());

    return 3;
  }

  //////////////////////////////////////////////////
  /*hxResult hx_close(int _target)
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
          _deviceinfo->limit[i][0] = rep.limit(i).minimum();
          _deviceinfo->limit[i][1] = rep.limit(i).maximum();
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
  }*/
}   // end extern "C"
