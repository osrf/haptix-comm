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

#include <cstring>
#include <iostream>
#include <array>
#include <ignition/transport.hh>
#include "haptix/comm/haptix.h"
#include "msg/hxCommand.pb.h"
#include "msg/hxRobot.pb.h"
#include "msg/hxSensor.pb.h"

extern "C" {
  /// \brief Different robots/simulators supported.
  std::string ProjectTopic = "haptix";
  std::string DekaTopic    = "deka";
  std::string MPLTopic     = "mpl";
  std::string GazeboTopic  = "gazebo";
  std::string MujocoTopic  = "mujoco";

  // Default to the Gazebo target.
  int g_target = 2;

  std::array<std::string, 4> RobotTopics =
      {DekaTopic, MPLTopic, GazeboTopic, MujocoTopic};

  /// \brief ignition transport node.
  ignition::transport::Node *haptixNode = NULL;

  /// \brief Error string, to be retrieved by hx_last_result()
  std::string lastResult;
  std::mutex lastResultLock;

  /// \brief Timeout used for the service requests (ms.).
  unsigned int Timeout = 1000;

  //////////////////////////////////////////////////
  /// \brief Private function that converts a protobuf hxSensor message to a
  /// C struct hxSensor.
  static void convert(const haptix::comm::msgs::hxSensor _in, hxSensor *_out)
  {
    // Initialize the C struct.
    memset(_out, 0, sizeof(hxSensor));

    _out->time_stamp.sec = _in.time_stamp().sec();
    _out->time_stamp.nsec = _in.time_stamp().nsec();
    for (int i = 0; (i < hxMAXMOTOR) && (i < _in.motor_pos_size()); ++i)
      _out->motor_pos[i] = _in.motor_pos(i);
    for (int i = 0; (i < hxMAXMOTOR) && (i < _in.motor_vel_size()); ++i)
      _out->motor_vel[i] = _in.motor_vel(i);
    for (int i = 0; (i < hxMAXMOTOR) && (i < _in.motor_torque_size()); ++i)
      _out->motor_torque[i] = _in.motor_torque(i);
    for (int i = 0; (i < hxMAXJOINT) && (i < _in.joint_pos_size()); ++i)
      _out->joint_pos[i] = _in.joint_pos(i);
    for (int i = 0; (i < hxMAXJOINT) && (i < _in.joint_vel_size()); ++i)
      _out->joint_vel[i] = _in.joint_vel(i);
    for (int i = 0; (i < hxMAXCONTACTSENSOR) &&
                    (i < _in.contact_size()); ++i)
      _out->contact[i] = _in.contact(i);
    for (int i = 0; (i < hxMAXIMU) && (i < _in.imu_linear_acc_size()); ++i)
    {
      _out->imu_linear_acc[i][0] = _in.imu_linear_acc(i).x();
      _out->imu_linear_acc[i][1] = _in.imu_linear_acc(i).y();
      _out->imu_linear_acc[i][2] = _in.imu_linear_acc(i).z();
    }
    for (int i = 0; (i < hxMAXIMU) &&
                    (i < _in.imu_angular_vel_size()); ++i)
    {
      _out->imu_angular_vel[i][0] = _in.imu_angular_vel(i).x();
      _out->imu_angular_vel[i][1] = _in.imu_angular_vel(i).y();
      _out->imu_angular_vel[i][2] = _in.imu_angular_vel(i).z();
    }
    for (int i = 0; (i < hxMAXIMU) &&
                    (i < _in.imu_orientation_size()); ++i)
    {
      _out->imu_orientation[i][0] = _in.imu_orientation(i).w();
      _out->imu_orientation[i][1] = _in.imu_orientation(i).x();
      _out->imu_orientation[i][2] = _in.imu_orientation(i).y();
      _out->imu_orientation[i][3] = _in.imu_orientation(i).z();
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
  hxResult hx_connect(const char * /*_host*/, int /*_port*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hx_close()
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hx_robot_info(hxRobotInfo *_robotinfo)
  {
    // Initialize the C struct.
    memset(_robotinfo, 0, sizeof(hxRobotInfo));

    haptix::comm::msgs::hxRobot req;
    haptix::comm::msgs::hxRobot rep;
    bool result;
    ignition::transport::Node *hxNode = getHxNodeInstance();

    // TODO: Why do we have to create an empty message here?
    req.set_motor_count(0);
    req.set_joint_count(0);
    req.set_contact_sensor_count(0);
    req.set_imu_count(0);
    haptix::comm::msgs::hxRobot::hxLimit *limit = req.add_joint_limit();
    limit->set_minimum(0.0);
    limit->set_maximum(0.0);
    req.set_update_rate(0.0);

    // Request the service.
    std::string service = "/" + ProjectTopic + "/" + RobotTopics[g_target] +
        "/GetRobotInfo";
    bool executed = hxNode->Request(service, req, Timeout, rep, result);

    if (executed)
    {
      if (result)
      {
        // Fill the struct with the response.
        _robotinfo->motor_count = rep.motor_count();
        _robotinfo->joint_count = rep.joint_count();
        _robotinfo->contact_sensor_count = rep.contact_sensor_count();
        _robotinfo->imu_count = rep.imu_count();

        // Fill the joint limit field.
        for (int i = 0; i < rep.joint_limit_size(); ++i)
        {
          _robotinfo->joint_limit[i][0] = rep.joint_limit(i).minimum();
          _robotinfo->joint_limit[i][1] = rep.joint_limit(i).maximum();
        }

        // Fill the motor limit field.
        for (int i = 0; i < rep.motor_limit_size(); ++i)
        {
          _robotinfo->motor_limit[i][0] = rep.motor_limit(i).minimum();
          _robotinfo->motor_limit[i][1] = rep.motor_limit(i).maximum();
        }

        _robotinfo->update_rate = rep.update_rate();

        return hxOK;
      }
      else
      {
        std::lock_guard<std::mutex> lock(lastResultLock);
        lastResult = "hx_robot_info() Service call failed.";
        std::cerr << lastResult << std::endl;
      }
    }
    else
    {
      std::lock_guard<std::mutex> lock(lastResultLock);
      lastResult = "hx_robot_info() Service call timed out.";
      std::cerr << lastResult << std::endl;
    }

    return hxERROR;
  }

  //////////////////////////////////////////////////
  hxResult hx_update(const hxCommand* _command, hxSensor* _sensor)
  {
    // Initialize the C struct.
    memset(_sensor, 0, sizeof(hxSensor));

    haptix::comm::msgs::hxCommand req;
    haptix::comm::msgs::hxSensor rep;
    bool result;
    ignition::transport::Node *hxNode = getHxNodeInstance();

    for (int i = 0; i < hxMAXMOTOR; ++i)
    {
      req.add_ref_pos(_command->ref_pos[i]);
      req.add_ref_vel_max(_command->ref_vel_max[i]);
      req.add_gain_pos(_command->gain_pos[i]);
      req.add_gain_vel(_command->gain_vel[i]);
    }
    req.set_ref_pos_enabled(_command->ref_pos_enabled);
    req.set_ref_vel_max_enabled(_command->ref_vel_max_enabled);
    req.set_gain_pos_enabled(_command->gain_pos_enabled);
    req.set_gain_vel_enabled(_command->gain_vel_enabled);

    // Request the service.
    std::string service = "/" + ProjectTopic + "/" + RobotTopics[g_target] +
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
      {
        std::lock_guard<std::mutex> lock(lastResultLock);
        lastResult = "hx_update() Service call failed.";
        std::cerr << lastResult << std::endl;
      }
    }
    else
    {
      std::lock_guard<std::mutex> lock(lastResultLock);
      lastResult = "hx_update() Service call timed out.";
      std::cerr << lastResult << std::endl;
    }

    return hxERROR;
  }

  //////////////////////////////////////////////////
  hxResult hx_read_sensors(hxSensor *_sensor)
  {
    // Sanity check.
    if (!_sensor)
      return hxERROR;

    haptix::comm::msgs::hxSensor req;
    // TODO: why do we have to fill in an empty message here?
    req.mutable_time_stamp()->set_sec(0);
    req.mutable_time_stamp()->set_nsec(0);
    haptix::comm::msgs::hxSensor rep;
    bool result;
    ignition::transport::Node *hxNode = getHxNodeInstance();

    // Request the service.
    std::string service = "/" + ProjectTopic + "/" + RobotTopics[g_target] +
        "/Read";
    bool executed = hxNode->Request(service, req, Timeout, rep, result);

    if (!executed)
    {
      std::lock_guard<std::mutex> lock(lastResultLock);
      lastResult = "hx_read_sensors() Service call timed out.";
      std::cerr << lastResult << std::endl;
      return hxERROR;
    }

    if (!result)
    {
      std::lock_guard<std::mutex> lock(lastResultLock);
      lastResult = "hx_read_sensors() Service call failed.";
      std::cerr << lastResult << std::endl;
      return hxERROR;
    }

    // Fill the struct with the response.
    convert(rep, _sensor);

    return hxOK;
  }

  const char *hx_last_result()
  {
    std::lock_guard<std::mutex> lock(lastResultLock);
    return lastResult.c_str();
  }

// end extern "C"
}
