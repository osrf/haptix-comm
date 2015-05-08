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

#include <chrono>
#include <string>
#include <stdio.h>
#include <ignition/transport.hh>
#include "msg/hxCommand.pb.h"
#include "msg/hxRobot.pb.h"
#include "msg/hxSensor.pb.h"
#include "test_config.h"

int numMotors = 4;
int numJoints = 5;
int numContactSensors = 6;
int numIMUs = 7;
float updateRate = 8.0;
unsigned int time_sec = 9;
unsigned int time_nsec = 10;

std::string deviceInfoTopic = "/haptix/gazebo/GetRobotInfo";
std::string updateTopic = "/haptix/gazebo/Update";

//////////////////////////////////////////////////
/// \brief Provide a service.
void onGetRobotInfo(const std::string &_service,
  const haptix::comm::msgs::hxRobot &/*_req*/,
  haptix::comm::msgs::hxRobot &_rep, bool &_result)
{
  _result = true;

  if (_service != deviceInfoTopic)
    _result = false;

  _rep.set_motor_count(numMotors);
  _rep.set_joint_count(numJoints);
  _rep.set_contact_sensor_count(numContactSensors);
  _rep.set_imu_count(numIMUs);
  _rep.set_update_rate(updateRate);

  for (int i = 0; i < numJoints; ++i)
  {
    haptix::comm::msgs::hxRobot::hxLimit *joint = _rep.add_joint_limit();
    joint->set_minimum(-i);
    joint->set_maximum(i);
  }

  for (int i = 0; i < numMotors; ++i)
  {
    haptix::comm::msgs::hxRobot::hxLimit *motor = _rep.add_motor_limit();
    motor->set_minimum(-i);
    motor->set_maximum(i);
  }
}

//////////////////////////////////////////////////
/// \brief Provide an "Update" service.
void onUpdate(const std::string &_service,
  const haptix::comm::msgs::hxCommand &/*_req*/,
  haptix::comm::msgs::hxSensor &_rep, bool &_result)
{
  _result = true;

  if (_service != updateTopic)
    _result = false;

  // Read the request parameters.
  // Debug output.
  /*std::cout << "Received a new motor command:" << std::endl;
  for (int i = 0; i < numMotors; ++i)
  {
    std::cout << "\tMotor " << i << ":" << std::endl;
    std::cout << "\t\t" << _req.ref_pos(i) << std::endl;
    std::cout << "\t\t" << _req.ref_vel(i) << std::endl;
    std::cout << "\t\t" << _req.gain_pos(i) << std::endl;
    std::cout << "\t\t" << _req.gain_vel(i) << std::endl;
  }*/

  // Create some dummy response.
  for (int i = 0; i < numMotors; ++i)
  {
    _rep.add_motor_pos(i);
    _rep.add_motor_vel(i + 1);
    _rep.add_motor_torque(i + 2);
  }

  for (int i = 0; i < numJoints; ++i)
  {
    _rep.add_joint_pos(i);
    _rep.add_joint_vel(i + 1);
  }

  for (int i = 0; i < numContactSensors; ++i)
    _rep.add_contact(i);

  for (int i = 0; i < numIMUs; ++i)
  {
    haptix::comm::msgs::imu *linear_acc = _rep.add_imu_linear_acc();
    linear_acc->set_x(i);
    linear_acc->set_y(i + 1);
    linear_acc->set_z(i + 2);
    haptix::comm::msgs::imu *angular_vel = _rep.add_imu_angular_vel();
    angular_vel->set_x(i + 3);
    angular_vel->set_y(i + 4);
    angular_vel->set_z(i + 5);
    haptix::comm::msgs::quaternion *orientation = _rep.add_imu_orientation();
    orientation->set_x(i + 6);
    orientation->set_y(i + 7);
    orientation->set_z(i + 8);
    orientation->set_w(i + 9);
  }

  _rep.mutable_time_stamp()->set_sec(time_sec);
  _rep.mutable_time_stamp()->set_nsec(time_nsec);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cerr << "Partition name has not be passed as argument" << std::endl;
    return -1;
  }

  // Set the partition name for this test.
  setenv("IGN_PARTITION", argv[1], 1);

  // Max lifetime of the program.
  int time = 5000;

  // Create a Haptix transport node.
  ignition::transport::Node node;

  // Advertise the "getdeviceinfo" service.
  if (!node.Advertise(deviceInfoTopic, onGetRobotInfo))
  {
    std::cerr << "Error advertising the [" << deviceInfoTopic << "] service."
              << std::endl;
  }

  // Advertise the "update" service.
  if (!node.Advertise(updateTopic, onUpdate))
  {
    std::cerr << "Error advertising the [" << updateTopic << "] service."
              << std::endl;
  }

  // Zzzz.
  std::this_thread::sleep_for(std::chrono::milliseconds(time));
}
