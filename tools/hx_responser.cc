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

#include <chrono>
#include <string>
#include <stdio.h>
#include <ignition/transport.hh>
#include "msg/hxCommand.pb.h"
#include "msg/hxRobot.pb.h"
#include "msg/hxSensor.pb.h"

int numMotors = 4;
int numJoints = 5;
int numContactSensors = 6;
int numIMUs = 7;

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

  for (int i = 0; i < numJoints; ++i)
  {
    haptix::comm::msgs::hxRobot::hxLimit *joint = _rep.add_joint_limit();
    joint->set_minimum(-i);
    joint->set_maximum(i);
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
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  int time = -1;

  if ( argc > 2 )
  {
    std::cout << "usage: " << argv[0] << " [TIME_MS]";
    return -1;
  }
  else if (argc == 2)
  {
    try
    {
      // Read the time parameter.
      time = std::stoi(argv[1]);
    }
    catch(const std::invalid_argument ex)
    {
      std::cerr << "<TIME_MS> argument must be a positive integer." << std::endl;
      return -1;
    }
  }

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

  if (time != -1)
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  else
  {
    // Zzzz.
    printf("Accepting service calls. Press [ENTER] to exit.\n");
    getchar();
  }
}
