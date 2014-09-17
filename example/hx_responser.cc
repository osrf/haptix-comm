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
#include <haptix/comm/haptix.h>
#include <haptix/comm/msg/hxCommand.pb.h>
#include <haptix/comm/msg/hxDevice.pb.h>
#include <haptix/comm/msg/hxSensor.pb.h>

int numMotors = 4;
int numJoints = 5;
int numContactSensors = 6;
int numIMUs = 7;

std::string deviceInfoTopic = "/haptix/deka/GetDeviceInfo";
std::string updateTopic = "/haptix/deka/Update";

std::chrono::time_point<std::chrono::system_clock> start, end;
long counter;

//////////////////////////////////////////////////
/// \brief Provide a service.
void onGetDeviceInfo(const std::string &_service,
  const haptix::comm::msgs::hxDevice &/*_req*/,
  haptix::comm::msgs::hxDevice &_rep, bool &_result)
{
  _result = true;

  if (_service != deviceInfoTopic)
    _result = false;

  _rep.set_nmotor(numMotors);
  _rep.set_njoint(numJoints);
  _rep.set_ncontactsensor(numContactSensors);
  _rep.set_nimu(numIMUs);

  for (int i = 0; i < numJoints; ++i)
  {
    haptix::comm::msgs::hxJointAngle *joint = _rep.add_limit();
    joint->set_min(-i);
    joint->set_max(i);
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
  _rep.set_timestamp(0);
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
    haptix::comm::msgs::imu *linacc = _rep.add_imu_linacc();
    linacc->set_x(i);
    linacc->set_y(i + 1);
    linacc->set_z(i + 2);
    haptix::comm::msgs::imu *angvel = _rep.add_imu_angvel();
    angvel->set_x(i + 3);
    angvel->set_y(i + 4);
    angvel->set_z(i + 5);
  }

  end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsedSec = end - start;
  ++counter;

  if (elapsedSec.count() > 1.0)
  {
    std::cout << "Receiving messages at " << counter / elapsedSec.count()
              << "Hz." << std::endl;
    counter = 0;
    start = end;
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  int time = -1;
  counter = 0;

  start = std::chrono::system_clock::now();

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
  if (!node.Advertise(deviceInfoTopic, onGetDeviceInfo))
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
