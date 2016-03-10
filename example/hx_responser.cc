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
#include <haptix/comm/haptix.h>

#ifdef _MSC_VER
#pragma warning(push, 0)
#endif
#include <haptix/comm/msg/hxCommand.pb.h>
#include <haptix/comm/msg/hxRobot.pb.h>
#include <haptix/comm/msg/hxSensor.pb.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

static const int motorCount         = 4;
static const int jointCount         = 5;
static const int contactSensorCount = 6;
static const int imuCount           = 7;
static const float updateRate       = 8;
static const int time_sec           = 9;
static const int time_nsec          = 10;

static const std::string RobotInfoTopic  = "/haptix/gazebo/GetRobotInfo";
static const std::string UpdateTopic     = "/haptix/gazebo/Update";
static const std::string SensorInfoTopic = "/haptix/gazebo/Read";

//////////////////////////////////////////////////
/// \brief Provide a "getRobotInfo" service.
void onGetRobotInfo(const haptix::comm::msgs::hxRobot &/*_req*/,
  haptix::comm::msgs::hxRobot &_rep, bool &_result)
{
  _result = true;

  _rep.Clear();

  _rep.set_motor_count(motorCount);
  _rep.set_joint_count(jointCount);
  _rep.set_contact_sensor_count(contactSensorCount);
  _rep.set_imu_count(imuCount);
  _rep.set_update_rate(updateRate);

  for (int i = 0; i < jointCount; ++i)
  {
    haptix::comm::msgs::hxRobot::hxLimit *joint = _rep.add_joint_limit();
    joint->set_minimum((float)(-i));
    joint->set_maximum((float)(i));
  }
}

//////////////////////////////////////////////////
/// \brief Provide an "Update" service.
void onUpdate(const haptix::comm::msgs::hxCommand &/*_req*/,
  haptix::comm::msgs::hxSensor &_rep, bool &_result)
{
  _result = true;

  // Read the request parameters.
  // Debug output.
  /*std::cout << "Received a new motor command:" << std::endl;
  for (int i = 0; i < motorCount; ++i)
  {
    std::cout << "\tMotor " << i << ":" << std::endl;
    std::cout << "\t\t" << _req.ref_pos(i) << std::endl;
    std::cout << "\t\t" << _req.ref_vel(i) << std::endl;
    std::cout << "\t\t" << _req.gain_pos(i) << std::endl;
    std::cout << "\t\t" << _req.gain_vel(i) << std::endl;
  }*/

  _rep.Clear();

  // Create some dummy response.
  for (int i = 0; i < motorCount; ++i)
  {
    _rep.add_motor_pos((float)(i));
    _rep.add_motor_vel((float)(i + 1));
    _rep.add_motor_torque((float)(i + 2));
  }

  for (int i = 0; i < jointCount; ++i)
  {
    _rep.add_joint_pos((float)(i));
    _rep.add_joint_vel((float)(i + 1));
  }

  for (int i = 0; i < contactSensorCount; ++i)
    _rep.add_contact((float)(i));

  for (int i = 0; i < imuCount; ++i)
  {
    haptix::comm::msgs::imu *linear_acc = _rep.add_imu_linear_acc();
    linear_acc->set_x((float)(i));
    linear_acc->set_y((float)(i + 1));
    linear_acc->set_z((float)(i + 2));
    haptix::comm::msgs::imu *angvel = _rep.add_imu_angular_vel();
    angvel->set_x((float)(i + 3));
    angvel->set_y((float)(i + 4));
    angvel->set_z((float)(i + 5));
  }
  _rep.mutable_time_stamp()->set_sec(time_sec);
  _rep.mutable_time_stamp()->set_nsec(time_nsec);
}

//////////////////////////////////////////////////
/// \brief Provide a "sensor update" service.
void onSensorRequest(const haptix::comm::msgs::hxSensor &/*_unused*/,
  haptix::comm::msgs::hxSensor &_rep, bool &_result)
{
  _result = true;

  _rep.Clear();

  // Create some dummy response.
  for (int i = 0; i < motorCount; ++i)
  {
    _rep.add_motor_pos((float)(i));
    _rep.add_motor_vel((float)(i + 1));
    _rep.add_motor_torque((float)(i + 2));
  }

  for (int i = 0; i < jointCount; ++i)
  {
    _rep.add_joint_pos((float)(i));
    _rep.add_joint_vel((float)(i + 1));
  }

  for (int i = 0; i < contactSensorCount; ++i)
    _rep.add_contact((float)(i));

  for (int i = 0; i < imuCount; ++i)
  {
    haptix::comm::msgs::imu *linear_acc = _rep.add_imu_linear_acc();
    linear_acc->set_x((float)(i));
    linear_acc->set_y((float)(i + 1));
    linear_acc->set_z((float)(i + 2));
    haptix::comm::msgs::imu *angvel = _rep.add_imu_angular_vel();
    angvel->set_x((float)(i + 3));
    angvel->set_y((float)(i + 4));
    angvel->set_z((float)(i + 5));
  }
  _rep.mutable_time_stamp()->set_sec(time_sec);
  _rep.mutable_time_stamp()->set_nsec(time_nsec);
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
    catch(const std::invalid_argument &/*_ex*/)
    {
      std::cerr << "<TIME_MS> argument must be a positive integer."
                << std::endl;
      return -1;
    }
  }

  // Create a Haptix transport node.
  ignition::transport::Node node;

  // Advertise the "getrobotinfo" service.
  if (!node.Advertise(RobotInfoTopic, onGetRobotInfo))
  {
    std::cerr << "Error advertising the [" << RobotInfoTopic << "] service."
              << std::endl;
  }

  // Advertise the "update" service.
  if (!node.Advertise(UpdateTopic, onUpdate))
  {
    std::cerr << "Error advertising the [" << UpdateTopic << "] service."
              << std::endl;
  }

  // Advertise the "readsensors" service.
  if (!node.Advertise(SensorInfoTopic, onSensorRequest))
  {
    std::cerr << "Error advertising the [" << SensorInfoTopic << "] service."
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
