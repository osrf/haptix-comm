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

#include <stdio.h>
#include <time.h>
#include <haptix/comm/haptix.h>

#ifdef _WIN32
#include <windows.h>
#endif

//////////////////////////////////////////////////
void printState(const hxRobotInfo *_robotInfo, const hxSensor *_sensor)
{
  int i;

  printf("\tMotors:\n");
  for (i = 0; i < _robotInfo->motor_count; ++i)
  {
    printf("\t\tMotor %d\n", i);
    printf("\t\t\tPosition: %f\n", _sensor->motor_pos[i]);
    printf("\t\t\tVelocity: %f\n", _sensor->motor_vel[i]);
    printf("\t\t\tTorque: %f\n" , _sensor->motor_torque[i]);
  }

  printf("\tJoints:\n");
  for (i = 0; i < _robotInfo->joint_count; ++i)
  {
    printf("\t\tJoint %d\n", i);
    printf("\t\t\tPosition: %f\n", _sensor->joint_pos[i]);
    printf("\t\t\tVelocity: %f\n", _sensor->joint_vel[i]);
  }

  printf("\tContact sensors:\n");
  for (i = 0; i < _robotInfo->contact_sensor_count; ++i)
  {
    printf("\t\t# %d\n", i);
    printf("\t\t\tvalue: %f\n", _sensor->contact[i]);
  }

  printf("\tIMUs:\n");
  for (i = 0; i < _robotInfo->imu_count; ++i)
  {
    printf("\t\t# %d\n", i);
    printf("\t\t\tLinear acceleration: (%f, %f, %f)\n",
      _sensor->imu_linear_acc[i][0], _sensor->imu_linear_acc[i][1],
      _sensor->imu_linear_acc[i][2]);
    printf("\t\t\tAngular velocity: (%f, %f, %f)\n",
      _sensor->imu_angular_vel[i][0], _sensor->imu_angular_vel[i][1],
      _sensor->imu_angular_vel[i][2]);
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  int i;
  int counter = 0;
  hxRobotInfo robotInfo;
  hxCommand cmd;
  hxSensor sensor;

  printf("\nRequesting robot information...\n\n");

  // Requesting robot information.
  if (hx_robot_info(&robotInfo) != hxOK)
  {
    printf("hx_getrobotinfo(): Request error.\n");
    return -1;
  }

  // Check results.
  printf("Robot information received:\n");
  printf("Num motors: %d\n", robotInfo.motor_count);
  printf("Num joints: %d\n", robotInfo.joint_count);
  printf("Num contact sensors: %d\n", robotInfo.contact_sensor_count);
  printf("Num IMUs: %d\n", robotInfo.imu_count);
  printf("Joint limits: \n");

  for (i = 0; i < robotInfo.joint_count; ++i)
  {
    printf("\tJoint %d:\n", i);
    printf("\t\t Min: %f\n", robotInfo.joint_limit[i][0]);
    printf("\t\t Max: %f\n", robotInfo.joint_limit[i][1]);
  }

  // Create a command.
  for (i = 0; i < robotInfo.motor_count; ++i)
  {
    cmd.ref_pos[i] = i;
    cmd.ref_vel_max[i] = i + 1;
    cmd.gain_pos[i] = i + 2;
    cmd.gain_vel[i] = i + 3;
  }

  // Send commands at ~100Hz.
  for (; ;)
  {
    if (hx_update(&cmd, &sensor) != hxOK)
    {
      printf("hx_update(): Request error.\n");
      continue;
    }

    // Print the state at ~1Hz.
    if (++counter == 100)
    {
      // Show the sensor output after hx_update().
      printf("Sensor state after hx_update():\n");
      printState(&robotInfo, &sensor);

      if (hx_read_sensors(&sensor) != hxOK)
      {
        printf("hx_update(): Request error.\n");
        continue;
      }

      // Show the sensor output after hx_readsensors().
      printf("Sensor state after hx_readsensors():\n");
      printState(&robotInfo, &sensor);

      counter = 0;
    }

    unsigned int sleeptime_us = 10000;
#ifdef _WIN32
    Sleep(sleeptime_us / 1e3);
#else
    usleep(sleeptime_us);
#endif
  }

  return 0;
}
