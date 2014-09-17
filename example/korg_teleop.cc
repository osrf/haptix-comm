
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
#include <unistd.h>
#include <haptix/comm/haptix.h>
#include "sliders/sliders.h"

void printState(const hxDeviceInfo *_deviceInfo, const hxSensor *_sensor)
{
  int i;

  printf("\tMotors:\n");
  for (i = 0; i < _deviceInfo->nmotor; ++i)
  {
    printf("\t\tMotor %d\n", i);
    printf("\t\t\tPosition: %f\n", _sensor->motor_pos[i]);
    printf("\t\t\tVelocity: %f\n", _sensor->motor_vel[i]);
    printf("\t\t\tTorque: %f\n" , _sensor->motor_torque[i]);
  }

  printf("\tJoints:\n");
  for (i = 0; i < _deviceInfo->njoint; ++i)
  {
    printf("\t\tJoint %d\n", i);
    printf("\t\t\tPosition: %f\n", _sensor->joint_pos[i]);
    printf("\t\t\tVelocity: %f\n", _sensor->joint_vel[i]);
  }

  printf("\tContact sensors:\n");
  for (i = 0; i < _deviceInfo->ncontactsensor; ++i)
  {
    printf("\t\t# %d\n", i);
    printf("\t\t\tvalue: %f\n", _sensor->contact[i]);
  }

  printf("\tIMUs:\n");
  for (i = 0; i < _deviceInfo->nIMU; ++i)
  {
    printf("\t\t# %d\n", i);
    printf("\t\t\tLinear acceleration: (%f, %f, %f)\n",
      _sensor->IMU_linacc[i][0], _sensor->IMU_linacc[i][1],
      _sensor->IMU_linacc[i][2]);
    printf("\t\t\tAngular velocity: (%f, %f, %f)\n",
      _sensor->IMU_angvel[i][0], _sensor->IMU_angvel[i][1],
      _sensor->IMU_angvel[i][2]);
  }
}


//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  int i;
  int n = 9;
  int m = 0;
  int counter = 0;
  hxDeviceInfo deviceInfo;
  hxCommand cmd;
  hxSensor sensor;

  Sliders board;
  board.rt_midi_open(1);
  //right now only analog controllers are supported


  printf("\nRequesting device information...\n\n");

  // Requesting device information.
  if (hx_getdeviceinfo(hxGAZEBO, &deviceInfo) != hxOK)
  {
    printf("hx_getdeviceinfo(): Request error.\n");
    return -1;
  }

  // Check results.
  printf("Device information received:\n");
  printf("Num motors: %d\n", deviceInfo.nmotor);
  printf("Num joints: %d\n", deviceInfo.njoint);
  printf("Num contact sensors: %d\n", deviceInfo.ncontactsensor);
  printf("Num IMUs: %d\n", deviceInfo.nIMU);
  printf("Joint limits: \n");

  for (i = 0; i < deviceInfo.njoint; ++i)
  {
    printf("\tJoint %d:\n", i);
    printf("\t\t Min: %f\n", deviceInfo.limit[i][0]);
    printf("\t\t Max: %f\n", deviceInfo.limit[i][1]);
  }

  if(deviceInfo.nmotor < n){
    n = deviceInfo.nmotor;
    printf("Not enough motors to use all sliders\n");
  } else if (deviceInfo.nmotor > n){
    printf("Not enough sliders for all motors\n");
    m = deviceInfo.nmotor - n;
    m = m > 9 ? 9 : m;
    printf("Using knobs 1-%d\n", m);
  }

  cmd.timestamp = 0; //hack
  for (; ;)
  {
    
    //Check the state of the sliders and make a command

    for(int i = 0; i < n; i++){
      float min = deviceInfo.limit[i][0];
      float max = deviceInfo.limit[i][1];
      cmd.ref_pos[i] = (max - min)* board.sliders[i] + min;
    }
    for(int j = 0; j < m; j++){
      float min = deviceInfo.limit[j+n][0];
      float max = deviceInfo.limit[j+n][1];
      cmd.ref_pos[j+n] = (max - min)* board.knobs[j] + min;
   
    }

    //Send the request
    if (hx_update(hxGAZEBO, &cmd, &sensor) != hxOK)
      printf("hx_update(): Request error.\n");
    else
      cmd.timestamp = sensor.timestamp;

    // Print the state at ~1Hz.
    /*if (++counter == 100)
    {
      printState(&deviceInfo, &sensor);
      counter = 0;
    }*/

    usleep(10000);
  }




  return 0;
}
