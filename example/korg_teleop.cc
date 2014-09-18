
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
#include "teleop.h"

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  int i;
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

  //Map of hxAPLMotor, index pairs

  std::map<hxAPLMotors, unsigned int> ctrl_mappings;
  //Rightmost slider is little finger (right-handed scheme)
  ctrl_mappings[motor_little_ab_ad] = 17;
  ctrl_mappings[motor_ring_ab_ad]   = 16;
  ctrl_mappings[motor_middle_ab_ad] = 15;
  ctrl_mappings[motor_index_ab_ad]  = 14;
  ctrl_mappings[motor_thumb_cmc_ab_ad]  = 13;
  ctrl_mappings[motor_thumb_cmc_fe]  = 12;

  ctrl_mappings[motor_little_mcp] = 8;
  ctrl_mappings[motor_ring_mcp]   = 7;
  ctrl_mappings[motor_middle_mcp] = 6;
  ctrl_mappings[motor_index_mcp]  = 5;
  ctrl_mappings[motor_thumb_mcp]  = 4;

  ctrl_mappings[motor_wrist_rot]  = 3;
  ctrl_mappings[motor_wrist_dev]  = 2;
  ctrl_mappings[motor_wrist_fe]  = 1;


  float sliders_initial[board.NUM_CHANNELS];
  float knobs_initial[board.NUM_CHANNELS];
  for(int k = 0; k < board.NUM_CHANNELS; k++){
    sliders_initial[k] = board.sliders[k];
    knobs_initial[k] = board.knobs[k];
  }

  for(i = 0; i < deviceInfo.nmotor; i++){
    cmd.ref_pos[i] = 0;
  }


  for (; ;)
  {
    
    //Check the state of the sliders and make a command

    for(std::map<hxAPLMotors, unsigned int>::iterator it = ctrl_mappings.begin(); it != ctrl_mappings.end(); it++){
      hxAPLMotors device_idx = it->first;
      unsigned int slider_idx = it->second;
      float slider_val;
      float min = deviceInfo.limit[device_idx][0];
      float max = deviceInfo.limit[device_idx][1];
      if(slider_idx < 9){
        if(board.sliders[slider_idx] == sliders_initial[slider_idx])
          continue;

        slider_val = board.sliders[slider_idx];
      } else {
        slider_idx %= 9;
        if(board.knobs[slider_idx] == knobs_initial[slider_idx])
          continue;

        slider_val = board.knobs[slider_idx];
      }
      cmd.ref_pos[device_idx] = (max-min)*slider_val + min;
    }

    coupling_v1(&cmd);
    

    //Send the request
    if (hx_update(hxGAZEBO, &cmd, &sensor) != hxOK)
      printf("hx_update(): Request error.\n");
    else
      cmd.timestamp = sensor.timestamp;

    usleep(10000);
  }

  return 0;
}
