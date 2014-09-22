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
#include <yaml-cpp/yaml.h>

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

  //TODO: Give some helpful text telling the user what to do

  //Map of hxAPLMotor, index pairs (to make sense of enum)
  YAML::Node string_mappings = YAML::LoadFile("../motor_indices.yaml");

  YAML::Node grasps = YAML::LoadFile("../grasps.yaml");
  
  float sliders_initial[board.NUM_CHANNELS];
  float knobs_initial[board.NUM_CHANNELS];
  for(int k = 0; k < board.NUM_CHANNELS; k++){
    sliders_initial[k] = board.sliders[k];
    knobs_initial[k] = board.knobs[k];
  }

  for (; ;)
  {
   
    for(i = 0; i < deviceInfo.nmotor; i++){
      cmd.ref_pos[i] = 0;
    }   
    //Check the state of the sliders and make a command

    for(YAML::const_iterator it=grasps.begin(); it != grasps.end(); it++){
      //Get the slider value and interpolate the grasp 
      unsigned int slider_idx = grasps[it->first]["slider"].as<unsigned int>();
      float slider_value = board.sliders[slider_idx];
      std::vector<float> grasp_vec = grasps[it->first]["grasp"].as<std::vector<float> >();
      for(int j = 0; j < grasp_vec.size(); j++){
        cmd.ref_pos[j] += slider_value*grasp_vec[j];
      }
    }

    //Send the request
    if (hx_update(hxGAZEBO, &cmd, &sensor) != hxOK)
      printf("hx_update(): Request error.\n");
    else
      cmd.timestamp = sensor.timestamp;

    usleep(10000);
  }

  return 0;
}
