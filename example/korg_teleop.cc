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
#include <climits>

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  int i;
  int counter = 0;
  int mode = 0;
  bool pressed = false;
  hxDeviceInfo deviceInfo;
  hxCommand cmd;
  hxSensor sensor;

  Sliders board;
  board.rt_midi_open(1);
  //right now only analog controllers are supported

  printf("\nRequesting device information...\n\n");

  // Requesting device information.
  if (hx_getdeviceinfo(hxMUJOCO, &deviceInfo) != hxOK)
  {
    printf("hx_getdeviceinfo(): Request error.\n");
    return -1;
  }

  //TODO: Give some helpful text telling the user what to do

  //Map of hxAPLMotor, index pairs (to make sense of enum)
  YAML::Node string_mappings = YAML::LoadFile("../motor_indices.yaml");
  YAML::Node slider_mappings = YAML::LoadFile("../slider_mappings.yaml");
  YAML::Node grasps = YAML::LoadFile("../grasps.yaml");


  std::map<unsigned int, unsigned int> ctrl_mappings;
  //Rightmost slider is little finger (right-handed scheme)
  for(YAML::const_iterator it=slider_mappings.begin(); it != slider_mappings.end(); it++){
    //string_mappings[it->first] = key, it->second = value
    unsigned int key = string_mappings[it->first.as<std::string>()].as<unsigned int>();
    ctrl_mappings[key] = it->second.as<unsigned int>();
  }
  
  float sliders_initial[board.NUM_CHANNELS];
  float knobs_initial[board.NUM_CHANNELS];
  for(int k = 0; k < board.NUM_CHANNELS; k++){
    sliders_initial[k] = board.sliders[k];
    knobs_initial[k] = board.knobs[k];
  }

  for(i = 0; i < deviceInfo.nmotor; i++){
    cmd.ref_pos[i] = 0;
  }

  printf("Use the sliders and knobs to control the fingers and wrist of the hand. Press the play button to switch between modes.\n");

  for (; ;)
  {
    if((!board.buttons[1]) && pressed){
      mode = (mode+1) % 2;
      pressed = false;
    }
    else if(board.buttons[1]){
      pressed = true;
    }

    if(mode == 0){ //Finger teleop
      for(std::map<unsigned int, unsigned int>::iterator it =
            ctrl_mappings.begin(); it != ctrl_mappings.end(); it++){
        unsigned int device_idx = it->first;
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

 
    } else { //Eigengrasps
      for(i = 0; i < deviceInfo.nmotor; i++){
        cmd.ref_pos[i] = 0;
      }
      //Check the state of the sliders and make a command

      for(YAML::const_iterator it=grasps.begin(); it != grasps.end(); it++){
        //Get the slider value and interpolate the grasp 
        unsigned int slider_idx = grasps[it->first]["slider"].as<unsigned int>();
        float slider_value = board.sliders[slider_idx];
        std::vector<float> grasp_vec = grasps[it->first]["grasp"].as<std::vector<float> >();

        float max = -INT_MAX;
        float scale = 0;
        for(int j = 0; j < grasp_vec.size(); j++){
          cmd.ref_pos[j] += slider_value*grasp_vec[j];
          if(cmd.ref_pos[j] > max){
            max = cmd.ref_pos[j];
            float jointlim = deviceInfo.limit[j][1];
            if( max > jointlim){
              scale = jointlim/max;
            }
          }
        }

        if(scale != 0){
          for(int j = 0; j < grasp_vec.size(); j++){
            cmd.ref_pos[j]*=scale;
          }
        }

      }
    }
    coupling_v1(&cmd);

    //Send the request
    if (hx_update(hxMUJOCO, &cmd, &sensor) != hxOK)
      printf("hx_update(): Request error.\n");
    else
      cmd.timestamp = sensor.timestamp;

    if(board.buttons[2]){
      printf("Sending command\n");
      for(int j = 0; j < deviceInfo.nmotor; j++){
        printf("%f\n", cmd.ref_pos[j]);
      }
      usleep(10000);
    }
  }

  return 0;
}
