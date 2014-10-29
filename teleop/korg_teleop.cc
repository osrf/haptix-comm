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
#include <yaml-cpp/yaml.h>
#include <climits>

class Grasp
{
  public:
    static const unsigned int grasp_size = 23;
  private:
    std::string name;
    unsigned int index;
    float grasp_vector[grasp_size];
  public:
    Grasp(const std::string n, const unsigned int i) : name(n), index(i){}
    unsigned int GetIndex() const
    {
      return index;
    }

    void SetGraspVectorAt(const unsigned int i, const float v)
    {
      if (i >= grasp_size){
        printf("Invalid index given in SetGraspVectorAt\n");
        return;
      }
      grasp_vector[i] = v;
    }

    float GetGraspVectorAt(const unsigned int i) const
    {
      if (i >= grasp_size){
        printf("Invalid index given in SetGraspVectorAt\n");
        return -1;
      }
      return grasp_vector[i];
    }
};

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  int i;
  int counter = 0;
  int mode = 0;
  bool pressed = false;

  // haptix_comm abstractions
  hxDeviceInfo deviceInfo;
  hxCommand cmd;
  hxSensor sensor;

  // Data structures for managing command information
  std::vector<Grasp> grasp_vectors;
  std::vector<std::pair<unsigned int, unsigned int> > ctrl_mappings;

  // slider board data
  Sliders board;
  float sliders_initial[board.NUM_CHANNELS];
  float knobs_initial[board.NUM_CHANNELS];

  // populate slider board from rtMidi driver
  board.rt_midi_open(1);

  printf("\nRequesting device information...\n\n");
  if (hx_getdeviceinfo(hxGAZEBO, &deviceInfo) != hxOK)
  {
    printf("hx_getdeviceinfo(): Request error.\n");
    return -1;
  }

  // Map of hxAPLMotor, index pairs (to make sense of enum)
  YAML::Node string_mappings = YAML::LoadFile("../config/motor_indices.yaml");
  YAML::Node slider_mappings = YAML::LoadFile("../config/slider_mappings.yaml");
  YAML::Node grasps = YAML::LoadFile("../config/grasps.yaml");

  for (YAML::const_iterator it=grasps.begin(); it != grasps.end(); it++)
  {
    const std::string grasp_name = it->first.as<std::string>();
    std::vector<float> grasp_vector = grasps[grasp_name]["grasp"].
                                        as<std::vector<float> >();
    int grasp_index = grasps[grasp_name]["slider"].as<int>();

    Grasp g(grasp_name, grasp_index);
    for (int i = 0; i < Grasp::grasp_size; i++)
    {
      g.SetGraspVectorAt(i, grasp_vector[i]);
    }
    grasp_vectors.push_back(g);
  }


  for (YAML::const_iterator it=slider_mappings.begin();
      it != slider_mappings.end(); it++)
  {
    unsigned int key =
              string_mappings[it->first.as<std::string>()].as<unsigned int>();
    ctrl_mappings.push_back(std::pair<unsigned int, unsigned int>(
                                          key, it->second.as<unsigned int>()));
  }
  
 for (int k = 0; k < board.NUM_CHANNELS; k++)
  {
    sliders_initial[k] = board.sliders[k];
    knobs_initial[k] = board.knobs[k];
  }

  for (i = 0; i < deviceInfo.nmotor; i++)
  {
    cmd.ref_pos[i] = 0;
  }

  printf("Use the sliders and knobs to control the fingers and wrist of the\
          hand. Press the play button to switch between modes.\n");

  for (; ;)
  {
    if ((!board.buttons[1]) && pressed)
    {
      mode = (mode+1) % 2;
      pressed = false;
    }
    else if (board.buttons[1])
    {
      pressed = true;
    }

    // Finger teleop
    if (mode == 0)
    {
      for (std::vector<std::pair<unsigned int, unsigned int>>::iterator it =
            ctrl_mappings.begin(); it != ctrl_mappings.end(); it++)
      {
        unsigned int device_idx = it->first;
        unsigned int slider_idx = it->second;
        float slider_val;
        float min = deviceInfo.limit[device_idx][0];
        float max = deviceInfo.limit[device_idx][1];
        if (slider_idx < board.NUM_CHANNELS)
        {
          if (board.sliders[slider_idx] == sliders_initial[slider_idx])
            continue;

          slider_val = board.sliders[slider_idx];
        }
        else
        {
          slider_idx %= board.NUM_CHANNELS;
          if(board.knobs[slider_idx] == knobs_initial[slider_idx])
            continue;

          slider_val = board.knobs[slider_idx];
        }
        cmd.ref_pos[device_idx] = (max-min)*slider_val + min;
      }

 
    }
    else
    {
      // Command pre-defined grasps

      // Reset the command
      for(i = 0; i < deviceInfo.nmotor; i++)
      {
        cmd.ref_pos[i] = 0;
      }
      // Check the state of the sliders and make a command
      float slider_total = 0;
      for (std::vector<Grasp>::iterator it = grasp_vectors.begin();
          it != grasp_vectors.end(); it++)
      {
        if(board.sliders[it->GetIndex()] > 0)
          slider_total += 1;
      }
      std::cout << "slider total: " << slider_total << std::endl;

      for (std::vector<Grasp>::iterator it = grasp_vectors.begin();
          it != grasp_vectors.end(); it++)
      {
        // Get the slider value and interpolate the grasp 
        unsigned int slider_idx = it->GetIndex();
        float slider_value = board.sliders[slider_idx];

        for (int j = 0; j < Grasp::grasp_size; j++)
        {
          if(slider_total > 0)
            cmd.ref_pos[j] += slider_value*it->GetGraspVectorAt(j)/(slider_total);
          if (cmd.ref_pos[j] > deviceInfo.limit[j][1])
          {
            cmd.ref_pos[j] = deviceInfo.limit[j][1];
          }
        }
      }
    }

    // Send the request
    if (hx_update(hxGAZEBO, &cmd, &sensor) != hxOK)
    {
      printf("hx_update(): Request error.\n");
    }
    else
    {
      cmd.timestamp = sensor.timestamp;
    }
  }

  return 0;
}
