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
#include <cstring>
#include <time.h>
#include <yaml-cpp/yaml.h>
#include <haptix/comm/haptix.h>
#include <ncurses.h>
#include "teleop.h"

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  //int i;
  int counter = 0;
  hxDeviceInfo deviceInfo;
  hxCommand cmd;
  hxSensor sensor;
  char c;

  const int maxnamesize = 128;
  char filename[maxnamesize];
  FILE* f;
  long filesize;
  char* yaml_str;
  char* buffer;

  // Get the keyboard control mappings from .yaml file.
  if(argc > 1){
    strncpy(filename, argv[1], maxnamesize);
  } else{
    printf("No yaml filename given, using default file commands.yaml.\n");
    strncpy(filename, "../commands.yaml", 32);
  }
  
  YAML::Node commands = YAML::LoadFile(filename);
  
  YAML::Node motors = YAML::LoadFile("../motor_indices.yaml");

  printf("\nRequesting device information...\n\n");

  // Requesting device information.
  if (hx_getdeviceinfo(hxGAZEBO, &deviceInfo) != hxOK)
  {
    printf("hx_getdeviceinfo(): Request error.\n");
    return -1;
  }

  initscr();

  //TODO: auto-generate help screen
  raw();
  noecho();
  printw("Press buttons to control the hand. Press ESC to quit.");
  refresh();

  for(int i = 0; i < deviceInfo.nmotor; i++){
    cmd.ref_pos[i] = 0;
  }
  // Send commands at ~100Hz.
  for (; ;)
  {

    c = getch();

    //printf("Got input %d\n", c);
    //Convert c to a string so we can use it to index into YAML nodes
    char command[1];
    sprintf(command, "%c", c);
    if(!commands[command].IsDefined()){
      if(c==27){ //q
        break;
      }
      
      continue;
    }
    std::string motor_name = commands[command]["motor_name"].as<std::string>();
    unsigned int motor_index = motors[motor_name].as<int>();
    float inc = commands[command]["increment"].as<float>();

    cmd.ref_pos[motor_index] += inc;

    coupling_v1(&cmd);

    if (hx_update(hxGAZEBO, &cmd, &sensor) != hxOK)
      printf("hx_update(): Request error.\n");
    else
      cmd.timestamp = sensor.timestamp;

    usleep(10000);
  }

  endwin();
  return 0;
}
