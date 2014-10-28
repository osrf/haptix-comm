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

#include <ignition/transport.hh>
#include <ignition/msgs.hh>
#include <ignition/math.hh>

#include <ncurses.h>

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

  ignition::transport::Node node("haptix");
  node.Advertise("arm_pose_inc");

  // Get the keyboard control mappings from .yaml file.
  if(argc > 1){
    strncpy(filename, argv[1], maxnamesize);
  } else{
    printf("No yaml filename given, using default file commands.yaml.\n");
    strncpy(filename, "../config/commands.yaml", 32);
  }
  
  YAML::Node commands = YAML::LoadFile(filename);
  
  YAML::Node motors = YAML::LoadFile("../config/motor_indices.yaml");

  YAML::Node arm_mappings = YAML::LoadFile("../config/arm_mappings.yaml");

  YAML::Node view_mappings = YAML::LoadFile("../config/view_mappings.yaml");

  printf("\nRequesting device information...\n\n");

  // Requesting device information.
  if (hx_getdeviceinfo(hxGAZEBO, &deviceInfo) != hxOK)
  {
    printf("hx_getdeviceinfo(): Request error.\n");
    return -1;
  }

  if( initscr() == NULL ){
    perror("Could not initialize ncurses.");
    return -1;
  }

  raw();
  noecho();
  printw("Press buttons to control the hand. Press ESC to quit.\n");
  printw("Commands:\n");
  for(YAML::const_iterator it = commands.begin(); it != commands.end(); it++){
    std::string key = it->first.as<std::string>();
    std::stringstream ss;
    ss << "\t" << key << ": Increment " << it->second["command_name"].as<std::string>() <<" by " << it->second["increment"].as<float>() << std::endl;

    printw(ss.str().c_str());
  }
  
  refresh();

  for(int i = 0; i < deviceInfo.nmotor; i++){
    cmd.ref_pos[i] = 0;
  }


  for (; ;)
  {

    c = getch();

    //Convert c to a string so we can use it to index into YAML nodes
    char command[1];
    sprintf(command, "%c", c);
    if(!commands[command].IsDefined()){
      if(c==27){ //ESC
        break;
      }
      
      continue;
    }

    std::string motor_name = commands[command]["command_name"].as<std::string>();

    float inc = commands[command]["increment"].as<float>();
    if(!motors[motor_name].IsDefined()){
      if(!arm_mappings[motor_name].IsDefined()){
          continue;
      }

      //Arm base pose control
      int index = arm_mappings[motor_name].as<int>();

      float pose_inc_args[6] = {0, 0, 0, 0, 0, 0};
      pose_inc_args[index] = inc;
      
      ignition::msgs::Pose msg;
      msg.mutable_position()->set_x(pose_inc_args[0]);
      msg.mutable_position()->set_y(pose_inc_args[1]);
      msg.mutable_position()->set_z(pose_inc_args[2]);
      ignition::math::Quaternion<float> q(pose_inc_args[3], pose_inc_args[4],
                                 pose_inc_args[5]);
      msg.mutable_orientation()->set_x(q.X());
      msg.mutable_orientation()->set_y(q.Y());
      msg.mutable_orientation()->set_z(q.Z());
      msg.mutable_orientation()->set_w(q.W());

      node.Publish("/haptix/arm_pose_inc", msg);
      
    } else {

      unsigned int motor_index = motors[motor_name].as<int>();

      cmd.ref_pos[motor_index] += inc;

      if (hx_update(hxGAZEBO, &cmd, &sensor) != hxOK)
        printf("hx_update(): Request error.\n");
      else
        cmd.timestamp = sensor.timestamp;
    }

    // Send commands at ~100Hz.
    usleep(10000);
  }

  endwin();
  return 0;
}
