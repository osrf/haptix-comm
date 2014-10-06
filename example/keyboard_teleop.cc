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

#include <ignition/transport.hh>
//#include <ignition/msgs/pose.pb.h>
#include <ignition/math.hh>

#include <gazebo/msgs/pose.pb.h>

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
    strncpy(filename, "../commands.yaml", 32);
  }
  
  YAML::Node commands = YAML::LoadFile(filename);
  
  YAML::Node motors = YAML::LoadFile("../motor_indices.yaml");

  YAML::Node arm_mappings = YAML::LoadFile("../arm_mappings.yaml");

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

  //TODO: auto-generate commands help screen
  raw();
  noecho();
  printw("Press buttons to control the hand. Press ESC to quit.\n");
  printw("Commands:\n");
  for(YAML::const_iterator it = commands.begin(); it != commands.end(); it++){
    std::string key = it->first.as<std::string>();
    std::stringstream ss;
    ss << "\t" << key << ": Increment " << it->second["motor_name"].as<std::string>() <<" by " << it->second["increment"].as<float>() << std::endl;

    printw(ss.str().c_str());
  }
  
  refresh();

  for(int i = 0; i < deviceInfo.nmotor; i++){
    cmd.ref_pos[i] = 0;
  }


  // Send commands at ~100Hz.
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

    std::string motor_name = commands[command]["motor_name"].as<std::string>();

    float inc = commands[command]["increment"].as<float>();
    if(!motors[motor_name].IsDefined()){
      //Arm base pose control
      int index = arm_mappings[motor_name].as<int>();

      float pose_inc_args[6] = {0, 0, 0, 0, 0, 0};
      pose_inc_args[index] = inc;
      ignition::math::Pose3<float> pose_inc(pose_inc_args[0], pose_inc_args[1],
                                            pose_inc_args[2], pose_inc_args[3],
                                            pose_inc_args[4], pose_inc_args[5]);
      gazebo::msgs::Pose msg;
      gazebo::msgs::Vector3d* vec_msg;
      vec_msg = msg.mutable_position();
      vec_msg->set_x(pose_inc.Pos().X());
      vec_msg->set_y(pose_inc.Pos().Y());
      vec_msg->set_z(pose_inc.Pos().Z());
      gazebo::msgs::Quaternion* quat_msg;
      quat_msg = msg.mutable_orientation();
      quat_msg->set_x(pose_inc.Rot().X());
      quat_msg->set_y(pose_inc.Rot().Y());
      quat_msg->set_z(pose_inc.Rot().Z());
      quat_msg->set_w(pose_inc.Rot().W());

      node.Publish("/haptix/arm_pose_inc", msg);
    } else {

      unsigned int motor_index = motors[motor_name].as<int>();

      cmd.ref_pos[motor_index] += inc;

      coupling_v1(&cmd);

      if (hx_update(hxGAZEBO, &cmd, &sensor) != hxOK)
        printf("hx_update(): Request error.\n");
      else
        cmd.timestamp = sensor.timestamp;
    }

    usleep(10000);
  }

  endwin();
  return 0;
}
