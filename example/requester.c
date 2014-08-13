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
#include <haptix/comm/AplControlInterface.h>
#include <haptix/comm/Comm.h>

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Create a transport node.
  NodePtr node = newNode();

  struct AplRobotCommand jointCmd;
  struct AplRobotState jointState;
  int result;

  // Fill the joint command.
  int i;
  for (i = 0; i < ACI_num_joints; ++i)
  {
    jointCmd.command[i].position = 1.0 * i;
    jointCmd.command[i].velocity = 2.0 * i;
    jointCmd.command[i].effort = 3.0 * i;
    jointCmd.command[i].kp_position = 4.0 * i;
    jointCmd.command[i].ki_position = 5.0 * i;
    jointCmd.command[i].kp_velocity = 6.0 * i;
    jointCmd.command[i].force = 7.0 * i;
  }

  int timer = 5000;

  // Request a service call.
  int done = nodeRequest(node, "/newJointCmd", jointCmd, timer,
    &jointState, &result);

  // Check results.
  if (done == 0)
  {
    if (result == 0)
    {
      printf("State received:\n");

      int i;
      for (i = 0; i < ACI_num_joints; ++i)
      {
        printf("Joint %d\n", i);
        printf("Position: %f\n", jointState.state[i].position);
        printf("Velocity: %f\n", jointState.state[i].velocity);
        printf("Effort: %f\n--\n", jointState.state[i].effort);
      }
    }
    else
      printf("Request error.");
  }
  else
    printf("Request timed out.");

  // Destroy the node.
  deleteNode(node);
}
