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
#include "haptix/comm/Api.h"
#include <haptix/comm/Comm.h>

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Create a transport node.
  NodePtr node = newNode();

  Arm_t jointCmd, jointState;
  int result;

  // Fill the input parameter.
  jointCmd.pos = 1.5;
  jointCmd.vel = 2.5;
  int timer = 5000;

  // Request a service call.
  int done = nodeRequest(node, "/echo", jointCmd, timer, &jointState, &result);

  // Check results.
  if (done == 0)
  {
    if (result == 0)
    {
      printf("Pos received: %f\n", jointState.pos);
      printf("Vel received: %f\n", jointState.vel);
    }
    else
      printf("Request error.");
  }
  else
    printf("Request timed out.");

  // Destroy the node.
  deleteNode(node);
}
