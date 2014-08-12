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
 /// \brief Provide an "echo" service.
 void echo(const char *_service, Arm_t _req, Arm_t *_rep, int *_result)
 {
   _rep->pos = _req.pos;
   _rep->vel = _req.vel;
   *_result = 0;
 }

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Create a transport node.
  NodePtr node = newNode();

  // Advertise an "echo" service.
  nodeAdvertise(node, "/echo", echo);

  // Zzzz.
  printf("Accepting service calls. Press [ENTER] to exit.");
  getchar();

  // Destroy the node.
  deleteNode(node);
}
