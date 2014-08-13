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
 /// \brief Provide an "echo" service.
 void callback(const char *_service, struct AplRobotCommand _req,
    struct AplRobotState *_rep, int *_result)
 {
    // Do something smart.
    int i;
    for (i = 0; i < ACI_num_joints; ++i)
    {
      _rep->state[i].position = i;
      _rep->state[i].velocity = i;
      _rep->state[i].effort = i;
    }
    *_result = 0;
 }

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Create a transport node.
  NodePtr node = newNode();

  // Advertise a service.
  nodeAdvertise(node, "/newJointCmd", callback);

  // Zzzz.
  printf("Accepting service calls. Press [ENTER] to exit.");
  getchar();

  // Destroy the node.
  deleteNode(node);
}
