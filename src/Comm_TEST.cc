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

#include "gtest/gtest.h"
#include "haptix/comm/AplControlInterface.h"
#include "haptix/comm/Comm.h"

//////////////////////////////////////////////////
/// \brief Provide a "dummy" service.
void callback(const char *_service, struct AplRobotCommand _req,
  struct AplRobotState *_rep, int *_result)
{
  // Check the name of the service received.
  EXPECT_STREQ(_service, "/test");

  // Check the request parameters.
  for (int i = 0; i < num_joints; ++i)
  {
    EXPECT_FLOAT_EQ(_req.command[i].position, 1.0 * i);
    EXPECT_FLOAT_EQ(_req.command[i].velocity,  2.0 * i + 1.0);
    EXPECT_FLOAT_EQ(_req.command[i].effort, 3.0 * i + 2.0);
    EXPECT_FLOAT_EQ(_req.command[i].kp_position, 4.0 * i + 3.0);
    EXPECT_FLOAT_EQ(_req.command[i].ki_position, 5.0 * i + 4.0);
    EXPECT_FLOAT_EQ(_req.command[i].kp_velocity, 6.0 * i + 5.0);
    EXPECT_FLOAT_EQ(_req.command[i].force, 7.0 * i + 6.0);
  }

  // Create some dummy response.
  for (int i = 0; i < num_joints; ++i)
  {
    _rep->state[i].position = i;
    _rep->state[i].velocity = i + 1.0;
    _rep->state[i].effort = i + 2.0;
  }
  *_result = 0;
}

//////////////////////////////////////////////////
/// \brief Check that we can use the C-wrapper.
TEST(CommTest, BasicUsage)
{
  // Create a transport node.
  NodePtr node = newNode();

  // Advertise an "test" service.
  nodeAdvertise(node, "/test", callback);

  struct AplRobotCommand jointCmd;
  struct AplRobotState jointState;
  int result;

  // Fill the joint command.
  for (int i = 0; i < num_joints; ++i)
  {
    jointCmd.command[i].position = 1.0 * i;
    jointCmd.command[i].velocity = 2.0 * i + 1.0;
    jointCmd.command[i].effort = 3.0 * i + 2.0;
    jointCmd.command[i].kp_position = 4.0 * i + 3.0;
    jointCmd.command[i].ki_position = 5.0 * i + 4.0;
    jointCmd.command[i].kp_velocity = 6.0 * i + 5.0;
    jointCmd.command[i].force = 7.0 * i + 6.0;
  }

  int timer = 5000;

  // Request a service call.
  int done = nodeRequest(node, "/test", jointCmd, timer, &jointState, &result);

  EXPECT_EQ(done, 0);
  ASSERT_EQ(result, 0);

  for (int i = 0; i < num_joints; ++i)
  {
    EXPECT_FLOAT_EQ(jointState.state[i].position, i);
    EXPECT_FLOAT_EQ(jointState.state[i].velocity, i + 1.0);
    EXPECT_FLOAT_EQ(jointState.state[i].effort, i + 2.0);
  }

  // Destroy the node.
  deleteNode(node);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
