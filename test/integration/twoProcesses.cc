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

#include <chrono>
#include <thread>
#include "gtest/gtest.h"
#include "haptix/comm/Api.h"
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
void runReplier()
{
  // Create a transport node.
  NodePtr node = newNode();

  // Advertise an "test" service.
  nodeAdvertise(node, "/test", callback);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

//////////////////////////////////////////////////
/// \brief Three different nodes running in two different processes. In the
/// subscriber processs there are two nodes. Both should receive the message.
/// After some time one of them unsubscribe. After that check that only one
/// node receives the message.
TEST(twoProcesses, SrvTwoProcs)
{
  pid_t pid = fork();

  if (pid == 0)
    runReplier();
  else
  {
    struct AplRobotCommand jointCmd;
    struct AplRobotState jointState;
    int result;
    int timer = 5000;

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

    // Create a transport node.
    NodePtr node1 = newNode();

    // Make a valid service call.
    int done = nodeRequest(node1, "/test", jointCmd, timer, &jointState, &result);

    // The service call should not expire.
    EXPECT_EQ(done, 0);

    // The remote service call returned succesfully.
    ASSERT_EQ(result, 0);

    // Check the response.
    for (int i = 0; i < num_joints; ++i)
    {
      EXPECT_FLOAT_EQ(jointState.state[i].position, i);
      EXPECT_FLOAT_EQ(jointState.state[i].velocity, i + 1.0);
      EXPECT_FLOAT_EQ(jointState.state[i].effort, i + 2.0);
    }

    // Make an invalid service request.
    auto t1 = std::chrono::system_clock::now();
    done = nodeRequest(node1, "unknown_service", jointCmd, timer, &jointState,
           &result);
    auto t2 = std::chrono::system_clock::now();

    double elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

    // Check if the elapsed time was close to the timeout.
    EXPECT_NEAR(elapsed, timer, 5.0);

    // Wait for the child process to return.
    int status;
    waitpid(pid, &status, 0);
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
