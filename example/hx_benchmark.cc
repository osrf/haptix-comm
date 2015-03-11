/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#ifdef _WIN32
#include <windows.h>
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <time.h>
#include <haptix/comm/haptix.h>
#include <chrono>
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#endif

typedef std::chrono::steady_clock::time_point Timestamp;

int running = 1;

//////////////////////////////////////////////////
void sigHandler(int signo)
{
  // Terminate the program.
  running = 0;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  int i;
  long counter = 0;
  hxRobotInfo robotInfo;
  hxCommand cmd;
  hxSensor sensor;

  // Capture SIGINT signal.
  if (signal(SIGINT, sigHandler) == SIG_ERR)
    printf("Error catching SIGINT\n");

  // Capture SIGTERM signal.
  if (signal(SIGTERM, sigHandler) == SIG_ERR)
    printf("Error catching SIGTERM\n");

  // Connect to the simulator / hardware
  if (hx_connect(NULL, 0) != hxOK)
  {
    printf("hx_connect(): Request error.\n");
    return -1;
  }

  // Requesting robot information.
  if (hx_robot_info(&robotInfo) != hxOK)
  {
    printf("hx_getrobotinfo(): Request error.\n");
    return -1;
  }

  Timestamp last = std::chrono::steady_clock::now();

  // Send commands as fast as we can.
  while (running == 1)
  {
    // Create a new command based on a sinusoidal wave.
    for (i = 0; i < robotInfo.motor_count; ++i)
    {
      // Set the desired position of this motor
      cmd.ref_pos[i] = 0.5 * sin(0.05 * 2.0 * M_PI * counter * 0.01);
      // We could set a desired maximum velocity
      // cmd.ref_vel_max[i] = 1.0;
      // We could set a desired controller position gain
      // cmd.gain_pos[i] = 1.0;
      // We could set a desired controller velocity gain
      // cmd.gain_vel[i] = 1.0;
    }
    // Indicate that the positions we set should be used.
    cmd.ref_pos_enabled = 1;
    // We're not setting it, so indicate that ref_vel_max should be ignored.
    cmd.ref_vel_max_enabled = 0;
    // We're not setting it, so indicate that gain_pos should be ignored.
    cmd.gain_pos_enabled = 0;
    // We're not setting it, so indicate that gain_vel should be ignored.
    cmd.gain_vel_enabled = 0;

    // Send the new joint command and receive the state update.
    if (hx_update(&cmd, &sensor) != hxOK)
    {
      printf("hx_update(): Request error.\n");
      continue;
    }

    ++counter;

    Timestamp now = std::chrono::steady_clock::now();
    // Elapsed time since the last update from this publisher.
    std::chrono::duration<double> elapsed = last - now;

    // This publisher has expired.
    if (std::chrono::duration_cast<std::chrono::milliseconds>
         (elapsed).count() >= 5000)
    {
      std::cout << "Running at " << counter / 5.0 << " Hz" << std::endl;
      counter = 0;
      last = std::chrono::steady_clock::now();
    }

    // Here is where you would do your other work, such as reading from EMG
    // sensors, decoding that data, computing your next control command,
    // etc.  In this example, we're just sleeping for 10ms.
    //
    // You might also want to sleep in your code, because there's a maximum
    // rate at which the limb can process new commands and produce new
    // sensor readings.  Depending on how long your computation takes, you
    // might want to wait here until it's time to send a new command.  Or
    // you might want to run as fast as possible, computing and sending new
    // commands constantly (but knowing that not all of them will be
    // executed by the limb).
  }

  // Disconnect from the simulator / hardware
  if (hx_close() != hxOK)
  {
    printf("hx_close(): Request error.\n");
    return -1;
  }

  return 0;
}
