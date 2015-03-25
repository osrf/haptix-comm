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
#include <cmath>
#include <iomanip>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <signal.h>
#include <stdio.h>
#include <time.h>
#include <haptix/comm/haptix.h>
#include <chrono>
#include <fstream>
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#endif

typedef std::chrono::steady_clock::time_point Timestamp;
typedef boost::accumulators::accumulator_set<long, boost::accumulators::stats<
          boost::accumulators::tag::mean,
          boost::accumulators::tag::median(boost::accumulators::with_p_square_quantile),
          boost::accumulators::tag::variance,
          boost::accumulators::tag::min,
          boost::accumulators::tag::max>> Stats;

int running = 1;

//////////////////////////////////////////////////
void sigHandler(int signo)
{
  // Terminate the program.
  running = 0;
}

//////////////////////////////////////////////////
void printStats(const Stats &_stats, Timestamp &_last)
{
  // Check if it's time to print stats.
  Timestamp now = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = now - _last;

  if (std::chrono::duration_cast<std::chrono::milliseconds>
       (elapsed).count() >= 1000)
  {
    // Print the header.
    static bool first = true;
    if (first)
    {
      std::cout << std::setw(10) << "Mean" << std::setw(10) << "Median"
              << std::setw(10) << "Min" << std::setw(10) << "Max"
              << std::setw(10) << "Stddev" << std::endl;
      first = false;
    }

    // Print a new row of stats.
    std::cout << std::fixed << std::setprecision(2)
              << std::setw(10) << boost::accumulators::mean(_stats)
              << std::setw(10) << boost::accumulators::median(_stats)
              << std::setw(10) << boost::accumulators::min(_stats)
              << std::setw(10) << boost::accumulators::max(_stats)
              << std::setw(10) << sqrt(boost::accumulators::variance(_stats))
              << std::endl;
    _last = std::chrono::steady_clock::now();
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  int i;
  hxRobotInfo robotInfo;
  hxCommand cmd;
  hxSensor sensor;
  float targetWristPos = 1.0;
  Stats stats;
  std::ofstream myfile;
  static const float kThreshold = 0.00001;

  myfile.open("1.dat");

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

  if (hx_read_sensors(&sensor) != hxOK)
  {
    printf("hx_read_sensors(): Request error.\n");
    return -1;
  }

  memset(&cmd, 0, sizeof(hxCommand));

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
    return -1;
  }

  // Let the hand reach the target.
  usleep(2000000);

  float dPos = std::abs(targetWristPos - sensor.motor_pos[2]);
  float lastDPos = dPos;

  cmd.ref_pos[2] = targetWristPos;
  hxTime cmdSent = sensor.time_stamp;
  hxTime last = sensor.time_stamp;

  // Send commands as fast as we can.
  while (running == 1)
  {
    // Send the new joint command and receive the state update.
    if (hx_update(&cmd, &sensor) != hxOK)
    {
      printf("hx_update(): Request error.\n");
      continue;
    }

    dPos = std::abs(targetWristPos - sensor.motor_pos[2]);

    if (lastDPos - dPos > kThreshold)
    {
      // Update stats.
      hxTime cmdApplied = sensor.time_stamp;

      long cmdElapsed = (((cmdApplied.sec - cmdSent.sec) * 1000000000)
        + (cmdApplied.nsec - cmdSent.nsec)) / 1000000.0;

      //std::chrono::duration<double> cmdElapsed = cmdApplied - cmdSent;
      //float ms = std::chrono::duration_cast<
      //  std::chrono::milliseconds>(cmdElapsed).count();
      stats(cmdElapsed);

      // Update log file.
      myfile << " " << cmdElapsed << std::endl;

      // Change wrist direction.
      targetWristPos = -targetWristPos;
      cmd.ref_pos[2] = targetWristPos;

      dPos = std::abs(targetWristPos - sensor.motor_pos[2]);
      cmdSent = sensor.time_stamp;
    }

    lastDPos = dPos;

    // Print stats if needed.
    printStats(stats, last);

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

  myFile.close();

  return 0;
}
