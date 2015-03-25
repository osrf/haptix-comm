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
#include <chrono>
#include <cmath>
#include <iomanip>
#include <thread>
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

#ifndef _WIN32
typedef boost::accumulators::accumulator_set<long, boost::accumulators::stats<
          boost::accumulators::tag::mean,
          boost::accumulators::tag::median(boost::accumulators::with_p_square_quantile),
          boost::accumulators::tag::variance,
          boost::accumulators::tag::min,
          boost::accumulators::tag::max>> Stats;
#endif

int running = 1;

//////////////////////////////////////////////////
void sigHandler(int signo)
{
  // Terminate the program.
  running = 0;
}

#ifndef _WIN32
//////////////////////////////////////////////////
void printStats(const Stats &_stats)
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
}
#endif

//////////////////////////////////////////////////
long elapsedMs(hxTime _time1, hxTime _time2)
{
  return ((_time1.sec + (_time1.nsec / 1000000000.0)) -
   (_time2.sec + (_time2.nsec / 1000000000.0))) * 1000;
}

//////////////////////////////////////////////////
void updateLogFile(const hxTime *_time, const long _elapsed,
  std::ofstream *_file)
{
  float fTime = _time->sec + (_time->nsec / 1000000000.0);
  *_file << fTime << " " << _elapsed << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  int i;
  hxRobotInfo robotInfo;
  hxCommand cmd;
  hxSensor sensor;
  float targetWristPos = 1.0;
#ifndef _WIN32
  Stats stats;
#endif
  std::ofstream logFile;
  static const float kThreshold = 0.00001;

  logFile.open("log2.dat", std::ios::out);

  // Capture SIGINT signal.
  if (signal(SIGINT, sigHandler) == SIG_ERR)
    printf("Error catching SIGINT\n");

  // Capture SIGTERM signal.
  if (signal(SIGTERM, sigHandler) == SIG_ERR)
    printf("Error catching SIGTERM\n");

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
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  if (hx_read_sensors(&sensor) != hxOK)
  {
    printf("hx_update(): Request error.\n");
    return -1;
  }

  float dPos = std::abs(targetWristPos - sensor.joint_pos[2]);
  float lastDPos = dPos;

  cmd.ref_pos[2] = targetWristPos;
  hxTime cmdSent = sensor.time_stamp;
  hxTime timeLastStatsPrinted = sensor.time_stamp;

  // Send commands as fast as we can.
  while (running == 1)
  {
    // Send the new joint command and receive the state update.
    if (hx_update(&cmd, &sensor) != hxOK)
    {
      printf("hx_update(): Request error.\n");
      continue;
    }

    dPos = std::abs(targetWristPos - sensor.joint_pos[2]);

    if (lastDPos - dPos > kThreshold)
    {
      // Update stats.
      hxTime cmdApplied = sensor.time_stamp;

      long cmdElapsed = elapsedMs(cmdApplied, cmdSent);
#ifndef _WIN32
      stats(cmdElapsed);
#endif

      // Update log file.
      updateLogFile(&cmdSent, cmdElapsed, &logFile);

      // Change wrist direction.
      targetWristPos = -targetWristPos;
      cmd.ref_pos[2] = targetWristPos;

      dPos = std::abs(targetWristPos - sensor.joint_pos[2]);
      cmdSent = sensor.time_stamp;
    }

    lastDPos = dPos;

#ifndef _WIN32
    // Print stats if needed.
    if (elapsedMs(sensor.time_stamp, timeLastStatsPrinted) > 2000)
    {
      printStats(stats);
      timeLastStatsPrinted = sensor.time_stamp;
    }
#endif
  }

  // Disconnect from the simulator / hardware
  if (hx_close() != hxOK)
  {
    printf("hx_close(): Request error.\n");
    return -1;
  }

  logFile.close();

  return 0;
}
