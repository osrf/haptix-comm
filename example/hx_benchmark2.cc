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
#ifndef _WIN32
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#endif
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
void reset()
{
  const float kThreshold = 0.001;
  hxCommand cmd;
  hxSensor sensor;

  memset(&cmd, 0, sizeof(hxCommand));

  cmd.ref_pos[0] = 0.0;
  cmd.ref_pos_enabled = 1;
  cmd.ref_vel_max_enabled = 0;
  cmd.gain_pos_enabled = 0;
  cmd.gain_vel_enabled = 0;

  do
  {
    // Send the new joint command and receive the state update.
    if (hx_update(&cmd, &sensor) != hxOK)
    {
      printf("hx_update(): Request error.\n");
      return;
    }
  } while (std::abs(sensor.motor_pos[0]) > kThreshold);
}

//////////////////////////////////////////////////
void stop()
{
  hxCommand cmd;
  hxSensor sensor;

  memset(&cmd, 0, sizeof(hxCommand));

  cmd.ref_pos_enabled = 1;
  cmd.ref_vel_max_enabled = 0;
  cmd.gain_pos_enabled = 0;
  cmd.gain_vel_enabled = 0;

  // Send the new joint command and receive the state update.
  if (hx_read_sensors(&sensor) != hxOK)
  {
    printf("hx_read_sensors(): Request error.\n");
    return;
  }

  // Keep the hand in contact.
  cmd.ref_pos[0] = sensor.motor_pos[0];
  if (hx_update(&cmd, &sensor) != hxOK)
  {
    printf("hx_update(): Request error.\n");
    return;
  }
}

//////////////////////////////////////////////////
void up(hxTime *_timestamp)
{
  hxCommand cmd;
  hxSensor sensor;

  memset(&cmd, 0, sizeof(hxCommand));

  cmd.ref_pos[0] = -1.0;
  cmd.ref_pos_enabled = 1;
  cmd.ref_vel_max_enabled = 0;
  cmd.gain_pos_enabled = 0;
  cmd.gain_vel_enabled = 0;

  // Send the new joint command and receive the state update.
  if (hx_update(&cmd, &sensor) != hxOK)
  {
    printf("hx_update(): Request error.\n");
    return;
  }

  *_timestamp = sensor.time_stamp;
}

//////////////////////////////////////////////////
void down(hxTime *_timestamp)
{
  hxCommand cmd;
  hxSensor sensor;

  memset(&cmd, 0, sizeof(hxCommand));

  cmd.ref_pos[0] = 1.0;
  cmd.ref_pos_enabled = 1;
  cmd.ref_vel_max_enabled = 0;
  cmd.gain_pos_enabled = 0;
  cmd.gain_vel_enabled = 0;

  // Send the new joint command and receive the state update.
  if (hx_update(&cmd, &sensor) != hxOK)
  {
    printf("hx_update(): Request error.\n");
    return;
  }

  *_timestamp = sensor.time_stamp;
}

//////////////////////////////////////////////////
long elapsedMs(hxTime _time1, hxTime _time2)
{
  return ((_time1.sec + (_time1.nsec / 1000000000.0)) -
   (_time2.sec + (_time2.nsec / 1000000000.0))) * 1000;
}

//////////////////////////////////////////////////
void waitForContact(hxTime *_timestamp)
{
  const float kContactThreshold = 0.001;
  hxSensor sensor;
  do
  {
    // Send the new joint command and receive the state update.
    if (hx_read_sensors(&sensor) != hxOK)
    {
      printf("hx_read_sensors(): Request error.\n");
      return;
    }
  } while (sensor.contact[6] <= kContactThreshold);

  *_timestamp = sensor.time_stamp;
}

//////////////////////////////////////////////////
void waitForNoContact(hxTime *_timestamp)
{
  const float kContactThreshold = 0.001;
  hxSensor sensor;
  do
  {
    // Send the new joint command and receive the state update.
    if (hx_read_sensors(&sensor) != hxOK)
    {
      printf("hx_read_sensors(): Request error.\n");
      return;
    }
  } while (sensor.contact[6] > kContactThreshold);

  *_timestamp = sensor.time_stamp;
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
  Stats stats;
  std::ofstream logFile;
  hxTime timeCmdSent;
  hxTime timeNow;
  hxTime timeLastStatsPrinted;

  logFile.open("log.dat");

  // Capture SIGINT signal.
  if (signal(SIGINT, sigHandler) == SIG_ERR)
    printf("Error catching SIGINT\n");

  // Capture SIGTERM signal.
  if (signal(SIGTERM, sigHandler) == SIG_ERR)
    printf("Error catching SIGTERM\n");

  if (hx_read_sensors(&sensor) != hxOK)
  {
    printf("hx_read_sensors(): Request error.\n");
    return -1;
  }
  timeLastStatsPrinted = sensor.time_stamp;

  reset();
  usleep(500000);

  while (running)
  {
    down(&timeCmdSent);
    // Start timer.
    waitForContact(&timeNow);
    // Stop timer.
    long elapsedUntilContact = elapsedMs(timeNow, timeCmdSent);
    stats(elapsedUntilContact);
    stop();
    updateLogFile(&timeCmdSent, elapsedUntilContact, &logFile);
    usleep(500000);

    up(&timeCmdSent);
    // Start timer.
    waitForNoContact(&timeNow);
    // Stop timer.
    long elapsedUntilNoContact = elapsedMs(timeNow, timeCmdSent);
    stats(elapsedUntilNoContact);

    updateLogFile(&timeCmdSent, elapsedUntilContact, &logFile);

#ifndef _WIN32
    long elapsedSinceLastStats = elapsedMs(timeNow, timeLastStatsPrinted);
    // Print stats if needed.
    if (elapsedMs(timeNow, timeLastStatsPrinted) > 2000)
    {
      printStats(stats);
      timeLastStatsPrinted = timeNow;
    }
#endif

    reset();
    usleep(500000);
  }

  logFile.close();
}
