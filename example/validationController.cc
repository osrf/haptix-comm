/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <iostream>
#include <fstream>
#include <gazebo/plugins/validation/ValidationController.hh>
#include <haptix/comm/haptix.h>

/// \brief
class WristLimitValidation : public gazebo::ValidationController
{

  public: WristLimitValidation(const std::string &_filePrefix)
    : filePrefix(_filePrefix)
  {
  }

  //////////////////////////////////////////////////
  void InitializeInitConds()
  {
    hxCommand cmd;
    hxSensor sensor;

    // Indicate that the positions we set should be used.
    cmd.ref_pos_enabled = 1;
    // We're not setting it, so indicate that ref_vel should be ignored.
    cmd.ref_vel_enabled = 0;
    // We're not setting it, so indicate that ref_vel_max should be ignored.
    cmd.ref_vel_max_enabled = 0;
    // We're not setting it, so indicate that gain_pos should be ignored.
    cmd.gain_pos_enabled = 0;
    // We're not setting it, so indicate that gain_vel should be ignored.
    cmd.gain_vel_enabled = 0;
    // Set the desired position of this motor
    cmd.ref_pos[0] =static_cast<float>(0);

    // Send the new joint command and receive the state update.
    if (hx_update(&cmd, &sensor) != hxOK)
    {
      std::cerr << "hx_update(): Request error" << std::endl;
    }
  };

  //////////////////////////////////////////////////
  // void UpdateInitConds()
  // {

  // };

  //////////////////////////////////////////////////
  bool Initializing() const
  {
    return this->currentState->timer.GetElapsed() < gazebo::common::Time(2.0);
  }

  //////////////////////////////////////////////////
  void InitializeRun()
  {
    std::string filename = this->filePrefix + std::to_string(this->counter)
      + ".log";
    if (this->logFile.is_open())
      this->logFile.close();

    this->logFile.open(filename);

    ++this->counter;
    this->step = 0;
  };

  //////////////////////////////////////////////////
  void UpdateRun()
  {
    hxCommand cmd;
    hxSensor sensor;

    // Indicate that the positions we set should be used.
    cmd.ref_pos_enabled = 1;
    // We're not setting it, so indicate that ref_vel should be ignored.
    cmd.ref_vel_enabled = 0;
    // We're not setting it, so indicate that ref_vel_max should be ignored.
    cmd.ref_vel_max_enabled = 0;
    // We're not setting it, so indicate that gain_pos should be ignored.
    cmd.gain_pos_enabled = 0;
    // We're not setting it, so indicate that gain_vel should be ignored.
    cmd.gain_vel_enabled = 0;
    // Set the desired position of this motor
    cmd.ref_pos[0] =static_cast<float>(
        350 * 0.5 * sin(0.05 * 2.0 * M_PI * this->step * 0.08));

    // Send the new joint command and receive the state update.
    if (hx_update(&cmd, &sensor) != hxOK)
    {
      std::cerr << "hx_update(): Request error" << std::endl;
    }

    usleep(20000);

    // Write time in the log file.
    this->logFile << sensor.time_stamp.sec << "." << sensor.time_stamp.nsec;
    // Write the state of wrist_rot joint in the log file.
    this->logFile << " " << sensor.joint_pos[0] << std::endl;

    ++this->step;
  }

  //////////////////////////////////////////////////
  bool Running() const
  {
    return this->step < 250;
  }

  /// \brief Run counter.
  unsigned int counter = 0;

  /// \brief ToDo.
  int step = 0;

  /// \brief ToDo.
  std::ofstream logFile;

  /// \brief ToDo.
  std::string filePrefix;
};

//////////////////////////////////////////////////
void usage()
{
  std::cout << "validationController <file_prefix>" << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc != 2)
  {
    usage();
    return -1;
  }

  WristLimitValidation controller(argv[1]);
  controller.Start();

  return 0;
}
