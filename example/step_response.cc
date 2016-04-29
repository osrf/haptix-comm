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
class StepResponseValidation : public gazebo::ValidationController
{

  public: StepResponseValidation(const std::string &_filePrefix, const bool _reference,
                               const int _motorIndex)
    : ValidationController(_reference),
      filePrefix(_filePrefix),
      motorIndex(_motorIndex)
  {
  }

  //////////////////////////////////////////////////
  void InitializeInitConds()
  {
    std::cout << "Initialize" << std::endl;
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
    for (auto i = 0; i < 6; ++i)
      cmd.ref_pos[i] = 0.0;

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
    std::cout << "InitializeRun" << std::endl;
    std::string filename = this->filePrefix + "_motor" +
      std::to_string(this->motorIndex) + "_run" + std::to_string(this->counter)
      + ".log";
    if (this->logFile.is_open())
      this->logFile.close();

    this->logFile.open(filename);

    ++this->counter;

    this->initialT = -1;
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

    if (hx_read_sensors(&sensor) != hxOK)
    {
      std::cerr << "hx_update(): Request error" << std::endl;
      return;
    }

    if (this->initialT < 0)
    {
      this->initialT = static_cast<double>(sensor.time_stamp.sec) +
        static_cast<double>(sensor.time_stamp.nsec) / 1e9;
    }

    this->currentT = (static_cast<double>(sensor.time_stamp.sec) +
        static_cast<double>(sensor.time_stamp.nsec) / 1e9) - this->initialT;

    double target;
    switch (this->motorIndex)
    {
      case 0:
        if (this->currentState->timer.GetElapsed() < gazebo::common::Time(3.0))
          target = -120;
        else if (this->currentState->timer.GetElapsed() < gazebo::common::Time(6.0))
          target = 0;
        else if (this->currentState->timer.GetElapsed() < gazebo::common::Time(9.0))
          target = 100;
        else
          target = 0;
        break;
      case 1:
        if (this->currentState->timer.GetElapsed() < gazebo::common::Time(3.0))
          target = -40;
        else if (this->currentState->timer.GetElapsed() < gazebo::common::Time(6.0))
          target = 0;
        else if (this->currentState->timer.GetElapsed() < gazebo::common::Time(9.0))
          target = 40;
        else
          target = 0;
        break;
      case 2:
        if (this->currentState->timer.GetElapsed() < gazebo::common::Time(3.0))
          target = 10;
        else if (this->currentState->timer.GetElapsed() < gazebo::common::Time(6.0))
          target = 0;
        else if (this->currentState->timer.GetElapsed() < gazebo::common::Time(9.0))
          target = 60;
        else
          target = 0;
        break;
      case 3:
        if (this->currentState->timer.GetElapsed() < gazebo::common::Time(3.0))
          target = 20;
        else if (this->currentState->timer.GetElapsed() < gazebo::common::Time(6.0))
          target = 0;
        else if (this->currentState->timer.GetElapsed() < gazebo::common::Time(9.0))
          target = 80;
        else
          target = 0;
        break;
      case 4:
        if (this->currentState->timer.GetElapsed() < gazebo::common::Time(3.0))
          target = 15;
        else if (this->currentState->timer.GetElapsed() < gazebo::common::Time(6.0))
          target = 0;
        else if (this->currentState->timer.GetElapsed() < gazebo::common::Time(9.0))
          target = 75;
        else
          target = 0;
        break;
      case 5:
        if (this->currentState->timer.GetElapsed() < gazebo::common::Time(3.0))
          target = 10;
        else if (this->currentState->timer.GetElapsed() < gazebo::common::Time(6.0))
          target = 0;
        else if (this->currentState->timer.GetElapsed() < gazebo::common::Time(9.0))
          target = 60;
        else
          target = 0;
        break;
      default:
        std::cout << "Incorrect motor index [" << this->motorIndex << "]"
                  << std::endl;   
        return;
    };

    for (auto i = 0; i < 6; ++i)
      cmd.ref_pos[i] = 0.0;
    cmd.ref_pos[this->motorIndex] = target;

    // Send the new joint command and receive the state update.
    if (hx_update(&cmd, &sensor) != hxOK)
    {
      std::cerr << "hx_update(): Request error" << std::endl;
      return;
    }

    int jointIndex;
    switch (this->motorIndex)
    {
      case 0:
        jointIndex = 0;
        break;
      case 1:
        jointIndex = 2;
        break;
      case 2:
        jointIndex = 3;
        break;
      case 3:
        jointIndex = 4;
        break;
      case 4:
        jointIndex = 5;
        break;
      case 5:
        jointIndex = 7;
        break;
      default:
        std::cout << "Incorrect motor index [" << this->motorIndex << "]"
                  << std::endl;   
        return;
    };

    // Write the log file.
    // # <time> <motor_index> <joint_index> <command> <state> 
    this->logFile << this->currentT << " " << this->motorIndex << " " << jointIndex << " "
                  << cmd.ref_pos[this->motorIndex] << " "
                  << sensor.joint_pos[jointIndex] << std::endl;

    usleep(20000);
  }

  //////////////////////////////////////////////////
  bool Running() const
  {
    return (this->currentState->timer.GetElapsed() < gazebo::common::Time(15.0));
  }

  /// \brief Run counter.
  unsigned int counter = 0;

  /// \brief ToDo.
  std::ofstream logFile;

  /// \brief ToDo.
  std::string filePrefix;

  /// \brief ToDo.
  int motorIndex;

  /// \brief ToDo.
  double initialT;

  /// \brief ToDo.
  double currentT;
};

//////////////////////////////////////////////////
void usage()
{
  std::cout << "step_response [--reference] <file_prefix> <motor_index>" << std::endl;
  std::cout << "   <motor_index> : [0-5]" << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc < 3 || argc > 4)
  {
    usage();
    return -1;
  }

  if ((argc == 3 && std::string(argv[1]) == "--reference") ||
      (argc == 3 && std::string(argv[2]) == "--reference"))
  {
    usage();
    return -1;
  }

  std::string prefix;
  int motorIndex;
  bool reference = false;
  if (argc == 3)
  {
    prefix = argv[1];
    motorIndex = std::stoi(argv[2]);
  }
  else if (argc == 4)
  {
    reference = true;
    if (std::string(argv[1]) == "--reference")
    {
      prefix = argv[2];
      motorIndex = std::stoi(argv[3]); 
    }
    else if (std::string(argv[2]) == "--reference")
    {
      prefix = argv[1];
      motorIndex = std::stoi(argv[3]);
    }
    else if (std::string(argv[3]) == "--reference")
    {
      prefix = argv[1];
      motorIndex = std::stoi(argv[2]);
    }
  }

  if (motorIndex < 0 || motorIndex > 5)
  {
    usage();
    return -1;
  }

  StepResponseValidation controller(prefix, reference, motorIndex);
  controller.Start();

  return 0;
}
