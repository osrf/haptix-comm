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
class SinusoidalValidation : public gazebo::ValidationController
{

  public: SinusoidalValidation(const std::string &_filePrefix, const bool _reference,
                               const int _motorIndex, const double _freq)
    : ValidationController(_reference),
      filePrefix(_filePrefix),
      motorIndex(_motorIndex),
      frequency(_freq)
  {
    switch (this->motorIndex)
    {
      case 0:
        this->yOffset = -10;
        this->amplitude = 110;
        break;
      case 1:
        this->amplitude = 40;
        break;
      case 2:
        this->yOffset = 35;
        this->amplitude = 30;
        break;
      case 3:
        this->yOffset = 50;
        this->amplitude = 40;
        break;
      case 4:
        this->yOffset = 45;
        this->amplitude = 30;
        break;
      case 5:
        this->yOffset = 35;
        this->amplitude = 25;
        break;
      default:
        std::cout << "Incorrect motor index [" << this->motorIndex << "]"
                  << std::endl;   
        this->amplitude = 0;
    };
    double t = 1.0 / this->frequency;
    this->x = -2.0 * t;
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

    cmd.ref_pos[this->motorIndex] = this->yOffset;

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
      + "_freq" + std::to_string(this->frequency) + ".log";
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

    for (auto i = 0; i < 6; ++i)
      cmd.ref_pos[i] = 0.0;

    cmd.ref_pos[this->motorIndex] = static_cast<float>(
      this->yOffset + this->amplitude * sin(2 * M_PI * this->frequency *
      this->currentT));

    // Send the new joint command and receive the state update.
    if (hx_update(&cmd, &sensor) != hxOK)
    {
      std::cerr << "hx_update(): Request error" << std::endl;
      return;
    }

    // Write time in the log file.
    this->logFile << this->currentT;

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
    this->logFile << " " << this->motorIndex << " " << jointIndex << " "
                  << cmd.ref_pos[this->motorIndex] << " "
                  << sensor.joint_pos[jointIndex] << std::endl;

    usleep(20000);
  }

  //////////////////////////////////////////////////
  bool Running() const
  {
    double period = 1.0 / this->frequency;
    return this->currentT <= 5 * period;
  }

  /// \brief Run counter.
  unsigned int counter = 0;

  /// \brief ToDo.
  int x;

  /// \brief ToDo.
  std::ofstream logFile;

  /// \brief ToDo.
  std::string filePrefix;

  /// \brief ToDo.
  int motorIndex;

  /// \brief ToDo.
  double amplitude;

  /// \brief ToDo.
  double frequency;

  /// \brief ToDo.
  double yOffset = 0;

  /// \brief ToDo.
  double initialT;

  /// \brief ToDo.
  double currentT;
};

//////////////////////////////////////////////////
void usage()
{
  std::cout << "sinusoidal [--reference] <file_prefix> <motor_index>"
            << " <freq>" << std::endl;
  std::cout << "   <motor_index> : [0-5]" << std::endl;
  std::cout << "   <freq> : A positive value (Hz)" << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc < 4 || argc > 5)
  {
    usage();
    return -1;
  }

  if ((argc == 4 && std::string(argv[1]) == "--reference") ||
      (argc == 4 && std::string(argv[2]) == "--reference") ||
      (argc == 4 && std::string(argv[3]) == "--reference"))
  {
    usage();
    return -1;
  }

  std::string prefix;
  int motorIndex;
  bool reference = false;
  double freq;
  if (argc == 4)
  {
    prefix = argv[1];
    motorIndex = std::stoi(argv[2]);
    freq = std::stod(argv[3]);
  }
  else if (argc == 5)
  {
    reference = true;
    if (std::string(argv[1]) == "--reference")
    {
      prefix = argv[2];
      motorIndex = std::stoi(argv[3]); 
      freq = std::stod(argv[4]);
    }
    else if (std::string(argv[2]) == "--reference")
    {
      prefix = argv[1];
      motorIndex = std::stoi(argv[3]);
      freq = std::stod(argv[4]);
    }
    else if (std::string(argv[3]) == "--reference")
    {
      prefix = argv[1];
      motorIndex = std::stoi(argv[2]);
      freq = std::stod(argv[4]);
    }
    else if (std::string(argv[4]) == "--reference")
    {
      prefix = argv[1];
      motorIndex = std::stoi(argv[2]);
      freq = std::stod(argv[3]);
    }
  }

  if (motorIndex < 0 || motorIndex > 5 || freq <= 0)
  {
    usage();
    return -1;
  }

  SinusoidalValidation controller(prefix, reference, motorIndex, freq);
  controller.Start();

  return 0;
}
