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

#ifndef __HAPTIX_COMM_TYPES_H
#define __HAPTIX_COMM_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Maximum number of motors.
#define hxMAXMOTOR              15

/// \brief Maximum number of joints.
#define hxMAXJOINT              25

/// \brief Maximum number of contact sensors.
#define hxMAXCONTACTSENSOR      10

/// \brief Maximum number of IMUs.
#define hxMAXIMU                10

/// \brief API return codes.
typedef enum
{
  hxOK = 0,                               // success
  hxBAD,                                  // a bad thing
  hxHORRIBLE                              // another bad thing
} hxResult;

/// \brief Communication targets.
typedef enum
{
  hxDEKA = 0,                             // DEKA physical arm
  hxMPL,                                  // MPL physical arm
  hxGAZEBO,                               // Gazebo simulator
  hxMUJOCO                                // MuJoCo simulator
} hxTarget;

/// \brief Device information.
struct _hxDeviceInfo
{
  /// \brief Number of motors.
  int nmotor;

  /// \brief Number of hinge joints.
  int njoint;

  /// \brief Number of contact sensors.
  int ncontactsensor;

  /// \brief Number of IMUs.
  int nIMU;

  /// \brief Minimum and maximum joint angles (rad).
  float limit[hxMAXJOINT][2];

  // Anything else we should provide here, as opposed to letting the user
  // extract the data they need from the XML model or robot specs?
};

/// \brief Sensor data.
struct _hxSensor
{
  /// \brief Motor position (rad).
  float motor_pos[hxMAXMOTOR];

  /// \brief Motor velocity (rad/s).
  float motor_vel[hxMAXMOTOR];

  /// \brief Torque applied by embedded controller (Nm).
  float motor_torque[hxMAXMOTOR];

  /// \brief Joint position (rad).
  float joint_pos[hxMAXJOINT];

  /// \brief Joint velocity (rad/s).
  float joint_vel[hxMAXJOINT];

  /// \brief Contact normal force (N).
  float contact[hxMAXCONTACTSENSOR];

  /// \brief 3D linear acceleration (m/s^2).
  float IMU_linacc[hxMAXIMU][3];

  /// \brief 3D angular velocity (rad/s).
  float IMU_angvel[hxMAXIMU][3];
};

/// \brief Motor commands.
struct _hxCommand
{
  /// \brief Reference positions.
  float ref_pos[hxMAXMOTOR];

  /// \brief Reference velocities.
  float ref_vel[hxMAXMOTOR];

  /// \brief Position feedback gains.
  float gain_pos[hxMAXMOTOR];

  /// \brief Velocity feedback gains.
  float gain_vel[hxMAXMOTOR];

  // Do the robots accept any other commands?
};

/// \def hxDeviceInfo
/// \brief Robot information.
typedef struct _hxDeviceInfo hxDeviceInfo;

/// \def hxSensor
/// \brief Sensor data.
typedef struct _hxSensor hxSensor;

/// \def hxCommand
/// \brief Motor commands.
typedef struct _hxCommand hxCommand;

#ifdef __cplusplus
}
#endif
#endif
