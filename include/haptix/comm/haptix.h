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

#ifndef __HAPTIX_COMM_HAPTIX_H
#define __HAPTIX_COMM_HAPTIX_H

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Maximum number of motors.
// #define hxMAXMOTOR              15

/// \brief Maximum number of joints.
//  Should be equeal to hxAPLJoints::num_joints
// #define hxMAXJOINT              24

/// \brief Maximum number of contact sensors.
#define hxMAXCONTACTSENSOR      10

/// \brief Maximum number of IMUs.
#define hxMAXIMU                10

/// \brief APL Joint enums.
typedef enum
{
  wrist_rot = 0,
  wrist_dev,
  wrist_fe,
  index_dip,
  index_mcp,
  index_pip,
  little_dip,
  little_mcp,
  little_pip,
  middle_dip,
  middle_mcp,
  middle_pip,
  thumb_cmc_ab_ad,
  little_ab_ad,
  ring_ab_ad,
  middle_ab_ad,
  index_ab_ad,
  thumb_cmc_fe,
  ring_dip,
  ring_mcp,
  ring_pip,
  thumb_mcp,
  thumb_dip,
  num_joints
} hxAPLJoints;

/// \brief APL motor enums.
typedef enum
{
  motor_wrist_rot = 0,
  motor_wrist_dev,
  motor_wrist_fe,
  motor_index_dip,
  motor_index_mcp,
  motor_index_pip,
  motor_little_dip,
  motor_little_mcp,
  motor_little_pip,
  motor_middle_dip,
  motor_middle_mcp,
  motor_middle_pip,
  motor_thumb_cmc_ab_ad,
  motor_little_ab_ad,
  motor_ring_ab_ad,
  motor_middle_ab_ad,
  motor_index_ab_ad,
  motor_thumb_cmc_fe,
  motor_ring_dip,
  motor_ring_mcp,
  motor_ring_pip,
  motor_thumb_mcp,
  motor_thumb_dip,
  num_motors
} hxAPLMotors;

/// \brief APL contact sensor enums.
typedef enum
{
  contact_index = 0,
  contact_little,
  contact_middle,
  contact_thumb,
  contact_ring,
  num_contact_sensors
} hxAPLContactSensors;

/// \brief APL IMU enums.
typedef enum
{
  imu_index = 0,
  imu_little,
  imu_middle,
  imu_thumb,
  imu_ring,
  num_imu_sensors
} hxAPLImuSensors;

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
  float limit[num_joints][2];

  // Anything else we should provide here, as opposed to letting the user
  // extract the data they need from the XML model or robot specs?
};

/// \brief Sensor data.
struct _hxSensor
{
  /// \brief Timestamp.
  double timestamp;

  /// \brief Motor position (rad).
  float motor_pos[num_motors];

  /// \brief Motor velocity (rad/s).
  float motor_vel[num_motors];

  /// \brief Torque applied by embedded controller (Nm).
  float motor_torque[num_motors];

  /// \brief Joint position (rad).
  float joint_pos[num_joints];

  /// \brief Joint velocity (rad/s).
  float joint_vel[num_joints];

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
  /// \brief Timestamp.
  double timestamp;

  /// \brief Reference positions.
  float ref_pos[num_motors];

  /// \brief Reference velocities.
  float ref_vel[num_motors];

  /// \brief Position feedback gains.
  float gain_pos[num_motors];

  /// \brief Velocity feedback gains.
  float gain_vel[num_motors];

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

/// \brief Connect to specified device/simulator target.
/// Multile calls to this function are allowed with different targets.
/// \param[in] _target Device to be connected.
/// \return 'hxOK' if the connection succeed or an error code otherwise.
hxResult hx_connect(int _target);

/// \brief Close connection to specified device/simulator target.
/// \param[in] _target Device to be disconnected.
/// \return 'hxOK' if the disconnection succeed or an error code otherwise.
hxResult hx_close(int _target);

/// \brief Get info for specified device/simulator target.
/// \param[in] _target Requested device.
/// \param[out] _deviceInfo Device information requested.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hx_getdeviceinfo(int _target,
                          hxDeviceInfo *_deviceinfo);

/// \brief Synchronous update at the rate supported by the device:
///   1. set motor command.
///   2. Advance simulation state and sleep for remainder of update step,
///      or wait for physical device to finish update.
///   3. Return simulated or physical sensor data.
/// \param[in] _target Device to update.
/// \param[in] _command New command to be sent.
/// \param[out] _sensor Sensor data received after the update.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hx_update(int _target,
                   const hxCommand *_command,
                   hxSensor *_sensor);

#ifdef __cplusplus
}
#endif
#endif
