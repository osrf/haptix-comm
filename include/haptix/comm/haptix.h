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

/// \file haptix.h
/// \brief Structures and functions for the primary HAPTIX API.

#ifndef __HAPTIX_COMM_HAPTIX_H
#define __HAPTIX_COMM_HAPTIX_H

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Maximum number of motors.
/// Defines the maximum number of motors across any particular device.
/// It is used when allocating sensor and command objects related to the motors.
/// The number of motors for a particular device is defined in _hxDeviceInfo.
#define hxMAXMOTOR              32

/// \brief Maximum number of joints.
/// Defines the maximum number of joints across any particular device.
/// It is used when allocating sensor properties related to the joints.
/// number of joints for a particular device is defined in _hxDeviceInfo.
#define hxMAXJOINT              32

/// \brief Maximum number of contact sensors.
/// Defines the maximum number of contact sensors across any particular device.
/// It is used when allocating an hxSensor.contact object.
/// The number of contact sensors for a particular device is defined in
/// _hxDeviceInfo.
#define hxMAXCONTACTSENSOR      32

/// \brief Maximum number of IMUs.
/// Defines the maximum number of inertial measurement units across any
/// particular device.
/// It is used when allocating sensor objects related to IMUs.
/// The number of IMUs for a particular device is defined in _hxDeviceInfo.
#define hxMAXIMU                32

/// \brief API return codes.
/// These return codes are used to specify the outcome of a haptix-comm
/// function (e.g. success or failure).
typedef enum
{
  /// success
  hxOK = 0,
  /// a bad thing
  hxBAD,
  /// another bad thing
  hxHORRIBLE
} hxResult;

/// \brief Communication targets.
/// This enumeration specifies the possible devices (physical or simulated)
/// that communicate over the HAPTIX API. Specifying the target may be important
/// in case a controller has different behavior across different devices.
typedef enum
{
  /// DEKA physical arm
  hxDEKA = 0,                            
  /// JHU APL MPL physical arm
  hxMPL,
  /// Gazebo simulator
  hxGAZEBO,
  /// MuJoCo simulator
  hxMUJOCO
} hxTarget;

/// \brief Device information.
/// This data structure specifies inherent properties of the device that
/// do not change during simulation (for
/// example, the number of joints in the robot arm).
///
/// It can be retrieved from a communication target by calling
/// hx_getdeviceinfo(int, hxDeviceInfo*).
struct _hxDeviceInfo
{
  /// \brief Number of motors.
  /// Motors are commanded through filling an hxCommand struct and calling
  /// hx_update(int, const hxCommand*, hxSensor*).
  ///
  /// The number of motors is less than or equal to the number of
  /// joints. For example, one motor may control several joints through
  /// kinematic joint coupling.
  int nmotor;

  /// \brief Number of hinge joints.
  /// The joints are passive and are moved as a side effect of commanding
  /// the motors. A joint may correspond directly to the movement of a motor,
  /// or it may be commanded indirectly through joint coupling.
  ///
  /// The number of joints is greater than or equal to the number of
  /// motors.
  int njoint;

  /// \brief Number of contact sensors.
  /// A contact sensor measures the magnitude of the force on that sensor.
  int ncontactsensor;

  /// \brief Number of IMUs (inertial measurement units).
  /// An IMU or inertial measurement unit measures the 3-dimensional linear
  /// acceleration vector and the 3-dimensional angular velocity vector
  /// experienced by the sensor.
  int nIMU;

  /// \brief Minimum and maximum motor angles (rad).
  /// An m by 2 array representing the angular limits of each motor in the
  /// device, where m is the maximum number of motors. Each 1x2 row of the array
  /// corresponds to a motor. The first entry in the row is the lower limit
  /// of the motor. The second entry is the upper limit of the motor.
  float limit[hxMAXMOTOR][2];
};

/// \brief Sensor data.
/// This data structure specifies the sensor information gained in a simulation
/// update.
///
/// It is an output of the function hx_update(int, const hxCommand*, hxSensor*).
struct _hxSensor
{
  /// \brief Motor position (rad).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// _hxDeviceInfo::nmotors-1 contain the angular positions for each motor.
  /// The ordering of these motor values is consistent across the different
  /// motor-related properties of _hxSensor.
  ///
  /// These values cannot exceed the minimum and maximum values specified in
  /// _hxDeviceInfo::limit.
  float motor_pos[hxMAXMOTOR];

  /// \brief Motor velocity (rad/s).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// _hxDeviceInfo::nmotors-1 contain the angular velocity for
  /// each motor. The ordering of
  /// these motor values is consistent across the different motor-related
  /// properties of hxSensor.
  float motor_vel[hxMAXMOTOR];

  /// \brief Torque applied by embedded controller (Nm).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// _hxDeviceInfo::nmotors-1 contain the torque for each motor.
  /// The ordering of
  /// these motor values is consistent across the different motor-related
  /// properties of _hxSensor.
  float motor_torque[hxMAXMOTOR];

  /// \brief Joint position (rad).
  /// An array of floats of size #hxMAXJOINT. Entries 0 through
  /// _hxDeviceInfo::njoint-1 contain the angular position for each joint.
  /// The ordering of these joint values is consistent with
  /// _hxSensor::joint_vel.
  float joint_pos[hxMAXJOINT];

  /// \brief Joint velocity (rad/s).
  /// An array of floats of size #hxMAXJOINT.
  /// Entries 0 through _hxDeviceInfo::njoint-1 contain the angular position
  /// for each joint.
  /// The ordering of these joint values is consistent with
  /// _hxSensor::joint_pos.
  float joint_vel[hxMAXJOINT];

  /// \brief Contact normal force (N).
  /// An array of floats of size #hxMAXCONTACTSENSOR. Entries 0 through
  /// _hxDeviceInfo::ncontactsensor contain the contact magnitude for each
  /// contact sensor.
  float contact[hxMAXCONTACTSENSOR];

  /// \brief 3D linear acceleration (m/s^2).
  /// An array of floats of size #hxMAXIMUx3 where each row is a 3-dimensional
  /// linear acceleration vector. The entries of each row are measured in
  /// meters per second squared and ordered (x, y, z).
  /// Entries 0 through _hxDeviceInfo::nimu-1 contain the acceleration vectors
  /// for each IMU.\n
  /// The ordering of these IMU values is consistent with _hxSensor::IMU_angvel.
  float IMU_linacc[hxMAXIMU][3];

  /// \brief 3D angular velocity (rad/s).
  /// An array of floats of size #hxMAXIMUx3 where each row is a 3-dimensional
  /// angular velocity vector. The entries of each row are measured in
  /// radians per second and ordered (x, y, z).
  /// Entries 0 through _hxDeviceInfo::nimu-1 contain the velocity vectors
  /// for each IMU.
  ///
  /// The ordering of these IMU values is consistent with _hxSensor::IMU_linacc.
  float IMU_angvel[hxMAXIMU][3];
};

/// \brief Motor commands.
struct _hxCommand
{
  /// \brief Timestamp.
  double timestamp;

  /// \brief Reference positions.
  float ref_pos[hxMAXMOTOR];

  /// \brief Reference velocities.
  float ref_vel[hxMAXMOTOR];

  /// \brief Position feedback gains.
  float gain_pos[hxMAXMOTOR];

  /// \brief Velocity feedback gains.
  float gain_vel[hxMAXMOTOR];
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
