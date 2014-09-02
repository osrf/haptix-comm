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

#ifndef __HAPTIX_COMM_COMM_H
#define __HAPTIX_COMM_COMM_H

#include "haptix/comm/AplControlInterface.h"
#include "haptix/comm/Helpers.h"

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
enum hxResult
{
  hxOK = 0,                               // success
  hxBAD,                                  // a bad thing
  hxHORRIBLE                              // another bad thing
};

/// \brief Communication targets.
enum hxTarget
{
  hxDEKA = 0,                             // DEKA physical arm
  hxMPL,                                  // MPL physical arm
  hxGAZEBO,                               // Gazebo simulator
  hxMUJOCO                                // MuJoCo simulator
};

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
                          hxDeviceInfo* _deviceinfo);

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
                   const hxCommand* _command,
                   hxSensor* _sensor);

//////////////////////////////////////////////////

/// \brief Pointer to the Haptix transport node.
typedef void* HaptixNodePtr;

/// \brief Create a new Haptix transport node.
/// \return A pointer to the new transport node created.
HAPTIX_VISIBLE HaptixNodePtr HaptixNewNode();

/// \brief Create a new Haptix transport node with a namespace.
/// \param[in] _ns All the topic names advertised by this node will be prefixed
/// with the value of '_ns'.
/// \return A pointer to the new transport node created.
HAPTIX_VISIBLE HaptixNodePtr HaptixNewNodeNamespace(const char *_ns);

/// \brief Advertise a new service.
/// \param[in] _node Transport node used to advertise the service.
/// \param[in] _service Name associated to the service.
/// \param[in] _cb Callback to handle the service request with the
/// following parameters:
/// \param[in] _service Requested service name.
/// \param[in] _req Struct containing the request.
/// \param[out] _rep Struct containing the response.
/// \param[out] _result Service call result.
HAPTIX_VISIBLE int HaptixAdvertise(HaptixNodePtr _node,
                                   const char *_service,
                                   void (*_cb)(const char *_service,
  struct AplRobotCommand _req, struct AplRobotState *_rep, int *_result));

/// \brief Request a new service using a blocking call.
/// \param[in] _service Service requested.
/// \param[in] _req Struct containing the request's parameters.
/// \param[in] _timeout The request will timeout after '_timeout' ms.
/// \param[out] _rep Struct containing the response.
/// \param[out] _result Result of the service call.
/// \return 0 when the request was executed or -1 if the timer expired.
HAPTIX_VISIBLE int HaptixRequest(HaptixNodePtr _node,
                                 const char *_service,
                                 struct AplRobotCommand _req,
                                 int _timeout,
                                 struct AplRobotState *_rep,
                                 int *_result);

/// \brief Destroy a Haptix transport node.
/// \param[in] _node Pointer to the node to be destroyed.
HAPTIX_VISIBLE void HaptixDeleteNode(HaptixNodePtr _node);


#ifdef __cplusplus
}
#endif
#endif
