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
/// \file haptix_sim.h
/// \brief Structures and functions for simulation specific functionality.
/// This header is not needed for use with Gazebo but added for code
/// compatibility with other simulators.

#ifndef __HAPTIX_COMM_HAPTIX_SIM_H
#define __HAPTIX_COMM_HAPTIX_SIM_H

#include "haptix/comm/haptix.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Maximum number of contacts.
#define hxsMAXCONTACT            30

/// \brief Maximum number of bodies.
#define hxsMAXBODY               30

/// \brief Simulated device information, beyond the information provided in
/// hxDeviceInfo.
struct _hxsDeviceInfo
{
  /// \brief Number of dynamic objects.
  int ndynamic;

  /// \brief Number of static objects.
  int nstatic;

  // Anything else we should provide here?
};

/// \brief Information about bodies.
struct _hxsBody
{
  /// \brief Position.
  float pos[hxsMAXBODY][3];

  /// \brief Orientation (unit quaternion).
  float quat[hxsMAXBODY][4];

  /// \brief Linear velocity.
  float linvel[hxsMAXBODY][3];

  /// \brief Angular velocity (expmap representation).
  float angvel[hxsMAXBODY][3];

  /// \brief Linear acceleration.
  float linacc[hxsMAXBODY][3];

  /// \brief Angular acceleration.
  float angacc[hxsMAXBODY][3];
};

/// \brief Information about joints.
struct _hxsJoint
{
  /// \brief Position.
  float pos[hxMAXJOINT];

  /// \brief Velocity.
  float vel[hxMAXJOINT];

  /// \brief Acceleration.
  float acc[hxMAXJOINT];

  /// \brief Torque due to actuation.
  float torque_motor[hxMAXJOINT];

  /// \brief Torque due to limits, damping, friction.
  float torque_passive[hxMAXJOINT];
};

/// \brief Information about contacts.
struct _hxsContact
{
  /// \brief Number of currently active contacts.
  int ncontact;

  /// Contact descriptors
  /// \brief contacting body 1.
  int body1[hxsMAXCONTACT];

  /// \brief Contacting body 2.
  int body2[hxsMAXCONTACT];


  /// Contact frame relative to global frame.
  /// \brief Contact point.
  float point[hxsMAXCONTACT][3];

  /// \brief Contact normal (unit vector).
  float normal[hxsMAXCONTACT][3];

  /// \brief First tangent (unit vector).
  float tangent1[hxsMAXCONTACT][3];

  /// \brief Second tangent (unit vector).
  float tangent2[hxsMAXCONTACT][3];


  /// Data in contact frame, with axis order (normal, tangent1, tangent2).
  /// \brief Normal distance.
  float distance[hxsMAXCONTACT];

  /// \brief Relative velocity.
  float velocity[hxsMAXCONTACT][3];

  /// \brief Contact force.
  float force[hxsMAXCONTACT][3];
};

/// \def hxsDeviceInfo
/// \brief Simulated device information.
typedef struct _hxsDeviceInfo hxsDeviceInfo;

/// \def hxsBody
/// \brief Information about bodies.
typedef struct _hxsBody hxsBody;

/// \def hxsJoint
/// \brief Information about joints.
typedef struct _hxsJoint hxsJoint;

/// \def hxsContact
/// \brief Information about contacts.
typedef struct _hxsContact hxsContact;

/// \brief Get simulation information.
/// \param[in] _target Requested device.
/// \param[out] _deviceinfo Device information requested.
/// \return 'hxOK' if the disconnection succeed or an error code otherwise.
hxResult hxs_getdeviceinfo(int _target,
                           hxsDeviceInfo *_deviceinfo);

/// \brief Get information about bodies.
/// \param[in] _target Requested device.
/// \param[out] _body Body information requested.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hxs_getbody(int _target,
                     hxsBody *_body);

/// \brief Get information about joints.
/// \param[in] _target Requested device.
/// \param[out] _joint Joint information requested.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hxs_getjoint(int _target,
                      hxsJoint *_joint);

/// \brief Get information about active contacts.
/// \param[in] _target Requested device.
/// \param[out] _contact Contact information requested.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hxs_getcontact(int _target,
                        hxsContact *_contact);

/// \brief Get Jacobian of global point attached to robot link (index between 1
/// and njoint-1) size of Jacobian matrix is 3-by-njoint, in row-major format.
/// \param[in] _target Requested device.
/// \param[in] _link Requested link index.
/// \param[in] _point Point information requested.
/// \param[out] _jacobian Jacobian information requested.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hxs_getjacobian(int _target,
                         int _link,
                         const float *_point,
                         float *_jacobian);

/// \brief Set simulation state (position and velocity) as follows:
/// the robot base and objects are set from hxBody.
/// the robot links are set from hxJoint via forward kinematics.
/// the robot link data in hxBody, and all acceleration and torque data are
/// ignored.
/// \param[in] _target Requested device.
/// \param[in] _body New body value.
/// \param[in] _joint New joint value.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hxs_setstate(int _target,
                      const hxsBody *_body,
                      const hxsJoint *_joint);

/// \brief Synchronous update with direct torque control:
///   1. set joint torques ignoring actuator model.
///   2. advance simulation state, sleep for remainder of update step only if
///      requested.
///   3. return simulated sensor data.
/// \param[in] _target Requested device.
/// \param[in] _torque New torque value.
/// \param[out] _sensor Sensor data received after the update.
/// \param[in] _flg_sleep Advance simulation step this time.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hxs_updatedirect(int _target,
                          const float *_torque,
                          hxSensor *_sensor,
                          int _flg_sleep);

#ifdef __cplusplus
}
#endif
#endif
