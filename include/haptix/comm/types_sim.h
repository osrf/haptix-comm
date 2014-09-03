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

#ifndef __HAPTIX_COMM_TYPES_SIM_H
#define __HAPTIX_COMM_TYPES_SIM_H

#include "haptix/comm/comm.h"

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

#ifdef __cplusplus
}
#endif
#endif
