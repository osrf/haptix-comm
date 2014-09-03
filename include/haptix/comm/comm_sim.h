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

#ifndef __HAPTIX_COMM_COMM_SIM_H
#define __HAPTIX_COMM_COMM_SIM_H

#include "haptix/comm/comm.h"
#include "haptix/comm/types_sim.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Get simulation information.
/// \param[in] _target Requested device.
/// \param[out] _deviceinfo Device information requested.
/// \return 'hxOK' if the disconnection succeed or an error code otherwise.
hxResult hxs_getdeviceinfo(int _target,
                           hxsDeviceInfo* _deviceinfo);

/// \brief Get information about bodies.
/// \param[in] _target Requested device.
/// \param[out] _body Body information requested.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hxs_getbody(int _target,
                     hxsBody* _body);

/// \brief Get information about joints.
/// \param[in] _target Requested device.
/// \param[out] _joint Joint information requested.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hxs_getjoint(int _target,
                      hxsJoint* _joint);

/// \brief Get information about active contacts.
/// \param[in] _target Requested device.
/// \param[out] _contact Contact information requested.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hxs_getcontact(int _target,
                        hxsContact* _contact);

/// \brief Get Jacobian of global point attached to robot link (index between 1
/// and njoint-1) size of Jacobian matrix is 3-by-njoint, in row-major format.
/// \param[in] _target Requested device.
/// \param[in] _link Requested link index.
/// \param[in] _point Point information requested.
/// \param[out] _jacobian Jacobian information requested.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hxs_getjacobian(int _target,
                         int _link,
                         const float* _point,
                         float* _jacobian);

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
                      const hxsBody* _body,
                      const hxsJoint* _joint);

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
                          const float* _torque,
                          hxSensor* _sensor,
                          int _flg_sleep);

#ifdef __cplusplus
}
#endif
#endif
