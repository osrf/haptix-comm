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

#include "haptix/comm/types.h"
#include "haptix/comm/Helpers.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Connect to specified device/simulator target.
/// Multile calls to this function are allowed with different targets.
/// \param[in] _target Device to be connected.
/// \return 'hxOK' if the connection succeed or an error code otherwise.
HAPTIX_VISIBLE hxResult hx_connect(int _target);

/// \brief Close connection to specified device/simulator target.
/// \param[in] _target Device to be disconnected.
/// \return 'hxOK' if the disconnection succeed or an error code otherwise.
HAPTIX_VISIBLE hxResult hx_close(int _target);

/// \brief Get info for specified device/simulator target.
/// \param[in] _target Requested device.
/// \param[out] _deviceInfo Device information requested.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
HAPTIX_VISIBLE hxResult hx_getdeviceinfo(int _target,
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
HAPTIX_VISIBLE hxResult hx_update(int _target,
                                  const hxCommand* _command,
                                  hxSensor* _sensor);

#ifdef __cplusplus
}
#endif
#endif
