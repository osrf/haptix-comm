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

/// \brief Pointer to the Haptix transport node.
typedef void* HaptixNodePtr;

/// \brief Create a new Haptix transport node.
/// \return A pointer to the new transport node created.
HAPTIX_VISIBLE HaptixNodePtr HaptixNewNode();

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
