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

#include "haptix/comm/Api.h"
#include "haptix/comm/Helpers.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Pointer to the transport node.
typedef void* NodePtr;

/// \brief Create a new transport node.
/// \return A pointer to the new transport node created.
HAPTIX_VISIBLE NodePtr newNode();

/// \brief Advertise a new service.
/// \param[in] _node Transport node used to advertise the service.
/// \param[in] _service Name associated to the service.
/// \param[in] _cb Callback to handle the service request with the
/// following parameters:
/// \param[in] _service Requested service name.
/// \param[in] _req Struct containing the request.
/// \param[out] _rep Struct containing the response.
/// \param[out] _result Service call result.
HAPTIX_VISIBLE int nodeAdvertise(NodePtr _node, const char *_service,
  void (*_cb)(const char *_service, Arm_t _req, Arm_t *_rep, int *_result));

/// \brief Request a new service using a blocking call.
/// \param[in] _service Service requested.
/// \param[in] _req Struct containing the request's parameters.
/// \param[in] _timeout The request will timeout after '_timeout' ms.
/// \param[out] _rep Struct containing the response.
/// \param[out] _result Result of the service call.
/// \return 0 when the request was executed or -1 if the timer expired.
HAPTIX_VISIBLE int nodeRequest(NodePtr _node, const char *_service, Arm_t _req,
  int _timeout, Arm_t *_rep, int *_result);

/// \brief Destroy a transport node.
/// \param[in] _node Pointer to the node to be destroyed.
HAPTIX_VISIBLE void deleteNode(NodePtr _node);

#ifdef __cplusplus
}
#endif
#endif
