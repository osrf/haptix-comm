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

#include "haptix/comm/Helpers.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void* NodePtr;

HAPTIX_VISIBLE NodePtr newNode();

HAPTIX_VISIBLE int nodeAdvertise(NodePtr n);

HAPTIX_VISIBLE int nodeRequest(NodePtr _node, char *_service, double _posReq,
  double _velReq, float _timeout, double *_posRes, double *_velRes,
  int _result);

HAPTIX_VISIBLE void deleteNode(NodePtr n);

#ifdef __cplusplus
}
#endif
#endif
