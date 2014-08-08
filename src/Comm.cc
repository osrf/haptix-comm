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

#include <iostream>
#include <ignition/transport.hh>
#include "haptix/comm/Comm.h"

extern "C" {
  NodePtr newNode()
  {
    std::cout << "Node constructor" << std::endl;
    return reinterpret_cast<void*>(new ignition::transport::Node());
  }

  void nodeSet(NodePtr /*_n*/, int /*_i*/)
  {
    std::cout << "NodeSet" << std::endl;
  }

  int nodeGet(NodePtr /*_n*/)
  {
    std::cout << "NodeGet" << std::endl;
    return 0;
  }

  void deleteNode(NodePtr /*_n*/)
  {
    std::cout << "Node destructor" << std::endl;
  }
}
