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
#include "haptix/comm/Arm.pb.h"
#include "haptix/comm/Comm.h"

extern "C" {
  //////////////////////////////////////////////////
  /// \brief Provide an "echo" service.
  void srvEcho(const std::string &/*_topic*/, const haptix::comm::Arm &_req,
    haptix::comm::Arm &_rep, bool &_result)
  {
    // Set the response's content.
    _rep.set_pos(_req.pos());
    _rep.set_vel(_req.vel());

    std::cout << "Service received" << std::endl;
    std::cout << "Pos: " << _req.pos() << std::endl;
    std::cout << "Vel: " << _req.vel() << std::endl;

    // The response succeed.
    _result = true;
  }

  NodePtr newNode()
  {
    std::cout << "Node constructor" << std::endl;
    return reinterpret_cast<void*>(new ignition::transport::Node());
  }

  void nodeSet(NodePtr /*_n*/, int /*_i*/)
  {
    std::cout << "NodeSet" << std::endl;
  }

  int nodeAdvertise(NodePtr _node)
  {
    // Advertise a service call.
    ignition::transport::Node *node =
      reinterpret_cast<ignition::transport::Node*>(_node);

    node->Advertise("/echo", srvEcho);
    std::cout << "Advertise" << std::endl;
    return 0;
  }

  int nodeRequest(NodePtr _node, char *_service, double _posReq,
  double _velReq, float _timeout, double *_posRes, double *_velRes, int _result)
  {
    std::cout << "nodeRequest" << std::endl;

    haptix::comm::Arm armReq, armRep;
    armReq.set_pos(_posReq);
    armReq.set_vel(_velReq);
    bool result;

    ignition::transport::Node *node =
      reinterpret_cast<ignition::transport::Node*>(_node);
    // Request the service.
    bool executed = node->Request("/echo", armReq, _timeout, armRep, result);
    *_posRes = armRep.pos();
    *_velRes = armRep.vel();

    std::cout << "Pos: " << armRep.pos() << std::endl;
    std::cout << "Vel: " << armRep.vel() << std::endl;

    return 0;
  }

  void deleteNode(NodePtr /*_n*/)
  {
    std::cout << "Node destructor" << std::endl;
  }
}
