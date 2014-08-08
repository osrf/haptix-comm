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
