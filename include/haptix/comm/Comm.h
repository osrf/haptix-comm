#ifndef __MYWRAPPER_H
#define __MYWRAPPER_H

#include "haptix/comm/Helpers.hh"

#ifdef __cplusplus
extern "C" {
#endif

typedef void* NodePtr;

HAPTIX_VISIBLE NodePtr newNode();

HAPTIX_VISIBLE void nodeSet(NodePtr n, int i);

HAPTIX_VISIBLE int nodeGet(NodePtr n);

HAPTIX_VISIBLE void deleteNode(NodePtr n);

#ifdef __cplusplus
}
#endif
#endif