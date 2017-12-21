#ifndef ABORT_H
#define ABORT_H

#include "db.h"

#define AbortIfNot(x, ret) \
{ \
    if (!(x)) \
    { \
        dbprintf(__FILE__ ":%d - AbortIfNot()\n", __LINE__); \
        return ret; \
    }\
}\

#define AbortIf(x, ret) \
{ \
    if ((x)) \
    { \
        dbprintf(__FILE__ ":%d - AbortIf()\n", __LINE__); \
        return ret; \
    }\
}\

#endif
