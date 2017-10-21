#ifndef ABORT_H
#define ABORT_H

#define AbortIfNot(x, ret) \
{ \
    if (!(x)) \
    { \
        return ret; \
    }\
}\

#define AbortIf(x, ret) \
{ \
    if ((x)) \
    { \
        return ret; \
    }\
}\

#endif
