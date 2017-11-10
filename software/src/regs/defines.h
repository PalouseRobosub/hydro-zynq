#ifndef REG_DEFINES_H
#define REG_DEFINES_H

#define TOKEN_PASTE2(x, y) x ## y
#define TOKEN_PASTE(x, y) TOKEN_PASTE2(x, y)

#define RESERVE(type, cnt) type TOKEN_PASTE(unused, __LINE__)[cnt]

#endif
