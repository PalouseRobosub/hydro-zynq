#ifndef SYSTEM_H
#define SYSTEM_H

#include "types.h"

result_t init_system();

void set_board_led(bool enabled);

tick_t get_system_time();

#endif
