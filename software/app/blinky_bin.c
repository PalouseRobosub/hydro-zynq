#include "abort.h"
#include "time_util.h"
#include "system.h"
#include "uart.h"
#include "inttypes.h"

#include "regs/gpio_regs.h"

void delay(uint32_t milliseconds)
{
    tick_t start = get_system_time();
    tick_t done = start + ms_to_ticks(milliseconds);
    while (get_system_time() < done);
}

int main()
{
    print("Init system\n");
    AbortIfNot(init_system(), fail);

    while (1)
    {
        tick_t now = get_system_time();
        print("Displaying LED at %f s, %d ms (%"PRId64")\n", ticks_to_seconds(now), ticks_to_ms(now), now);

        /*
         * Write the on-board LED high.
         */
        set_board_led(true);

        delay(500);

        /*
         * Write the on-board LED low.
         */
        set_board_led(false);
        delay(500);
    }
}
