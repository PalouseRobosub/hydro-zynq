#include "abort.h"
#include "time_util.h"
#include "system.h"
#include "uart.h"

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

    if (&gpio_regs->DATA[1] != 0xe000a044)
    {
        print("Bad DATA1 addr\n");
    }

    if (&gpio_regs->DIRM_0 != 0xe000a204)
    {
        print("Bad DIRM0 addr\n");
    }

    if (&gpio_regs->OEN_1 != 0xe000a248)
    {
        print("Bad DIRM0 addr\n");
    }

    while (1)
    {
        /*
         * Write the on-board LED high.
         */
        print("Enabling LED\n");
        set_board_led(true);

        print("Waiting 500ms\n");
        delay(500);

        /*
         * Write the on-board LED low.
         */
        print("Disabling LED\n");
        set_board_led(false);
        print("Waiting 500ms\n");
        delay(500);
    }
}
