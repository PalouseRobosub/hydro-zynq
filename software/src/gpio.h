#ifndef GPIO_H
#define GPIO_H

#include "types.h"
#include "regs/AxiGpioRegs.h"

typedef struct gpio_driver_t
{
    struct AxiGpioRegs * regs;
    uint32_t size;
} gpio_driver_t;

result_t init_gpio(gpio_driver_t *gpio, uint32_t base_address, const uint32_t size);

result_t set_gpio(gpio_driver_t *gpio, const uint32_t pin, const bool high);

result_t read_gpio(gpio_driver_t *gpio, const uint32_t pin, bool *status);

#endif // GPIO_H
