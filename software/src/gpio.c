#include "gpio.h"

#include "abort.h"
#include "types.h"

#include "regs/AxiGpioRegs.h"

/**
 * Initializes a GPIO driver.
 *
 * @param[inout] gpio The driver to initialize.
 * @param base_address The base address of the AXI GPIO module.
 * @param size The number of output bits controlled by the driver.
 *
 * @return Success or fail.
 */
result_t init_gpio(gpio_driver_t *gpio,
                   uint32_t base_address,
                   const uint32_t size)
{
    AbortIfNot(gpio, fail);
    AbortIfNot(base_address, fail);
    AbortIf(size > 64, fail);

    gpio->regs = (AxiGpioRegs *)base_address;
    gpio->size = size;

    return success;
}

/**
 * Sets a GPIO output state.
 *
 * @param gpio The GPIO driver to use.
 * @param pin The pin to change the state of.
 * @param high Specified true to set the output high. Low otherwise.
 *
 * @return Success or fail.
 */
result_t set_gpio(gpio_driver_t *gpio,
                  const uint32_t pin,
                  const bool high)
{
    AbortIfNot(gpio, fail);
    AbortIfNot(gpio->regs, fail);

    AbortIf(pin >= gpio->size, fail);
    AbortIf(pin > 64, fail);

    uint32_t *reg = (pin >= 32)? &gpio->regs->GPIO2_DATA :
                                 &gpio->regs->GPIO_DATA;

    if (high)
    {
        *reg |= 1 << (pin % 32);
    }
    else
    {
        *reg &= ~(1 << (pin % 32));
    }

    return success;
}

/**
 * Reads the state of a GPIO pin.
 *
 * @param gpio The GPIO driver to use.
 * @param pin The pin to read.
 * @param[out] status Location to store read pin status.
 *
 * @return Success or fail.
 */
result_t read_gpio(gpio_driver_t *gpio,
                   const uint32_t pin,
                   bool *status)
{
    AbortIfNot(gpio, fail);
    AbortIfNot(gpio->regs, fail);
    AbortIfNot(status, fail);

    AbortIf(pin >= gpio->size, fail);
    AbortIf(pin > 64, fail);

    const uint32_t reg = (pin >= 32)? gpio->regs->GPIO2_DATA :
                                      gpio->regs->GPIO_DATA;

    *status = (reg & (1 << (pin % 32)))? true : false;

    return success;
}
