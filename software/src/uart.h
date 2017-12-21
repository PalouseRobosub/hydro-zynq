#ifndef UART_H
#define UART_H

#include "types.h"

void print_string(char *str);

void uprintf(const char fmt[], ...);

void uart_putchar(char c);

#endif
