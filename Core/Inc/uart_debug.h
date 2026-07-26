/*
 * uart_debug.h
 *
 *  Created on: Jun 3, 2026
 *      Author: ben
 */

#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define UART_DEBUG_BUFFER_SIZE 256

/**
 * UART debug print helper (variadic, printf-style).
 *
 * Examples:
 *   ser_printf("Hello %s\n", "world");
 *   ser_printf("ADC values: L=%lu, R=%lu\n", adc[1], adc[4]);
 *   ser_printf("Calib: L=%u, R=%u (samples: %u)\n", adc[1], adc[4], samples);
 */
void ser_printf(const char *format, ...);

#endif /* UART_DEBUG_H */
