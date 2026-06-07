/*
 * uart_debug.c
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
 * Debug Print Funktion (variadisch - wie printf)
 *
 * Beispiele:
 *   uart_printf("Hallo %s\n", "Welt");
 *   uart_printf("ADC Werte: L=%lu, R=%lu\n", adc[1], adc[4]);
 *   uart_printf("Calib: L=%u, R=%u (Samples: %u)\n", adc[1], adc[4], samples);
 */
void ser_printf(const char *format, ...);

#endif /* INC_UART_DEBUG_C_ */
