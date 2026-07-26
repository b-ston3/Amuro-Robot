/*
 * uart_debug.c
 *
 *  Created on: Jun 3, 2026
 *      Author: ben
 */

#include "uart_debug.h"
#include "main.h"

// Shared UART handle and print buffer defined in main.c.
extern UART_HandleTypeDef huart2;
extern char string_buf[100];

/**
 * Variadic debug print helper that sends printf-style output over UART.
 */
void ser_printf(const char *format, ...) {
	va_list args;
	va_start(args, format);

	// Format message into the shared debug buffer.
	uint8_t len = vsnprintf(string_buf, UART_DEBUG_BUFFER_SIZE, format, args);

	va_end(args);

	// Transmit message over UART2.
	if (len > 0) {
		HAL_UART_Transmit(&huart2, (uint8_t*) string_buf, len, HAL_MAX_DELAY);
	}
}
