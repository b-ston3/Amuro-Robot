/*
 * uart_debug.c
 *
 *  Created on: Jun 3, 2026
 *      Author: ben
 */

#include "uart_debug.h"
#include "main.h"

// Externes UART Handle (aus main.c)
extern UART_HandleTypeDef huart2;
extern char string_buf[100];

/**
 * Variadische Debug Print Funktion (wie printf, aber über UART)
 *
 * Diese Funktion arbeitet wie printf(), aber sendet das Ergebnis
 * über UART statt auf die Console.
 */
void ser_printf(const char *format, ...) {
	va_list args;
	va_start(args, format);

	// Formatiere String in Buffer
	uint8_t len = vsnprintf(string_buf, UART_DEBUG_BUFFER_SIZE, format, args);

	va_end(args);

	// Sende über UART
	if (len > 0) {
		HAL_UART_Transmit(&huart2, (uint8_t*) string_buf, len, HAL_MAX_DELAY);
	}
}
