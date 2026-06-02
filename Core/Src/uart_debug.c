/*
 * uart_debug.c
 *
 *  Created on: 02.06.2026
 *      Author: benst
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
void uart_printf(const char *format, ...) {
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

/**
 * Spezialisierte Print-Funktion für ADC-Werte
 * Einfach und schnell für häufige Ausgaben
 */
void uart_print_adc(uint32_t left, uint32_t right) {
	uart_printf("%lu, %lu\n", left, right);
}

/**
 * Spezialisierte Print-Funktion für Encoder-Kalibrierung
 */
void uart_print_encoder_calib(uint32_t left, uint32_t right, uint16_t samples) {
	uart_printf("Calib: L=%lu, R=%lu (Samples: %u)\n", left, right, samples);
}

/**
 * Spezialisierte Print-Funktion für Encoder-Zustände
 */
void uart_print_encoder_state(uint32_t left, uint32_t right, uint8_t state_left,
		uint8_t state_right) {
	uart_printf("L=%lu, R=%lu (States: %u, %u)\n", left, right, state_left,
			state_right);
}
