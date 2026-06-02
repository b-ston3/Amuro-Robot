/*
 * uart_debug.h
 *
 *  Created on: 02.06.2026
 *      Author: benst
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
void uart_printf(const char* format, ...);

/**
 * Einfache Debug Print mit fest definiertem Format
 * Schneller, wenn man die gleiche Ausgabe öfter braucht
 *
 * Beispiele:
 *   uart_print_adc(adc[1], adc[4]);
 *   uart_print_encoder_calib(adc[1], adc[4], encoder_left.sample_count);
 */
void uart_print_adc(uint32_t left, uint32_t right);
void uart_print_encoder_calib(uint32_t left, uint32_t right, uint16_t samples);
void uart_print_encoder_state(uint32_t left, uint32_t right, uint8_t state_left, uint8_t state_right);

#endif
