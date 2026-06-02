/*
 * encoder_calib.h
 *
 *  Created on: 02.06.2026
 *      Author: benst
 */

#ifndef ENCODER_CALIB_H
#define ENCODER_CALIB_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "uart_debug.h"

/**
 * Encoder Calibration Module
 *
 * Funktionalität:
 * - Nyquist-konforme Abtastrate (für 24 Flanken/Umdrehung bei ~10 U/s => 480 Hz mindestens)
 * - Spike-Schutz durch Moving Average + Median
 * - Komplette Rad-Abtastung für imperfekte weiße Flächen
 * - Robuste Schwellwertberechnung
 */

#define ENCODER_CALIB_NUM_SAMPLES    240  // Für 1 komplette Umdrehung bei sicherer Abtastrate
#define ENCODER_CALIB_MOVING_AVG_SIZE 5   // 5er Moving Average
#define ENCODER_CALIB_SPIKE_THRESHOLD 1000 // ADC-Sprung > 1000 = Spike (ADC 12-bit: 0-4095)

typedef enum {
	ENCODER_CALIB_IDLE, ENCODER_CALIB_RUNNING, ENCODER_CALIB_COMPLETE
} EncoderCalibState_t;

typedef struct {
	// Raw Samples
	uint16_t samples[ENCODER_CALIB_NUM_SAMPLES];
	uint16_t sample_count;

	// Moving Average Buffer
	uint16_t moving_avg_buffer[ENCODER_CALIB_MOVING_AVG_SIZE];
	uint8_t moving_avg_index;

	// Last Value für Spike-Erkennung
	uint16_t last_value;

	// Kalibrierungsergebnisse
	uint16_t min_value;
	uint16_t max_value;
	uint16_t threshold_low;
	uint16_t threshold_high;
	uint16_t median_value;

	// Betriebszustand
	uint8_t current_state;     // 0 = LOW (schwarz), 1 = HIGH (weiß)
	EncoderCalibState_t calib_state;

} EncoderCalib_t;

/**
 * Vergleichsfunktion für qsort
 */
//static int compare_uint16(const void *a, const void *b);

/**
 * Initialisierung
 */
void EncoderCalib_Init(EncoderCalib_t *enc);

/**
 * Kalibrierung starten (löscht alte Samples)
 */
void EncoderCalib_StartCalibration(EncoderCalib_t *enc);

/**
 * Neue Messwert während Kalibrierung hinzufügen
 * Entfernt Spikes automatisch via Moving Average
 * Gibt true zurück wenn Kalibrierung abgeschlossen
 */
bool EncoderCalib_AddSample(EncoderCalib_t *enc, uint16_t adc_value);

/**
 * Kalibrierung abschließen und Schwellwerte berechnen
 * threshold_margin: Sicherheitsspanne in % vom Wertebereich (z.B. 0.1 = 10%)
 */
void EncoderCalib_Finalize(EncoderCalib_t *enc, float threshold_margin);

/**
 * Zustand aktualisieren basierend auf aktuellem ADC-Wert
 * Schmitt-Trigger Hysterese ist bereits in threshold_low/high eingebaut
 */
void EncoderCalib_UpdateState(EncoderCalib_t *enc, uint16_t adc_value);

/**
 * Debug: Schwellwerte und Zustand ausgeben
 */
void EncoderCalib_PrintCalibData(EncoderCalib_t *enc);

/**
 * Helper: Gibt den aktuellen Zustand zurück (0=LOW, 1=HIGH)
 */
uint8_t EncoderCalib_GetState(EncoderCalib_t *enc);

#endif
