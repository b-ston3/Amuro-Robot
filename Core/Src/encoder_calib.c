/*
 * encoder_calib.c
 *
 *  Created on: 02.06.2026
 *      Author: benst
 */

#include "encoder_calib.h"

/**
 * Vergleichsfunktion für qsort
 */
static int encoder_calib_compare_uint16(const void *a, const void *b) {
	uint16_t val_a = *(const uint16_t*) a;
	uint16_t val_b = *(const uint16_t*) b;

	if (val_a < val_b)
		return -1;
	if (val_a > val_b)
		return 1;
	return 0;
}

/**
 * Hilfsfunktion: Median aus Samples berechnen (wird in Finalize verwendet)
 */
static uint16_t encoder_calib_median(uint16_t *data, uint16_t count) {
	// Einfache Median-Berechnung via Bubble Sort
	// Für kleine Arrays (240 Elemente) ist das OK
	uint16_t sorted[ENCODER_CALIB_NUM_SAMPLES];
	memcpy(sorted, data, count * sizeof(uint16_t));

	qsort(sorted, count, sizeof(uint16_t), encoder_calib_compare_uint16);

	// Mittelelement (oder Durchschnitt der beiden mittleren bei gerader Anzahl)
	if (count % 2 == 1) {
		return sorted[count / 2];
	} else {
		return (sorted[count / 2 - 1] + sorted[count / 2]) / 2;
	}
}

/**
 * Hilfsfunktion: Moving Average berechnen
 */
static uint16_t encoder_calib_moving_average(EncoderCalib_t *enc) {
	uint32_t sum = 0;
	for (int i = 0; i < ENCODER_CALIB_MOVING_AVG_SIZE; i++) {
		sum += enc->moving_avg_buffer[i];
	}
	return (uint16_t) (sum / ENCODER_CALIB_MOVING_AVG_SIZE);
}

/**
 * Spike-Erkennung für starke Abweichungen
 */
static bool encoder_calib_is_valid_sample(EncoderCalib_t *enc,
		uint16_t new_value) {
	if (enc->sample_count == 0) {
		// Erstes Sample akzeptieren
		return true;
	}

	// Prüfe auf großen Sprung (Spike)
	int32_t diff = (int32_t) new_value - (int32_t) enc->last_value;
	if (diff < 0)
		diff = -diff;

	if (diff > ENCODER_CALIB_SPIKE_THRESHOLD) {
		// Spike erkannt, Wert ignorieren
		return false;
	}

	return true;
}

/**
 * Moving Average Filter zur Glättung
 */
static uint16_t encoder_calib_filter_value(EncoderCalib_t *enc,
		uint16_t raw_value) {
	// Füge in Ringbuffer ein
	enc->moving_avg_buffer[enc->moving_avg_index] = raw_value;
	enc->moving_avg_index = (enc->moving_avg_index + 1)
			% ENCODER_CALIB_MOVING_AVG_SIZE;

	// Gib gefilterten Wert zurück
	return encoder_calib_moving_average(enc);
}

/**
 * Initialisierung
 */
void EncoderCalib_Init(EncoderCalib_t *enc) {
	memset(enc, 0, sizeof(EncoderCalib_t));
	enc->calib_state = ENCODER_CALIB_IDLE;
	enc->current_state = 0;
	enc->last_value = 2048; // Mittlerer ADC-Wert als Startwert (12-bit: 0-4095)
	enc->min_value = 4095;
	enc->max_value = 0;

	// Moving Average Buffer mit Mittelwert initialisieren
	for (int i = 0; i < ENCODER_CALIB_MOVING_AVG_SIZE; i++) {
		enc->moving_avg_buffer[i] = 2048;
	}
}

/**
 * Kalibrierung starten
 */
void EncoderCalib_StartCalibration(EncoderCalib_t *enc) {
	enc->calib_state = ENCODER_CALIB_RUNNING;
	enc->sample_count = 0;
	enc->min_value = 4095;
	enc->max_value = 0;
	enc->last_value = 2048;
	enc->moving_avg_index = 0;

	// Moving Average Buffer zurücksetzen
	for (int i = 0; i < ENCODER_CALIB_MOVING_AVG_SIZE; i++) {
		enc->moving_avg_buffer[i] = 2048;
	}
}

/**
 * Neue Messwert während Kalibrierung hinzufügen
 *
 * Ablauf:
 * 1. Spike-Erkennung
 * 2. Moving Average Filterung
 * 3. Min/Max Tracking
 * 4. Speicherung
 * 5. Rückgabe: true wenn genug Samples
 */
bool EncoderCalib_AddSample(EncoderCalib_t *enc, uint16_t adc_value) {
	// Nur während Kalibrierung
	if (enc->calib_state != ENCODER_CALIB_RUNNING) {
		return false;
	}

	// Prüfe auf Spike
	if (!encoder_calib_is_valid_sample(enc, adc_value)) {
		// Spike ignoriert, aber geben false zurück (kein Sample akzeptiert)
		return false;
	}

	// Filtere über Moving Average
	uint16_t filtered_value = encoder_calib_filter_value(enc, adc_value);

	// Aktualisiere last_value für nächste Spike-Prüfung
	enc->last_value = adc_value;

	// Min/Max Tracking (auf gefilterten Wert)
	if (filtered_value < enc->min_value) {
		enc->min_value = filtered_value;
	}
	if (filtered_value > enc->max_value) {
		enc->max_value = filtered_value;
	}

	// Speichere Sample
	if (enc->sample_count < ENCODER_CALIB_NUM_SAMPLES) {
		enc->samples[enc->sample_count] = filtered_value;
		enc->sample_count++;
	}

	// Kalibrierung abgeschlossen wenn genug Samples
	if (enc->sample_count >= ENCODER_CALIB_NUM_SAMPLES) {
		enc->calib_state = ENCODER_CALIB_COMPLETE;
		return true;
	}

	return false;
}

/**
 * Kalibrierung finalisieren und Schwellwerte berechnen
 *
 * threshold_margin: z.B. 0.1 = 10% Sicherheitsspanne vom Wertebereich
 *
 * Strategie:
 * - Berechne Median (robust gegen Ausreißer)
 * - Berechne unteren/oberen Schwellwert basierend auf Median
 *   LOW  = Median - (Range * margin)  [schwarze Fläche]
 *   HIGH = Median + (Range * margin)  [weiße Fläche, auch wenn imperfekt]
 */
void EncoderCalib_Finalize(EncoderCalib_t *enc, float threshold_margin) {
	if (enc->calib_state != ENCODER_CALIB_COMPLETE) {
		return;
	}

	// Berechne Median aller Samples
	enc->median_value = encoder_calib_median(enc->samples, enc->sample_count);

	// Berechne Range
	uint16_t range = enc->max_value - enc->min_value;

	// Berechne Schwellwerte mit Margin für Hysterese
	// Der Margin schafft Abstand zur Spitze des Wertebereichs
	uint16_t margin = (uint16_t) (range * threshold_margin);

	enc->threshold_low = enc->min_value + margin; // Oberer Schwellwert für LOW->HIGH
	enc->threshold_high = enc->max_value - margin; // Unterer Schwellwert für HIGH->LOW

	// Sicherheit: LOW < HIGH
	if (enc->threshold_low >= enc->threshold_high) {
		// Fallback: Teile Bereich 50/50
		uint16_t mid = (enc->min_value + enc->max_value) / 2;
		enc->threshold_low = mid - 100;
		enc->threshold_high = mid + 100;
	}
}

/**
 * Zustand aktualisieren (Schmitt-Trigger)
 *
 * Hysterese-Logik:
 * - Wenn current_state == 0 (LOW):  state wird 1 (HIGH) nur wenn adc_value > threshold_high
 * - Wenn current_state == 1 (HIGH): state wird 0 (LOW) nur wenn adc_value < threshold_low
 * - Dazwischen: state bleibt gleich (Hysterese)
 */
void EncoderCalib_UpdateState(EncoderCalib_t *enc, uint16_t adc_value) {
	if (enc->calib_state != ENCODER_CALIB_COMPLETE) {
		// Kalibrierung nicht fertig, Zustand nicht aktualisieren
		return;
	}

	if (enc->current_state == 0) {
		// Im LOW Zustand
		if (adc_value > enc->threshold_high) {
			enc->current_state = 1;
		}
	} else {
		// Im HIGH Zustand
		if (adc_value < enc->threshold_low) {
			enc->current_state = 0;
		}
	}
}

/**
 * Aktuellen Zustand auslesen
 */
uint8_t EncoderCalib_GetState(EncoderCalib_t *enc) {
	return enc->current_state;
}

/**
 * Debug: Kalibrierungsdaten ausgeben (über UART)
 */
void EncoderCalib_PrintCalibData(EncoderCalib_t *enc) {
	uart_printf("=== Encoder Calibration Data ===\r\n");
	uart_printf("State: %d (0=Idle, 1=Running, 2=Complete)\r\n", enc->calib_state);
	uart_printf("Samples collected: %d / %d\r\n", enc->sample_count,
	ENCODER_CALIB_NUM_SAMPLES);
	uart_printf("Min value: %u\r\n", enc->min_value);
	uart_printf("Max value: %u\r\n", enc->max_value);
	uart_printf("Median value: %u\r\n", enc->median_value);
	uart_printf("Range: %u\r\n", enc->max_value - enc->min_value);
	uart_printf("Threshold LOW  (transition 1->0): %u\r\n", enc->threshold_low);
	uart_printf("Threshold HIGH (transition 0->1): %u\r\n", enc->threshold_high);
	uart_printf("Current state: %u (0=LOW/Black, 1=HIGH/White)\r\n", enc->current_state);
	uart_printf("================================\r\n");
}
