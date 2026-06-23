/*
 * task_searchLine.c
 *
 *  Created on: Jun 23, 2026
 *      Author: ben
 */

#include "task_searchLine.h"
#include "task_linefollower.h"
#include "motor-control.h"
#include "encoder.h"
#include "uart_debug.h"

#define SWEEP_100_DEG_TICKS 31
#define BLACK_THRESHOLD 2800
#define REGULATION_INTERVAL 20u

static uint32_t last_regulation_time = 0;
static SEARCH_LINE_STATE search_state = SEARCH_INIT;
static float base_speed = 0.0f;
static LINE_LAST_SEEN primary_direction;

extern volatile uint32_t adc[6];

// Hilfsfunktion zur Linienerkennung
static uint8_t is_line_found(void) {
	if (adc[5] > BLACK_THRESHOLD || adc[0] > BLACK_THRESHOLD
			|| adc[2] > BLACK_THRESHOLD) {
		return 1;
	}
	return 0;
}

void taskSearchLine_Init(float speed) {
	base_speed = speed;
	search_state = SEARCH_INIT;
	last_regulation_time = HAL_GetTick();
}

void taskSearchLine(void) {
	uint32_t current_time = HAL_GetTick();
	uint32_t time_since_regulation = current_time - last_regulation_time;

	switch (search_state) {
	case SEARCH_INIT:
		primary_direction = taskLineFollower_GetLastSeen();
		reset_l_enc_tickcounter();
		reset_r_enc_tickcounter();
		search_state = SEARCH_SWEEP_PRIMARY_OUT;
		last_regulation_time = current_time;
		break;

	case SEARCH_SWEEP_PRIMARY_OUT:
		if (time_since_regulation >= REGULATION_INTERVAL) {
			// Abbruchbedingung: Linie gefunden!
			if (is_line_found()) {
				motorControl('l', 'f', 0);
				motorControl('r', 'f', 0);
				search_state = SEARCH_SUCCESS;
				last_regulation_time = current_time;
				break;
			}

			if (primary_direction == LINE_LEFT) {
				motorControl('l', 'f', 0);
				motorControl('r', 'f', base_speed);
				if (get_r_enc_tickcounter() > SWEEP_100_DEG_TICKS) {
					reset_l_enc_tickcounter();
					reset_r_enc_tickcounter();
					search_state = SEARCH_SWEEP_PRIMARY_IN;
				}
			} else { // RIGHT oder MIDDLE als Default nach rechts
				motorControl('l', 'f', base_speed);
				motorControl('r', 'f', 0);
				if (get_l_enc_tickcounter() > SWEEP_100_DEG_TICKS) {
					reset_l_enc_tickcounter();
					reset_r_enc_tickcounter();
					search_state = SEARCH_SWEEP_PRIMARY_IN;
				}
			}
			last_regulation_time = current_time;
		}
		break;

	case SEARCH_SWEEP_PRIMARY_IN:
		if (time_since_regulation >= REGULATION_INTERVAL) {
			if (is_line_found()) {
				motorControl('l', 'f', 0);
				motorControl('r', 'f', 0);
				search_state = SEARCH_SUCCESS;
				last_regulation_time = current_time;
				break;
			}

			// Drehe zurück in die Ausgangsposition
			if (primary_direction == LINE_LEFT) {
				motorControl('l', 'f', 0);
				motorControl('r', 'b', base_speed);
				if (get_r_enc_tickcounter() > SWEEP_100_DEG_TICKS) {
					reset_l_enc_tickcounter();
					reset_r_enc_tickcounter();
					search_state = SEARCH_SWEEP_SECONDARY_OUT;
				}
			} else {
				motorControl('l', 'b', base_speed);
				motorControl('r', 'f', 0);
				if (get_l_enc_tickcounter() > SWEEP_100_DEG_TICKS) {
					reset_l_enc_tickcounter();
					reset_r_enc_tickcounter();
					search_state = SEARCH_SWEEP_SECONDARY_OUT;
				}
			}
			last_regulation_time = current_time;
		}
		break;

	case SEARCH_SWEEP_SECONDARY_OUT:
		if (time_since_regulation >= REGULATION_INTERVAL) {
			if (is_line_found()) {
				motorControl('l', 'f', 0);
				motorControl('r', 'f', 0);
				search_state = SEARCH_SUCCESS;
				last_regulation_time = current_time;
				break;
			}
			// In die andere Richtung absuchen
			if (primary_direction == LINE_LEFT) {
				motorControl('l', 'f', base_speed);
				motorControl('r', 'f', 0);
				if (get_l_enc_tickcounter() > SWEEP_100_DEG_TICKS) {
					reset_l_enc_tickcounter();
					reset_r_enc_tickcounter();
					search_state = SEARCH_SWEEP_SECONDARY_IN;
				}
			} else {
				motorControl('l', 'f', 0);
				motorControl('r', 'f', base_speed);
				if (get_r_enc_tickcounter() > SWEEP_100_DEG_TICKS) {
					reset_l_enc_tickcounter();
					reset_r_enc_tickcounter();
					search_state = SEARCH_SWEEP_SECONDARY_IN;
				}
			}
			last_regulation_time = current_time;
		}
		break;

	case SEARCH_SWEEP_SECONDARY_IN:
		if (time_since_regulation >= REGULATION_INTERVAL) {
			if (is_line_found()) {
				motorControl('l', 'f', 0);
				motorControl('r', 'f', 0);
				search_state = SEARCH_SUCCESS;
				last_regulation_time = current_time;
				break;
			}

			// Zurück zur Mitte
			if (primary_direction == LINE_LEFT) {
				motorControl('l', 'b', base_speed);
				motorControl('r', 'f', 0);
				if (get_l_enc_tickcounter() > SWEEP_100_DEG_TICKS) {
					motorControl('l', 'f', 0);
					search_state = SEARCH_FAILED_GAP;
				}
			} else {
				motorControl('l', 'f', 0);
				motorControl('r', 'b', base_speed);
				if (get_r_enc_tickcounter() > SWEEP_100_DEG_TICKS) {
					motorControl('r', 'f', 0);
					search_state = SEARCH_FAILED_GAP;
				}
			}
			last_regulation_time = current_time;
		}
		break;

	case SEARCH_SUCCESS:
	case SEARCH_FAILED_GAP:
		motorControl('l', 'f', 0);
		motorControl('r', 'f', 0);
		last_regulation_time = current_time;
		break;
	}
}

SEARCH_LINE_STATE taskSearchLine_GetState(void) {
	return search_state;
}
