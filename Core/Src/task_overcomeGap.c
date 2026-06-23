/*
 * task_overcomeGap.c
 *
 *  Created on: Jun 23, 2026
 *      Author: ben
 */

#include "task_overcomeGap.h"
#include "task_drive_forward.h"
#include "task_rotate.h"
#include "motor-control.h"
#include "encoder.h"

#define GAP_STEP_MM             40.0f   // Abstand zwischen den Drehetappen
#define GAP_SWEEP_DEG           10.0f   // Drehwinkel in Drehetappe
#define MAX_GAP_MM              200.0f  // Sicherheitsabschaltung

#define ENCODER_TICKS_PER_MM    0.20f
#define BLACK_THRESHOLD         2800    // Schwellenwert für die Linienerkennung

#define REGULATION_INTERVAL     20u

static OVERCOME_GAP_STATE gap_state = GAP_INIT;
static float base_speed = 0.0f;
static float total_distance_driven = 0.0f;

static uint32_t last_regulation_time = 0;

extern volatile uint32_t adc[6];

static uint8_t is_line_found(void) {
	if (adc[5] > BLACK_THRESHOLD || adc[0] > BLACK_THRESHOLD
			|| adc[2] > BLACK_THRESHOLD) {
		return 1;
	}
	return 0;
}

void taskOvercomeGap_Init(float speed) {
	base_speed = speed;
	gap_state = GAP_INIT;
	total_distance_driven = 0.0f;
	last_regulation_time = HAL_GetTick();
}

void taskOvercomeGap(void) {
	uint32_t current_time = HAL_GetTick();
	uint32_t time_since_regulation = current_time - last_regulation_time;

	if (time_since_regulation >= REGULATION_INTERVAL) {
		last_regulation_time = current_time;

		if (gap_state >= GAP_DRIVE_STEP_INIT
				&& gap_state <= GAP_RETURN_CENTER) {
			if (is_line_found()) {
				// Sub-Tasks sicher beenden
				taskDriveForward_Stop();
				taskRotate_Stop();

				// Anhalten
				motorControl('l', 'f', 0);
				motorControl('r', 'f', 0);
				gap_state = GAP_SUCCESS;
				return;
			}
		}

		switch (gap_state) {
		case GAP_INIT:
			total_distance_driven = 0.0f;
			gap_state = GAP_DRIVE_STEP_INIT;
			break;

			// 1.: Ein Stück geradeaus fahren
		case GAP_DRIVE_STEP_INIT:
			reset_l_enc_tickcounter();
			reset_r_enc_tickcounter();
			taskDriveForward_Init(base_speed);
			taskDriveForward();
			gap_state = GAP_DRIVE_STEP;
			break;

		case GAP_DRIVE_STEP:
			taskDriveForward();

			int32_t current_ticks = (get_l_enc_tickcounter()
					+ get_r_enc_tickcounter()) / 2;
			if (current_ticks > (GAP_STEP_MM * ENCODER_TICKS_PER_MM)) {
				taskDriveForward_Stop();
				total_distance_driven += GAP_STEP_MM;

				if (total_distance_driven >= MAX_GAP_MM) {
					gap_state = GAP_FAILED;
				} else {
					gap_state = GAP_CHECK_LEFT_INIT;
				}
			}
			break;

			// 2.: Kurz nach links "schauen"
		case GAP_CHECK_LEFT_INIT:
			taskRotate_Init(GAP_SWEEP_DEG, 'b', 'f'); // Auf der Stelle nach links
			taskRotate();
			gap_state = GAP_CHECK_LEFT;
			break;

		case GAP_CHECK_LEFT:
			taskRotate();
			if (taskRotate_GetState() == ROTATION_STOP) {
				gap_state = GAP_CHECK_RIGHT_INIT;
			}
			break;

			// 3.: Nach rechts "schauen" (Doppelter Winkel, da wir aktuell links stehen)
		case GAP_CHECK_RIGHT_INIT:
			taskRotate_Init(GAP_SWEEP_DEG * 2.0f, 'f', 'b'); // Auf der Stelle nach rechts
			taskRotate();
			gap_state = GAP_CHECK_RIGHT;
			break;

		case GAP_CHECK_RIGHT:
			taskRotate();
			if (taskRotate_GetState() == ROTATION_STOP) {
				gap_state = GAP_RETURN_CENTER_INIT;
			}
			break;

			// 4.: Wieder mittig ausrichten
		case GAP_RETURN_CENTER_INIT:
			taskRotate_Init(GAP_SWEEP_DEG, 'b', 'f'); // Zurück zur Mitte
			taskRotate();
			gap_state = GAP_RETURN_CENTER;
			break;

		case GAP_RETURN_CENTER:
			taskRotate();
			if (taskRotate_GetState() == ROTATION_STOP) {
				// Scan abgeschlossen, nichts gefunden -> Nächstes Stück geradeaus
				gap_state = GAP_DRIVE_STEP_INIT;
			}
			break;

		case GAP_SUCCESS:
			// Die Super-State-Machine wechselt jetzt wieder in den Linefollower
			break;

		case GAP_FAILED:
			// Lücke war zu groß -> Roboter stoppen
			motorControl('l', 'f', 0);
			motorControl('r', 'f', 0);
			break;
		}
	}
}

OVERCOME_GAP_STATE taskOvercomeGap_GetState(void) {
	return gap_state;
}
