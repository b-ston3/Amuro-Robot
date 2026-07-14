/*
 * task_avoidObstacle.c
 *
 *  Created on: Jun 23, 2026
 *      Author: ben
 */

#include "task_avoidObstacle.h"
#include "task_drive_forward.h"
#include "task_rotate.h"
#include "motor-control.h"
#include "encoder.h"

#define AVOID_SPEED             35.0f   // Basis-Geschwindigkeit
#define AVOID_BACKUP_MM         40.0f

// Schritt 1: Rechtsdrehung
#define AVOID_ANGLE_1_DEG       45.0f

// Schritt 2: Erstes Stück geradeaus
#define AVOID_DRIVE_1_MM        105.0f

// Schritt 3: Erste Linksdrehung
#define AVOID_ANGLE_2_DEG       45.0f

// Schritt 4: Langes Stück am Hindernis vorbei
#define AVOID_DRIVE_2_MM        125.0f	//105.0f

// Schritt 5: Zweite Linksdrehung
#define AVOID_ANGLE_3_DEG       67.5f

// Schritt 6: Zurück in Richtung Linie
#define AVOID_DRIVE_3_MM        100.0f
#define AVOID_DRIVE_3_SEARCH_IN_MM 80.0f

// Schritt 7: Finale Rechtsdrehung zur Linie
#define AVOID_RETURN_MAX_DEG    120.0f  // Maximalwinkel als Fallback für die letzte Drehung

// Hardware-Konstanten
#define ENCODER_TICKS_PER_MM    0.190f	//0.20f
#define BLACK_THRESHOLD         3100

static AVOID_OBSTACLE_STATE avoid_state = AVOID_INIT;
extern volatile uint32_t adc[6];

void taskAvoidObstacle_Init(void) {
	avoid_state = AVOID_INIT;
}

void taskAvoidObstacle(void) {
	switch (avoid_state) {
	case AVOID_INIT:
		reset_l_enc_tickcounter();
		reset_r_enc_tickcounter();
		avoid_state = AVOID_BACKUP;
		break;

	case AVOID_BACKUP:
		motorControl('l', 'b', AVOID_SPEED);
		motorControl('r', 'b', AVOID_SPEED);
		if (get_l_enc_tickcounter()
				> (AVOID_BACKUP_MM * ENCODER_TICKS_PER_MM)) {
			motorControl('l', 'f', 0);
			motorControl('r', 'f', 0);
			reset_l_enc_tickcounter();
			reset_r_enc_tickcounter();
			// Drehung nach rechts
			taskRotate_Init(AVOID_ANGLE_1_DEG, 'f', 'b');
			avoid_state = AVOID_TURN_RIGHT_1;
		}
		break;

	case AVOID_TURN_RIGHT_1:
		taskRotate();
		if (taskRotate_GetState() == ROTATION_STOP) {
			taskDriveForward_Init(AVOID_SPEED);
			reset_l_enc_tickcounter();
			reset_r_enc_tickcounter();
			avoid_state = AVOID_DRIVE_1;
		}
		break;

	case AVOID_DRIVE_1:
		taskDriveForward();
		if (get_l_enc_tickcounter()
				> (AVOID_DRIVE_1_MM * ENCODER_TICKS_PER_MM)) {
			taskDriveForward_Stop();
			reset_l_enc_tickcounter();
			reset_r_enc_tickcounter();
			// Drehung nach links
			taskRotate_Init(AVOID_ANGLE_2_DEG, 'b', 'f');
			avoid_state = AVOID_TURN_LEFT_1;
		}
		break;

	case AVOID_TURN_LEFT_1:
		taskRotate();
		if (taskRotate_GetState() == ROTATION_STOP) {
			taskDriveForward_Init(AVOID_SPEED);
			reset_l_enc_tickcounter();
			reset_r_enc_tickcounter();
			avoid_state = AVOID_DRIVE_2;
		}
		break;

	case AVOID_DRIVE_2:
		taskDriveForward();
		if (get_l_enc_tickcounter()
				> (AVOID_DRIVE_2_MM * ENCODER_TICKS_PER_MM)) {
			taskDriveForward_Stop();
			reset_l_enc_tickcounter();
			reset_r_enc_tickcounter();
			// Drehung nach links
			taskRotate_Init(AVOID_ANGLE_3_DEG, 'b', 'f');
			avoid_state = AVOID_TURN_LEFT_2;
		}
		break;

	case AVOID_TURN_LEFT_2:
		taskRotate();
		if (taskRotate_GetState() == ROTATION_STOP) {
			taskDriveForward_Init(AVOID_SPEED);
			reset_l_enc_tickcounter();
			reset_r_enc_tickcounter();
			avoid_state = AVOID_DRIVE_3;
		}
		break;

	case AVOID_DRIVE_3:
		taskDriveForward();
		if ((get_l_enc_tickcounter()
				> AVOID_DRIVE_3_SEARCH_IN_MM * ENCODER_TICKS_PER_MM)
				&& (adc[5] > BLACK_THRESHOLD || adc[0] > BLACK_THRESHOLD
						|| adc[2] > BLACK_THRESHOLD)) {
			taskRotate_Stop();
			motorControl('l', 'f', 0);
			motorControl('r', 'f', 0);
			avoid_state = AVOID_SUCCESS;
			break;
		}
		if (get_l_enc_tickcounter()
				> (AVOID_DRIVE_3_MM * ENCODER_TICKS_PER_MM)) {
			taskDriveForward_Stop();
			reset_l_enc_tickcounter();
			reset_r_enc_tickcounter();
			// Finale Drehung nach rechts
			taskRotate_Init(AVOID_RETURN_MAX_DEG, 'f', 'b');
			avoid_state = AVOID_TURN_RIGHT_TO_LINE;
		}
		break;

	case AVOID_TURN_RIGHT_TO_LINE:
		// Sensor erkennt Linie --> Drehung abbrechen
		if (adc[5] > BLACK_THRESHOLD || adc[0] > BLACK_THRESHOLD
				|| adc[2] > BLACK_THRESHOLD) {
			taskRotate_Stop();
			motorControl('l', 'f', 0);
			motorControl('r', 'f', 0);
			avoid_state = AVOID_SUCCESS;
			break;
		}

		taskRotate();

		// Fallback: Falls Roboter sich komplett gedreht hat und keine Linie gefunden hat
		if (taskRotate_GetState() == ROTATION_STOP) {
			avoid_state = AVOID_SUCCESS;
		}
		break;

	case AVOID_SUCCESS:
		// Wartet darauf, von der Super-State-Machine in der main.c umgeschaltet zu werden
		break;
	}
}

AVOID_OBSTACLE_STATE taskAvoidObstacle_GetState(void) {
	return avoid_state;
}
