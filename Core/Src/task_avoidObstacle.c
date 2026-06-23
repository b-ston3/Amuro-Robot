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

#define OBSTACLE_SIZE_IN_MM 200.0f
#define ENCODER_TICKS_PER_MM    0.20f

#define OBSTACLE_AVOID_SPEED 35.0f
#define BLACK_THRESHOLD 2800

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
		motorControl('l', 'b', OBSTACLE_AVOID_SPEED);
		motorControl('r', 'b', OBSTACLE_AVOID_SPEED);
		if (get_l_enc_tickcounter() > 50) {
			motorControl('l', 'f', 0);
			motorControl('r', 'f', 0);
			taskRotate_Init(90.0f, 'b', 'f');
			avoid_state = AVOID_TURN_LEFT_1;
		}
		break;

	case AVOID_TURN_LEFT_1:
		taskRotate();
		if (taskRotate_GetState() == ROTATION_STOP) {
			taskDriveForward_Init(OBSTACLE_AVOID_SPEED);
			reset_l_enc_tickcounter();
			reset_r_enc_tickcounter();
			avoid_state = AVOID_DRIVE_PAST;
		}
		break;

	case AVOID_DRIVE_PAST:
		taskDriveForward();
		if (get_l_enc_tickcounter() > (OBSTACLE_SIZE_IN_MM * ENCODER_TICKS_PER_MM)) {
			taskDriveForward_Stop();
			taskRotate_Init(90.0f, 'f', 'b');
			avoid_state = AVOID_TURN_RIGHT_1;
		}
		break;

	case AVOID_TURN_RIGHT_1:
		taskRotate();
		if (taskRotate_GetState() == ROTATION_STOP) {
			taskDriveForward_Init(OBSTACLE_AVOID_SPEED);
			avoid_state = AVOID_DRIVE_TO_LINE;
		}
		break;

	case AVOID_DRIVE_TO_LINE:
		// Fährt blind geradeaus --Linie gefunden--> bricht sofort ab
		if (adc[5] > BLACK_THRESHOLD || adc[0] > BLACK_THRESHOLD || adc[2] > BLACK_THRESHOLD) {
			taskDriveForward_Stop();
			taskRotate_Init(90.0f, 'f', 'b');
			avoid_state = AVOID_TURN_RIGHT_2;
			break;
		}
		taskDriveForward();
		break;

	case AVOID_TURN_RIGHT_2:
		taskRotate();
		if (taskRotate_GetState() == ROTATION_STOP) {
			avoid_state = AVOID_SUCCESS;
		}
		break;

	case AVOID_SUCCESS:
		break;
	}
}

AVOID_OBSTACLE_STATE taskAvoidObstacle_GetState(void) {
	return avoid_state;
}
