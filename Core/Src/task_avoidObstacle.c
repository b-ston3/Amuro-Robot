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

#define AVOID_SPEED             35.0f   // Base speed for the maneuver sequence.
#define AVOID_BACKUP_MM         40.0f

// Step 1: initial right turn.
#define AVOID_ANGLE_1_DEG       45.0f

// Step 2: first forward segment.
#define AVOID_DRIVE_1_MM        105.0f

// Step 3: first left turn.
#define AVOID_ANGLE_2_DEG       45.0f

// Step 4: longer forward segment past the obstacle.
#define AVOID_DRIVE_2_MM        125.0f

// Step 5: second left turn.
#define AVOID_ANGLE_3_DEG       67.5f

// Step 6: drive toward the expected line position.
#define AVOID_DRIVE_3_MM        100.0f
#define AVOID_DRIVE_3_SEARCH_IN_MM 80.0f

// Step 7: final right turn to reacquire the line.
#define AVOID_RETURN_MAX_DEG    120.0f  // Max fallback angle for the final turn.

// Hardware conversion constants.
#define ENCODER_TICKS_PER_MM    0.190f
#define BLACK_THRESHOLD         3100

// Shared ADC buffer produced by main.c DMA callback.
static AVOID_OBSTACLE_STATE avoid_state = AVOID_INIT;
extern volatile uint32_t adc[6];

void taskAvoidObstacle_Init(void) {
	avoid_state = AVOID_INIT;
}

void taskAvoidObstacle(void) {
	/*
	 * Local obstacle-avoidance sequence.
	 * main.c enters this task after bumper detection and exits on AVOID_SUCCESS.
	 */
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
			// Turn right.
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
			// Turn left.
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
			// Turn left.
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
			// Final right turn to search for the line.
			taskRotate_Init(AVOID_RETURN_MAX_DEG, 'f', 'b');
			avoid_state = AVOID_TURN_RIGHT_TO_LINE;
		}
		break;

	case AVOID_TURN_RIGHT_TO_LINE:
		// Stop rotating immediately when any line sensor detects black.
		if (adc[5] > BLACK_THRESHOLD || adc[0] > BLACK_THRESHOLD
				|| adc[2] > BLACK_THRESHOLD) {
			taskRotate_Stop();
			motorControl('l', 'f', 0);
			motorControl('r', 'f', 0);
			avoid_state = AVOID_SUCCESS;
			break;
		}

		taskRotate();

		// Fallback: stop searching after the bounded turn completes.
		if (taskRotate_GetState() == ROTATION_STOP) {
			avoid_state = AVOID_SUCCESS;
		}
		break;

	case AVOID_SUCCESS:
		// main.c consumes AVOID_SUCCESS and switches back to ROBOT_FOLLOW_LINE.
		break;
	}
}

AVOID_OBSTACLE_STATE taskAvoidObstacle_GetState(void) {
	return avoid_state;
}
