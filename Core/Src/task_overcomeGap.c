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

#define GAP_STEP_MM             50.0f   // Forward step between scan sweeps.
#define GAP_SWEEP_DEG           17.0f   // Rotation angle per scan sweep.
#define MAX_GAP_MM              300.0f  // Safety stop if no line is found.

#define ENCODER_TICKS_PER_MM    0.190f
#define BLACK_THRESHOLD         3100    // Line-detection threshold from ADC sensors.

#define REGULATION_INTERVAL     20u

static OVERCOME_GAP_STATE gap_state = GAP_INIT;
static float base_speed = 0.0f;
static float total_distance_driven = 0.0f;

static uint32_t last_regulation_time = 0;

// Shared ADC buffer produced by main.c DMA callback.
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
				// Stop active sub-tasks before returning control.
				taskDriveForward_Stop();
				taskRotate_Stop();

				// Stop both motors and report success to main state machine.
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

			// 1) Drive one forward step.
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

			// 2) Sweep left briefly.
		case GAP_CHECK_LEFT_INIT:
			reset_l_enc_tickcounter();
			reset_r_enc_tickcounter();
			taskRotate_Init(GAP_SWEEP_DEG, 'b', 'f'); // Rotate in place to the left.
			taskRotate();
			gap_state = GAP_CHECK_LEFT;
			break;

		case GAP_CHECK_LEFT:
			taskRotate();
			if (taskRotate_GetState() == ROTATION_STOP) {
				gap_state = GAP_CHECK_RIGHT_INIT;
			}
			break;

			// 3) Sweep right across center to the opposite side.
		case GAP_CHECK_RIGHT_INIT:
			reset_l_enc_tickcounter();
			reset_r_enc_tickcounter();
			taskRotate_Init(GAP_SWEEP_DEG * 2.0f, 'f', 'b'); // Rotate in place to the right.
			taskRotate();
			gap_state = GAP_CHECK_RIGHT;
			break;

		case GAP_CHECK_RIGHT:
			taskRotate();
			if (taskRotate_GetState() == ROTATION_STOP) {
				gap_state = GAP_RETURN_CENTER_INIT;
			}
			break;

			// 4) Return to center heading.
		case GAP_RETURN_CENTER_INIT:
			reset_l_enc_tickcounter();
			reset_r_enc_tickcounter();
			taskRotate_Init(GAP_SWEEP_DEG, 'b', 'f'); // Rotate back to center.
			taskRotate();
			gap_state = GAP_RETURN_CENTER;
			break;

		case GAP_RETURN_CENTER:
			taskRotate();
			if (taskRotate_GetState() == ROTATION_STOP) {
				// Completed one scan cycle without line detection; advance to next step.
				gap_state = GAP_DRIVE_STEP_INIT;
			}
			break;

		case GAP_SUCCESS:
			// main.c consumes GAP_SUCCESS and switches back to ROBOT_FOLLOW_LINE.
			break;

		case GAP_FAILED:
			// Gap scan exceeded allowed distance; keep robot stopped.
			motorControl('l', 'f', 0);
			motorControl('r', 'f', 0);
			break;
		}
	}
}

OVERCOME_GAP_STATE taskOvercomeGap_GetState(void) {
	return gap_state;
}
