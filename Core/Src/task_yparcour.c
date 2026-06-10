/*
 * task_yparcour.c
 *
 *  Created on: 09.06.2026
 *      Author: benst
 */

#include "task_yparcour.h"
#include "encoder.h"
#include "motor-control.h"
#include "uart_debug.h"
#include "task_drive_forward.h"
#include "task_rotate.h"

#define REGULATION_INTERVAL 50u

#define ENCODER_TICKS_PER_MM 2.00f

static YPARCOUR_TASK_STATE yparcour_task_state = YPARCOUR_INIT;
static float base_speed = 0.0f;
static uint32_t last_regulation_time = 0u;

static float clampf(float val, float lo, float hi) {
	if (val < lo) {
		return lo;
	}
	if (val > hi) {
		return hi;
	}
	return val;
}

static void stop_motors(void) {
	motorControl('l', 'f', 0);
	motorControl('r', 'f', 0);
}

static void run_forward_segment(float distance_mm,
		YPARCOUR_TASK_STATE next_state) {
	switch (taskDriveForward_GetState()) {
	case DRIVE_INIT:
		//reset both encoder tick counter
		reset_l_enc_tickcounter();
		reset_r_enc_tickcounter();

		taskDriveForward_Init(base_speed);
		break;

	case DRIVE_RUNNING:
		int32_t left_ticks = (int32_t) get_l_enc_tickcounter();
		int32_t right_ticks = (int32_t) get_r_enc_tickcounter();
		int32_t progress_ticks = (int32_t) (((float) ((left_ticks + right_ticks
				+ 1) / 2)) - 0.5f);
		int32_t target_ticks = (int32_t) (distance_mm * ENCODER_TICKS_PER_MM);

		if (progress_ticks >= target_ticks) {
			taskDriveForward_Stop();
			ser_printf(
					"PARCOURS: state=%d done (forward %.0fmm), ticks L=%ld R=%ld target=%ld\n",
					yparcour_task_state, distance_mm, left_ticks, right_ticks,
					target_ticks);
			break;
		}

		taskDriveForward();
		ser_printf(
				"PARCOURS: state=%d fwd=%.0fmm progress=%ld/%ld ticks L=%ld R=%ld vL=%.1f vR=%.1f\n",
				yparcour_task_state, distance_mm, progress_ticks, target_ticks,
				left_ticks, right_ticks, taskDriveForward_getLeftSpeed(),
				taskDriveForward_getRightSpeed());
		break;

	case DRIVE_STOP:
		yparcour_task_state = next_state;
		reset_l_enc_tickcounter();
		reset_r_enc_tickcounter();
		break;

	default:
		break;
	}
}

static void run_rotation_segment(float angle_deg, char left_dir, char right_dir,
		YPARCOUR_TASK_STATE next_state) {
	switch (taskRotate_GetState()) {
	case ROTATION_INIT:
		taskRotate_Init(angle_deg, left_dir, right_dir);
		break;

	case ROTATION_RUNNING:
		taskRotate();
		break;

	case ROTATION_STOP:
		yparcour_task_state = next_state;
		reset_l_enc_tickcounter();
		reset_r_enc_tickcounter();
		break;

	default:
		break;
	}
}

void taskYParcour_Init(float speed) {
	base_speed = clampf(speed, 0.0f, 100.0f);
	yparcour_task_state = YPARCOUR_INIT;
	last_regulation_time = HAL_GetTick();
	reset_l_enc_tickcounter();
	reset_r_enc_tickcounter();

	ser_printf("PARCOURS: init speed=%.1f, calib ticks/mm=%.2f ticks\n",
			base_speed, ENCODER_TICKS_PER_MM);
}

void taskYParcour(void) {
	switch (yparcour_task_state) {
	case YPARCOUR_INIT:
		reset_l_enc_tickcounter();
		reset_r_enc_tickcounter();
		yparcour_task_state = YPARCOUR_SEGMENT_1_FORWARD_425;
		ser_printf("PARCOURS: start state=%d\n", yparcour_task_state);
		break;

	case YPARCOUR_SEGMENT_1_FORWARD_425:
		run_forward_segment(425.0f, YPARCOUR_SEGMENT_2_ROTATE_RIGHT_150);
		break;

	case YPARCOUR_SEGMENT_2_ROTATE_RIGHT_150:
		run_rotation_segment(150.0f, 'f', 'b', YPARCOUR_SEGMENT_3_FORWARD_250);
		break;

	case YPARCOUR_SEGMENT_3_FORWARD_250:
		run_forward_segment(250.0f, YPARCOUR_SEGMENT_4_ROTATE_LEFT_75);
		break;

	case YPARCOUR_SEGMENT_4_ROTATE_LEFT_75:
		run_rotation_segment(75.0f, 'b', 'f', YPARCOUR_SEGMENT_5_FORWARD_200);
		break;

	case YPARCOUR_SEGMENT_5_FORWARD_200:
		run_forward_segment(200.0f, YPARCOUR_FINISHED);
		break;

	case YPARCOUR_FINISHED:
		stop_motors();
		break;

	default:
		stop_motors();
		yparcour_task_state = YPARCOUR_FINISHED;
		break;
	}
}

void taskYParcour_Stop(void) {
	stop_motors();
	yparcour_task_state = YPARCOUR_FINISHED;
}

YPARCOUR_TASK_STATE taskYParcour_GetState(void) {
	return yparcour_task_state;
}
