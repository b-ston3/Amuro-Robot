/*
 * task_parcours.c
 *
 *  Created on: 07.06.2026
 *      Author: benst
 */

#include "task_parcours.h"
#include "encoder.h"
#include "motor-control.h"
#include "uart_debug.h"

#define REGULATION_INTERVAL 50u
#define FORWARD_KP 0.5f
#define FORWARD_MAX_CORRECTION 20.0f

#define ROTATION_KP 0.30f
#define ROTATION_MIN_SPEED 16.0f
#define ROTATION_MAX_SPEED 45.0f
#define ROTATION_BALANCE_KP 0.5f
#define ROTATION_MAX_BALANCE 12.0f

#define ENCODER_TICKS_PER_MM 2.00f
#define ENCODER_TICKS_PER_DEG 1.10f

static PARCOURS_TASK_STATE parcours_task_state = PARCOURS_INIT;
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

static int32_t get_segment_progress_ticks(void) {
	return ((int32_t) get_l_enc_tickcounter()
			+ (int32_t) get_r_enc_tickcounter()) / 2;
}

static void run_forward_segment(float distance_mm,
		PARCOURS_TASK_STATE next_state) {
	int32_t left_ticks = (int32_t) get_l_enc_tickcounter();
	int32_t right_ticks = (int32_t) get_r_enc_tickcounter();
	int32_t progress_ticks = (left_ticks + right_ticks) / 2;
	int32_t target_ticks = (int32_t) (distance_mm * ENCODER_TICKS_PER_MM);
	uint32_t current_time = HAL_GetTick();

	if (progress_ticks >= target_ticks) {
		stop_motors();
		ser_printf(
				"PARCOURS: state=%d done (forward %.0fmm), ticks L=%ld R=%ld target=%ld\n",
				parcours_task_state, distance_mm, left_ticks, right_ticks,
				target_ticks);
		parcours_task_state = next_state;
		reset_l_enc_tickcounter();
		reset_r_enc_tickcounter();
		last_regulation_time = current_time;
		return;
	}

	if ((current_time - last_regulation_time) >= REGULATION_INTERVAL) {
		int32_t encoder_diff = left_ticks - right_ticks;
		float correction = clampf(FORWARD_KP * (float) encoder_diff,
		-FORWARD_MAX_CORRECTION, FORWARD_MAX_CORRECTION);
		float left_speed = clampf(base_speed - correction, 0.0f, 100.0f);
		float right_speed = clampf(base_speed + correction, 0.0f, 100.0f);

		motorControl('l', 'f', (double) left_speed);
		motorControl('r', 'f', (double) right_speed);

		ser_printf(
				"PARCOURS: state=%d fwd=%.0fmm progress=%ld/%ld ticks L=%ld R=%ld vL=%.1f vR=%.1f\n",
				parcours_task_state, distance_mm, progress_ticks, target_ticks,
				left_ticks, right_ticks, left_speed, right_speed);

		last_regulation_time = current_time;
	}
}

static void run_rotation_segment(float angle_deg, char left_dir, char right_dir,
		PARCOURS_TASK_STATE next_state) {
	int32_t left_ticks = (int32_t) get_l_enc_tickcounter();
	int32_t right_ticks = (int32_t) get_r_enc_tickcounter();
	int32_t progress_ticks = get_segment_progress_ticks();
	int32_t target_ticks = (int32_t) (angle_deg * ENCODER_TICKS_PER_DEG);
	uint32_t current_time = HAL_GetTick();

	if (progress_ticks >= target_ticks) {
		stop_motors();
		ser_printf(
				"PARCOURS: state=%d done (rot %.0fdeg), ticks L=%ld R=%ld target=%ld\n",
				parcours_task_state, angle_deg, left_ticks, right_ticks,
				target_ticks);
		parcours_task_state = next_state;
		reset_l_enc_tickcounter();
		reset_r_enc_tickcounter();
		last_regulation_time = current_time;
		return;
	}

	if ((current_time - last_regulation_time) >= REGULATION_INTERVAL) {
		int32_t remaining_ticks = target_ticks - progress_ticks;
		int32_t encoder_diff = left_ticks - right_ticks;
		float raw_speed = ROTATION_KP * (float) remaining_ticks;
		float base_rotation_speed = clampf(raw_speed, ROTATION_MIN_SPEED,
		ROTATION_MAX_SPEED);
		float balance = clampf(ROTATION_BALANCE_KP * (float) encoder_diff,
		-ROTATION_MAX_BALANCE, ROTATION_MAX_BALANCE);
		float left_speed = clampf(base_rotation_speed - balance, 0.0f, 100.0f);
		float right_speed = clampf(base_rotation_speed + balance, 0.0f, 100.0f);

		motorControl('l', left_dir, (double) left_speed);
		motorControl('r', right_dir, (double) right_speed);

		ser_printf(
				"PARCOURS: state=%d rot=%.0fdeg progress=%ld/%ld ticks L=%ld R=%ld vL=%.1f vR=%.1f\n",
				parcours_task_state, angle_deg, progress_ticks, target_ticks,
				left_ticks, right_ticks, left_speed, right_speed);

		last_regulation_time = current_time;
	}
}

void taskParcours_Init(double speed) {
	base_speed = clampf((float) speed, 0.0f, 100.0f);
	parcours_task_state = PARCOURS_INIT;
	last_regulation_time = HAL_GetTick();
	reset_l_enc_tickcounter();
	reset_r_enc_tickcounter();

	ser_printf("PARCOURS: init speed=%.1f, calib ticks/mm=%.2f ticks/deg=%.2f\n",
			base_speed, ENCODER_TICKS_PER_MM, ENCODER_TICKS_PER_DEG);
}

void taskParcours(void) {
	switch (parcours_task_state) {
	case PARCOURS_INIT:
		reset_l_enc_tickcounter();
		reset_r_enc_tickcounter();
		parcours_task_state = PARCOURS_SEGMENT_1_FORWARD_425;
		ser_printf("PARCOURS: start state=%d\n", parcours_task_state);
		break;

	case PARCOURS_SEGMENT_1_FORWARD_425:
		run_forward_segment(425.0f, PARCOURS_SEGMENT_2_ROTATE_RIGHT_150);
		break;

	case PARCOURS_SEGMENT_2_ROTATE_RIGHT_150:
		run_rotation_segment(150.0f, 'f', 'b',
				PARCOURS_SEGMENT_3_FORWARD_250);
		break;

	case PARCOURS_SEGMENT_3_FORWARD_250:
		run_forward_segment(250.0f, PARCOURS_SEGMENT_4_ROTATE_LEFT_75);
		break;

	case PARCOURS_SEGMENT_4_ROTATE_LEFT_75:
		run_rotation_segment(75.0f, 'b', 'f', PARCOURS_SEGMENT_5_FORWARD_200);
		break;

	case PARCOURS_SEGMENT_5_FORWARD_200:
		run_forward_segment(200.0f, PARCOURS_FINISHED);
		break;

	case PARCOURS_FINISHED:
		stop_motors();
		break;

	default:
		stop_motors();
		parcours_task_state = PARCOURS_FINISHED;
		break;
	}
}

void taskParcours_Stop(void) {
	stop_motors();
	parcours_task_state = PARCOURS_FINISHED;
}

PARCOURS_TASK_STATE taskParcours_GetState(void) {
	return parcours_task_state;
}
