/*
 * task_rotate.c
 *
 *  Created on: 09.06.2026
 *      Author: benst
 */

#include "task_rotate.h"
#include "encoder.h"
#include "motor-control.h"

#define REGULATION_INTERVAL 50u

#define KP 0.000030f
#define MIN_SPEED 32.0f
#define MAX_SPEED 52.0f
#define CORRECTION_KP 0.000050f
#define MAX_CORRECTION 12.0f

#define ENCODER_TICKS_PER_DEG 0.15381896410f	//0.14198673610f	//0.134513750f	//0.1356975f	//0.138065f	//0.1428f	//0.13333f

static ROTATION_TASK_STATE rotation_task_state = ROTATION_INIT;
static uint32_t last_regulation_time = 0u;
static float deg = 0.0f;
static char l_dir = 'f';
static char r_dir = 'f';

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

void taskRotate_Init(float angle_deg, char left_dir, char right_dir) {
	deg = angle_deg;
	l_dir = left_dir;
	r_dir = right_dir;
	rotation_task_state = ROTATION_INIT;
	last_regulation_time = HAL_GetTick();
}

void taskRotate(void) {
	uint32_t current_time = HAL_GetTick();
	uint32_t time_since_regulation = current_time - last_regulation_time;

	switch (rotation_task_state) {
	case ROTATION_INIT:
		//reset both encoder tick counter
		reset_l_enc_tickcounter();
		reset_r_enc_tickcounter();

		rotation_task_state = ROTATION_RUNNING;
		last_regulation_time = current_time;
		break;

	case ROTATION_RUNNING:
		int32_t left_ticks = (int32_t) get_l_enc_tickcounter();
		int32_t right_ticks = (int32_t) get_r_enc_tickcounter();
		int32_t progress_ticks = (int32_t) (((float) ((left_ticks + right_ticks
				+ 1) / 2)) - 0.5f);
		int32_t target_ticks = (int32_t) (deg * ENCODER_TICKS_PER_DEG);

		if (progress_ticks >= target_ticks) {
			rotation_task_state = ROTATION_STOP;
			break;
		}

		if (time_since_regulation >= REGULATION_INTERVAL) {
			int32_t remaining_ticks = target_ticks - progress_ticks;
			int32_t encoder_diff = left_ticks - right_ticks;
			float raw_speed = KP * (float) remaining_ticks;
			float base_rotation_speed = clampf(raw_speed, MIN_SPEED,
			MAX_SPEED);
			float correction = clampf(CORRECTION_KP * (float) encoder_diff,
					-MAX_CORRECTION, MAX_CORRECTION);
			float left_speed = clampf(base_rotation_speed - correction, 0.0f,
					100.0f);
			float right_speed = clampf(base_rotation_speed + correction, 0.0f,
					100.0f);

			motorControl('l', l_dir, left_speed);
			motorControl('r', r_dir, right_speed);

			last_regulation_time = current_time;
		}
		break;

	case ROTATION_STOP:
		stop_motors();
		reset_l_enc_tickcounter();
		reset_r_enc_tickcounter();
		last_regulation_time = current_time;
		break;

	default:
		break;
	}
}

void taskRotate_Stop(void) {
	rotation_task_state = ROTATION_STOP;
}

ROTATION_TASK_STATE taskRotate_GetState(void) {
	return rotation_task_state;
}
