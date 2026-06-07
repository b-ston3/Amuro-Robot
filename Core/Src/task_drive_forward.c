/*
 * task_drive_forward.c
 *
 *  Created on: 07.06.2026
 *      Author: benst
 */

#include "task_drive_forward.h"
#include "motor-control.h"
#include "encoder.h"

static DRIVE_TASK_STATE drive_task_state = DRIVE_INIT;
static float base_speed = 0;
static uint32_t last_regulation_time = 0;
static float left_speed = 0;
static float right_speed = 0;

//Regelungs-Parameter
#define REGULATION_INTERVAL 50u	// in ms wie oft regeln
#define KP 0.5f					// Proportional-Verstärkung
#define MAX_CORRECTION 20.0f	// [%]  Max. Korrektur pro Seite

// Motor-Kompensation (Hardware-Unterschiede ausgleichen)
#define LEFT_MOTOR_OFFSET 0.0f
#define RIGHT_MOTOR_OFFSET 0.0f

static float clampf(float val, float lo, float hi) {
	if (val < lo)
		return lo;
	if (val > hi)
		return hi;
	return val;
}

/**
 * @brief Initialize the drive forward task
 * @param speed: Base speed 0-100%
 * @retval None
 */
void taskDriveForward_Init(double speed) {
	base_speed = (float) speed;
	drive_task_state = DRIVE_INIT;
	last_regulation_time = HAL_GetTick();
}

void taskDriveForward(void) {
	uint32_t current_time = HAL_GetTick();
	uint32_t time_since_regulation = current_time - last_regulation_time;

	switch (drive_task_state) {
	case DRIVE_INIT:
		motorControl('l', 'f', (double) base_speed);
		motorControl('r', 'f', (double) base_speed);

		//reset both encoder tick counter
		reset_l_enc_tickcounter();
		reset_r_enc_tickcounter();

		drive_task_state = DRIVE_RUNNING;
		last_regulation_time = current_time;
		break;
	case DRIVE_RUNNING:
		if (time_since_regulation >= REGULATION_INTERVAL) {
			// Reset regulation timer
			last_regulation_time = current_time;

			int32_t left_ticks = (int32_t) get_l_enc_tickcounter();
			int32_t right_ticks = (int32_t) get_r_enc_tickcounter();
			int32_t encoder_diff = left_ticks - right_ticks;

			float correction = KP * (float) encoder_diff;

			correction = clampf(correction, -MAX_CORRECTION, MAX_CORRECTION);

			left_speed = base_speed - correction + LEFT_MOTOR_OFFSET;
			right_speed = base_speed + correction + RIGHT_MOTOR_OFFSET;

			// Clamp speeds to valid range (0-100%)
			left_speed = clampf(left_speed, 0.0f, 100.0f);
			right_speed = clampf(right_speed, 0.0f, 100.0f);

			// Apply regulated speeds to both motors
			motorControl('r', 'f', (double) right_speed);
			motorControl('l', 'f', (double) left_speed);
		}
		break;
	case DRIVE_STOP:
		// Stop motors
		motorControl('l', 'f', 0);
		motorControl('r', 'f', 0);
		break;
	default:
		break;
	}
}

/**
 * @brief Stop the drive forward task
 * @retval None
 */
void taskDriveForward_Stop(void) {
	drive_task_state = DRIVE_STOP;
}

/**
 * @brief Get current drive task state
 * @retval Current state
 */
DRIVE_TASK_STATE taskDriveForward_GetState(void) {
	return drive_task_state;
}
