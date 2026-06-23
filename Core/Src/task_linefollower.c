/*
 * task_linefollower.c
 *
 *  Created on: 16.06.2026
 *      Author: benst
 */

#include "task_linefollower.h"
#include "motor-control.h"
#include "uart_debug.h"

static LINEFOLLOWER_TASK_STATE linefollower_task_state = LINE_INIT;
static float base_speed = 0.0f;
static uint32_t last_regulation_time = 0;
static float left_speed = 0.0f;
static float right_speed = 0.0f;

static LINE_LAST_SEEN line_last_seen = LINE_MIDDLE;

// Zugriff auf das globale ADC-Array aus der main.c
// adc[5]: lineSensor_left, adc[0]: lineSensor_middle, adc[2]: lineSensor_right
extern volatile uint32_t adc[6];

// Regelungs-Parameter
#define REGULATION_INTERVAL 20u     // in ms wie oft regeln
#define KP 0.027f                   // Proportional-Verstärkung
#define MAX_CORRECTION 30.0f

#define RIGHT_SENSOR_OFFSET 210     // Hardware-Unterschied der Sensoren

#define BLACK_THRESHOLD 2800        // ADC-Werte unter diesem Wert gelten als "Weiß"

static float clampf(float val, float lo, float hi) {
	if (val < lo)
		return lo;
	if (val > hi)
		return hi;
	return val;
}

/**
 * @brief Initialize the line follower task
 * @param speed: Base speed 0-100%
 * @retval None
 */
void taskLineFollower_Init(float speed) {
	base_speed = speed;
	linefollower_task_state = LINE_INIT;
	last_regulation_time = HAL_GetTick();
}

void taskLineFollower(void) {
	uint32_t current_time = HAL_GetTick();
	uint32_t time_since_regulation = current_time - last_regulation_time;

	switch (linefollower_task_state) {
	case LINE_INIT:
		motorControl('l', 'f', base_speed);
		motorControl('r', 'f', base_speed);

		linefollower_task_state = LINE_RUNNING;
		last_regulation_time = current_time;
		break;

	case LINE_RUNNING:
		if (time_since_regulation >= REGULATION_INTERVAL) {
			last_regulation_time = current_time;
			uint32_t raw_left = adc[5];
			uint32_t raw_middle = adc[0];
			uint32_t raw_right = adc[2];

			if (raw_left < BLACK_THRESHOLD && raw_middle < BLACK_THRESHOLD
					&& raw_right < BLACK_THRESHOLD) {
				linefollower_task_state = LINE_LOST;
				break;
			}

			if (raw_left >= BLACK_THRESHOLD
					&& raw_left + RIGHT_SENSOR_OFFSET >= raw_right) {
				line_last_seen = LINE_LEFT;
			} else if (raw_right >= BLACK_THRESHOLD
					&& raw_right >= raw_left + RIGHT_SENSOR_OFFSET) {
				line_last_seen = LINE_RIGHT;
			} else if (raw_middle >= BLACK_THRESHOLD) {
				line_last_seen = LINE_MIDDLE;
			}

			int32_t left_val = (int32_t) raw_left + RIGHT_SENSOR_OFFSET;
			int32_t right_val = (int32_t) raw_right;

			int32_t sensor_diff = left_val - right_val;
			float correction = KP * (float) sensor_diff;
			correction = clampf(correction, -MAX_CORRECTION, MAX_CORRECTION);

			left_speed = base_speed - correction;
			right_speed = base_speed + correction;

			left_speed = clampf(left_speed, 0.0f, 100.0f);
			right_speed = clampf(right_speed, 0.0f, 100.0f);

			motorControl('r', 'f', right_speed);
			motorControl('l', 'f', left_speed);
		}
		break;

	case LINE_LOST:
		motorControl('l', 'f', 0);
		motorControl('r', 'f', 0);
		last_regulation_time = current_time;
		break;

	case LINE_STOP:
		motorControl('l', 'f', 0);
		motorControl('r', 'f', 0);
		last_regulation_time = current_time;
		break;

	default:
		break;
	}
}

/**
 * @brief Stop the line follower task
 * @retval None
 */
void taskLineFollower_Stop(void) {
	linefollower_task_state = LINE_STOP;
}

/**
 * @brief Get current line follower task state
 * @retval Current state
 */
LINEFOLLOWER_TASK_STATE taskLineFollower_GetState(void) {
	return linefollower_task_state;
}

float taskLineFollower_getLeftSpeed(void) {
	return left_speed;
}

float taskLineFollower_getRightSpeed(void) {
	return right_speed;
}

LINE_LAST_SEEN taskLineFollower_GetLastSeen(void) {
	return line_last_seen;
}
