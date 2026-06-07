/*
 * task_LED.c
 *
 *  Created on: Jun 3, 2026
 *      Author: ben
 */

#include "task_LED.h"

#define LED3_ON_TIME    500   // milliseconds
#define LED3_OFF_TIME   500   // milliseconds

typedef enum {
	eStateA,  // LED ON state
	eStateB   // LED OFF state
} LED_STATE;

static LED_STATE taskLED_state = eStateA;
static uint32_t taskLED_timeout = 0;

/**
 * @brief  Initialize the LED3 task state and timeout
 * @retval None
 */
void taskLED_Init(void) {
	taskLED_state = eStateA;
	taskLED_timeout = HAL_GetTick();
}

/**
 * @brief  Cooperative multitasking LED3 blinking task
 *         Returns immediately without blocking
 *         Must be called periodically from main loop
 * @retval None
 */
void taskLED(void) {
	uint32_t current_time = HAL_GetTick();
	uint32_t elapsed_time = current_time - taskLED_timeout;

	switch (taskLED_state) {
	case eStateA:
		// LED ON state - turn on LED3 and wait for ON_TIME
		if (elapsed_time >= LED3_ON_TIME) {
			// Transition: Turn off LED3 and switch state
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
			taskLED_state = eStateB;
			taskLED_timeout = current_time;  // Reset timeout
		} else {
			// Still in ON state - keep LED on
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		}
		break;

	case eStateB:
		// LED OFF state - keep LED off and wait for OFF_TIME
		if (elapsed_time >= LED3_OFF_TIME) {
			// Transition: Turn on LED3 and switch state
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			taskLED_state = eStateA;
			taskLED_timeout = current_time;  // Reset timeout
		} else {
			// Still in OFF state - keep LED off
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		}
		break;
	}
}
