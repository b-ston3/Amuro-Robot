/*
 * motor-control.c
 *
 *  Created on: 28.05.2026
 *      Author: benst
 */

#include "motor-control.h"

static int pwm(double speed, char inv) {
	if (!inv) {
		return 65535 * (speed / 100);
	} else if (inv) {
		return 65535 * (1 - (speed / 100));
	}
	return 0;
}

void motorControl(char motor, char dir, double speed) {
	if (motor == 'l') {
		switch (dir) {
		//forward
		case 'f':
			HAL_GPIO_WritePin(GPIOA, phase2_L_Pin, GPIO_PIN_SET);
			TIM1->CCR2 = pwm(speed, 1);
			break;
			//backward
		case 'b':
			HAL_GPIO_WritePin(GPIOA, phase2_L_Pin, GPIO_PIN_RESET);
			TIM1->CCR2 = pwm(speed, 0);
			break;
		default:
			break;
		}
	}
	//right Motor
	else if (motor == 'r') {
		switch (dir) {
		case 'f':
			HAL_GPIO_WritePin(GPIOB, phase2_R_Pin, GPIO_PIN_RESET);
			TIM1->CCR3 = pwm(speed, 0);
			break;
		case 'b':
			HAL_GPIO_WritePin(GPIOB, phase2_R_Pin, GPIO_PIN_SET);
			TIM1->CCR3 = pwm(speed, 1);
			break;
		default:
			break;
		}
	}
}
