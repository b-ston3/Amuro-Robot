/*
 * motor-control.c
 *
 *  Created on: 28.05.2026
 *      Author: benst
 */

#include "motor-control.h"

volatile double speed_left = 0;
volatile double speed_right = 0;

typedef enum {
	FORWARD, BACKWARDS, TURN, STOP
} DRIVE_STATE;
static DRIVE_STATE drive_state = STOP;

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
		speed_left = speed;
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
		speed_right = speed;
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

void drive_forward(double speed) {
	if (drive_state != FORWARD) {
		motorControl('l', 'f', speed);
		motorControl('r', 'f', speed);
	} else {
		motorControl('l', 'f', (((get_l_enc_tickcounter() - get_r_enc_tickcounter())
				/ get_l_enc_tickcounter()) * 100) + speed_left);
	}
}
