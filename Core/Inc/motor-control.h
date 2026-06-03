/*
 * motor-control.h
 *
 *  Created on: 28.05.2026
 *      Author: benst
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "tim.h"
#include "gpio.h"

//static int pwm(double speed, char inv);
void motorControl(char motor, char dir, double speed);

#endif /* INC_MOTOR_CONTROL_H_ */
