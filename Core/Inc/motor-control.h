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
#include "encoder.h"

void motorControl(char motor, char dir, double speed);

#endif /* INC_MOTOR_CONTROL_H_ */
