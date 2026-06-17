/*
 * task_linefollower.h
 *
 *  Created on: 16.06.2026
 *      Author: benst
 */

#ifndef INC_TASK_LINEFOLLOWER_H_
#define INC_TASK_LINEFOLLOWER_H_

#include "main.h"

typedef enum {
	LINE_INIT, LINE_RUNNING, LINE_LOST, LINE_STOP
} LINEFOLLOWER_TASK_STATE;

typedef enum {
	LINE_LEFT, LINE_MIDDLE, LINE_RIGHT
} LINE_LAST_SEEN;

void taskLineFollower_Init(float speed);
void taskLineFollower(void);
void taskLineFollower_Stop(void);
LINEFOLLOWER_TASK_STATE taskLineFollower_GetState(void);
//void taskLineFollower_setSensors(uint32_t raw_left, uint32_t raw_middle, uint32_t raw_right);
float taskLineFollower_getLeftSpeed(void);
float taskLineFollower_getRightSpeed(void);

#endif /* INC_TASK_LINEFOLLOWER_H_ */
