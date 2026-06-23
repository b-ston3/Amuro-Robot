/*
 * task_avoidObstacle.h
 *
 *  Created on: Jun 23, 2026
 *      Author: ben
 */

#ifndef INC_TASK_AVOIDOBSTACLE_H_
#define INC_TASK_AVOIDOBSTACLE_H_

typedef enum {
	AVOID_INIT,
	AVOID_BACKUP,
	AVOID_TURN_LEFT_1,
	AVOID_DRIVE_PAST,
	AVOID_TURN_RIGHT_1,
	AVOID_DRIVE_TO_LINE,
	AVOID_TURN_RIGHT_2,
	AVOID_SUCCESS
} AVOID_OBSTACLE_STATE;

void taskAvoidObstacle_Init(void);
void taskAvoidObstacle(void);
AVOID_OBSTACLE_STATE taskAvoidObstacle_GetState(void);

#endif /* INC_TASK_AVOIDOBSTACLE_H_ */
