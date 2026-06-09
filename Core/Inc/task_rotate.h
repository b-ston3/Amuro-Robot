/*
 * task_rotate.h
 *
 *  Created on: 09.06.2026
 *      Author: benst
 */

#ifndef INC_TASK_ROTATE_H_
#define INC_TASK_ROTATE_H_

typedef enum {
	ROTATION_INIT, ROTATION_RUNNING, ROTATION_STOP
} ROTATION_TASK_STATE;

void taskRotate_Init(float angle_deg, char left_dir,
		char right_dir);
void taskRotate(void);
void taskRotate_Stop(void);
ROTATION_TASK_STATE taskRotate_GetState(void);

#endif /* INC_TASK_ROTATE_H_ */
