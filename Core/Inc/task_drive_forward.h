/*
 * task_drive_forward.h
 *
 *  Created on: 07.06.2026
 *      Author: benst
 */

#ifndef INC_TASK_DRIVE_FORWARD_H_
#define INC_TASK_DRIVE_FORWARD_H_

typedef enum {
	DRIVE_INIT, DRIVE_RUNNING, DRIVE_STOP
} DRIVE_TASK_STATE;

void taskDriveForward_Init(float speed);
void taskDriveForward(void);
void taskDriveForward_Stop(void);
DRIVE_TASK_STATE taskDriveForward_GetState(void);
float taskDriveForward_getLeftSpeed(void);
float taskDriveForward_getRightSpeed(void);

#endif /* INC_TASK_DRIVE_FORWARD_H_ */
