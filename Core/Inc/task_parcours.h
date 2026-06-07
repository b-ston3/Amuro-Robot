/*
 * task_parcours.h
 *
 *  Created on: 07.06.2026
 *      Author: benst
 */

#ifndef INC_TASK_PARCOURS_H_
#define INC_TASK_PARCOURS_H_

typedef enum {
	PARCOURS_INIT = 0,
	PARCOURS_SEGMENT_1_FORWARD_425,
	PARCOURS_SEGMENT_2_ROTATE_RIGHT_150,
	PARCOURS_SEGMENT_3_FORWARD_250,
	PARCOURS_SEGMENT_4_ROTATE_LEFT_75,
	PARCOURS_SEGMENT_5_FORWARD_200,
	PARCOURS_FINISHED
} PARCOURS_TASK_STATE;

void taskParcours_Init(double speed);
void taskParcours(void);
void taskParcours_Stop(void);
PARCOURS_TASK_STATE taskParcours_GetState(void);

#endif /* INC_TASK_PARCOURS_H_ */
