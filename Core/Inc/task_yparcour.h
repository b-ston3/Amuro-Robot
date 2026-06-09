/*
 * task_yparcour.h
 *
 *  Created on: 09.06.2026
 *      Author: benst
 */

#ifndef INC_TASK_YPARCOUR_H_
#define INC_TASK_YPARCOUR_H_

typedef enum {
	YPARCOUR_INIT = 0,
	YPARCOUR_SEGMENT_1_FORWARD_425,
	YPARCOUR_SEGMENT_2_ROTATE_RIGHT_150,
	YPARCOUR_SEGMENT_3_FORWARD_250,
	YPARCOUR_SEGMENT_4_ROTATE_LEFT_75,
	YPARCOUR_SEGMENT_5_FORWARD_200,
	YPARCOUR_FINISHED
} YPARCOUR_TASK_STATE;

void taskYParcour_Init(float speed);
void taskYParcour(void);
void taskYParcour_Stop(void);
YPARCOUR_TASK_STATE taskYParcour_GetState(void);

#endif /* INC_TASK_YPARCOUR_H_ */
