/*
 * task_yparcour.h
 *
 *  Created on: 09.06.2026
 *      Author: benst
 */

#ifndef INC_TASK_YPARCOUR_H_
#define INC_TASK_YPARCOUR_H_

#include "main.h"

typedef enum {
	YPARCOUR_INIT = 0,
	YPARCOUR_SEGMENT_1_FORWARD,
	YPARCOUR_SEGMENT_2_ROTATE,
	YPARCOUR_SEGMENT_3_FORWARD,
	YPARCOUR_SEGMENT_4_ROTATE,
	YPARCOUR_SEGMENT_5_FORWARD,
	YPARCOUR_FINISHED
} YPARCOUR_TASK_STATE;

void taskYParcour_Init(float speed);
void taskYParcour(void);
void taskYParcour_Stop(void);
YPARCOUR_TASK_STATE taskYParcour_GetState(void);

#endif /* INC_TASK_YPARCOUR_H_ */
