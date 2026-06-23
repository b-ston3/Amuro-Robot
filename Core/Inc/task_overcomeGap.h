/*
 * task_overcomeGap.h
 *
 *  Created on: Jun 23, 2026
 *      Author: ben
 */

#ifndef INC_TASK_OVERCOMEGAP_H_
#define INC_TASK_OVERCOMEGAP_H_

#include "main.h"

typedef enum {
	GAP_INIT,
	GAP_DRIVE_STEP_INIT,
	GAP_DRIVE_STEP,
	GAP_CHECK_LEFT_INIT,
	GAP_CHECK_LEFT,
	GAP_CHECK_RIGHT_INIT,
	GAP_CHECK_RIGHT,
	GAP_RETURN_CENTER_INIT,
	GAP_RETURN_CENTER,
	GAP_SUCCESS,
	GAP_FAILED
} OVERCOME_GAP_STATE;

void taskOvercomeGap_Init(float speed);
void taskOvercomeGap(void);
OVERCOME_GAP_STATE taskOvercomeGap_GetState(void);

#endif /* INC_TASK_OVERCOMEGAP_H_ */
