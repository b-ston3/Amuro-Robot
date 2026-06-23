/*
 * task_searchLine.h
 *
 *  Created on: Jun 23, 2026
 *      Author: ben
 */

#ifndef INC_TASK_SEARCHLINE_H_
#define INC_TASK_SEARCHLINE_H_

#include "main.h"

typedef enum {
	SEARCH_INIT,
	SEARCH_SWEEP_PRIMARY_OUT, // Dreht in vermutete Richtung
	SEARCH_SWEEP_PRIMARY_IN,  // Dreht zurück zur Mitte
	SEARCH_SWEEP_SECONDARY_OUT, // Dreht in andere Richtung
	SEARCH_SWEEP_SECONDARY_IN,  // Dreht zurück zur Mitte
	SEARCH_SUCCESS,
	SEARCH_FAILED_GAP
} SEARCH_LINE_STATE;

void taskSearchLine_Init(float speed);
void taskSearchLine(void);
SEARCH_LINE_STATE taskSearchLine_GetState(void);

#endif /* INC_TASK_SEARCHLINE_H_ */
