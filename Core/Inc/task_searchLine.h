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
	SEARCH_SWEEP_PRIMARY_OUT,   // Sweep first toward the side where the line was last seen.
	SEARCH_SWEEP_PRIMARY_IN,    // Return from the primary sweep to center.
	SEARCH_SWEEP_SECONDARY_OUT, // Sweep to the opposite side.
	SEARCH_SWEEP_SECONDARY_IN,  // Return from the secondary sweep to center.
	SEARCH_SUCCESS,
	SEARCH_FAILED_GAP
} SEARCH_LINE_STATE;

void taskSearchLine_Init(float speed);
void taskSearchLine(void);
SEARCH_LINE_STATE taskSearchLine_GetState(void);

#endif /* INC_TASK_SEARCHLINE_H_ */
