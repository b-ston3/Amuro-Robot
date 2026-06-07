/*
 * encoder.c
 *
 *  Created on: 02.06.2026
 *      Author: benst
 */

#include "encoder.h"

#define ENCODER_LEFT_UPPER_THRESHOLD 3000
#define ENCODER_LEFT_LOWER_THRESHOLD 580
#define ENCODER_RIGHT_UPPER_THRESHOLD 3300
#define ENCODER_RIGHT_LOWER_THRESHOLD 240

volatile uint32_t l_enc_tickcounter = 0;
volatile uint8_t l_enc_last_state = 0;
volatile uint32_t r_enc_tickcounter = 0;
volatile uint8_t r_enc_last_state = 0;

uint8_t schmitt_trigger(char motor, uint16_t inputSignal) {
	uint8_t signal = 0;
	switch (motor) {
	case 'l':
		if (inputSignal > ENCODER_LEFT_UPPER_THRESHOLD) {
			signal = 1;
			if (l_enc_last_state == 0) {
				l_enc_last_state = 1;
				l_enc_tickcounter++;
			}
		} else if (inputSignal < ENCODER_LEFT_LOWER_THRESHOLD) {
			signal = 0;
			if (l_enc_last_state == 1) {
				l_enc_last_state = 0;
				l_enc_tickcounter++;
			}
		}
		break;
	case 'r':
		if (inputSignal > ENCODER_RIGHT_UPPER_THRESHOLD) {
			signal = 1;
			if (r_enc_last_state == 0) {
				r_enc_last_state = 1;
				r_enc_tickcounter++;
			}
		} else if (inputSignal < ENCODER_RIGHT_LOWER_THRESHOLD) {
			signal = 0;
			if (r_enc_last_state == 1) {
				r_enc_last_state = 0;
				r_enc_tickcounter++;

			}
			break;
			default:
			break;
		}
	}
	return signal;
}
