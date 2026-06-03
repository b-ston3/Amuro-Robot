/*
 * encoder.h
 *
 *  Created on: 02.06.2026
 *      Author: benst
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>

uint8_t schmitt_trigger(char motor, uint16_t inputSignal);

uint32_t get_l_enc_tickcounter();
uint32_t get_r_enc_tickcounter();

#endif /* INC_ENCODER_H_ */
