/*
 * output.h
 *
 *  Created on: 28.07.2020
 *      Author: Bartosz
 */

#ifndef OUTPUT_H_
#define OUTPUT_H_

#include "main.h"

#define NB_OUTPUTS			10

#define STATE_ON			1
#define STATE_OFF			0

#define TIME_PERIO_PROP		10 // 200ms / 10 ms / 2

typedef struct{
	uint8_t state;
	uint8_t perio;
	uint8_t actual_time;

	uint16_t pin;
	GPIO_TypeDef * port;
}output;

output _output[NB_OUTPUTS];


void init_outputs(void);
void update_outputs(void);
void set_state_output(output * _out);


#endif /* OUTPUT_H_ */
