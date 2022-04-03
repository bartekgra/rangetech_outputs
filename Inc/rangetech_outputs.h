/*
 * rangetech_pos.h
 *
 *  Created on: 10.04.2020
 *      Author: Bartosz
 */

#ifndef RANGETECH_OM_H_
#define RANGETECH_OM_H_

#include "main.h"
#include "corona_protocol.h"
#include "output.h"


enum main_states{
	_calib_communication,
	_work,
	_error
} main_state;

enum calib_com_states{
	_calib_module_com,
	_request_for_set_id_address,
	_request_for_start_msg
} calib_com_state;


#define PERIODE_CALIB_COM			100
#define PERIODE_CALIB_ENGINE		40
#define PERIODE_WORK				0
#define PERIODE_ERROR				5

uint16_t actual_counter_led_state;
uint16_t periode_led_state;
uint8_t actual_state_led_state;


void init_range_outputs(void);
void loop_outputs_10ms(void);
void calib_communication(void);
void calib_engines(void);
void work(void);
void error(void);

void init_led_state(void);
void led_state_update(void);


#endif /* RANGETECH_POS_H_ */
