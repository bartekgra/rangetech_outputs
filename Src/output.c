/*
 * output.c
 *
 *  Created on: 28.07.2020
 *      Author: Bartosz
 */


#include "output.h"

void init_outputs(void){
	_output[0].pin = ENABLE_1_Pin;
	_output[0].port = ENABLE_1_GPIO_Port;

	_output[1].pin = ENABLE_2_Pin;
	_output[1].port = ENABLE_2_GPIO_Port;

	_output[2].pin = ENABLE_3_Pin;
	_output[2].port = ENABLE_3_GPIO_Port;

	_output[3].pin = ENABLE_4_Pin;
	_output[3].port = ENABLE_4_GPIO_Port;

	_output[4].pin = ENABLE_5_Pin;
	_output[4].port = ENABLE_5_GPIO_Port;

	_output[5].pin = ENABLE_6_Pin;
	_output[5].port = ENABLE_6_GPIO_Port;

	_output[6].pin = ENABLE_CO_1_Pin;
	_output[6].port = ENABLE_CO_1_GPIO_Port;

	_output[7].pin = ENABLE_CO_2_Pin;
	_output[7].port = ENABLE_CO_2_GPIO_Port;

	_output[8].pin = ENABLE_CO_3_Pin;
	_output[8].port = ENABLE_CO_3_GPIO_Port;

	_output[9].pin = ENABLE_CO_4_Pin;
	_output[9].port = ENABLE_CO_4_GPIO_Port;

	for(int i = 0; i < NB_OUTPUTS; i++){
		_output[i].state = STATE_OFF;
		set_state_output(&_output[i]);
	}
}

void update_outputs(void){
	for(int i = 0; i < NB_OUTPUTS; i++){
		if(_output[i].perio == 0){
			_output[i].actual_time = 0;
		}else{
			_output[i].actual_time++;
			if(_output[i].actual_time > _output[i].perio * TIME_PERIO_PROP){
				_output[i].actual_time = 0;
				if(_output[i].state == STATE_ON){
					_output[i].state = STATE_OFF;
				}else{
					_output[i].state = STATE_ON;
				}
				set_state_output(&_output[i]);
			}
		}
	}
}

void set_state_output(output * _out){
	if(_out->state == STATE_ON){
		HAL_GPIO_WritePin(_out->port, _out->pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(_out->port, _out->pin, GPIO_PIN_RESET);
	}
}
