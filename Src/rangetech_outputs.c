/*
 * rangetech_pos.c
 *
 *  Created on: 08.04.2020
 *      Author: Bartosz
 */


#include "rangetech_outputs.h"


void init_range_outputs(void){
	init_led_state();
	init_outputs();
	init_corona(WIRED_MODE);
}

void loop_outputs_10ms(void){
	led_state_update();
	communication_step_it();
	update_outputs();

	switch(main_state){
	case _calib_communication:
		periode_led_state = PERIODE_CALIB_COM;
		calib_communication();
		break;
	case _work:
		periode_led_state = PERIODE_WORK;
		work();
		break;
	case _error:
		periode_led_state = PERIODE_ERROR;
		error();
		break;
	}
}

void calib_communication(void){
	static uint32_t counter_tmp = 0;
	static uint8_t data_tmp[4] = {
			0,0,0,0
	};

	switch(calib_com_state){
	case _calib_module_com:
		if(get_configured_flag() == 1){
			data_tmp[0] = rand_number() % 0xFF;
			data_tmp[1] = rand_number() % 0xFF;
			data_tmp[2] = rand_number() % 0xFF;
			data_tmp[3] = rand_number() % 0xFF;
			calib_com_state = _request_for_set_id_address;
		}
		break;
	case _request_for_set_id_address:
		if(get_device_address() == 0xFFFF){
			if(++counter_tmp == ADDRESS_REQUEST_PERIOD){
				counter_tmp = 0;
				transmit_event(EV_ASK_FOR_ADDRESS, MASTER_ADDRESS, data_tmp);
			}
		}else{
			calib_com_state = _request_for_start_msg;
		}
		break;
	case  _request_for_start_msg:
		if(get_unix_timestamp() == 0){
			if(++counter_tmp == START_REQUEST_PERIOD){
				counter_tmp = 0;
				data_tmp[0] = DEVICE_TYPE;
				transmit_event(EV_MANIFEST, MASTER_ADDRESS, data_tmp);
			}
		}else{
			main_state = _work;
		}
		break;
	}
}


void work(void){
}

void error(void){
}

void init_led_state(void){
	periode_led_state = PERIODE_CALIB_COM;
}

void led_state_update(void){
	if(++actual_counter_led_state >= periode_led_state){
		actual_counter_led_state = 0;
		if(actual_state_led_state == 0 || periode_led_state == 0){
			actual_state_led_state = 1;
		}else{
			actual_state_led_state = 0;
		}
	}

	if(actual_state_led_state == 1){
		HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_RESET);
	}

}
