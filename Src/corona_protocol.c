/*
 * com_korona.c
 *
 *  Created on: 14.04.2020
 *      Author: Bartosz
 */

/**Sposob impelentacji protokolu corona w dowolnym projekcie:
 * 1. Ustawic odpowiednie peryferium uart: #define UART_PERIPH		huartn
 * 		gdzie: n = 1,2,...
 * 2. Ustawic odpowiedni numer urzadzenia: #define DEVICE_TYPE		n
 * 		gdzie: n = 1 //pos, n = 2 //ad
 * 3. Ustawic odpowiedni rozmiar pamieci flash stm32: #define FLASH_SIZE	n
 * 		np. dla STM32F411RC n = 0x40000
 *
 * 4. Umiescic funkcje z koncowka "_it" w odpowiednich przerwaniach:
 *		"read_byte_it()" - przerwanie od odczytania 1 bajtu przez peryferium huartn:
 *			HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 *		"end_of_transmission_it()" - przerwanie od zakonczenia wyslania calej ramki danych:
 *			HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
 *		"communication_step_it()" - przerwanie wykonywane cyklicznie z okresem 10ms:
 *			HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 * 5. Umiescic funkcje "init_corona(mode)" w miejscu implementacji programu
 * 		mode = WIRED_MODE, LORA_MODE lub FSK_MODE
 *
 **Sposob wyslania eventu:
 * 		nalezy skorzystac z funkcji: transmit_event(event, address, *data)
 * 		gdzie:
 * 			event - #define odpowiedniego eventu
 *			address - adres odbiorcy
 *			*data - wskaznik na tablice danych do wyslania
 *		funkcja zwraca:
 *			COM_OK - w przypadku gdy odpowiednia ramka danych zostala dodana do buforu nadawczego
 *			COM_ERROR - kiedy nie zakonczyla sie poprzednia tranzakcja, wiec nie mozliwe jest nadpisanie buforu nadawczego
 *
 **Sposob rozszerzania protokolu corona:
 * 	jesli chcemy dodac akcje o nowym numerze czynnosci nalezy:
 * 		1. Dodac #define z przedrostkiem "AC_" w dziale "MASTER -> SLAVE (ACTION)" z unikatowym numerm czynnosci
 * 		2. Dodac pozycje w "action_table[][3]":
 * 			#1 pozycja - nowo stworzony #define w punkcie 1.
 * 			#2 pozycja - ilosc danych odbieranych (request)
 * 			#3 pozycja - ilosc danych wysylanych (response)
 * 		3. Nalezy dodac pozycje w funkcji do_action() z odpowiednia reakcja na nowa akcje
 *
 * 	jesli chcemy dodac event o nowym numerze czynnosci nalezy:
 * 		1. Dodac #define z przedrostkiem "EV_" w dziale "SLAVE -> MASTER (EVENT)" z unikatowym numerm czynnosci
 * 		2. Dodac pozycje w "event_table[][3]":
 * 			#1 pozycja - nowo stworzony #define w punkcie 1.
 * 			#2 pozycja - ilosc danych wysylanych (request)
 * 			#3 pozycja - ilosc danych odbieranych (event)
 * 		3. Nalezy dodac pozycje w funkcji do_event() z odpowiednia reakcja na odpowiedz nowego eventu
 *
 *
 * */

#include "corona_protocol.h"

uint32_t debug_tmp;
HAL_StatusTypeDef status_debug;

uint8_t action_table[][3]={	// action, num request bytes, num response bytes
		{AC_SCAN, 						0, 1},
		{AC_SET_DEVICE_ADDRESSS, 		2, 2},
		{AC_SET_GROUP_ADDRESS, 			1, 1},
		{AC_GET_COMMUNICATION_ERROR, 	0, 1},
		{AC_SET_OUTPUT, 				2, 0},
		{AC_TOGGLE_OUTPUT, 				1, 0},
		{AC_PWM_OUTPUT, 				2, 0},
		{AC_GET_OUTPUT_STATES, 			0, 10}
};

uint8_t event_table[][3]={ // event, num request bytes, num response bytes
		{EV_MANIFEST,					1, 7},
		{EV_ASK_FOR_ADDRESS,			4, 6}
};


void init_corona(uint8_t mode){
	led_rx.port = GREEN_LED_GPIO_Port;
	led_rx.pin = GREEN_LED_Pin;

	led_tx.port = RED_LED_GPIO_Port;
	led_tx.pin = RED_LED_Pin;

	com_states.communication_mode = mode;

	if(com_states.communication_mode == WIRED_MODE){
		com_states.configured_com_flag = 1;
	}

	group_address = *(uint32_t*)(START_FLASH_ADDRESS + FLASH_SIZE__ - GROUP_ADDRESS_OFFSET);
	device_address = *(uint32_t*)(START_FLASH_ADDRESS + FLASH_SIZE__ - DEVICE_ADDRESS_OFFSET);

	HAL_Delay(1);

	if(group_address == 0xFF){
		set_group_address(BASE_GROUP_ADDRESS);
	}

	HAL_UART_Receive_IT(&UART_PERIPH, &data_uart_rx, 1);
}

void read_byte_it(void){
	com_states.timeout_frame_cycle = 0;
	buffer_rx[head_position_buffer_rx] = data_uart_rx;
	if(++head_position_buffer_rx == BUFFER_RX_SIZE) head_position_buffer_rx = 0;
	if(HAL_UART_Receive_IT(&UART_PERIPH, &data_uart_rx, 1) == HAL_BUSY){
		com_states.receive_it_flag = 1;
	}
}

void end_of_transmission_it(void){
	HAL_GPIO_WritePin(UART_EN_GPIO_Port, UART_EN_Pin, GPIO_PIN_RESET);
	com_states.msg_tx_pending_flag = 0;
	if(com_states.configured_com_flag == 1){
		if(com_states.sending_response_flag == 1){
			com_states.msg_tx_pending_flag = 1;
			com_states.sending_response_flag = 0;
			HAL_GPIO_WritePin(UART_EN_GPIO_Port, UART_EN_Pin, GPIO_PIN_SET);
			HAL_UART_Transmit_IT(&UART_PERIPH, buffer_response, head_position_buffer_response);
			active_led(&led_tx);
		} else if(com_states.event_to_transmision_flag == 1){
			com_states.msg_tx_pending_flag = 1;
			com_states.event_to_transmision_flag = 0;
			HAL_GPIO_WritePin(UART_EN_GPIO_Port, UART_EN_Pin, GPIO_PIN_SET);
			HAL_UART_Transmit_IT(&UART_PERIPH, buffer_tx, head_position_buffer_tx);
			active_led(&led_tx);
		}
	}
}

void communication_step_it(void){
	if(com_states.configured_com_flag == 1){
		update_led(&led_rx);
		update_led(&led_tx);
		receive();
		retransmit_event();
		update_unix_timestamp();

		if(com_states.receive_it_flag){
			if(HAL_UART_Receive_IT(&UART_PERIPH, &data_uart_rx, 1) == HAL_OK){
				com_states.receive_it_flag = 0;
			}
		}
	}
}

void set_config_command_to_buffer_tx(char* table){
	uint8_t iter = 0;
	while(table[iter] != 0){
		buffer_tx[iter] = table[iter];
		iter++;
	}
	head_position_buffer_tx = iter;
}

void set_new_line_at_end_buffer_tx(void){
	buffer_tx[head_position_buffer_tx] = new_line_character;
	head_position_buffer_tx++;
}

uint8_t get_config_response_ok(void){
	if(get_number_of_ready_bytes_rx() >= 4){
		if(buffer_rx[tail_position_buffer_rx] == ok_config_response[0] &&
				buffer_rx[(tail_position_buffer_rx + 1) % BUFFER_RX_SIZE] == ok_config_response[1] &&
				buffer_rx[(tail_position_buffer_rx + 2) % BUFFER_RX_SIZE] == carriage_return_character &&
				buffer_rx[(tail_position_buffer_rx + 3) % BUFFER_RX_SIZE] == new_line_character){
			remove_all_bytes_rx();
			return COM_OK;
		}else{
			remove_last_byte_rx();
		}
	}
	return COM_ERROR;
}

void receive(void){
	uint8_t tmp_data_size = 0;
	while(get_number_of_ready_bytes_rx() > 0 && buffer_rx[tail_position_buffer_rx] != START_BYTE){
		remove_last_byte_rx();
	}

	if(get_number_of_ready_bytes_rx() >= NUMBER_OF_PREAMBLE_BYTES){
		if(get_preamble_rx_params() == COM_ERROR){
			remove_last_byte_rx();
			return;
		}
		if(preamble_params_rx.action_or_event == ACTION){
			tmp_data_size = preamble_params_rx.data_request_size;
		} else if(preamble_params_rx.action_or_event == EVENT){
			tmp_data_size = preamble_params_rx.data_response_size;
		}
		if(get_number_of_ready_bytes_rx() >= NUMBER_OF_PREAMBLE_BYTES + tmp_data_size + NUMBER_OF_CRC_BYTES){
			if(check_rx_crc(tmp_data_size) == COM_ERROR){
				remove_last_byte_rx();
				return;
			}else{
				active_led(&led_rx);
				if(preamble_params_rx.action_or_event == EVENT && preamble_params_tx.event == preamble_params_rx.event){
					receive_response();
				}else if(preamble_params_rx.action_or_event == ACTION){
					receive_action();
				}
				remove_all_bytes_rx();
			}
		}
	}

	if(com_states.timeout_frame_cycle == TIMEOUT_FRAME_CYCLE){
		remove_all_bytes_rx();
	} else {
		com_states.timeout_frame_cycle ++;
	}
}

void receive_response(void){
	do_event();
	com_states.timeout_transaction = 0;
	com_states.transaction_pending_flag = 0;
	com_states.event_to_transmision_flag = 0;
	com_states.number_of_retransmision = 0;
}

void receive_action(void){
	set_preamble_to_response();
	do_action();
	set_crc_to_response();
	com_states.sending_response_flag = 1;
	send_response();
}

void remove_last_byte_rx(void){
	tail_position_buffer_rx++;
	if(tail_position_buffer_rx == BUFFER_RX_SIZE) tail_position_buffer_rx = 0;
}

void remove_all_bytes_rx(void){
	tail_position_buffer_rx = head_position_buffer_rx;
}

uint16_t get_number_of_ready_bytes_rx(void){
	if(tail_position_buffer_rx > head_position_buffer_rx){
		return (uint16_t)(BUFFER_RX_SIZE - tail_position_buffer_rx + head_position_buffer_rx);
	} else if(tail_position_buffer_rx < head_position_buffer_rx){
		return (uint16_t)(head_position_buffer_rx - tail_position_buffer_rx);
	} else {
		return 0;
	}
}

uint8_t get_preamble_rx_params(void){
	uint32_t size_tab;
	preamble_params_rx.start_byte = buffer_rx[tail_position_buffer_rx];
	preamble_params_rx.group_address = buffer_rx[(tail_position_buffer_rx + 1) % BUFFER_RX_SIZE];
	preamble_params_rx.sender_address = buffer_rx[(tail_position_buffer_rx + 2) % BUFFER_RX_SIZE] << 8 |
			buffer_rx[(tail_position_buffer_rx + 3) % BUFFER_RX_SIZE];
	preamble_params_rx.receiver_address = buffer_rx[(tail_position_buffer_rx + 4) % BUFFER_RX_SIZE] << 8 |
				buffer_rx[(tail_position_buffer_rx + 5) % BUFFER_RX_SIZE];
	preamble_params_rx.action = buffer_rx[(tail_position_buffer_rx + 6) % BUFFER_RX_SIZE];
	preamble_params_rx.event = buffer_rx[(tail_position_buffer_rx + 6) % BUFFER_RX_SIZE];

	if(preamble_params_rx.group_address != group_address) return COM_ERROR;
	if(preamble_params_rx.receiver_address != device_address) return COM_ERROR;
	size_tab = sizeof(action_table)/sizeof(action_table[0]);
	for(int i = 0; i < size_tab; i++){
		if(preamble_params_rx.action == action_table[i][0]){
			preamble_params_rx.table_position = i;
			preamble_params_rx.data_request_size = action_table[i][1];
			preamble_params_rx.data_response_size = action_table[i][2];
			preamble_params_rx.action_or_event = ACTION;
			return COM_OK;
		}
	}
	size_tab = sizeof(event_table)/sizeof(event_table[0]);
	for(int i = 0; i < size_tab; i++){
		if (preamble_params_rx.event == event_table[i][0]){
			preamble_params_rx.table_position = i;
			preamble_params_rx.data_request_size = event_table[i][1];
			preamble_params_rx.data_response_size = event_table[i][2];
			preamble_params_rx.action_or_event = EVENT;
			return COM_OK;
		}
	}
	return COM_ERROR;
}


void set_preamble_to_response(void){
	buffer_response[0] = START_BYTE;
	buffer_response[1] = group_address;
	buffer_response[2] = (device_address >> 8) & 0xFF;
	buffer_response[3] = device_address & 0xFF;
	buffer_response[4] = (preamble_params_rx.sender_address >> 8) & 0xFF;
	buffer_response[5] = preamble_params_rx.sender_address & 0xFF;
	buffer_response[6] = preamble_params_rx.action;
	head_position_buffer_response = NUMBER_OF_PREAMBLE_BYTES + preamble_params_rx.data_response_size + NUMBER_OF_CRC_BYTES;
}

void do_event(void){
	uint16_t start_idx_data = (tail_position_buffer_rx + NUMBER_OF_PREAMBLE_BYTES) % BUFFER_RX_SIZE;
	uint16_t new_tmp_address;
	switch(preamble_params_rx.event){
	case EV_MANIFEST:
		new_tmp_address = (uint16_t)((buffer_rx[(start_idx_data + 6) % BUFFER_RX_SIZE]) |
				buffer_rx[(start_idx_data + 5) % BUFFER_RX_SIZE] << 8);
		if(new_tmp_address == get_device_address()){
			set_unix_timestamp(buffer_rx[(start_idx_data) % BUFFER_RX_SIZE] << 24 |
							   buffer_rx[(start_idx_data + 1) % BUFFER_RX_SIZE] << 16|
							   buffer_rx[(start_idx_data + 2) % BUFFER_RX_SIZE] << 8 |
							   buffer_rx[(start_idx_data + 3) % BUFFER_RX_SIZE],
							   buffer_rx[(start_idx_data + 4) % BUFFER_RX_SIZE]);
		}else{
			set_device_address(new_tmp_address);
		}
		break;

	case EV_ASK_FOR_ADDRESS:
		if(buffer_rx[(start_idx_data) % BUFFER_RX_SIZE] == buffer_tx[NUMBER_OF_PREAMBLE_BYTES] &&
				buffer_rx[(start_idx_data + 1) % BUFFER_RX_SIZE] == buffer_tx[NUMBER_OF_PREAMBLE_BYTES + 1] &&
				buffer_rx[(start_idx_data + 2) % BUFFER_RX_SIZE] == buffer_tx[NUMBER_OF_PREAMBLE_BYTES + 2] &&
				buffer_rx[(start_idx_data + 3) % BUFFER_RX_SIZE] == buffer_tx[NUMBER_OF_PREAMBLE_BYTES + 3]){
			set_device_address((uint16_t)((buffer_rx[(start_idx_data + 5) % BUFFER_RX_SIZE]) |
					buffer_rx[(start_idx_data + 4) % BUFFER_RX_SIZE] << 8));
		}
		break;
	}
}

void do_action(void){
	uint16_t start_idx_data = (tail_position_buffer_rx + NUMBER_OF_PREAMBLE_BYTES) % BUFFER_RX_SIZE;
	switch(preamble_params_rx.action){
	case AC_SCAN:
		buffer_response[NUMBER_OF_PREAMBLE_BYTES] = DEVICE_TYPE;
		buffer_response[NUMBER_OF_PREAMBLE_BYTES + 1] = (device_address >> 8) & 0xFF;
		buffer_response[NUMBER_OF_PREAMBLE_BYTES + 2] = device_address & 0xFF;
		break;

	case AC_SET_DEVICE_ADDRESSS:
		unix_timestamp = 0;
		main_state = _calib_communication;
		calib_com_state = _request_for_start_msg;
		set_device_address(buffer_rx[(start_idx_data + 1) % BUFFER_RX_SIZE] | (uint16_t)(buffer_rx[start_idx_data] << 8));
		buffer_response[NUMBER_OF_PREAMBLE_BYTES] = buffer_rx[start_idx_data];
		buffer_response[NUMBER_OF_PREAMBLE_BYTES + 1] = buffer_rx[(start_idx_data + 1) % BUFFER_RX_SIZE];
		break;

	case AC_SET_GROUP_ADDRESS:
		set_group_address(buffer_rx[start_idx_data]);
		buffer_response[NUMBER_OF_PREAMBLE_BYTES] = buffer_rx[start_idx_data];
		break;

	case AC_GET_COMMUNICATION_ERROR:
		buffer_response[NUMBER_OF_PREAMBLE_BYTES] = com_states.number_of_lost_msg;
		com_states.number_of_lost_msg = 0;
		break;

	case AC_SET_OUTPUT:
		if(main_state == _work){
			if(buffer_rx[start_idx_data] <= NB_OUTPUTS){
				_output[buffer_rx[start_idx_data] - 1].perio = 0;
				if(buffer_rx[(start_idx_data + 1) % BUFFER_RX_SIZE] == STATE_ON){
					_output[buffer_rx[start_idx_data] - 1].state = STATE_ON;
				}else if(buffer_rx[(start_idx_data + 1) % BUFFER_RX_SIZE] == STATE_OFF){
					_output[buffer_rx[start_idx_data] - 1].state = STATE_OFF;
				}
				set_state_output(&_output[buffer_rx[start_idx_data] - 1]);
			}
		}
		break;

	case AC_TOGGLE_OUTPUT:
		if(main_state == _work){
			if(buffer_rx[start_idx_data] <= NB_OUTPUTS){
				_output[buffer_rx[start_idx_data] - 1].perio = 0;
				if(_output[buffer_rx[start_idx_data] - 1].state == STATE_ON){
					_output[buffer_rx[start_idx_data] - 1].state = STATE_OFF;
				}else{
					_output[buffer_rx[start_idx_data] - 1].state = STATE_ON;
				}
				set_state_output(&_output[buffer_rx[start_idx_data] - 1]);
			}
		}
		break;

	case AC_PWM_OUTPUT:
		if(main_state == _work){
			if(buffer_rx[start_idx_data] <= NB_OUTPUTS){
				_output[buffer_rx[start_idx_data] - 1].perio = buffer_rx[(start_idx_data + 1) % BUFFER_RX_SIZE];
			}
		}
		break;

	case AC_GET_OUTPUT_STATES:
		if(main_state == _work){
			for(int i = 0; i < NB_OUTPUTS; i ++){
				buffer_response[NUMBER_OF_PREAMBLE_BYTES + i] = _output[i].state;
			}
		}
		break;
	}
}

void send_response(void){
	if(com_states.msg_tx_pending_flag == 0){
		com_states.sending_response_flag = 0;
		com_states.msg_tx_pending_flag = 1;
		HAL_GPIO_WritePin(UART_EN_GPIO_Port, UART_EN_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit_IT(&UART_PERIPH, buffer_response, head_position_buffer_response);
		active_led(&led_tx);
	}
}


uint8_t transmit_event(uint8_t event, uint16_t address, uint8_t *data){
	uint16_t crc_16;
	if(com_states.transaction_pending_flag == 0 && com_states.configured_com_flag == 1){
		com_states.transaction_pending_flag = 1;

		if(set_preamble_tx_params(event, address) == COM_ERROR){
			return COM_ERROR;
		}

		buffer_tx[0] = START_BYTE;
		buffer_tx[1] = group_address;
		buffer_tx[2] = (device_address >> 8) & 0xFF;
		buffer_tx[3] = device_address & 0xFF;
		buffer_tx[4] = (address >> 8) & 0xFF;
		buffer_tx[5] = address & 0xFF;
		buffer_tx[6] = event;
		for(int i = 0; i < preamble_params_tx.data_request_size; i++){
			buffer_tx[7 + i] = data[i];
		}
		crc_16 = calculate_crc16(buffer_tx, BUFFER_TX_SIZE, 0, NUMBER_OF_PREAMBLE_BYTES + preamble_params_tx.data_request_size);
		buffer_tx[NUMBER_OF_PREAMBLE_BYTES + preamble_params_tx.data_request_size] = (crc_16 >> 8) & 0xFF;
		buffer_tx[NUMBER_OF_PREAMBLE_BYTES + preamble_params_tx.data_request_size + 1] = crc_16 & 0xFF;
		head_position_buffer_tx = NUMBER_OF_PREAMBLE_BYTES + preamble_params_tx.data_request_size + NUMBER_OF_CRC_BYTES;
		com_states.event_to_transmision_flag = 1;
		send_event();
		return COM_OK;

	}else{
		return COM_ERROR;
	}
}

uint8_t set_preamble_tx_params(uint8_t event, uint16_t address){
	uint32_t size_tab;
	preamble_params_tx.start_byte = START_BYTE;
	preamble_params_tx.group_address = group_address;
	preamble_params_tx.sender_address = device_address;
	preamble_params_tx.receiver_address = address;
	preamble_params_tx.event = event;
	size_tab = sizeof(event_table)/sizeof(event_table[0]);
	for(int i = 0; i < size_tab; i++){
		if(preamble_params_tx.event == event_table[i][0]){
			preamble_params_tx.table_position = i;
			preamble_params_tx.data_request_size = event_table[i][1];
			preamble_params_tx.data_response_size = event_table[i][2];
			preamble_params_tx.action_or_event = EVENT;

			return COM_OK;
		}
	}
	return COM_ERROR;
}

void send_event(void){
	if(com_states.msg_tx_pending_flag == 0 && com_states.sending_response_flag == 0){
		com_states.msg_tx_pending_flag = 1;
		com_states.event_to_transmision_flag = 0;
		HAL_GPIO_WritePin(UART_EN_GPIO_Port, UART_EN_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit_IT(&UART_PERIPH, buffer_tx, head_position_buffer_tx);
		active_led(&led_tx);
	} else {
		com_states.event_to_transmision_flag = 1;
	}
}

void retransmit_event(void){
	if(com_states.transaction_pending_flag == 1 && com_states.event_to_transmision_flag == 0){
		if(com_states.timeout_transaction == TIMEOUT_TRANSACTION){
			com_states.number_of_retransmision ++;
			if(com_states.number_of_retransmision == MAX_RETRANSMISION_NUMBER + 1){
				com_states.timeout_transaction = 0;
				com_states.transaction_pending_flag = 0;
				com_states.event_to_transmision_flag = 0;
				com_states.number_of_retransmision = 0;
				com_states.number_of_lost_msg ++;
			}else{
				com_states.timeout_transaction = 0;
				if(preamble_params_tx.event != EV_MANIFEST && preamble_params_tx.event != EV_ASK_FOR_ADDRESS){
					send_event();
				}
			}
		}else{
			com_states.timeout_transaction ++;
		}
	}else{
		com_states.timeout_transaction = 0;
	}
}


void update_unix_timestamp(void){
	if(unix_timestamp != 0){
		if(++unix_10ms == 100){
			unix_10ms = 0;
			unix_timestamp ++;
		}
	}
}


uint8_t get_group_address(void){
	return group_address;
}

void set_group_address(uint8_t address){
	group_address = address;
	update_flash_flag = 1;
}

uint16_t get_device_address(void){
	return device_address;
}

void set_device_address(uint16_t address){
	device_address = address;
	update_flash_flag = 1;
}

void update_flash_param(void){
	FLASH_EraseInitTypeDef type_erase;
	if(update_flash_flag == 1 && com_states.msg_tx_pending_flag == 0){
		HAL_FLASH_Unlock();
		type_erase.TypeErase = FLASH_TYPEERASE_PAGES;
		type_erase.Page = FLASH_PAGE_NB - 1;
		type_erase.NbPages = 1;
		HAL_FLASHEx_Erase(&type_erase, &debug_tmp);
		status_debug = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, START_FLASH_ADDRESS + FLASH_SIZE__ - DEVICE_ADDRESS_OFFSET, (uint32_t)device_address);
		status_debug = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, START_FLASH_ADDRESS + FLASH_SIZE__ - GROUP_ADDRESS_OFFSET, group_address);
		HAL_FLASH_Lock();
		update_flash_flag = 0;
	}
}

uint32_t get_unix_timestamp(void){
	return unix_timestamp;
}

uint8_t get_time_10ms(void){
	return unix_10ms;
}

void set_unix_timestamp(uint32_t time, uint8_t time_10ms){
	unix_timestamp = time;
	unix_10ms = time_10ms;
}

uint8_t get_configured_flag(void){
	return com_states.configured_com_flag;
}

void set_uart_baudrate(uint32_t baudrate){
	com_states.baudrate = baudrate;
	UART_PERIPH.Init.BaudRate = com_states.baudrate;
	HAL_UART_Init(&UART_PERIPH);
	HAL_UART_Receive_IT(&UART_PERIPH, &data_uart_rx, 1);
}

void active_led(led_communication * led){
	HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
	led->counter = TIME_ACTIVE_LED;
	led->state = 1;
}

void update_led(led_communication * led){
	if(led->state == 1){
		if(led->counter > 0){
			led->counter--;
		}else{
			HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
			led->state = 0;
		}
	}
}

uint8_t check_rx_crc(uint8_t data_size){
	uint16_t position_crc = (tail_position_buffer_rx + NUMBER_OF_PREAMBLE_BYTES + data_size);
	uint16_t read_crc = buffer_rx[position_crc % BUFFER_RX_SIZE] << 8 | buffer_rx[(position_crc + 1) % BUFFER_RX_SIZE];
	uint16_t calculated_crc = calculate_crc16(buffer_rx, BUFFER_RX_SIZE, tail_position_buffer_rx,
			data_size + NUMBER_OF_PREAMBLE_BYTES);
	if(read_crc == calculated_crc)
		return COM_OK;
	else
		return COM_ERROR;
}

void set_crc_to_response(void){
	uint16_t crc_16 = calculate_crc16(buffer_response, BUFFER_RESPONSE_SIZE, 0, NUMBER_OF_PREAMBLE_BYTES + preamble_params_rx.data_response_size);
	buffer_response[NUMBER_OF_PREAMBLE_BYTES + preamble_params_rx.data_response_size] = (crc_16 >> 8) & 0xFF;
	buffer_response[NUMBER_OF_PREAMBLE_BYTES + preamble_params_rx.data_response_size + 1] = crc_16 & 0xFF;
}

uint16_t calculate_crc16(uint8_t *data_buffer, uint16_t buffer_size, uint16_t start_frame_position, uint16_t frame_size){
	uint16_t crc16 = 0;
	uint8_t tbl_idx;
	for(uint16_t i = 0; i < frame_size; i++){
		tbl_idx = ((crc16 >> 8) ^ data_buffer[(i + start_frame_position) % buffer_size]) & 0xFF;
		crc16 = (crc16_tab[tbl_idx] ^ (crc16 << 8)) & 0xFFFF;
	}

	return crc16;
}

uint16_t crc16_tab[256] = {
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
		0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
		0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
		0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
		0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
		0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
		0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
		0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
		0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

