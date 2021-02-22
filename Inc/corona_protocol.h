/*
 * com_korona.h
 *
 *  Created on: 14.04.2020
 *      Author: Bartosz
 */

#ifndef COM_KORONA_H_
#define COM_KORONA_H_
#include "main.h"
#include "rangetech_outputs.h"


#define WIRED_MODE					1
#define LORA_MODE					2
#define FSK_MODE					3

#define START_BAUDRATE 				9600
#define WORKING_BAUDRATE			115200

#define COM_OK						1
#define COM_ERROR 					2

#define UART_PERIPH					huart1
extern UART_HandleTypeDef 			UART_PERIPH;

#define MASTER_ADDRESS				1

#define ADDRESS_REQUEST_PERIOD		400		// [x 10ms]
#define START_REQUEST_PERIOD		400		// [x 10ms]

uint8_t update_flash_flag;
uint8_t group_address;
uint16_t device_address;
uint32_t unix_timestamp;
uint8_t unix_10ms;


#define ACTION 						1
#define EVENT						2
#define START_BYTE					0xBB
#define NUMBER_OF_PREAMBLE_BYTES	7	// start 1b | group 1b | sender 2b | receiver 2b | action 1b
#define NUMBER_OF_CRC_BYTES			2
uint16_t crc16_tab[256];
struct preamble_params{
	uint8_t start_byte;
	uint8_t group_address;
	uint16_t sender_address;
	uint16_t receiver_address;
	uint8_t action;
	uint8_t event;

	uint8_t action_or_event;
	uint8_t table_position;
	uint8_t data_request_size;
	uint8_t data_response_size;
}preamble_params_rx, preamble_params_tx;



#define TIMEOUT_FRAME_CYCLE			3	// * 10 ms
#define TIMEOUT_TRANSACTION			8	// * 10 ms
#define MAX_RETRANSMISION_NUMBER	4

struct communication_states{
	uint8_t communication_mode;
	uint8_t configured_com_flag;
	uint8_t msg_tx_pending_flag;
	uint8_t sending_response_flag;
	uint8_t event_to_transmision_flag;
	uint8_t transaction_pending_flag;
	uint8_t number_of_retransmision;
	uint8_t timeout_frame_cycle;
	uint8_t number_of_lost_msg;
	uint8_t timeout_transaction;
	uint32_t baudrate;

	uint8_t dist_info_flag;
	uint8_t end_calib_flag;
	uint8_t	drive_error_flag;
	uint8_t low_tension_flag;
}com_states;


#define DEVICE_TYPE					6 //OUTPUTS

/*	MASTER -> SLAVE (ACTION)	*/
#define AC_SCAN						1
#define AC_SET_DEVICE_ADDRESSS		2
#define AC_SET_GROUP_ADDRESS		3
#define AC_GET_COMMUNICATION_ERROR	4

#define AC_SET_OUTPUT				19
#define AC_TOGGLE_OUTPUT			20
#define AC_PWM_OUTPUT				21
#define AC_GET_OUTPUT_STATES		22

//#define

/*	SLAVE -> MASTER	(EVENT) */
#define EV_MANIFEST					255
#define	EV_ASK_FOR_ADDRESS			254

//#define


extern TIM_HandleTypeDef htim17;

#define START_FLASH_ADDRESS 		0x08000000
#define FLASH_SIZE__				0x20000
#define GROUP_ADDRESS_OFFSET		0x10
#define	DEVICE_ADDRESS_OFFSET		0x20

#define BASE_GROUP_ADDRESS			0x0D


#define BUFFER_RX_SIZE				400
#define BUFFER_TX_SIZE				100
#define BUFFER_RESPONSE_SIZE		100

uint8_t data_uart_rx;
uint8_t buffer_rx[BUFFER_RX_SIZE];
uint16_t head_position_buffer_rx;
uint16_t tail_position_buffer_rx;

uint8_t buffer_tx[BUFFER_TX_SIZE];
uint16_t head_position_buffer_tx;

uint8_t buffer_response[BUFFER_RESPONSE_SIZE];
uint16_t head_position_buffer_response;


const uint8_t new_line_character;
const uint8_t carriage_return_character;
const char* ok_config_response;
static const char* config_lora_command[];
static const char* config_fsk_command[];


#define TIME_ACTIVE_LED			5

typedef struct{
	uint8_t state;
	uint8_t counter;

	uint16_t pin;
	GPIO_TypeDef * port;
}led_communication;

led_communication led_rx, led_tx;


void init_corona(uint8_t mode);

void read_byte_it(void);
void end_of_transmission_it(void);
void communication_step_it(void);

void calibration_module(void);
void set_config_command_to_buffer_tx(char* table);
void set_new_line_at_end_buffer_tx(void);
uint8_t get_config_response_ok(void);

void receive(void);
void receive_response(void);
void receive_action(void);
void remove_last_byte_rx(void);
void remove_all_bytes_rx(void);
uint16_t get_number_of_ready_bytes_rx(void);
uint8_t get_preamble_rx_params(void);
void set_preamble_to_response(void);
void do_event(void);
void do_action(void);
void send_response(void);

uint8_t transmit_event(uint8_t event, uint16_t address, uint8_t *data);
uint8_t set_preamble_tx_params(uint8_t event, uint16_t address);
void send_event(void);
void retransmit_event(void);
void update_unix_timestamp(void);

uint8_t get_group_address(void);
void set_group_address(uint8_t address);
uint16_t get_device_address(void);
void set_device_address(uint16_t address);
void update_flash_param(void);
uint32_t get_unix_timestamp(void);
uint8_t get_time_10ms(void);
void set_unix_timestamp(uint32_t time, uint8_t time_10ms);
uint8_t get_configured_flag(void);
void set_uart_baudrate(uint32_t baudrate);

void active_led(led_communication * led);
void update_led(led_communication * led);

uint8_t check_rx_crc(uint8_t data_size);
void set_crc_to_response(void);
uint16_t calculate_crc16(uint8_t *data_buffer, uint16_t buffer_size, uint16_t start_frame_position, uint16_t frame_size);

#endif /* COM_KORONA_H_ */
