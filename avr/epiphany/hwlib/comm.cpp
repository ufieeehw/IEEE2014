#include "comm.h"
#include <avr/io.h>
#include <string.h>

#include "uart.h"
static const char start_sentinel = '%';
static const uint8_t max_message_length = 128;

struct MessageData {
	uint8_t actual_length;
	char buffer[max_message_length];	
}; 

static MessageData message;
bool message_pending = false;

static void comm_message_transmit(MessageData m){
	uint8_t transmitted = 0;
	while(uart_put(start_sentinel) == false){}
	while(uart_put((char)(m.actual_length)) == false){}
		
	while(transmitted < m.actual_length){
		while(uart_put(m.buffer[transmitted]) == false){};
		transmitted++;
	}
}

bool comm_message_send(uint8_t msg_type, char* msg_body, uint8_t msg_length){
	if(message_pending || ((msg_length + 1) > max_message_length)){
		return false;
	}	
	
	message.actual_length = msg_length + 1;
	message.buffer[0] = (char)msg_type;
	memcpy(&(message.buffer[1]), msg_body, msg_length);
	
	message_pending = true;
	comm_message_transmit(message);
	message_pending = false;
	
	return true;
}
