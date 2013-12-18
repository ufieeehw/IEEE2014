#ifndef UART_H_
#define UART_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>

void uart_init();
bool uart_put(char ch);
int uart_get();
bool uart_send_msg_block(uint8_t type, char* msg);
bool uart_send_msg_noblock(uint8_t type, char* msg);

typedef enum RxStates_enum {
	WaitingForStart,
	GrabLength,
	ReceivingMessage,
	Processing
} RxStates_t;

typedef enum MessageTypes_enum {
	EchoRequest = '?',
	EchoReply = 0x01,
	ACKValid = '_',
	ACKInvalid = 0x03
}MessageTypes_t;

char* uart_get_msg(void);
extern bool message_to_be_handled;

#endif /* UART_H_ */
