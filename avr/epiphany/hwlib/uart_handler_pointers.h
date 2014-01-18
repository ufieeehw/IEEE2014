/*
 * uart_handler_pointers.h
 *
 * Created: 1/11/2014 9:49:32 PM
 *  Author: Mason
 */ 


#ifndef UART_HANDLER_POINTERS_H_
#define UART_HANDLER_POINTERS_H_

#include "uart.h"
#include "pid.h"
#include "motor.h"
#include "clock.h"
#include "twi.h"
#include "mpu6050.h"


typedef void (*HandlerPointer)(char*, uint8_t);
extern HandlerPointer HandlerPointers[6] = {
	NULL,						// 0x00 AckValid
	NULL,						// 0x01 AckInvalid
	uart_echo_request,			// 0x02
	uart_echo_reply,			// 0x03
	pid_set_speed_handler,		// 0x04
	pid_get_odometry_handler	// 0x05
};



#endif /* UART_HANDLER_POINTERS_H_ */