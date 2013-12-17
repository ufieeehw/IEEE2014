

#ifndef COMM_H_
#define COMM_H_

#include <avr/io.h>
#include <string.h>

bool comm_message_send(uint8_t msg_type, char* msg_body, uint8_t msg_length);



#endif /* COMM_H_ */
