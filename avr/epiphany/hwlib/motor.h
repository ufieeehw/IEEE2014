#ifndef MOTOR_H_
#define MOTOR_H_

#include <avr/io.h>

enum MotorDirection {
	MOTOR_NEUTRAL = 0,
	MOTOR_FORWARD = 1,
	MOTOR_BACKWARD = 2	
};

enum MotorNum {
	MOTOR_1,
	MOTOR_2,
	MOTOR_3,
	MOTOR_4	
};

void motor_init();
void motor_set_direction(MotorNum motor_num, MotorDirection direction);
bool motor_set_effort(MotorNum num, uint16_t new_effort);


#endif /* MOTOR_H_ */
