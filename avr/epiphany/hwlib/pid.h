#ifndef PID_H_
#define PID_H_

#include "motor.h"

enum wheelNum{
	WHEEL1,
	WHEEL2,
	WHEEL3,
	WHEEL4
};

void pid_init();
void pid_setTunings(float Kp, float Ki, float Kd, wheelNum num);
float pid_getSpeed(wheelNum num);
void pid_setSpeed(float speed, wheelNum num);
void pid_set_speed_handler(char* message, uint8_t len);
void pid_get_odometry_handler(char* message, uint8_t len);
void pid_get_speed_multiplier_handler(char* messsage, uint8_t len);
void pid_set_speed_multiplier_handler(char* messsage, uint8_t len);


#endif /* PID_H_ */
