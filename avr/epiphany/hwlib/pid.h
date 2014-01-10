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
int pid_convertSpeedToEffort(float speed, wheelNum num);
float pid_getSpeed(wheelNum num);
void pid_setSpeed(float speed, MotorDirection direction, wheelNum num);


#endif /* PID_H_ */
