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
double pid_getSpeed(wheelNum num);


#endif /* PID_H_ */
