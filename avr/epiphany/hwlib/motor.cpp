#include "motor.h"

#include <avr/io.h>

/*
 *	Epiphany DIY Quad Motor configuration:
 *		PWM signals: PORTF -- CCA through CCD -- Motors 1, 2, 3, 4
 *		Motor direction: PORTK -- 1:0, 3:2, 5:4, 7:6 -- Motors 1, 2, 3, 4
 */

