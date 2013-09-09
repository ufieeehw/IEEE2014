#include "motor.h"

#include <avr/io.h>

/*
 *	Epiphany DIY Quad Motor configuration:
 *		PWM signals: PORTF -- CCA through CCD -- Motors 1, 2, 3, 4
 *		Motor direction: PORTK -- 1:0, 3:2, 5:4, 7:6 -- Motors 1, 2, 3, 4
 *
 *		Each PWM signal is connected to an enable on a STmicro L6205. Each of 
 *		those drivers supports two motors. There are two drivers onboard the 
 *		Epiphany board.
 *
 *
 *	Note: The Epiphany DIY motor drivers can control 2 "high power" motors as 
 *	well as 4 "regular" motors. We will use the "regular" motor configuration.
 */

static const int motor_max_effort = 1024; 
				// PWM duty cycles will be (effort/motor_max_effort)*100%.
				// We choose 1024 because: with a 32 MHz system (and peripheral) clock,
				// 32 MHz / 1024 timer ticks = PWM period of 1/(32 kHz) 
				//		i.e. 32 kHz switching frequency.


void motor_init(){
	
	TCF0.PER = motor_max_effort; // max effort corresponds to 100% duty cycle, and PER for single slope PWM
	TCF0.CTRLA = TC_CLKSEL_DIV1_gc; // use clkPER/1 for maximum resolution
	TCF0.CTRLB = TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC_WGMODE_SS_gc;
	
	motor_set_direction(MOTOR_1, MOTOR_NEUTRAL);
	motor_set_direction(MOTOR_2, MOTOR_NEUTRAL);
	motor_set_direction(MOTOR_3, MOTOR_NEUTRAL);
	motor_set_direction(MOTOR_4, MOTOR_NEUTRAL);

	PORTK.DIRSET = 0xFF;
	PORTF.DIRSET = 0x0F;
}

void motor_set_direction(MotorNum motor_num, MotorDirection direction){
	uint8_t current_directions = PORTK.IN;
	uint8_t clear_mask = ~(0b11 << (2*motor_num));
	
	PORTK.OUT = (current_directions & clear_mask) | (direction << (2*motor_num));
}

bool motor_set_effort(MotorNum num, uint16_t new_effort){
	if(new_effort > 1024){
		return false;
	}
	
	switch(num){
		case MOTOR_1:
			TCF0.CCA = new_effort;
			break;
		case MOTOR_2:
			TCF0.CCB = new_effort;
			break;
		case MOTOR_3:
			TCF0.CCC = new_effort;
			break;
		case MOTOR_4:
			TCF0.CCD = new_effort;
			break;
		default:
			return false;
	}
	
	return true;
}
	
