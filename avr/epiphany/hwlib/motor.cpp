#include "motor.h"

#include <avr/io.h>
#include <math.h>

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

static TC0_t &tc_motor = TCF0;
static PORT_t &port_pwm = PORTF;
static PORT_t &port_direction = PORTK; 

void motor_init(){
	
	tc_motor.PER = motor_max_effort; // max effort corresponds to 100% duty cycle, and PER for single slope PWM
	tc_motor.CTRLA = TC_CLKSEL_DIV1_gc; // use clkPER/1 for maximum resolution
	tc_motor.CTRLB = TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC_WGMODE_SS_gc;
	
	motor_set_direction(MOTOR_1, MOTOR_NEUTRAL);
	motor_set_direction(MOTOR_2, MOTOR_NEUTRAL);
	motor_set_direction(MOTOR_3, MOTOR_NEUTRAL);
	motor_set_direction(MOTOR_4, MOTOR_NEUTRAL);

	port_direction.DIRSET = 0xFF;
	port_pwm.DIRSET = 0x0F;
}

void motor_set_direction(MotorNum motor_num, MotorDirection direction){
	if(motor_num == MOTOR_1 || motor_num == MOTOR_3) {
		if(direction == MOTOR_FORWARD) direction = MOTOR_BACKWARD;
		else if(direction == MOTOR_BACKWARD) direction = MOTOR_FORWARD;
	}
	uint8_t current_directions = port_direction.IN;
	uint8_t clear_mask = ~(0b11 << (2*motor_num));
	
	port_direction.OUT = (current_directions & clear_mask) | (direction << (2*motor_num));
}

bool motor_set_effort(MotorNum num, uint16_t new_effort){
	if(new_effort > 1024){
		//return false;
		new_effort = 1024;
	}
	
	switch(num){
		case MOTOR_1:
			tc_motor.CCA = new_effort;
			break;
		case MOTOR_2:
			tc_motor.CCB = new_effort;
			break;
		case MOTOR_3:
			tc_motor.CCC = new_effort;
			break;
		case MOTOR_4:
			tc_motor.CCD = new_effort;
			break;
		default:
			return false;
	}
	
	return true;
}

void motor_set_velocity(MotorNum motor_num, float velocity) {
	if(velocity < 0) {
		motor_set_direction(motor_num, MOTOR_BACKWARD);
	} else {
		motor_set_direction(motor_num, MOTOR_FORWARD);
	}
	float speed = fabs(velocity);
	if(speed >= 1023) speed = 1023;
	motor_set_effort(motor_num, (uint16_t)speed);
}
