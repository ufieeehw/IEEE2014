#include "PID.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Configuration Values
static TC0_t &pid_tick_timer = TCC0;
#define PID_TICK_OVF TCC0_OVF_vect
static const uint16_t pid_tick_timer_period = 32000; //Will generate an interrupt every 1ms


void pid_init() {
	
	// Initialize the 1ms PID tick timer
	pid_tick_timer.CTRLA = TC_CLKSEL_DIV1_gc;	//Uses timer on portC since all timers on PORT E and D will be used for wheels.
	pid_tick_timer.CTRLB = 0x00;
	pid_tick_timer.CTRLC = 0x00;
	pid_tick_timer.CTRLD = 0x00;
	pid_tick_timer.CTRLE = 0x00;
	pid_tick_timer.PER = pid_tick_timer_period;		
	pid_tick_timer.INTCTRLA = TC_OVFINTLVL_LO_gc;
	
}

ISR(PID_TICK_OVF) {
	// Insert code to be run during PID tick.
}
