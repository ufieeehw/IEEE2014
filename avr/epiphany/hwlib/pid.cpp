#include "PID.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Configuration Values
static TC0_t &pid_tick_timer = TCC0;
#define PID_TICK_OVF TCC0_OVF_vect
static const uint16_t pid_tick_timer_period = 32000; //Will generate an interrupt every 1ms
static const uint8_t AVG_LENGTH = 100;

// File-Scope Variables
volatile uint16_t average_CCA;
int measurements[AVG_LENGTH];

void pid_init() {
	
	// Initialize a motor frequency measurement timer.
	PORTE.DIRCLR = 0x01;
	PORTE.PIN0CTRL = PORT_ISC_BOTHEDGES_gc;
	TCE0.CTRLA = TC_CLKSEL_DIV64_gc;
	TCE0.CTRLB = TC0_CCAEN_bm;
	TCE0.CTRLC = 0x00;
	TCE0.CTRLD = TC_EVACT_FRQ_gc | TC_EVSEL_CH0_gc;
	TCE0.CTRLE = 0x00;
	TCE0.INTCTRLB = TC0_CCAINTLVL0_bm;
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTE_PIN0_gc;
	
	// Initialize the 1ms PID tick timer
	pid_tick_timer.CTRLA = TC_CLKSEL_DIV1_gc;	//Uses timer on portC since all timers on PORT E and D will be used for wheels.
	pid_tick_timer.CTRLB = 0x00;
	pid_tick_timer.CTRLC = 0x00;
	pid_tick_timer.CTRLD = 0x00;
	pid_tick_timer.CTRLE = 0x00;
	pid_tick_timer.PER = pid_tick_timer_period;		
	pid_tick_timer.INTCTRLA = TC_OVFINTLVL_LO_gc;
	
}

// Returns an average of the last AVG_LENGTH CCA values.
static uint16_t pid_measureCCA() {
	uint32_t average = 0;	
	
	// Go through the elements of the average matrix and move them up one place, towards the end.
	// Also, add them up so that we can later divide them to get their average.
	for(int i = 0; i < (AVG_LENGTH - 1); i++) {
		measurements[i + 1] = measurements[i];
		average += measurements[i];
	}
	// Get the latest measurement
	measurements[0] = TCE0.CCA;
	average += measurements[0]; // Stick the latest CCA measurement in the first spot
	average /= AVG_LENGTH;	// Average it all out.
	return (uint16_t)average;
}

ISR(PID_TICK_OVF) {
	// Insert code to be run during PID tick.
}

ISR(TCE0_CCA_vect){
	average_CCA = pid_measureCCA();
}