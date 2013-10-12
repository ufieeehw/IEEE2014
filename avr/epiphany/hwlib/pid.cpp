#include "PID.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Configuration Values
static TC0_t &pid_tick_timer = TCC0;
#define PID_TICK_OVF TCC0_OVF_vect
static const uint16_t pid_tick_timer_period = 32000; //Will generate an interrupt every 1ms
static const uint8_t AVG_ARRAY_LENGTH = 5;
static const double CCA_TO_RAD_MULTIPLIER = 6770.674;

// File-Scope Variables and Structures
struct pid_wheel_data {
	int AVG_measurements[AVG_ARRAY_LENGTH];
	volatile int AVG_measurement_array_position;
	volatile double AVG_speed;
} ;

static pid_wheel_data wheelData[4];

void pid_init() {	
	// Initialize motor frequency measurement timers.
	PORTE.DIRCLR = 0x02;
	PORTE.PIN0CTRL = PORT_ISC_BOTHEDGES_gc;
	TCE0.CTRLA = TC_CLKSEL_DIV64_gc;
	TCE0.CTRLB = TC0_CCAEN_bm;
	TCE0.CTRLC = 0x00;
	TCE0.CTRLD = TC_EVACT_FRQ_gc | TC_EVSEL_CH0_gc;
	TCE0.CTRLE = 0x00;
	TCE0.INTCTRLB = TC0_CCAINTLVL1_bm;
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTE_PIN0_gc;

	PORTE.PIN1CTRL = PORT_ISC_BOTHEDGES_gc;
	TCE1.CTRLA = TC_CLKSEL_DIV64_gc;
	TCE1.CTRLB = TC1_CCAEN_bm;
	TCE1.CTRLC = 0x00;
	TCE1.CTRLD = TC_EVACT_FRQ_gc | TC_EVSEL_CH1_gc;
	TCE1.CTRLE = 0x00;
	TCE1.INTCTRLB = TC1_CCAINTLVL1_bm;
	EVSYS.CH1MUX = EVSYS_CHMUX_PORTE_PIN1_gc;
	
	PORTD.DIRCLR = 0x02;
	PORTD.PIN0CTRL = PORT_ISC_BOTHEDGES_gc;
	TCD0.CTRLA = TC_CLKSEL_DIV64_gc;
	TCD0.CTRLB = TC0_CCAEN_bm;
	TCD0.CTRLC = 0x00;
	TCD0.CTRLD = TC_EVACT_FRQ_gc | TC_EVSEL_CH2_gc;
	TCD0.CTRLE = 0x00;
	TCD0.INTCTRLB = TC1_CCAINTLVL1_bm;
	EVSYS.CH2MUX = EVSYS_CHMUX_PORTD_PIN0_gc;

	PORTD.PIN1CTRL = PORT_ISC_BOTHEDGES_gc;
	TCD1.CTRLA = TC_CLKSEL_DIV64_gc;
	TCD1.CTRLB = TC1_CCAEN_bm;
	TCD1.CTRLC = 0x00;
	TCD1.CTRLD = TC_EVACT_FRQ_gc | TC_EVSEL_CH3_gc;
	TCD1.CTRLE = 0x00;
	TCD1.INTCTRLB = TC1_CCAINTLVL1_bm;
	EVSYS.CH3MUX = EVSYS_CHMUX_PORTD_PIN1_gc;
	
	// Initialize the 1ms PID tick timer
	pid_tick_timer.CTRLA = TC_CLKSEL_DIV1_gc;	//Uses timer on portC since all timers on PORT E and D will be used for wheels.
	pid_tick_timer.CTRLB = 0x00;
	pid_tick_timer.CTRLC = 0x00;
	pid_tick_timer.CTRLD = 0x00;
	pid_tick_timer.CTRLE = 0x00;
	pid_tick_timer.PER = pid_tick_timer_period;
	pid_tick_timer.INTCTRLA = TC_OVFINTLVL_LO_gc;
	
}

static void pid_measureSpeed(uint16_t measured_CCA, wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	
	uint16_t tempAvgVal = 0;
	
	// Array operates as FIFO, w/ elements added to end of array, and values shifted forward to make room for new value.
	if(data.AVG_measurement_array_position == AVG_ARRAY_LENGTH) {
		for(int i = 0; i < AVG_ARRAY_LENGTH - 1; i++)
			data.AVG_measurements[i] = data.AVG_measurements[i+1];
		data.AVG_measurement_array_position--;
	}
	data.AVG_measurements[data.AVG_measurement_array_position++] = measured_CCA;
	
	for(int i = 0; i < data.AVG_measurement_array_position; i++)
		tempAvgVal += data.AVG_measurements[i];
	
	tempAvgVal /= (double)data.AVG_measurement_array_position;
	data.AVG_speed = (1.0/tempAvgVal)*CCA_TO_RAD_MULTIPLIER;
}

double pid_getSpeed(wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	return data.AVG_speed;
}

ISR(PID_TICK_OVF) {
	// Insert code to be run during PID tick.
}

ISR(TCE0_CCA_vect){
	pid_measureSpeed(TCE0.CCA, WHEEL1);
}

ISR(TCE1_CCA_vect){
	pid_measureSpeed(TCE1.CCA, WHEEL2);
}

ISR(TCD0_CCA_vect){
	pid_measureSpeed(TCD0.CCA, WHEEL3);
}

ISR(TCD1_CCA_vect){
	pid_measureSpeed(TCD1.CCA, WHEEL4);
}