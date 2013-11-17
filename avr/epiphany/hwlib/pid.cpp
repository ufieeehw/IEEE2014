#include "PID.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Configuration Values
static TC0_t &pid_tick_timer = TCC0;
#define PID_TICK_OVF TCC0_OVF_vect
static const uint16_t pid_tick_timer_period = 32000; //Will generate an interrupt every 1ms
static const uint8_t AVG_ARRAY_LENGTH = 5;
static const float CCA_TO_RAD_MULTIPLIER = 6770.674;
static const int sampleTime = 1;

static const float wheelConversionConstants[4][2] =	{	
															{0.0899, -30.96},	// Wheel 1
															{0.06878, -19.61},  // Wheel 2
															{0.08496, -30.78},  // Wheel 3
															{0.08894, -31.51}   // Wheel 4
														};

// File-Scope Variables and Structures
struct pid_wheel_data {
	// Wheel Direction
	MotorDirection direction = MOTOR_NEUTRAL;
	
	// Values for measuring the speed of the wheel
	int AVG_measurements[AVG_ARRAY_LENGTH] = {0};
	volatile int AVG_measurement_array_position = 0;
	volatile float AVG_speed = 0;
	
	// Values for the actual pid controller
	float input = 0, output = 0, setpoint = 0, errSum = 0, lastErr = 0, kp = 0, ki = 0, kd = 0;
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
	
	pid_setTunings(50,1,1,WHEEL1);
	pid_setTunings(50,1,1,WHEEL2);
	pid_setTunings(100,1,1,WHEEL3);
	pid_setTunings(100,1,1,WHEEL4);
	
}

static void pid_compute(wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	
	// Compute all working error variables
	float error = data.setpoint - data.AVG_speed;
	data.errSum += error;
	float dErr = (error - data.lastErr);
	
	//Compute the output
	data.output = (data.kp * error) + (data.ki * data.errSum) + (data.kd * dErr);
	
	//Remember some things for later
	data.lastErr = error;
}

void pid_setTunings(float Kp, float Ki, float Kd, wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	float sampleTimeInSec = ((float)sampleTime);
	data.kp = Kp;
	data.ki = Ki*sampleTimeInSec;
	data.kd = Kd/sampleTimeInSec;
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
	
	tempAvgVal /= (float)data.AVG_measurement_array_position;
	data.AVG_speed = (1.0/tempAvgVal)*CCA_TO_RAD_MULTIPLIER;
}

int pid_convertSpeedToEffort(float speed, wheelNum num) {
	return (speed - wheelConversionConstants[num][1])/wheelConversionConstants[num][0];
}

float pid_getSpeed(wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	return data.AVG_speed;
}

void pid_setSpeed(float speed, MotorDirection dir, wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	data.setpoint = speed;
	if(dir == 1) {
		if((num == WHEEL2) || (num == WHEEL4)) data.direction = MOTOR_BACKWARD;
		else data.direction = MOTOR_FORWARD;
	} else if(dir == 2) {
		if((num == WHEEL2) || (num == WHEEL4)) data.direction = MOTOR_FORWARD;
		else data.direction = MOTOR_BACKWARD;
	} else data.direction = MOTOR_NEUTRAL;
}


ISR(PID_TICK_OVF) {
	pid_compute(WHEEL1);
	pid_compute(WHEEL2);
	pid_compute(WHEEL3);
	pid_compute(WHEEL4);
	motor_set_direction(MOTOR_1, wheelData[WHEEL1].direction);
	motor_set_effort(MOTOR_1, pid_convertSpeedToEffort(wheelData[WHEEL1].output,WHEEL1));
	motor_set_direction(MOTOR_2, wheelData[WHEEL2].direction);
	motor_set_effort(MOTOR_2, pid_convertSpeedToEffort(wheelData[WHEEL2].output,WHEEL2));
	motor_set_direction(MOTOR_3, wheelData[WHEEL3].direction);
	motor_set_effort(MOTOR_3, pid_convertSpeedToEffort(wheelData[WHEEL3].output,WHEEL3));
	motor_set_direction(MOTOR_4, wheelData[WHEEL4].direction);
	motor_set_effort(MOTOR_4, pid_convertSpeedToEffort(wheelData[WHEEL4].output,WHEEL4));
	
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