#include "pid.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "uart.h"
#include "../init.h"

#define constP 120
#define constI 4000
#define constD 0.0004

// Configuration Values
static TC0_t &pid_tick_timer = TCC0;
#define PID_TICK_OVF TCC0_OVF_vect

static PORT_t &wheelPort1 = PORTA; // Originally PORTE
static PORT_t &wheelPort2 = PORTB; // Originally PORTD


static float radPerTick = (2*M_PI)/1856.0;
static const float sampleTime = 10e-3;


// File-Scope Variables and Structures
struct pid_wheel_data {
	// Wheel Direction
	//MotorDirection direction = MOTOR_NEUTRAL;
	
	// Values for measuring the speed of the wheel
	volatile float AVG_speed = 0;
	volatile int32_t ticks = 0;
	
	// Values for the actual pid controller
	float input = 0, output = 0, setpoint = 0, errSum = 0, lastErr = 0, kp = 0, ki = 0, kd = 0;

	int32_t pid_last_ticks = 0;
	int32_t odometry_last_ticks = 0;
} ;

static pid_wheel_data wheelData[4];

void pid_init() {	
	// Initialize motor frequency measurement timers.
	wheelPort1.DIRCLR = 0x03;
	wheelPort1.PIN0CTRL = PORT_ISC_BOTHEDGES_gc;
	wheelPort1.INT0MASK = 0x05;
	wheelPort1.INTCTRL = 0x05;

	wheelPort1.PIN1CTRL = PORT_ISC_BOTHEDGES_gc;
	wheelPort1.INT1MASK = 0x0A;
	
	wheelPort2.DIRCLR = 0x03;
	wheelPort2.PIN0CTRL = PORT_ISC_BOTHEDGES_gc;
	wheelPort2.INT0MASK = 0x05;
	wheelPort2.INTCTRL = 0x05;

	wheelPort2.PIN1CTRL = PORT_ISC_BOTHEDGES_gc;
	wheelPort2.INT1MASK = 0x0A;

	
	// Initialize the PID tick timer
	pid_tick_timer.CTRLA = TC_CLKSEL_DIV64_gc;	//Uses timer on portC since all timers on PORT E and D will be used for wheels.
	pid_tick_timer.CTRLB = 0x00;
	pid_tick_timer.CTRLC = 0x00;
	pid_tick_timer.CTRLD = 0x00;
	pid_tick_timer.CTRLE = 0x00;
	pid_tick_timer.PER = round(F_CPU * sampleTime / 64.);
	pid_tick_timer.INTCTRLA = TC_OVFINTLVL_LO_gc;
	
	pid_setTunings(constP, constI, constD, WHEEL1);
	pid_setTunings(constP, constI, constD, WHEEL2);
	pid_setTunings(constP, constI, constD, WHEEL3);
	pid_setTunings(constP, constI, constD, WHEEL4);
	
}

static void pid_compute(wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	
	// Compute all working error variables
	float error = data.setpoint - data.AVG_speed;
	data.errSum += error;
	float dErr = (error - data.lastErr);
	
	if(data.ki * data.errSum >= 1024) {
		data.errSum = 1024/data.ki;
	} else if(data.ki * data.errSum <= -1024) {
		data.errSum = -1024/data.ki;
	}
	
	//Compute the output
	data.output = (data.kp * error) + (data.ki * data.errSum) + (data.kd * dErr);
	
	//Remember some things for later
	data.lastErr = error;
}

void pid_setTunings(float Kp, float Ki, float Kd, wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	data.kp = Kp;
	data.ki = Ki*sampleTime;
	data.kd = Kd/sampleTime;
}

static void pid_measureSpeed(wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	
	cli();
	int32_t ticks = data.ticks;
	sei();
	
	data.AVG_speed = (ticks - data.pid_last_ticks)*radPerTick/sampleTime;
	
	data.pid_last_ticks = ticks;
}

float pid_getSpeed(wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	return data.AVG_speed;
}

void pid_setSpeed(float speed, wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	data.setpoint = speed;
}

/*******************************************************************
-----> Handler Functions <-----
*******************************************************************/

// Handler that sets wheel speeds based on message.
// Arguments:
//	message := [Wheel1B0, Wheel1B1, Wheel1B2, Wheel1B3, ... ,  Wheel4B0, Wheel4B1, Wheel4B2, Wheel4B3] 
//		16 bytes, little endian 32bit numbers that represent the desired wheel speed
//		multiplied by 1000.  Pass a pointer to the low byte of 
//	len := the length of the message.  E.g. the number of bytes in the array
void pid_set_speed_handler(char* message, uint8_t len) {
	cli();

	for(int i = 0; i < 4; i++) pid_setSpeed(uart_int32_to_float(&message[4*i]), (wheelNum)i);
	
	sei();
}

void pid_get_speed_handler(char* message, uint8_t len) {
	char wheelSpeeds[16];
	
	uart_float_to_char32((char*)&wheelSpeeds[0], pid_getSpeed(WHEEL1));
	uart_float_to_char32((char*)&wheelSpeeds[4], pid_getSpeed(WHEEL2));
	uart_float_to_char32((char*)&wheelSpeeds[8], pid_getSpeed(WHEEL3));
	uart_float_to_char32((char*)&wheelSpeeds[12], pid_getSpeed(WHEEL4));
	
	uart_send_msg_block(PIDgetSpeed, wheelSpeeds, 17);
}


static void pid_get_odometry(float* returnData) {
	for(int i = 0; i < 4; i++) {
		pid_wheel_data &data = wheelData[i];
		
		cli();
		int32_t ticks = data.ticks;
		sei();
		
		volatile int32_t tmp = ticks - data.odometry_last_ticks;
		volatile float tmp2 = tmp * radPerTick;
		
		returnData[i] = tmp2;
		
		data.odometry_last_ticks = ticks;
	}
}

void pid_get_odometry_handler(char* message, uint8_t len) {

	char odometry[16];
	float wheelOdometry[4];
	pid_get_odometry(wheelOdometry);
	
	for(int i = 0; i < 4; i++) {
		uart_float_to_char32(odometry+i*4, wheelOdometry[i]);
	}

	uart_send_msg_block(PIDgetOdometry, odometry, 17);

}

static inline void pid_set_speed_multiplier(float val) {
	radPerTick = val;
}

static inline float pid_get_speed_multiplier() {
	return radPerTick;
}

// Because the multiplier is a floating point value, we'll multiply it by 1000 first, and then send it.
// Or, if we're recieving it, we'll divide it by 1000.
void pid_get_speed_multiplier_handler(char* message, uint8_t len) {
	char multiplier[2];
	uart_float_to_char16(multiplier, pid_get_speed_multiplier());
	uart_send_msg_block(PIDgetMultiplier, multiplier, 3);
}

void pid_set_speed_multiplier_handler(char* message, uint8_t len) {
	pid_set_speed_multiplier(uart_int16_to_float(message));
}


ISR(PID_TICK_OVF) {
	pid_measureSpeed(WHEEL1);
	pid_measureSpeed(WHEEL2);
	pid_measureSpeed(WHEEL3);
	pid_measureSpeed(WHEEL4);
	pid_compute(WHEEL1);
	pid_compute(WHEEL2);
	pid_compute(WHEEL3);
	pid_compute(WHEEL4);
	motor_set_velocity(MOTOR_1, wheelData[WHEEL1].output);
	motor_set_velocity(MOTOR_2, wheelData[WHEEL2].output);
	motor_set_velocity(MOTOR_3, wheelData[WHEEL3].output);
	motor_set_velocity(MOTOR_4, wheelData[WHEEL4].output);
}

unsigned int grayToBinary(unsigned int num)
{
	unsigned int mask;
	for (mask = num >> 1; mask != 0; mask = mask >> 1)
	{
		num = num ^ mask;
	}
	return num;
}

ISR(PORTA_INT0_vect){
	// pins 0 and 2
	static int8_t old_value = 0;
	int8_t new_value = grayToBinary(((wheelPort1.IN & 4) >> 1) | (wheelPort1.IN & 1));
	
	int8_t difference = new_value - old_value;
	if(difference > 2) difference -= 4;
	if(difference < -2) difference += 4;
	
	wheelData[0].ticks += difference;
	
	old_value = new_value;
	
}

ISR(PORTA_INT1_vect){
	
	// pins 1 and 3
	static int8_t old_value = 0;
	int8_t new_value = grayToBinary(((wheelPort1.IN & 8) >> 1) | (wheelPort1.IN & 2));
	
	int8_t difference = new_value - old_value;
	if(difference > 2) difference -= 4;
	if(difference < -2) difference += 4;
	
	wheelData[1].ticks += difference;
	
	old_value = new_value;
}

ISR(PORTB_INT0_vect){
	// pins 0 and 2
	static int8_t old_value = 0;
	int8_t new_value = grayToBinary(((wheelPort2.IN & 4) >> 1) | (wheelPort2.IN & 1));
		
	int8_t difference = new_value - old_value;
	if(difference > 2) difference -= 4;
	if(difference < -2) difference += 4;
		
	wheelData[2].ticks += difference;
		
	old_value = new_value;
}

ISR(PORTB_INT1_vect){
	// pins 1 and 3
	static int8_t old_value = 0;
	int8_t new_value = grayToBinary(((wheelPort2.IN & 8) >> 1) | (wheelPort2.IN & 2));
	
	int8_t difference = new_value - old_value;
	if(difference > 2) difference -= 4;
	if(difference < -2) difference += 4;
	
	wheelData[3].ticks += difference;
	
	old_value = new_value;
}
	

