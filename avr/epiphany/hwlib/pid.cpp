#include "pid.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "uart.h"

#define constP 30
#define constI 1
#define constD 0.1

// Configuration Values
static TC0_t &pid_tick_timer = TCC0;
#define PID_TICK_OVF TCC0_OVF_vect

static PORT_t &wheelPort1 = PORTE;
static TC0_t &wheel1Timer = TCE0;
static TC1_t &wheel2Timer = TCE1;
static PORT_t &wheelPort2 = PORTD;
static TC0_t &wheel3Timer = TCD0;
static TC1_t &wheel4Timer = TCD1;

#define PID_Wheel1_Event_Channel EVSYS_CHMUX_PORTE_PIN0_gc
#define PID_Wheel2_Event_Channel EVSYS_CHMUX_PORTE_PIN1_gc
#define PID_Wheel3_Event_Channel EVSYS_CHMUX_PORTD_PIN0_gc
#define PID_Wheel4_Event_Channel EVSYS_CHMUX_PORTD_PIN1_gc

// Multipliers
// (xTicks/10ms)(1000ms/1sec)(1rev/464ticks)(2piRad/1rev) = blah
static float TICKS_TO_RAD_MULTIPLIER = (1/464.0)*(2*M_PI);
static const uint16_t pid_tick_timer_period = 5000; //Will generate an interrupt every 1ms
static const int sampleTimeMs = 10;
static const float max_wheel_speed = 70.0;


// File-Scope Variables and Structures
struct pid_wheel_data {
	// Wheel Direction
	//MotorDirection direction = MOTOR_NEUTRAL;
	
	// Values for measuring the speed of the wheel
	volatile float AVG_speed = 0;
	volatile int individual_ticks = 0;
	volatile int ticks = 0;
	
	// Values for the actual pid controller
	float input = 0, output = 0, setpoint = 0, errSum = 0, lastErr = 0, kp = 0, ki = 0, kd = 0;
	
	int32_t odometry_ticks = 0;
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
	pid_tick_timer.PER = pid_tick_timer_period;
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
	
	//Compute the output
	data.output = (data.kp * error) + (data.ki * data.errSum) + (data.kd * dErr);
	
	//Remember some things for later
	data.lastErr = error;
}

void pid_setTunings(float Kp, float Ki, float Kd, wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	float sampleTimeInSec = ((float)sampleTimeMs);
	data.kp = Kp;
	data.ki = Ki*sampleTimeInSec;
	data.kd = Kd/sampleTimeInSec;
}

static void pid_measureSpeed(wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	
	cli();
	data.ticks = data.individual_ticks;
	data.individual_ticks = 0;
	sei();
	
	data.odometry_ticks += data.ticks;
	data.AVG_speed = data.ticks*TICKS_TO_RAD_MULTIPLIER/10e-3;
}

float pid_getSpeed(wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	return data.AVG_speed;
}

void pid_setSpeed(float speed, wheelNum num) {
	pid_wheel_data &data = wheelData[num];
	data.setpoint = speed;
}

// Handler that sets wheel speeds based on message.
// Arguments:
//	message := a message that contains the following:
//		4 bytes, each is a signed representation of
//		the speed of the wheels from -100 to 100.
//		This means that if you want to pass this message
//		you should format convert 4 values between -100
//		and 100, and put them into characters in an array.
//	len := the length of the message.  E.g. the number of bytes in the array
void pid_set_speed_handler(char* message, uint8_t len) {
	cli();
	
	int8_t	wheel1percent = (int8_t)message[0], 
			wheel2percent = (int8_t)message[1], 
			wheel3percent = (int8_t)message[2], 
			wheel4percent = (int8_t)message[3];
	
	pid_setSpeed(max_wheel_speed * wheel1percent/100.0, WHEEL1);
	pid_setSpeed(max_wheel_speed * wheel2percent/100.0, WHEEL2);
	pid_setSpeed(max_wheel_speed * wheel3percent/100.0, WHEEL3);
	pid_setSpeed(max_wheel_speed * wheel4percent/100.0, WHEEL4);
	sei();
}

static uint32_t* pid_get_odometry(uint32_t* returnData) {
	for(int i = 0; i < 4; i++) {
		returnData[i] = wheelData[i].odometry_ticks;
		wheelData[i].odometry_ticks = 0;
	}
	return returnData;
}

void pid_get_odometry_handler(char* message, uint8_t len) {
	char odometry[16];
	uint32_t wheelOdometry[4];
	pid_get_odometry(wheelOdometry);
	
	for(int i = 0; i < 4; i++) {
		odometry[i*4  ] = (char)((wheelOdometry[i] >> 24) & 0xFF);
		odometry[i*4+1] = (char)((wheelOdometry[i] >> 16) & 0xFF);
		odometry[i*4+2] = (char)((wheelOdometry[i] >> 8)  & 0xFF);
		odometry[i*4+3] = (char)((wheelOdometry[i])       & 0xFF);
	}

	uart_send_msg_block(PIDgetOdometry, odometry, 17);
}

static inline void pid_set_speed_multiplier(float val) {
	TICKS_TO_RAD_MULTIPLIER = val;
}

static inline float pid_get_speed_multiplier() {
	return TICKS_TO_RAD_MULTIPLIER;
}

// Because the multiplier is a floating point value, we'll multiply it by 1000 first, and then send it.
// Or, if we're recieving it, we'll divide it by 1000.
void pid_get_speed_multiplier_handler(char* messsage, uint8_t len) {
	
}

void pid_set_speed_multiplier_handler(char* messsage, uint8_t len) {
	for(int i = 0; i < len; i++) {
		
	}
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

ISR(PORTE_INT0_vect){
	// pins 0 and 2
	static int8_t old_value = 0;
	int8_t new_value = grayToBinary(((wheelPort1.IN & 4) >> 1) | (wheelPort1.IN & 1));
	
	int8_t difference = new_value - old_value;
	if(difference > 2) difference -= 4;
	if(difference < -2) difference += 4;
	
	wheelData[0].individual_ticks += difference;
	
	old_value = new_value;
	
}

ISR(PORTE_INT1_vect){
	
	// pins 1 and 3
	static int8_t old_value = 0;
	int8_t new_value = grayToBinary(((wheelPort1.IN & 8) >> 1) | (wheelPort1.IN & 2));
	
	int8_t difference = new_value - old_value;
	if(difference > 2) difference -= 4;
	if(difference < -2) difference += 4;
	
	wheelData[1].individual_ticks += difference;
	
	old_value = new_value;
}

ISR(PORTD_INT0_vect){
	// pins 0 and 2
	static int8_t old_value = 0;
	int8_t new_value = grayToBinary(((wheelPort2.IN & 4) >> 1) | (wheelPort2.IN & 1));
		
	int8_t difference = new_value - old_value;
	if(difference > 2) difference -= 4;
	if(difference < -2) difference += 4;
		
	wheelData[2].individual_ticks += difference;
		
	old_value = new_value;
}

ISR(PORTD_INT1_vect){
	// pins 1 and 3
	static int8_t old_value = 0;
	int8_t new_value = grayToBinary(((wheelPort2.IN & 8) >> 1) | (wheelPort2.IN & 2));
	
	int8_t difference = new_value - old_value;
	if(difference > 2) difference -= 4;
	if(difference < -2) difference += 4;
	
	wheelData[3].individual_ticks += difference;
	
	old_value = new_value;
}
	

