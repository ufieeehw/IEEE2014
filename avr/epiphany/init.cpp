#include "init.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "hwlib/uart.h"
#include "hwlib/pid.h"
#include "hwlib/motor.h"

void init() {
	init_clocks();
	init_modules();
	init_interrupts();
}

void init_clocks() {
	OSC.CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm; // enable 32 Mhz clock (while leaving the current 2 Mhz clock enabled)
	while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { } // wait for it to stabilize
	CPU_CCP = CCP_IOREG_gc; // enable access to protected registers
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch to the 32 Mhz clock
}

void init_interrupts() {
	PMIC.CTRL = PMIC_RREN_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();
}

// All initialization functions for peripherals should be placed here.
void init_modules() {
	uart_init();
//	motor_init();
//	pid_init();
}
