// Graphing sketch

// This code takes a serial port and graphs the four inputs.
// In this case, it's the motor inputs.
import processing.serial.*;

Serial myPort;        // The serial port
int xPos = 1,         // horizontal position of the graph
    xPosPast = xPos;
float referenceValPast = 0,
     measuredVal1Past = 0,
     measuredVal2Past = 0,
     measuredVal3Past = 0,
     measuredVal4Past = 0;


void setup () {
  // set the window size:
  size(500, 800);        

  // List all the available serial ports
  println(Serial.list());
  // I know that the first port in the serial list on my mac
  // is always my  Arduino, so I open Serial.list()[0].
  // Open whatever port is the one you're using.
  myPort = new Serial(this, Serial.list()[2], 19200);
  // don't generate a serialEvent() unless you get a newline character:
  myPort.bufferUntil('\n');
  // set inital background:
  background(0);
}
void draw () {
  // everything happens in the serialEvent()
}

String readStringUntil(int measuredVal1, Serial s) {
    byte temp[] = s.readBytesUntil(measuredVal1);
    if (temp == null) {
      return null;
    } else {
      return new String(temp);
    }
}

void keyPressed () {
  myPort.write(key);
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  String inString = readStringUntil('\n', myPort);
  //println(inString);
  String inStringSplit[];

  if (inString != null) {
    // trim off any whitespace:
    inStringSplit = trim(inString).split(" ");
    // convert to an int and map to the screen height:
    float measuredVal1 = float(inStringSplit[1]); 
    float measuredVal2 = float(inStringSplit[2]);
    float measuredVal3 = float(inStringSplit[3]);
    float measuredVal4 = float(inStringSplit[4]);
    float referenceVal = float(inStringSplit[0]);
    

 

    measuredVal1 = map(measuredVal1, 0, 35, 0, height);
    measuredVal2 = map(measuredVal2, 0, 35, 0, height);
    measuredVal3 = map(measuredVal3, 0, 35, 0, height);
    measuredVal4 = map(measuredVal4, 0, 35, 0, height);
    referenceVal = map(referenceVal, 0,35,0, height);

    // draw the line:

 stroke(1,255,50);
  line(xPos - 1, height - referenceValPast, xPos, height - referenceVal);
  
 stroke(255,95,100); 
  line(xPos - 1, height - measuredVal1Past, xPos, height - measuredVal1);
  
  stroke(255,0,50);
  line(xPos - 1, height - measuredVal2Past, xPos, height - measuredVal2);

  stroke(255,0,255);
  line(xPos - 1, height - measuredVal3Past, xPos, height - measuredVal3);

  stroke(0,255,255);
  line(xPos - 1, height - measuredVal4Past, xPos, height - measuredVal4);
  
    referenceValPast = referenceVal;
    measuredVal1Past  = measuredVal1;
    measuredVal2Past  = measuredVal2;
    measuredVal3Past  = measuredVal3;
    measuredVal4Past  = measuredVal4;

    // at the edge of the screen, go back to the beginning:
    if (xPos >= width) {
      xPos = 0;
      background(0); 
    } 
    else {
      // increment the horizontal position:
      xPos++;
    }
  }
}


/*
#define F_CPU 32000000

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "init.h"
#include "hwlib/motor.h"x
#include "hwlib/pid.h"
#include "hwlib/uart.h"	

char rec;
int refspeed = 10;
int temp = 0; 
MotorDirection mdir = MOTOR_BACKWARD;

int main() {
	init();
	while(1) {
		rec = uart_get();
		switch(rec) {
			case 'q': refspeed++; break;
			case 'a': refspeed--; break;
			case 'z': temp = refspeed; 
					  refspeed = 0; 
					  break;
			case 'x': refspeed = temp; break;
			
			default: 
						pid_setSpeed(refspeed, mdir, WHEEL1);
						pid_setSpeed(refspeed, mdir, WHEEL2);
						pid_setSpeed(refspeed, mdir, WHEEL3);
						pid_setSpeed(refspeed, mdir, WHEEL4);
						printf("%i %f %f %f %f\n", refspeed, pid_getSpeed(WHEEL1), pid_getSpeed(WHEEL2), pid_getSpeed(WHEEL3), pid_getSpeed(WHEEL4));
						break;
		}
		
		_delay_ms(20);

		
	}
		
}
*/
