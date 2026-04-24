/* 
* HBridgeDriver.cpp
* Driver for brushed DC motors using an H-bridge driver with direction set via control pin. Each channel is an axle with a motor. 
* Can support 10 motors.
* Structure:
* 1) Top level, a master list of pins in either the PWM or digitals arrays. Slots to which physical pins are assigned.
* these are ppwms and pdigitals here, which hold pointers to these top level arrays.
* 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
* these are indexed by channel. motorDrive here. 
* 3) A PWM level indexed by channel. The current power or level value of the PWM timer. Standardized to reflect the control applied.
* for instance, if the PS3 controller delivers -1000 to 1000 thats the range. If the controller is using the button that has -1 off and a max of 1000
* then the range is 0 to 2000 to reflect the fact that no reverse operation is going to be in effect for something like a LED or pump.
* 4) Optionally, a duration and a minimum PWM level, or here a minimum motor power. The duration represents a maximum interval before the PWM timer
* is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
* The minimum PWM level is the bottom limit for the PWM value, or in this case the minimum motor power. It is indexed by channel and the value is
* the range that comes from the main controller, before any processing into a timer value.
* the top level abstract class AbstractMotorControl contains these values.
* Created: 10/2/2016 1:42:24 PM
* Author: jg
*/
#define LOW 0
#define HIGH 1
#include "HBridgeDriver.h"
#include "../Configuration_adv.h"


int HBridgeDriver::commandEmergencyStop(int status)
{
	for(int j=0; j < 10; j++) {
		int pindex = motorDrive[j][0];
		if(pindex != 255) {
			//ppwms[pindex]->init();
			ppwms[pindex]->pwmOff();
		}
	}
	fault_flag = 16;
	resetSpeeds();
	resetEncoders();
	return status;
}
/*
* Add a new PWM instance to this motor controller.
* channel - the controller channel from 1 to 10, each successively higher channel will reset maximum channel count. Each channel is an axle/motor.
* pin_number - the index in the PWM array defined in 'setMotors', this is the next blank slot available
* dir_pin - the direction pin for this channel
* dir_default - the default direction the motor starts in
*/ 
int HBridgeDriver::createPWM(uint8_t channel, uint8_t pin_number, uint8_t dir_pin, uint8_t dir_default) {
	// See if pin assigned
	int foundPin = -1;
	int pinSlot = -1;
	for(int i = 0; i < 32; i++) {
		if(!pdigitals[i]) {
			if(pinSlot == -1)
				pinSlot = i;
		} else {
			if(pdigitals[i]->pin == dir_pin) {
				foundPin = i;
				break;
			}
		}
	}
	// didnt find pin, didnt find a slot
	if(foundPin == -1 && pinSlot == -1) {
		return -1; // slots full...
	}
	if( channel <= 0 || channel > 10)
		return -99;

	if( getChannels() < channel ) 
		setChannels(channel);
	// Set up the digital direction pin
	Digital* dpin;
	if(foundPin == -1) {
	 	dpin = new Digital(dir_pin);
		pdigitals[pinSlot] = dpin;
	} else
		dpin = pdigitals[foundPin];
	dpin->pinMode(PinMode::OUTPUT);

	// set up PWM
	int pindex;
	for(pindex = 0; pindex < 10; pindex++) {
		if( !ppwms[pindex] ) {
				break;
		}
	}
	if( ppwms[pindex] )
		return -2;

	currentDirection[channel-1] = dir_default;
	defaultDirection[channel-1] = dir_default;
			
	motorDrive[channel-1][0] = pindex;
	motorDrive[channel-1][1] = dir_pin;
	PWM* ppin = new PWM(pin_number);
	ppwms[pindex] = ppin;
	ppwms[pindex]->init();
	return 0;
}
int HBridgeDriver::checkSafeShutdown() {
	int fault_flag = 0;
	for(int i = 1; i <= getChannels(); i++) {
		if(get_on_time_us(i) > watchdogMax) {
			int pindex = motorDrive[i-1][0];
			if(pindex != 255 && ppwms[pindex]) {
				ppwms[pindex]->pwmOff();
				fault_flag |= (1 << (i-1)); // set bit for this channel
			}
		}
	}
	return fault_flag;
}
/*
* Command the bridge driver power level. Manage direction pin. If necessary limit min and max power and
* scale to the MOTORPOWERSCALE if > 0. After calculation and saved values in the 0-1000 range scale it to 0-255 for 8 bit PWM.
* Each channel is an axle/motor
*/
int HBridgeDriver::commandMotorPower(int16_t p[10]) {
		// check shutdown override
		if( MOTORSHUTDOWN )
			return 0;
		int foundPin = 0;
	for(int motorChannel = 1; motorChannel <= getChannels(); motorChannel++) {
		int motorPower = p[motorChannel-1];
		motorSpeed[motorChannel-1] = motorPower;
		// get mapping of channel to pin
		// see if we need to make a direction change, check array of [PWM pin][dir pin][dir]
		if( currentDirection[motorChannel-1]) { // if dir 1, we are going what we define as 'forward' 
			if( motorPower < 0 ) { // and we want to go backward
				// reverse dir, send dir change to pin
				for(int i = 0; i < 10; i++) {
					if(pdigitals[i] && pdigitals[i]->pin == motorDrive[motorChannel-1][1]) {
							//pdigitals[i]->setPin(motorDrive[motorChannel-1][1]);
							pdigitals[i]->pinMode(OUTPUT);
							// default is 0 (LOW), if we changed the direction to reverse wheel rotation call the opposite dir change signal
							defaultDirection[motorChannel-1] ? pdigitals[i]->digitalWrite(HIGH) : pdigitals[i]->digitalWrite(LOW);
							currentDirection[motorChannel-1] = 0; // set new direction value
							motorPower = -motorPower; // absolute val
							foundPin = 1;
							break;
					}
				}
			} else { // wieter weiter
				foundPin = 1;
			}
		} else { // dir is 0
			if( motorPower > 0 ) { // we are going 'backward' as defined by our initial default direction and we want 'forward'
				// reverse, send dir change to pin
				for(int i = 0; i < 10; i++) {
					if(pdigitals[i] && pdigitals[i]->pin == motorDrive[motorChannel-1][1]) {
						//pdigitals[i]->setPin(motorDrive[motorChannel-1][1]);
						pdigitals[i]->pinMode(OUTPUT);
						// default is 0 (HIGH), if we changed the direction to reverse wheel rotation call the opposite dir change signal
						defaultDirection[motorChannel-1] ? pdigitals[i]->digitalWrite(LOW) : pdigitals[i]->digitalWrite(HIGH);
						currentDirection[motorChannel-1] = 1;
						foundPin = 1;
						break;
					}
				}
			} else { // backward with more backwardness
				// If less than 0 take absolute value, if zero dont play with sign
				if( motorPower ) motorPower = -motorPower;
				foundPin = 1;
			}
		}
		if(!foundPin) {
			return commandEmergencyStop(2);
		}	
		if( motorPower != 0 && motorPower < minMotorPower[motorChannel-1])
				motorPower = minMotorPower[motorChannel-1];
		if( motorPower > MAXMOTORPOWER ) // cap it at max
				motorPower = MAXMOTORPOWER;
		// Scale motor power if necessary and save it in channel speed array with proper sign for later use
		if( MOTORPOWERSCALE != 0 )
				motorPower /= MOTORPOWERSCALE;
		//
		// Reset encoders on new speed setting
		resetEncoders();
		if(motorDrive[motorChannel-1][0] != 255 && ppwms[motorDrive[motorChannel-1][0]]) {
			int pindex = motorDrive[motorChannel-1][0];
			//ppwms[pindex]->attachInterrupt(motorDurationService[motorChannel-1]);// last param TRUE indicates an overflow interrupt
			ppwms[pindex]->pwmWrite(true,motorPower);
			fault_flag = 0;
		} else {
			fault_flag = 16;
		}
		last_command_time[motorChannel-1] = (motorPower == 0 ? 0 : time_us_64());
	}
	watchdog = watchdogMax;
	shutdownRequested = false;
	shutdownLogged = false;
	return fault_flag;
}

void HBridgeDriver::getDriverInfo(uint8_t ch, char* outStr) {
	char cout[OUT_BUFFER_SIZE];
	char dout1[10];
	char dout3[10];
	char dout5[10];
	char dout7[10];
	
	if( motorDrive[ch-1][0] == 255 ) {
		itoa(-1, dout1, 10);
	} else {
		itoa(ppwms[motorDrive[ch-1][0]]->pin, dout1, 10);
		itoa(get_slice(ch), dout5, 10);
		itoa(ppwms[motorDrive[ch-1][0]]->get_pwm_channel(), dout7, 10);
	}
	
	itoa(motorDrive[ch-1][1], dout3, 10);

	if( motorDrive[ch-1][0] == 255 ) {
		sprintf(cout,"HB-PWM UNINITIALIZED Pin:%s, Dir Pin:%s\r\n\0", dout1, dout3);
	} else {
		sprintf(cout,"HB-PWM Pin:%s, Dir Pin:%s, Slice:%s, PWM Channel:%s\r\n\0", dout1, dout3, dout5, dout7);
	}
	
	for(int i=0; i < OUT_BUFFER_SIZE; ++i){
		 outStr[i] = cout[i];
		 if(!outStr[i])
		 break;
	}

}

//HBridgeDriver hBridgeDriver;