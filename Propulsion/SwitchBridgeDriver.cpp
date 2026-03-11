/* 
* SwitchBridgeDriver.cpp
* A motor driver that uses straight GPIO switching rather than PWM, for an on/off drive motor setup.
* Structure:
* 1) Top level, a master list of pins in the digitals array. Slots to which physical pins are assigned.
* these are pdigitals here, which hold pointers to these top level array.
* 2) In each class and subclass, a sublist of pins. Multidimensional array which holds the index to the top level pins.
* these are indexed by channel. motorDrive here.
* Since the M1 and M2 inputs are split, we add an additional motorDriveB multidimensional array for the second input.
* NOTE: The pin assignment for the IO pins is arranged in the index array one after the other, so the assignment for M1 is at pindex
* and the assignment for M2 is at pindex+1. This does NOT mean the pins have to be next to each other, its just that the indexes in
* the arrays are arranged one after the other for convenience.
* 3) A GPIO pin list indexed by channel. The level value is either a + or - value for on or off.
* 4) Optionally, a duration. The duration represents a maximum interval before the GPIO pin
* is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
* the top level abstract class AbstractMotorControl contains these values.
* Created: 10/20/2020 1:39:44 PM
* Author: groff
*/
#include "SwitchBridgeDriver.h"
#include "..\Configuration_adv.h"
// default constructor
SwitchBridgeDriver::SwitchBridgeDriver()
{
} //SwitchBridgeDriver

// default destructor
SwitchBridgeDriver::~SwitchBridgeDriver()
{
} //~SwitchBridgeDriver

int SwitchBridgeDriver::commandEmergencyStop(int status)
{
	for(int j=0; j < 10; j++) {
		int pindex = motorDrive[j][0];
		if(pindex != 255) {
			pdigitals[pindex]->digitalWrite(LOW);
		}
		pindex = motorDriveB[j][0];
		if(pindex != 255) {
				pdigitals[pindex]->digitalWrite(LOW);
		}
	}
	fault_flag = 16;
	resetSpeeds();
	resetEncoders();
	return status;
}

/*
* Add a new Digital pin instance to this motor controller.
* channel - the controller channel from 1 to 10
* pin_numberA - the index in the PWM array defined in 'setMotors' for M1, this is the next blank slot available
* pin_numberB - the index in the PWM array defined in 'setMotors' for M2
* enb_pin - the enable pin for this channel
* dir_default - the default direction the motor starts in
*/
void SwitchBridgeDriver::createDigital(uint8_t channel, uint8_t pin_numberA, uint8_t pin_numberB, uint8_t enable_pin, uint8_t dir_default) {
	if( getChannels() < channel ) setChannels(channel);
	if( assignPin(pin_numberA) && assignPin(pin_numberB)) {
		// Set up the digital direction pin
			int foundPin = 0;
			// Set up the digital enable pin, we want to be able to re-use these pins for multiple channels on 1 controller
			if( assignPin(enable_pin) ) {
				Digital* dpin = new Digital(enable_pin);
				dpin->pinMode(OUTPUT);
				for(int i = 0; i < 10; i++) {
					if(!pdigitals[i]) {
						pdigitals[i] = dpin;
						foundPin = 1;
						break;
					}
				}
				if(!foundPin) {
					delete dpin;
					return; // no slots?
				}
			} else { // cant assign, it may be already assigned
				for(int i = 0; i < 10; i++) {
					if(pdigitals[i]->pin == enable_pin) {
						//dpin = pdigitals[i];
						foundPin = 1;
						break;
					}
				}
				if(!foundPin) {
					return; // slots full...
				}
			}
		
			int pindex;
			for(pindex = 0; pindex < 9; pindex++) {
				if( !pdigitals[pindex] && !pdigitals[pindex+1])
				break;
			}
			if( pdigitals[pindex] || pdigitals[pindex+1])
				return;
			currentDirection[channel-1] = dir_default;
			defaultDirection[channel-1] = dir_default;
			
			motorDrive[channel-1][0] = pindex;
			motorDrive[channel-1][1] = enable_pin;
			//
			motorDriveB[channel-1][0] = pindex+1;
			// determines which input pin PWM signal goes to, motorDrive[0] or motorDriveB[0], which is at pindex, or pindex+1 in ppwms
			motorDriveB[channel-1][1] = dir_default;
			Digital* ppinA = new Digital(pin_numberA);
			pdigitals[pindex] = ppinA;
			pdigitals[pindex]->pinMode(OUTPUT);
			Digital* ppinB = new Digital(pin_numberB);
			pdigitals[pindex+1] = ppinB;
			pdigitals[pindex+1]->pinMode(OUTPUT);
		}
}

/*
* Command the power level. Manage enable pin. This inherits from AbstractMotorControl, but the motorPower is merely a +/- 
* forward/back value since we have a switched on/off control. It always max on forward, max on back
*/
int SwitchBridgeDriver::commandMotorPower(uint8_t motorChannel, int16_t motorPower) {
	// check shutdown override
	if( MOTORSHUTDOWN )
		return 0;
	int foundPin = 0;
	motorSpeed[motorChannel-1] = motorPower; //why? +/-
	// set enable pin
	for(int i = 0; i < 10; i++) {
		if(pdigitals[i] && pdigitals[i]->pin == motorDrive[motorChannel-1][1]) {
			//pdigitals[i]->setPin(motorDrive[motorChannel-1][1]);
			pdigitals[i]->pinMode(OUTPUT);
			pdigitals[i]->digitalWrite(HIGH);
			foundPin = 1;
			break;
		}
	}
	// get mapping of channel to pin
	// see if we need to make a direction change, check array of [PWM pin][dir pin][dir]
	if( currentDirection[motorChannel-1]) { // if dir 1, we are going what we define as 'forward'
		if( motorPower < 0 ) { // and we want to go backward
			// reverse dir, depending on default direction, we either send to PWM pin or PWM pin+1
			// default is 0 (LOW), if we changed the direction to reverse wheel rotation call the opposite dir change signal
			//defaultDirection[motorChannel-1] ? pdigitals[i]->digitalWrite(HIGH) : pdigitals[i]->digitalWrite(LOW);
			defaultDirection[motorChannel-1] ? motorDriveB[motorChannel-1][1] = 1 : motorDriveB[motorChannel-1][1] = 0;
			currentDirection[motorChannel-1] = 0; // set new direction value
			motorPower = -motorPower; // absolute val
		}
	} else { // dir is 0
		if( motorPower > 0 ) { // we are going 'backward' as defined by our initial default direction and we want 'forward'
			// reverse, indicate an alteration of the input pin
			// default is 0 (HIGH), if we changed the direction to reverse wheel rotation call the opposite dir change signal
			//defaultDirection[motorChannel-1] ? pdigitals[i]->digitalWrite(LOW) : pdigitals[i]->digitalWrite(HIGH);
			defaultDirection[motorChannel-1] ? motorDriveB[motorChannel-1][1] = 0 : motorDriveB[motorChannel-1][1] = 1;
			currentDirection[motorChannel-1] = 1;
		} else { // backward with more backwardness
			// If less than 0 take absolute value, if zero dont play with sign
			if( motorPower ) motorPower = -motorPower;
		}
	}
	if(!foundPin) {
		return commandEmergencyStop(6);
	}
	//
	// Reset encoders on new speed setting
	resetEncoders();
	// If we have a linked distance sensor. check range and possibly skip
	// If we are setting power 0, we are stopping anyway
	if( !checkUltrasonicShutdown()) {
		// element 0 of motorDrive has index to PWM array
		int pindex = motorDrive[motorChannel-1][0];
		// add the offset to the input pin, which will be 0 or 1 depending on above logic
		pindex += motorDriveB[motorChannel-1][1];
		// turn off all pins
		pdigitals[pindex]->digitalWrite(LOW);
	}
	fault_flag = 0;
	return 0;
}

void SwitchBridgeDriver::getDriverInfo(uint8_t ch, char* outStr) {
	char cout[OUT_BUFFER_SIZE];
	char dout1[5];
	char dout2[5];
	char dout3[5];
	char dout4[5];
	char dpin0[5];
	char dpin1[5];
	char dpin2[5];
	char dpin3[5];
	char dpin4[5];
	char dpin5[5];
	char dpin6[5];
	char dpin7[5];
	char dpin8[5];
	char dpin9[5];
	if( motorDrive[ch-1][0] == 255 ) {
		itoa(-1, dout1, 10);
		itoa(-1, dout3, 10);
	} else {
		itoa(pdigitals[motorDrive[ch-1][0]]->pin, dout1, 10);
		itoa(pdigitals[motorDrive[ch-1][0]]->mode, dout3, 10);
	}
	if( motorDriveB[ch-1][0] == 255 ) {
		itoa(-1, dout2, 10);
	} else {
		itoa(pdigitals[motorDriveB[ch-1][0]]->pin, dout2, 10);
	}
	
	itoa(motorDrive[ch-1][1], dout4, 10);
	if(pdigitals[0])
		itoa(pdigitals[0]->pin, dpin0, 10);
	else
		itoa(0, dpin0, 10);
	if(pdigitals[1])
		itoa(pdigitals[1]->pin, dpin1, 10);
	else
		itoa(0, dpin1, 10);
	if(pdigitals[2])
		itoa(pdigitals[2]->pin, dpin2, 10);
	else
		itoa(0, dpin2, 10);
	if(pdigitals[3])
		itoa(pdigitals[3]->pin, dpin3, 10);
	else
		itoa(0, dpin3, 10);
	if(pdigitals[4])
		itoa(pdigitals[4]->pin, dpin4, 10);
	else
		itoa(0, dpin4, 10);
	if(pdigitals[5])
		itoa(pdigitals[5]->pin, dpin5, 10);
	else
		itoa(0, dpin5, 10);
	if(pdigitals[6])
		itoa(pdigitals[6]->pin, dpin6, 10);
	else
		itoa(0, dpin6, 10);
	if(pdigitals[7])
		itoa(pdigitals[7]->pin, dpin7, 10);
	else
		itoa(0, dpin7, 10);
	if(pdigitals[8])
		itoa(pdigitals[8]->pin, dpin8, 10);
	else
		itoa(0, dpin8, 10);
	if(pdigitals[9])
		itoa(pdigitals[9]->pin, dpin9, 10);
	else
		itoa(0, dpin9, 10);
	if( motorDrive[ch-1][0] == 255 ) {
		sprintf(cout,"SB-Digital UNITIALIZED PinA:%s, Digital PinB:%s, Mode:%s, Enable Pin:%s\r\nDir Pins:0=%s,1=%s,2=%s,3=%s,4=%s,5=%s,6=%s,7=%s,8=%s,9=%s\0",
			dout1, dout2, dout3 ,dout4, dpin0, dpin1, dpin2, dpin3, dpin4, dpin5, dpin6, dpin7, dpin8, dpin9);
	} else {
		sprintf(cout,"SB-Digital PinA:%s, Digital PinB:%s, Mode:%s, Enable Pin:%s\r\nDir Pins:0=%s,1=%s,2=%s,3=%s,4=%s,5=%s,6=%s,7=%s,8=%s,9=%s\0",
			dout1, dout2, dout3 ,dout4, dpin0, dpin1, dpin2, dpin3, dpin4, dpin5, dpin6, dpin7, dpin8, dpin9);
	}
	for(int i=0; i < OUT_BUFFER_SIZE; ++i){
		 outStr[i] = cout[i];
		 if(!outStr[i])
			break;
	}
}
