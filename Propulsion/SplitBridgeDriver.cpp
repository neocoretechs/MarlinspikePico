/* 
* SplitBridgeDriver.cpp
* This is a type of H bridge with 2 separate inputs for M1 and M2, such that is may function as several types of drivers.
* H-bridge, half bridge, or ganged half bridge.
* Both half-bridges may operate independently or they can be ganged together in parallel to
* support approximately double the current of the H-bridge or single half-bridge configuration.
* PA PWM Fwd
* PB PWM Rev
* EA Enable
* ----- Presumably a forward motor and a reverse motor (or circuit for forward, and one for reverse)
* PA PWM Motor 1
* PB PWM Motor 2
* EA Enable Mot 1
* -----
* PA PWM M1
* PB Tie to PA
* EA Enable
*
* Structure:
* 1) Top level, a master list of pins in either the PWM or digitals arrays. Slots to which physical pins are assigned.
* these are ppwms and pdigitals here, which hold pointers to these top level arrays.
* 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
* these are indexed by channel. motorDrive here. All PWM has a pin, a prescale, and a resolution. We standardize the resolution to 8 bits typically.
* Since the M1 and M2 inputs are split, we add an additional motorDriveB multidimensional array for the second input.
* NOTE: The pin assignment for the timer pins is arranged in the index array one after the other, so the assignment for M1 is at pindex
* and the assignment for M2 is at pindex+1. This does NOT mean the pins have to be next to each other, its just that the indexes in
* the arrays are arranged one after the other for convenience.
* 3) A PWM level indexed by channel. The current power or level value of the PWM timer. Standardized to reflect the control applied.
* for instance, if the PS3 controller delivers -1000 to 1000 thats the range. If the controller is using the button that has -1 off and a max of 1000
* then the range is 0 to 2000 to reflect the fact that no reverse operation is going to be in effect for something like a LED or pump.
* 4) Optionally, a duration and a minimum PWM level, or here a minimum motor power. The duration represents a maximum interval before the PWM timer
* is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
* The minimum PWM level is the bottom limit for the PWM value, or in this case the minimum motor power. It is indexed by channel and the value is
* the range that comes from the main controller, before any processing into a timer value.
* the top level abstract class AbstractMotorControl contains these values.
* Created: 10/16/2020 12:40:41 PM, 3/25/2026
* Author: Groff
*/

#include "SplitBridgeDriver.h"

int SplitBridgeDriver::commandEmergencyStop(int status)
{
	HBridgeDriver::commandEmergencyStop(status);
	for(int j=0; j < 10; j++) {
		int pindex = motorDrive[j][1];
		if(pindex != 255) {
			pdigitals[pindex]->digitalWrite(LOW);
		}
		pindex = motorDriveB[j][0];
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
* channel - the controller channel from 1 to 10
* pin_numberA - the index in the PWM array defined in 'setMotors' for M1, this is the next blank slot available
* pin_numberB - the index in the PWM array defined in 'setMotors' for M2
* enb_pin - the enable pin for this channel
* dir_default - the default direction the motor starts in
*/
int SplitBridgeDriver::createPWM(uint8_t channel, uint8_t pin_numberA, uint8_t pin_numberB, uint8_t enable_pin, uint8_t dir_default) {
		// See if pin assigned
	int foundPin = 0;
	int pinSlot = 0;
	for(int i = 0; i < 32; i++) {
		if(!pdigitals[i])
			pinSlot = i;
		else
			if(pdigitals[i]->pin == enable_pin) {
				foundPin = i;
				break;
			}
	}
	// didnt find pin, didnt find a slot
	if(!foundPin && !pinSlot) {
		return -1; // slots full...
	}
		
	if( getChannels() < channel ) 
		setChannels(channel);
	// Set up the digital direction pin
	Digital* dpin;
	if(!foundPin) {
	 	dpin = new Digital(enable_pin);
		pdigitals[pinSlot] = dpin;
	} else
		dpin = pdigitals[foundPin];
	dpin->pinMode(PinMode::OUTPUT);

	int pindex;
	for(pindex = 0; pindex < 9; pindex++) {
		if( !ppwms[pindex] && !ppwms[pindex+1])
		break;
	}
	// did we find candidates?
	if( ppwms[pindex] || ppwms[pindex+1])
		return -2;

	currentDirection[channel-1] = dir_default;
	defaultDirection[channel-1] = dir_default;
			
	motorDrive[channel-1][0] = pindex;
	motorDrive[channel-1][1] = enable_pin;
	//
	motorDriveB[channel-1][0] = pindex+1;
	// determines which input pin PWM signal goes to, motorDrive[0] or motorDriveB[0], which is at pindex, or pindex+1 in ppwms
	motorDriveB[channel-1][1] = dir_default;
	
	PWM* ppinA = new PWM(pin_numberA);
	ppwms[pindex] = ppinA;
	ppwms[pindex]->init();
	//ppwms[pindex]->setSafeShutdown(true, ppwms[pindex]->watchdogMax);
	PWM* ppinB = new PWM(pin_numberB);
	ppwms[pindex+1] = ppinB;
	ppwms[pindex+1]->init();
	//ppwms[pindex+1]->setSafeShutdown(true, ppwms[pindex+1]->watchdogMax);
	return 0;
}
int SplitBridgeDriver::checkSafeShutdown(uint slice) {
	int fault_flag = 0;
	for(int i = 1; i <= getChannels(); i++) {
		int pindex = motorDrive[i-1][0];
		if(pindex == 255)
			continue;
		auto *p = ppwms[pindex];
		if(!p)
			continue;
		bool safe = p->safeShutdown;
		bool slice_ok = (p->get_slice() == slice);
		int64_t on_time = p->get_on_time_us();
		int64_t watchdog = (int64_t)p->watchdogMax;
		//if(pindex != 255 && ppwms[pindex] && ppwms[pindex]->safeShutdown && 
		//	ppwms[pindex]->get_slice() == slice && ppwms[pindex]->get_on_time_us() > ppwms[pindex]->watchdogMax) {
		if( safe && slice_ok && on_time > watchdog) {
			ppwms[pindex]->pwmOff();
			fault_flag |= (1 << (i-1)); // set bit for this channel
		}
		pindex = motorDriveB[i-1][0];
		if(pindex == 255)
			continue;
		p = ppwms[pindex];
		if(!p)
			continue;
		safe = p->safeShutdown;
		slice_ok = (p->get_slice() == slice);
		on_time = p->get_on_time_us();
		watchdog = (int64_t)p->watchdogMax;
		//if(pindex != 255 && ppwms[pindex] && ppwms[pindex]->safeShutdown && 
		//	ppwms[pindex]->get_slice() == slice && ppwms[pindex]->get_on_time_us() > ppwms[pindex]->watchdogMax) {
		if( safe && slice_ok && on_time > watchdog) {
			ppwms[pindex]->pwmOff();
			fault_flag |= (1 << (i-1)); // set bit for this channel
		}
	}
	return fault_flag;
}
void SplitBridgeDriver::setSafeShutdown(volatile uint8_t* active_mask_buffer) {
	for(int i = 1; i <= getChannels(); i++) {
		int pindex = motorDrive[i-1][0];
		if(pindex != 255 && ppwms[pindex]) {
				ppwms[pindex]->setSafeShutdown(true, ppwms[pindex]->watchdogMax);
				ppwms[pindex]->setup_slice_dma(active_mask_buffer);
		}
		pindex = motorDriveB[i-1][0];
		if(pindex != 255 && ppwms[pindex] ) {
				ppwms[pindex]->setSafeShutdown(true, ppwms[pindex]->watchdogMax);
				ppwms[pindex]->setup_slice_dma(active_mask_buffer);
		}
	}
}
/*
* Command the bridge driver power level. Manage enable pin. If necessary limit min and max power and
* scale to the MOTORPOWERSCALE if > 0. After calculation and saved values in the 0-1000 range scale it to 0-255 for 8 bit PWM.
*/
int SplitBridgeDriver::commandMotorPower(uint8_t motorChannel, int16_t motorPower) {
	// check shutdown override
	if( MOTORSHUTDOWN )
		return 0;
	int foundPin = 0;

	// set enable pin
	for(int i = 0; i < 32; i++) {
			if(pdigitals[i] && pdigitals[i]->pin == motorDrive[motorChannel-1][1]) {
				//pdigitals[i]->setPin(motorDrive[motorChannel-1][1]);
				pdigitals[i]->pinMode(PinMode::OUTPUT);
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
		return commandEmergencyStop(4);
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
	// If we have a linked distance sensor. check range and possibly skip
	// If we are setting power 0, we are stopping anyway
	if( !checkUltrasonicShutdown()) {
		// find the PWM pin and get the object we set up in M3 to write to power level
		// element 0 of motorDrive has index to PWM array
		int pindex = motorDrive[motorChannel-1][0];
		// writing power 0 sets mode 0 and timer turnoff
		if(motorPower > 0) {
			ppwms[pindex]->init();
			//ppwms[pindex]->attachInterrupt(motorDurationService[motorChannel-1]);// last param TRUE indicates an overflow interrupt
			ppwms[pindex]->pwmWrite(true, motorPower);
			motorSpeed[motorChannel-1] = motorPower; //why? +/-
			pindex = motorDriveB[motorChannel-1][1];
			ppwms[pindex]->init();
			//ppwms[pindex]->attachInterrupt(motorDurationService[motorChannel-1]);// last param TRUE indicates an overflow interrupt
			ppwms[pindex]->pwmWrite(false, motorPower);
		} else { // motorPower < 0
			ppwms[pindex]->init();
			//ppwms[pindex]->attachInterrupt(motorDurationService[motorChannel-1]);// last param TRUE indicates an overflow interrupt
			ppwms[pindex]->pwmWrite(false, motorPower);
			motorSpeed[motorChannel-1] = motorPower; //why? +/-
			pindex = motorDriveB[motorChannel-1][1];
			ppwms[pindex]->init();
			//ppwms[pindex]->attachInterrupt(motorDurationService[motorChannel-1]);// last param TRUE indicates an overflow interrupt
			ppwms[pindex]->pwmWrite(true, motorPower);
		}
		fault_flag = 0;
		return fault_flag;
	}
	fault_flag = 16;
	return 0;
}

void SplitBridgeDriver::getDriverInfo(uint8_t ch, char * outStr) {
	char cout[OUT_BUFFER_SIZE];
	char dout1[10];
	char dout2[10];
	char dout4[10];
	
	if( motorDrive[ch-1][0] == 255 ) {
		itoa(-1, dout1, 10);
	} else {
		itoa(ppwms[motorDrive[ch-1][0]]->pin, dout1, 10);
	}
	if( motorDriveB[ch-1][0] == 255 ) {
		itoa(-1, dout2, 10);
	} else {
		itoa(ppwms[motorDriveB[ch-1][0]]->pin, dout2, 10);
	}
	itoa(motorDriveB[ch-1][1], dout4, 10);

	if( motorDrive[ch-1][0] == 255 ) {
		sprintf(cout,"SB-PWM CHANNEL UNITIALIZED PinA:%s, PWM PinB:%s, Enable Pin:%s\r\n\0", dout1, dout2, dout4);
	} else {
		sprintf(cout,"SB-PWM PinA:%s, PWM PinB:%s, Enable Pin:%s\r\n\0", dout1, dout2, dout4);
	}
	for(int i=0; i < OUT_BUFFER_SIZE; ++i){
		 outStr[i] = cout[i];
		 if(!outStr[i])
			break;
	 }
}

//SplitBridgeDriver splitBridgeDriver;