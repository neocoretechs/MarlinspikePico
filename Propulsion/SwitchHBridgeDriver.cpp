	#include "SwitchHBridgeDriver.h"
    /**
	 * Add a new Digital pin instance to this motor controller.
	 * param channel the controller channel from 1 to channels
	 * param pin_number the index in the GPIO pin array defined in 'setMotors' for M1
	 * param dir_pin the enable pin for this channel
	 */
	void SwitchHBridgeDriver::createDigital(uint8_t channel, uint8_t pin_number, uint8_t dir_pin, uint8_t dir_default) {
	    if( getChannels() < channel )
	        setChannels(channel);
	    // Set up the digital direction pin, we want to be able to re-use these pins for multiple channels on 1 controller
	    Digital* dpin = new Digital(dir_pin);
	    dpin->pinMode(OUTPUT);
	    int pindex = 0;
	    for(; pindex < 9; pindex++) {
		    if(!pdigitals[pindex])
		    break;
	    }
	    if(pdigitals[pindex]) {
            delete dpin;
			return;
        }
	    currentDirection[channel-1] = dir_default;
	    defaultDirection[channel-1] = dir_default;
	    motorDrive[channel-1][1] = dir_pin;
	// determines which input pin PWM signal goes to, motorDrive[0], which is at pindex, or pindex+1 in ppwms
	pdigitals[pindex] = dpin;
	pdigitals[pindex]->pinMode(OUTPUT);
    Digital* ppinA = new Digital(pin_number);
    pindex = 0;
	for(; pindex < 9; pindex++) {
		if(!pdigitals[pindex])
		break;
	}
	if(pdigitals[pindex]) {
        delete ppinA;
		return;
    }
	pdigitals[pindex] = ppinA;
	pdigitals[pindex]->pinMode(OUTPUT);
    motorDrive[channel-1][0] = pindex;
	}
	
	int SwitchHBridgeDriver::commandEmergencyStop(int status) {
		for(int j=1; j <= getChannels(); j++) {
			int pindex = getMotorDigitalPin(j);
			if(pindex != 255) {
				pdigitals[pindex]->pinMode(PinMode::OUTPUT);// enable off
				pdigitals[pindex]->digitalWrite(LOW);
			}
		}
		fault_flag = status;
		resetSpeeds();
		resetEncoders();
		return status;
	}
/*
* Command the power level. Manage enable pin. This inherits from AbstractMotorControl, but the motorPower is merely a +/- 
* forward/back value since we have a switched on/off control. It always max on forward, max on back
*/
int SwitchHBridgeDriver::commandMotorPower(uint8_t motorChannel, int16_t motorPower) {
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
		// turn off all pins
		pdigitals[pindex]->digitalWrite(LOW);
	}
	motorSpeed[motorChannel-1] = motorPower; //why? +/-
	fault_flag = 0;
	return 0;
}

void SwitchHBridgeDriver::getDriverInfo(uint8_t ch, char* outStr) {
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
		sprintf(cout,"SHB-Digital UNITIALIZED Pin:%s, Mode:%s, Enable Pin:%s\r\nDir Pins:0=%s,1=%s,2=%s,3=%s,4=%s,5=%s,6=%s,7=%s,8=%s,9=%s\0",
			dout1, dout3 ,dout4, dpin0, dpin1, dpin2, dpin3, dpin4, dpin5, dpin6, dpin7, dpin8, dpin9);
	} else {
		sprintf(cout,"SHB-Digital Pin:%s, Mode:%s, Enable Pin:%s\r\nDir Pins:0=%s,1=%s,2=%s,3=%s,4=%s,5=%s,6=%s,7=%s,8=%s,9=%s\0",
			dout1, dout3 ,dout4, dpin0, dpin1, dpin2, dpin3, dpin4, dpin5, dpin6, dpin7, dpin8, dpin9);
	}
	for(int i=0; i < OUT_BUFFER_SIZE; ++i){
		 outStr[i] = cout[i];
		 if(!outStr[i])
			break;
	}
}
