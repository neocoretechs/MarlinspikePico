/* 
* VariablePWMDriver.cpp
* IMPORTANT: Channels are referenced by the M and G code as a value from 1-10 but the indexes are from 0-9 in the arrays
* of channels, therefore, always subtract 1 when going from methods that take values from the processing loop, such as:
* commandMotorPower
* getDriverInfo
*
* Structure: 
* 1) Top level, a master list of pins in either the PWM or digitals arrays. Slots to which physical pins are assigned.
* these are ppwms and pdigitals here, which hold pointers to these top level arrays.
* 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
* these are indexed by channel. PWMDrive here. All PWM has a pin, a prescale, and a resolution. We standardize the resolution to 8 bits typically.
* 3) A PWM level indexed by channel. The current power or level value of the PWM timer. Standardized to reflect the control applied.
* for instance, if the PS3 controller delivers -1000 to 1000 thats the range. If the controller is using the button that has -1 off and a max of 1000
* then the range is 0 to 2000 to reflect the fact that no reverse operation is going to be in effect for something like a LED or pump.
* 4) Optionally, a PWM duration and a minimum PWM level. The PWM duration represents a maximum time interval before the PWM timer
* is automatically shut down. The minimum PWM level is the bottom limit for the PWM value. It is indexed by channel and the value is
* the range that comes from the main controller, before any processing into a timer value.
* These arrays can be located in the top level abstract class AbstractPWMControl.
* Created: 10/29/2020 3:15:01 PM
* Author: Groff
*/
#include "VariablePWMDriver.h"
#include "Configuration_adv.h"

int VariablePWMDriver::commandEmergencyStop(int status) {
	for(int j=0; j < 10; j++) {
		int pindex = pwmDrive[j][0];
		if(pindex != 255) {
				pdigitals[pindex]->digitalWrite(LOW);
		}
		pindex = pwmDrive[j][0];
		if(pindex != 255) {
			ppwms[pindex]->init(ppwms[pindex]->pin);
			ppwms[pindex]->pwmOff();
		}
	}
	fault_flag = 16;
	resetLevels();
	return status;
}
/*
* Add a new PWM instance to this controller.
* channel - the controller channel from 1 to 10, each successively higher channel will reset maximum channel count. Each channel is a device on an 
* Hbridge split into separate outputs, or a motor driver attached to an LED array, etc.
* pin_number - the index in the PWM array defined in 'setPWM', this is the next blank slot available
* enable_pin - the enable pin for this channel. Assumed that low is disabled, high is enable.
* timer_pre - timer prescale default 1 = no prescale
* timer_res - timer resolution in bits - default 8
*/
void  VariablePWMDriver::createPWM(uint channel, uint pin_number, uint enable_pin, int timer_pre, int timer_res) {
	// Attempt to assign PWM pin, lock to 8 bits no prescale, mode 2 CTC
	if( getChannels() < channel ) setChannels(channel);
	int foundPin = 0;
	Digital* dpin = new Digital(enable_pin);
	dpin->pinMode(PinMode::OUTPUT);
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
		
	// find slot for new PWM pin and init
	int pindex;
	for(pindex = 0; pindex < 10; pindex++) {
		if( !ppwms[pindex] )
			break;
	}
	if( ppwms[pindex] ) // already assigned, slots full
			return;
				
	pwmDrive[channel-1][0] = pindex;
	pwmDrive[channel-1][1] = enable_pin;
	pwmDrive[channel-1][2] = timer_pre;
	pwmDrive[channel-1][3] = timer_res;
	PWM* ppin = new PWM(pin_number);
	ppwms[pindex] = ppin;
	ppwms[pindex]->init(pin_number);
}
/*
* Command the driver power level. Manage enable pin. If necessary limit min and max power and
* scale to the SCALE if > 0. After calculation and saved values in the 0-2000 range scale it to 0-255 for 8 bit PWM.
* Instead of our -1000 to 1000 range from stick we change it to 0-2000 by adding 1000 since no reverse is relevant.
* In the case of Ps3 controller the axis will have the zero point of the stick as halfway, and full stick back
* as 0, in the case of a trigger, the output is from -1 to 1 with no output being -1, so it will function as desired and expected.
* Each channel is a PWM driven device.
*/
int VariablePWMDriver::commandPWMLevel(uint8_t pwmChannel, int16_t pwmPower) {
	// check shutdown override
	if( PWMSHUTDOWN )
		return 0;
	int foundPin = 0;
	pwmPower += 1000;
	pwmLevel[pwmChannel-1] = pwmPower;
	// get mapping of channel to pin
	for(int i = 0; i < 10; i++) {
		if(pdigitals[i] && pdigitals[i]->pin ==  pwmDrive[pwmChannel-1][1]) {
				//pdigitals[i]->setPin(motorDrive[motorChannel-1][1]);
				pdigitals[i]->pinMode(PinMode::OUTPUT);
				pdigitals[i]->digitalWrite(HIGH);
				foundPin = 1;
				break;
		}
	}
	if(!foundPin) {
		return commandEmergencyStop(7);
	}                                                                                                                                                     
	// scale motor power from 0-2000 to our 0-255 8 bit timer val
	pwmPower /= 8;
	//
	if( pwmPower != 0 && pwmPower < minPWMLevel[pwmChannel-1])
		pwmPower = minPWMLevel[pwmChannel-1];
	if( pwmPower > MAXPWMLEVEL ) // cap it at max
		pwmPower = MAXPWMLEVEL;
	// Scale motor power if necessary and save it in channel speed array with proper sign for later use
	if( PWMPOWERSCALE != 0 )
		pwmPower /= PWMPOWERSCALE;
	//
	// find the PWM pin and get the object we set up in M3 to write to power level
	//int timer_mode = 2;
	//int timer_pre = pwmDrive[pwmChannel-1][2]; // prescale from M3
	//int timer_res = pwmDrive[pwmChannel-1][3]; // timer resolution in bits from M3
	// element 0 of motorDrive has index to PWM array
	int pindex = pwmDrive[pwmChannel-1][0];
	// writing power 0 sets mode 0 and timer turnoff
	ppwms[pindex]->init(ppwms[pindex]->pin);
	//ppwms[pindex]->attachInterrupt(motorDurationService[motorChannel-1]);// last param TRUE indicates an overflow interrupt
	ppwms[pindex]->pwmWrite(true, pwmPower);
	fault_flag = 0;
	return 0;
}

void VariablePWMDriver::getDriverInfo(uint8_t ch, char* outStr) {
	char cout[OUT_BUFFER_SIZE];
	char dout1[5];
	char dout2[5];
	char dout3[5];
	char dout4[5];
	char dout5[5];
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
	if( pwmDrive[ch-1][0] == 255 ) {
		itoa(-1, dout1, 10);
		itoa(-1, dout2, 10);
	} else {
		itoa(ppwms[pwmDrive[ch-1][0]]->pin, dout1, 10);
	}	
	itoa(pwmDrive[ch-1][1], dout3, 10);
	itoa(pwmDrive[ch-1][2], dout4, 10);
	itoa(pwmDrive[ch-1][3], dout5, 10);
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
	if( pwmDrive[ch-1][0] == 255 ) {
		sprintf(cout,"VP-PWM UNITIALIZED Pin:%s, Mode:%s, Enable Pin:%s, Timer Prescale:%s, Timer Res.:%s\r\nDir Pins:0=%s,1=%s,2=%s,3=%s,4=%s,5=%s,6=%s,7=%s,8=%s,9=%s\0",
			dout1, dout2 , dout3 , dout4 , dout5, dpin0, dpin1, dpin2, dpin3, dpin4, dpin5, dpin6, dpin7, dpin8, dpin9);
	} else {
		sprintf(cout,"VP-PWM Pin:%s, Mode:%s, Enable Pin:%s, Timer Prescale:%s, Timer Res.:%s\r\nDir Pins:0=%s,1=%s,2=%s,3=%s,4=%s,5=%s,6=%s,7=%s,8=%s,9=%s\0",
			dout1, dout2 , dout3 , dout4 , dout5, dpin0, dpin1, dpin2, dpin3, dpin4, dpin5, dpin6, dpin7, dpin8, dpin9);
	}
	 for(int i=0; i < OUT_BUFFER_SIZE; ++i){
		 outStr[i] = cout[i];
		 if(!outStr[i])
			 break;
	 }
}

// default constructor
VariablePWMDriver::VariablePWMDriver()
{
} //VariablePWMDriver
