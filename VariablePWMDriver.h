/* 
* VariablePWMDriver.h
* Driver to control a PWM device that is not a propulsion motor, such as LED or pump, and as such
* has no recognition of odometry or safety interlock or ultrasonic distance shutdown.
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

#ifndef __VARIABLEPWMDRIVER_H__
#define __VARIABLEPWMDRIVER_H__
#include "AbstractPWMControl.h"
#include <stdlib.h>
class VariablePWMDriver : public AbstractPWMControl
{
//variables
public:
protected:
	int MAXPWMLEVEL = 2000;
private:
//functions
public:
	VariablePWMDriver(): AbstractPWMControl(){};
	int commandPWMLevel(uint8_t ch, int16_t p) override;
	int commandEmergencyStop(int status) override;
	int isConnected(void) override { return true; }
	void setPWMs(PWM** pwm) { ppwms = pwm; }
	void setEnablePins(Digital** dpin) { pdigitals = dpin; }
	void setMaxPWMLevel(int p) override { MAXPWMLEVEL = p; }
	uint getPWMLevelPin(uint channel) { return pwmDrive[channel-1][0]; }
	uint getPWMEnablePin(uint channel) {return pwmDrive[channel-1][1]; }
	int createPWM(uint8_t channel, uint8_t pin_number, uint8_t enable_pin) override;
	void getDriverInfo(uint8_t ch, char* outStr) override;
	int queryFaultFlag(void) override { return fault_flag; }
	int queryStatusFlag(void) override { return status_flag; }
	
	Digital** pdigitals;
	bool usesPWM(uint8_t pin) override {
		for(int i = 0; i < 10; i++) {
			if(pwmDrive[i][0] != 255 && pwmDrive[i][0] && 
				ppwms[pwmDrive[i][0]] && ppwms[pwmDrive[i][0]]->pin == pin)
			 return true;
		}
		return false;
	}
	bool usesDigital(uint8_t pin) {
		for(int i = 0; i < 10; i++) {
			if(pwmDrive[i][1] != 255 && pwmDrive[i][1] && 
				pdigitals[pwmDrive[i][1]] && pdigitals[pwmDrive[i][1]]->pin == pin)
			 return true;
		}
		return false;
	}

}; //VariablePWMDriver

#endif //__VARIABLEPWMDRIVER_H__
