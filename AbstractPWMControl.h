/* 
* AbstractPWMControl.h
* Abstract class to facilitate control of multi channel or single channel PWM based devices that are not motors.
* Used to drive things like LED arrays and pumps that need variable speed controls without accompanying motor functionality
* like ultrasonic shutdown safety or hall effect counters for odometry or dead man switching. Provides the
* functionality of enable pin, etc to use H bridge driver or MOSFET circuit to drive things other than motors.
* IMPORTANT: Channels are referenced by the M and G code as a value from 1-10 but the indexes are from 0-9 in the arrays
* of channels, therefore, always subtract 1 when going from methods that take values from the processing loop, such as:
* commandMotorPower
* getDriverInfo

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
* 

* channel - the controller channel from 1 to 10, each successively higher channel will reset maximum channel count. Each channel is a device on an
* Hbridge split into separate outputs, or a motor driver attached to an LED array, etc.
* pin_number - the index in the PWM array defined in 'setPWM', this is the next blank slot available
* enable_pin - the enable pin for this channel. Assumed that low is disabled, high is enable.
* timer_pre - timer prescale default 1 = no prescale
* timer_res - timer resolution in bits - default 8
* Created: 10/29/2020 9:54:23 AM
* Author: Groff
*/

#ifndef __ABSTRACTPWMCONTROL_H__
#define __ABSTRACTPWMCONTROL_H__
#include "WPWM.h"
#include <stdio.h>
#include "DigitalPin.h"
class AbstractPWMControl
{
private:
	uint channels = 0;

protected:
	int status_flag = 0;
	PWM** ppwms;
	//Digital** pdigitals;
	// 10 possible drive channels, index is by channel-1.
	// pwmDrive[channel] [[PWM array index][dir pin][timer prescale][timer resolution]
	// PWM params array by channel:
	// 0-pin index to pwm array(default 255)
	// 1-enable pin
	// 2-timer prescale (1-none)
	// 3-timer resolution (8 bit)
	uint pwmDrive[10][4]={{255,0,1,8},{255,0,1,8},{255,0,1,8},{255,0,1,8},{255,0,1,8},{255,0,1,8},{255,0,1,8},{255,0,1,8},{255,0,1,8},{255,0,1,8}};
	int32_t maxPWMDuration[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}; // number of milliseconds operation before auto shutdown
	// 10 channels of last PWM value
	int pwmLevel[10] = {0,0,0,0,0,0,0,0,0,0};
	uint32_t minPWMLevel[10] = {0,0,0,0,0,0,0,0,0,0}; // Offset to add to G6, use with care, meant to compensate for electrical differences
	int PWMPOWERSCALE = 0; // scale, divisor for PWM level to reduce 0-1000 scale if non zero
	uint PWMSHUTDOWN = 0; // Override of PWM controls, puts it in irons
	int fault_flag = 0;
//functions
public:
	virtual ~AbstractPWMControl();
	void setDuration(uint ch, uint32_t durx) { maxPWMDuration[ch-1] = durx; }
	void setMinPWMLevel(uint ch, uint32_t mpow) { minPWMLevel[ch-1] = mpow;}
	virtual int commandPWMLevel(uint ch, int p)=0;
	virtual int commandEmergencyStop(int status)=0;
	virtual int isConnected(void)=0;
	virtual void getDriverInfo(uint ch, char* outStr)=0;
	virtual int queryFaultFlag(void)=0;
	virtual int queryStatusFlag(void)=0;
	virtual void setMaxPWMLevel(int p)=0;
	virtual bool usesPWM(uint8_t pin)=0;
	virtual bool usesDigital(uint8_t pin) { return false; }
	uint32_t getMaxPWMDuration(uint ch) { return maxPWMDuration[ch-1]; }
	uint32_t getMinPWMLevel(uint ch) { return minPWMLevel[ch-1] ; }
	int getPWMLevel(uint ch) { return pwmLevel[ch-1]; }
	void setChannels(uint ch) { channels = ch; }
	uint getChannels(void) { return channels; }
	void resetLevels(void);
	void setPWMShutdown(void) { commandEmergencyStop(1); PWMSHUTDOWN = 1;}
	void setPWMRun(void) { commandEmergencyStop(0); PWMSHUTDOWN = 0;}
	uint getPWMShutdown(void) { return PWMSHUTDOWN; }
	void setPWMPowerScale(int p) { PWMPOWERSCALE = p; }
}; //AbstractPWMControl

#endif //__ABSTRACTPWMCONTROL_H__
