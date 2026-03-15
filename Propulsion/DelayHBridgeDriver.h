/* 
* DelayHBridgeDriver.h
* Generic driver for a collection of H bridge driven brushed DC motor channels.
* The DelayHBridgeDriver creates a deal between direction change to allow the 
* motor to stop before reversing direction. This is to protect the motor and the driver from damage due to rapid direction changes.
* Structure:
* 1) Top level, a master list of pins in either the PWM or digitals arrays. Slots to which physical pins are assigned.
* these are ppwms and pdigitals here, which hold pointers to these top level arrays.
* 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
* these are indexed by channel. motorDrive here. All PWM has a pin, a prescale, and a resolution. We standardize the resolution to 8 bits typically.
* 3) A PWM level indexed by channel. The current power or level value of the PWM timer. Standardized to reflect the control applied.
* for instance, if the PS3 controller delivers -1000 to 1000 thats the range. If the controller is using the button that has -1 off and a max of 1000
* then the range is 0 to 2000 to reflect the fact that no reverse operation is going to be in effect for something like a LED or pump.
* 4) Optionally, a duration and a minimum PWM level, or here a minimum motor power. The duration represents a maximum interval before the PWM timer
* is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
* The minimum PWM level is the bottom limit for the PWM value, or in this case the minimum motor power. It is indexed by channel and the value is
* the range that comes from the main controller, before any processing into a timer value.
* the top level abstract class AbstractMotorControl contains these values.
* Created: 3/11/2026 1:42:24 PM
* Author: Jonathan Grof Copyright (C) NeoCoreTechs 2026
*/

#ifndef __DELAYHBRIDGEDRIVER_H__
#define __DELAYHBRIDGEDRIVER_H__
#include "HBridgeDriver.h"
#include <stdio.h>

class DelayHBridgeDriver : public HBridgeDriver
{
//variables
public:
protected:
//functions
private:
public:
	DelayHBridgeDriver(int maxPower) : HBridgeDriver(maxPower){};
	int commandMotorPower(uint8_t ch, int16_t p);
	void getDriverInfo(uint8_t ch, char* outStr);
protected:
private:
	DelayHBridgeDriver( const DelayHBridgeDriver &c );
	DelayHBridgeDriver& operator=( const DelayHBridgeDriver &c );

}; //DelayHBridgeDriver
//extern DelayHBridgeDriver delayHBridgeDriver;
#endif //__DELAYHBRIDGEDRIVER_H__
