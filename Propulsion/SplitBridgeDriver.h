/* 
* SplitBridgeDriver.h
* This is a driver for an H bridge that has separate M1 and M2 inputs.
* Structure:
* 1) Top level, a master list of pins in either the PWM or digitals arrays. Slots to which physical pins are assigned.
* these are ppwms and pdigitals here, which hold pointers to these top level arrays.
* 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
* these are indexed by channel. motorDrive here. All PWM has a pin, 
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
#ifndef __SPLITBRIDGEDRIVER_H__
#define __SPLITBRIDGEDRIVER_H__
#include "HBridgeDriver.h"
class SplitBridgeDriver : public HBridgeDriver
{
//variables
public:
protected:
private:
	uint8_t motorDriveB[10][4]={{255,0,1,8},{255,0,1,8},{255,0,1,8},{255,0,1,8},{255,0,1,8},{255,0,1,8},{255,0,1,8},{255,0,1,8},{255,0,1,8},{255,0,1,8}};
//functions
public:
	SplitBridgeDriver(int maxPower) : HBridgeDriver(maxPower){};
	int commandEmergencyStop(int status);
	int createPWM(uint8_t channel, uint8_t pin_numberA, uint8_t pin_numberB, uint8_t enb_pin, uint8_t dir_default);
	int commandMotorPower(uint8_t motorChannel, int16_t motorPower) override;
	uint8_t getMotorPWMPinB(uint8_t channel) { return motorDriveB[channel-1][0]; }
	void getDriverInfo(uint8_t ch, char* outStr) override;
protected:
private:
	SplitBridgeDriver( const SplitBridgeDriver &c ) = delete;
	SplitBridgeDriver& operator=( const SplitBridgeDriver &c ) = delete;

}; //SplitBridgeDriver

#endif //__SPLITBRIDGEDRIVER_H__
