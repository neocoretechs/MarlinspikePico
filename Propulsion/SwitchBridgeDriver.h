/* 
* SwitchBridgeDriver.h
* A motor driver that uses straight GPIO switching rather than PWM, for an on/off drive motor setup.
*
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
* Author: Groff
*/
#ifndef __SWITCHBRIDGEDRIVER_H__
#define __SWITCHBRIDGEDRIVER_H__
#include "AbstractMotorControl.h"
#include <stdio.h>

class SwitchBridgeDriver : public AbstractMotorControl
{
//variables
public:
protected:
	Digital** pdigitals;
	// 5 possible drive wheels, index is by channel-1.
	// motorDrive[channel] [[Digitals array index][dir pin]
	// PWM params array by channel:
	// 0-pin index to Digital pins array(default 255)
	// 1-direction pin
	uint8_t motorDrive[10][2]={{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0}};
	uint8_t motorDriveB[10][2]={{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0}};
	int status_flag = 0;
private:

//functions
public:
	SwitchBridgeDriver(int maxPower) : AbstractMotorControl(maxPower){};
	uint8_t getMotorDigitalPin(uint8_t channel) { return motorDrive[channel-1][0]; }
	uint8_t getMotorDigitalPinB(uint8_t channel) { return motorDriveB[channel-1][0]; }
	int commandMotorPower(int16_t p[10]) override;
	int commandEmergencyStop(int status) override;
	int isConnected(void) override { return true; }
	void setPins(Digital** pins) { pdigitals = pins; }
	uint8_t getMotorEnablePin(uint8_t channel) {return motorDrive[channel-1][1]; }
	void createDigital(uint8_t channel, uint8_t pin_number, uint8_t pin_numberB, uint8_t dir_pin, uint8_t dir_default);
	void getDriverInfo(uint8_t ch, char* outStr) override;
	int queryFaultFlag(void) override { return fault_flag; }
	int queryStatusFlag(void) override { return status_flag; }
	int checkSafeShutdown() override { return 0; }
	int get_dma_chan(uint8_t channel) override { return -1; }
	int get_slice(uint8_t channel) override { return 0; }
protected:
private:
	SwitchBridgeDriver( const SwitchBridgeDriver &c ) = delete;
	SwitchBridgeDriver& operator=( const SwitchBridgeDriver &c ) = delete;

}; //SwitchBridgeDriver

#endif //__SWITCHBRIDGEDRIVER_H__
