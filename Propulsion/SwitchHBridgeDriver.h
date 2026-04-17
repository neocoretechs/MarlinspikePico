/**
* SwitchHBridgeDriver:
* A motor driver that uses straight GPIO switching rather than PWM, for an on/off drive motor setup.
* The SwitchHBridgeDriver extends and differs from the SwitchBridgeDriver in that it uses one direction pin and one signal
* pin vs the 2 half bridge orientation of {@link SwitchBridgeDriver} that takes an enable pin and 2 signal pins 9one for each half bridge)
* Structure:
* 1) Top level, a master list of pins in the digitals array. Slots to which physical pins are assigned.
* 2) In each class and subclass, a sublist of pins. Multidimensional array which holds the index to the top level pins.
* these are indexed by channel.
* 3) A GPIO pin list indexed by channel. The level value is either a high or low for on or off.
* 4) Optionally, a duration. The duration represents a maximum interval before the GPIO pin
* is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
* the top level abstract class AbstractMotorControl contains these values.
* Jonathan Groff Copyright (C) NeoCoreTechs 2022
*
*/
#ifndef __SWITCHHBRIDGEDRIVER_H__
#define __SWITCHHBRIDGEDRIVER_H__
#include "SwitchBridgeDriver.h"
#include <stdio.h>

class SwitchHBridgeDriver : public SwitchBridgeDriver {

	/**
	 * Add a new Digital pin instance to this motor controller.
	 * channel the controller channel from 1 to channels
	 * pin_number the index in the GPIO pin array defined in 'setMotors' for M1

	 * dir_pin the enable pin for this channel 
	 */
	public:
	SwitchHBridgeDriver(int maxPower) : SwitchBridgeDriver(maxPower){};
    void createDigital(uint8_t channel, uint8_t pin_number, uint8_t dir_pin, uint8_t dir_default);
	int commandMotorPower(uint8_t channel, int16_t motorPower) override;
	int commandEmergencyStop(int status) override;
	void getDriverInfo(uint8_t ch, char* outStr) override;
	int checkSafeShutdown(uint slice) override { return 0; }
	void setSafeShutdown(volatile uint8_t* active_mask_buffer) override { return; }
	int get_dma_chan(uint8_t channel) override { return -1; }
	uint get_slice(uint8_t channel) override { return 0; }
    private:
	SwitchHBridgeDriver( const SwitchHBridgeDriver &c ) = delete;
	SwitchHBridgeDriver& operator=( const SwitchHBridgeDriver &c ) = delete;

};
#endif
