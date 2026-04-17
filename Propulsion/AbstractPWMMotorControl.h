/* 
* AbstractPWMMotorControl.h
*
* Created: 10/31/2020 12:57:21 PM
* Author: Groff
*/


#ifndef __ABSTRACTPWMMOTORCONTROL_H__
#define __ABSTRACTPWMMOTORCONTROL_H__
#include <math.h>
#include <atomic>
#include "AbstractMotorControl.h"

class AbstractPWMMotorControl : public AbstractMotorControl
{
//functions
public:
	AbstractPWMMotorControl(int maxPower) : AbstractMotorControl(maxPower){}
	virtual int commandMotorPower(uint8_t ch, int16_t p)=0;//make AbstractMotorControl not inst
	void setMinMotorPower(uint8_t ch, int mpow) { 
		if (ch >= 1 && ch <= getChannels())
            minMotorPower[ch - 1] = mpow;
	}
	void setMaxMotorPower(int p) override { MAXMOTORPOWER = p; }
	void setMotorPowerScale(int p) override { MOTORPOWERSCALE = p;}
	virtual void resetMaxMotorPower()=0;//set back to the maximum power, subclass sets
	virtual int createPWM(uint8_t channel, uint8_t pin_number, uint8_t dir_pin, uint8_t dir_default)=0;
	virtual int setSafeShutdown(volatile uint8_t* active_mask_buffer) = 0;
	virtual int get_dma_chan(uint8_t channel)=0;
	virtual int get_slice(uint8_t channel)=0;
}; //AbstractPWMMotorControl

#endif //__ABSTRACTPWMMOTORCONTROL_H__
