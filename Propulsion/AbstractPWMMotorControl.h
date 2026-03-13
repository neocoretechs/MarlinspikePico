/* 
* AbstractPWMMotorControl.h
*
* Created: 10/31/2020 12:57:21 PM
* Author: Groff
*/


#ifndef __ABSTRACTPWMMOTORCONTROL_H__
#define __ABSTRACTPWMMOTORCONTROL_H__
#include <math.h>
#include "AbstractMotorControl.h"

class AbstractPWMMotorControl : public AbstractMotorControl
{
//functions
public:
	AbstractPWMMotorControl(uint32_t maxPower) : AbstractMotorControl(maxPower){}
	virtual ~AbstractPWMMotorControl() = default;
	virtual int commandMotorPower(uint ch, int p)=0;//make AbstractMotorControl not inst
	void setMinMotorPower(uint8_t ch, uint32_t mpow) { 
		if (ch >= 1 && ch <= getChannels())
            minMotorPower[ch - 1] = abs((int)mpow) / 4;
	}
	void setMaxMotorPower(int p) { MAXMOTORPOWER = std::abs(p)/4; }
	void setMotorPowerScale(int p) { MOTORPOWERSCALE = std::abs(p)/4;}
	virtual void resetMaxMotorPower()=0;//set back to the maximum power, subclass sets

}; //AbstractPWMMotorControl

#endif //__ABSTRACTPWMMOTORCONTROL_H__
