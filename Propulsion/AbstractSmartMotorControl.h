/* 
* AbstractSmartMotorControl.h
*
* Created: 10/31/2020 12:57:03 PM
* Author: Groff
*/


#ifndef __ABSTRACTSMARTMOTORCONTROL_H__
#define __ABSTRACTSMARTMOTORCONTROL_H__


class AbstractSmartMotorControl : public AbstractMotorControl
{
//functions
public:
	AbstractSmartMotorControl(uint32_t maxPower) {MAXMOTORPOWER = maxPower;}
	~AbstractSmartMotorControl(){};
	void setMinMotorPower(uint8_t ch, uint32_t mpow) {minMotorPower[ch-1] = mpow;}
	void setMaxMotorPower(int p) {MAXMOTORPOWER = p;}
	void setMotorPowerScale(int p) {MOTORPOWERSCALE = p;}
	virtual void resetMaxMotorPower()=0;//set back to the maximum power, subclass sets

}; //AbstractSmartMotorControl

#endif //__ABSTRACTSMARTMOTORCONTROL_H__
