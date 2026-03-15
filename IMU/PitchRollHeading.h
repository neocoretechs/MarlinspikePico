/*
 * PitchRollHeading.h
 *
 * Created: 4/20/2014 10:04:44 PM
 *  Author: jg
 */ 

#ifndef PITCHROLLHEADING_H_
#define PITCHROLLHEADING_H_

#include "../WInterruptService.h"
#include "../DigitalPin.h"
#include "Adafruit_10DOF.h"

class PitchRollHeading {
	public:
	InterruptService *imuService;
	InterruptService *gyroInt;
	InterruptService *accelInt;
	InterruptService *magInt;
	Digital* gyroIntPin;
	Digital* accelIntPin;
	Digital* magIntPin;
	/* Update this with the correct SLP for accurate altitude measurements */
	float seaLevelPressure = 0;//SENSORS_PRESSURE_SEALEVELHPA;
	bool interruptsActive;
	Adafruit_10DOF* dof;
	Adafruit_LSM303_Accel_Unified* accel;
	Adafruit_LSM303_Mag_Unified* mag;
	Adafruit_BMP085_Unified* bmp;
	Adafruit_L3GD20_Unified* gyro;
	Wire* wire;
    uint8_t _addr;
    int32_t _sensorID;
	//PitchRollHeading(Wire* w) : wire(w){}

/*************************************************************************
    Initializes all the sensors used by this example
*************************************************************************/
    PitchRollHeading(Wire *w);
    void initSensors();

    /*************************************************************************
        Get the roll/pitch/heading/altitude/temperature
    **************************************************************************/
    sensors_vec_t getPitchRollHeading(void);
};

#endif /* PITCHROLLHEADING_H_ */