/*
 * IMUInterruptService.h
 * General interrupt service servicing IMU interrupt events from gyro, magnetometer, accelerometer
 * Created: 4/22/2014 6:32:40 PM
 * Author: jg
 */ 

#ifndef IMUINTERRUPTSERVICE_H_
#define IMUINTERRUPTSERVICE_H_
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include "WInterruptService.h"
#include "pico/stdlib.h"
#include "pins_pico/stdlib.h"
#include "PitchRollHeading.h"
#include "Adafruit_Sensor.h"

class IMUInterruptService: public InterruptService {
	private:

	public:
	PitchRollHeading* imu;
	sensors_vec_t orientation;
	IMUInterruptService(PitchRollHeading* imu) { this->imu = imu; }
	//Pin Change Interrupt Service Routine.
	void IMUInterruptService::service(void)
	{
		if( imu->interruptsActive) {
			orientation = imu->getPitchRollHeading();
		}
	}
	
	sensors_vec_t getOrientation() { return orientation; }

};


#endif /* IMUINTERRUPTSERVICE_H_ */