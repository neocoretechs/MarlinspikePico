/*
 * PitchRollHeading.cpp
 *
 * Created: 4/22/2014 6:58:06 PM
 *  Author: jg
 */
#include "IMUInterruptService.h" 
#include "PitchRollHeading.h"

PitchRollHeading::PitchRollHeading() {
		/* Assign a unique ID to the sensors */
		dof   = new Adafruit_10DOF();
		accel = new Adafruit_LSM303_Accel_Unified(LSM303_ADDRESS_ACCEL);
		mag   = new Adafruit_LSM303_Mag_Unified(LSM303_ADDRESS_MAG);
		bmp   = new Adafruit_BMP085_Unified(BMP085_ADDRESS);
		gyro  = new Adafruit_L3GD20_Unified();
		//interruptsActive = false;
		// set up pin change interrupts
		//gyroInt = new PCInterrupts();
		//accelInt = new PCInterrupts();
		//magInt = new PCInterrupts();
		//imuService = new IMUInterruptService(this);
		//gyroIntPin = new Digital(67); // pin 67 for now, put in configs later
		//accelIntPin = new Digital(68);
		//magIntPin = new Digital(52);
		//gyroIntPin->pinMode(INPUT);
		//accelIntPin->pinMode(INPUT);
		//magIntPin->pinMode(INPUT);
		//gyroInt->attachInterrupt(gyroIntPin->pin,imuService,CHANGE);
		//accelInt->attachInterrupt(accelIntPin->pin,imuService,CHANGE);
		//magInt->attachInterrupt(magIntPin->pin,imuService,CHANGE);
		initSensors();
}

void PitchRollHeading::initSensors()
{
		//volatile int a  = accel->begin();
		volatile int b = mag->begin();
		//volatile int c = gyro->begin();
		//gyro->enableAutoRange(true);
		//volatile int d = bmp->begin();
		//interruptsActive = true; // from now on IMUInterruptService can call getPichRollHeading		
}

sensors_vec_t PitchRollHeading::getPitchRollHeading(void)
{
	interruptsActive = false;
	sensors_event_t accel_event;
	sensors_event_t mag_event;
	sensors_event_t bmp_event;
	sensors_event_t gyro_event;
	sensors_vec_t   orientation;

	/* Calculate pitch and roll from the raw accelerometer data */
	//accel->getEvent(&accel_event);
	//volatile bool ago = dof->accelGetOrientation(&accel_event, &orientation);
	/* 'orientation' should have valid .roll and .pitch fields */
	//Serial.print(F("Roll: "));
	//Serial.print(orientation.roll);
	//Serial.print(F("; "));
	//Serial.print(F("Pitch: "));
	//Serial.print(orientation.pitch);
	//Serial.print(F("; "));
	//}
	
	/* Calculate the heading using the magnetometer */
	mag->getEvent(&mag_event);
	volatile bool mgo = dof->magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);
	/* 'orientation' should have valid .heading data now */
	//Serial.print(F("Heading: "));
	//Serial.print(orientation.heading);
	//Serial.print(F("; "));
	//}
	
	//dof->magTiltCompensation(SENSOR_AXIS_Z, &mag_event, &accel_event);
	//volatile bool fgo = dof->fusionGetOrientation(&accel_event, &mag_event, &orientation);
	/*
	gyro->getEvent(&gyro_event);
	volatile float x = gyro_event.gyro.x; //rads/sec
	volatile float y = gyro_event.gyro.y;
	volatile float z = gyro_event.gyro.z;
	*/
	/* Calculate the altitude using the barometric pressure sensor */
	/*
	bmp->getEvent(&bmp_event);
	if (bmp_event.pressure)
	{
		// Get ambient temperature in C
		float temperature;
		bmp->getTemperature(&temperature);
		//Convert atmospheric pressure, SLP and temp to altitude
		//Serial.print(F("Alt: "));
		volatile float pta = bmp->pressureToAltitude(seaLevelPressure,
		bmp_event.pressure,
		temperature);
		//Serial.print(F(" m; "))
		//Serial.print(F("Temp: "));
		//Serial.print(temperature);
		//Serial.print(F(" C"));
	}
	*/
	//Serial.println(F(""));
	//delay(1000);
	//interruptsActive = true;
	return orientation;
}