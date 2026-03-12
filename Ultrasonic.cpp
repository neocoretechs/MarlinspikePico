/*
 * Ultrasonic.cpp
 *
 * Created: 7/25/2014 11:42:28 PM
 *  Author: jg
 */
#include "Ultrasonic.h"
#include <time.h>

#define LOW 0
#define HIGH 1
float Ultrasonic::getRange()
{
	// The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
	// Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
	pin->pinMode(OUTPUT);
	pin->digitalWrite(LOW);
	sleep_us(2);
	pin->digitalWrite(HIGH);
	sleep_us(5);
	pin->digitalWrite(LOW);

	// The same pin is used to read the signal from the PING))): a HIGH
	// pulse whose duration is the time (in microseconds) from the sending
	// of the ping to the reception of its echo off of an object.
	pin->pinMode(INPUT);
	duration = pin->pulseIn(HIGH);

	// convert the time into a distance
	return microsecondsToCentimeters();// return centimeters
}

/*The measured distance from the range 0 to 400 Centimeters*/
long Ultrasonic::microsecondsToCentimeters(void)
{
	return duration/29/2;	
}
/*The measured distance from the range 0 to 157 Inches*/
long Ultrasonic::microsecondsToInches(void)
{
	return duration/74/2;	
}
