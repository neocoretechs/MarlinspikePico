/* 
* AbstractMotorControl.cpp
* Class to maintain the abstract collection of propulsion channels that comprise the traction power.
* Be they brushed motors driven by H-bridge, brushless DC, or a smart controller that uses a high level AT command protocol
* Controllers are channel-oriented. They manage collection of objects representing channels, or conceptually, wheels.
* IMPORTANT: Channels are referenced by the M and G code as a value from 1-10 but the indexes are from 0-9 in the arrays
* of channels, therefore, always subtract 1 when going from methods that take values from the processing loop, such as:
* commandMotorPower
* getDriverInfo
*
* Structure:
* 1) Since we can drive motors via PWM or switched on/off GPIO pins, with either split inputs with separate enable pins or
* one enable pin on a forward/reverse controller, we delegate those functions to the subclasses.
* 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
* these are indexed by channel. ultrasonicIndex, minMotorDist, etc. here. All PWM has a pin, a prescale, and a resolution.
* We standardize the resolution to 8 bits typically.
* 3) Optionally, a duration and a minimum motor power. The duration represents a an abstraction of the maximum interval before the device
* is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
* The minimum motor power is an unsigned value that is added to the base power level typically to compensate for differences in
* motor integrity affecting values that represent the same speed on different channels.
* 4) The motorSpeed is indexed by channel and the value is the range that comes from the main controller, before any processing into a timer value.
* 5) the current direction and default direction have different meanings depending on subclass.
* Created: 10/2/2016 12:53:49 PM
* Author: jg
*/

#include "AbstractMotorControl.h"

/*
* Link ultrasonic sensor to controller. Modify element in ultrasonicIndex array to point to Ultrasonic object.
* us - the Ultrasonic sensor array
* upin - the pin we are linking to, in the element array, if not there we ignore
* distance - min distance before shutdown
* facing - 1 the sensor is facing forward (default), 0 - sensor facing backward
*/
void AbstractMotorControl::linkDistanceSensor(Ultrasonic** us, uint8_t upin, uint32_t distance, uint8_t facing) {
	usensor = us;
	for(int i = 0; i < 10; i++) {
		if( us[i] && us[i]->getPin() == upin ) {
			ultrasonicIndex[i][0] = i;
			ultrasonicIndex[i][1] = facing;
			minMotorDist[i] = distance;
			return;
		}
	}	
}
/*
* check all linked ultrasonic sensors, if something is in minimum range, and it is in the direction
* of current travel as defined by the currentDirection array and the direction the sensor is facing, shut down all channels.
* First check to see if any channels are active.
* The premise is that the distance from the sensor to 'front' of robot is set to prevent impact
*/
bool AbstractMotorControl::checkUltrasonicShutdown() {
		bool shutdown = false;
		for(int i = 0; i < 10; i++)
			if( motorSpeed[i] != 0 ) {
				break;
			}
		if( shutdown )
			return shutdown;
		// If we have a linked distance sensor. check range and possibly skip
		// ultrasonicIndex corresponds to ultrasonic object pointer array, element 0 points to Ultrasonic array element
		for(int i = 0; i < 10; i++) {
			if( ultrasonicIndex[i][0] != 255 ) {
				// does the direction of sensor match movement direction?
				// we stop if Moving backwards with backward facing sensor or forward with forward facing
				// If motor is mirrored such the speed commands are reversed, then default direction should initially be 1
				// So the decision to stop is based on distance from obstacle, the current direction of travel,
				// the desired direction of travel, and the way the sensor is facing.
				if( !currentDirection[i] && !ultrasonicIndex[i][1] ||
					 currentDirection[i] && ultrasonicIndex[i][1] ) {
					if( usensor[ultrasonicIndex[i][0]]->getRange() < minMotorDist[i] ) {
						//commandEmergencyStop();
						shutdown = true;
						break;
					}
				}
			}
		}
		if( shutdown ) commandEmergencyStop(8);
		return shutdown;
}

void AbstractMotorControl::createEncoder(uint8_t channel, uint8_t encode_pin) {
		wheelEncoderService[channel-1] = new CounterInterruptService(maxMotorDuration[channel-1]);
		wheelEncoder[channel-1] = new PCInterrupts();
		wheelEncoder[channel-1]->attachInterrupt(encode_pin, wheelEncoderService[channel-1], CHANGE);
}
/*
* If we are using an encoder check the interval since last command.
* Interrupt service counter counts number of timer compare match resets.
* If number is exceeded issue shutdown and await next G5.
* This shutdown is to prevent unchecked freewheeling.
*/
bool AbstractMotorControl::checkEncoderShutdown() {
	bool running = false;
	for(int i = 0; i < 10; i++)
		if( motorSpeed[i] != 0 ) {
			running = true;
			break;
		}
	if( !running )
		return running;
	for(int j = 0; j < 10; j++) { // by channel
		if( wheelEncoderService[j] ) {
			  int cntxmd = wheelEncoderService[j]->get_counter();
			  if( cntxmd >= maxMotorDuration[j] ) {
					commandEmergencyStop(10);
					return true;
			  }
		}
	}
	return false;
}

int AbstractMotorControl::getEncoderCount(uint8_t ch) {
	if( wheelEncoderService[ch-1] )
		return wheelEncoderService[ch-1]->get_counter();
	return -1;
}

void AbstractMotorControl::resetEncoders(void) {
	for(int i = 0; i < 10; i++) {
		if( wheelEncoderService[i] ) {
			wheelEncoderService[i]->set_counter(0);
		}
	}
}

void AbstractMotorControl::resetSpeeds(void) {
	for(int i = 0; i < 10; i++) motorSpeed[i] = 0; // all channels down
}

// virtual destructor
AbstractMotorControl::~AbstractMotorControl() {} //~AbstractMotorControl
