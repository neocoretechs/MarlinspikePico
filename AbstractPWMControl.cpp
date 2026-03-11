/* 
* AbstractPWMControl.cpp
* Abstract class to facilitate control of multi channel or single channel PWM based devices that are not motors.
* Used to drive things like LED arrays and pumps that need variable speed controls WITHOUT accompanying motor functionality
* like ultrasonic shutdown safety or hall effect counters for odometry or dead man switching. Provides the
* functionality of enable pin, etc to use H bridge driver or MOSFET circuit to drive things other than motors.
* IMPORTANT: Channels are referenced by the M and G code as a value from 1-10 but the indexes are from 0-9 in the arrays
* of channels, therefore, always subtract 1 when going from methods that take values from the processing loop, such as:
* commandMotorPower
* getDriverInfo

* Structure:
* 1) Top level, a master list of pins in either the PWM or digitals arrays. Slots to which physical pins are assigned.
* these are ppwms and pdigitals here, which hold pointers to these top level arrays.
* 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
* these are indexed by channel. PWMDrive here. All PWM has a pin, a prescale, and a resolution. We standardize the resolution to 8 bits typically.
* 3) A PWM level indexed by channel. The current power or level value of the PWM timer. Standardized to reflect the control applied.
* for instance, if the PS3 controller delivers -1000 to 1000 thats the range. If the controller is using the button that has -1 off and a max of 1000
* then the range is 0 to 2000 to reflect the fact that no reverse operation is going to be in effect for something like a LED or pump.
* 4) Optionally, a PWM duration and a minimum PWM level. The PWM duration represents a maximum time interval before the PWM timer
* is automatically shut down. The minimum PWM level is the bottom limit for the PWM value. It is indexed by channel and the value is
* the range that comes from the main controller, before any processing into a timer value.
* Created: 10/29/2020 9:54:23 AM
* Author: Groff
*/
#include "AbstractPWMControl.h"

// functions
void AbstractPWMControl::resetLevels(void) {
	for(int i = 0; i < 10; i++) pwmLevel[i] = 0; // all channels down
}

// virtual destructor
AbstractPWMControl::~AbstractPWMControl()
{
} //~AbstractPWMControl
