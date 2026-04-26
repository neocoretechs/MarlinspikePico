/* 
* DelayHBridgeDriver.cpp
* Driver for brushed DC motors using an H-bridge driver with direction set via control pin. Each channel is an axle with a motor. 
* Can support 10 motors.
* The DelayHBridgeDriver creates a delay between direction change to allow the 
* motor to stop before reversing direction. This is to protect the motor and the driver from damage due to rapid direction changes.
* Structure:
* 1) Top level, a master list of pins in either the PWM or digitals arrays. Slots to which physical pins are assigned.
* these are ppwms and pdigitals here, which hold pointers to these top level arrays.
* 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
* these are indexed by channel. motorDrive here. All PWM has a pin, a prescale, and a resolution. We standardize the resolution to 8 bits typically.
* 3) A PWM level indexed by channel. The current power or level value of the PWM timer. Standardized to reflect the control applied.
* for instance, if the PS3 controller delivers -1000 to 1000 thats the range. If the controller is using the button that has -1 off and a max of 1000
* then the range is 0 to 2000 to reflect the fact that no reverse operation is going to be in effect for something like a LED or pump.
* 4) Optionally, a duration and a minimum PWM level, or here a minimum motor power. The duration represents a maximum interval before the PWM timer
* is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
* The minimum PWM level is the bottom limit for the PWM value, or in this case the minimum motor power. It is indexed by channel and the value is
* the range that comes from the main controller, before any processing into a timer value.
* the top level abstract class AbstractMotorControl contains these values.
* Created: 10/2/2016 1:42:24 PM
* Author: jg
*/

#include "DelayHBridgeDriver.h"
#include "../Configuration_adv.h"


void DelayHBridgeDriver::getDriverInfo(uint8_t ch, char* outStr) {
	if(ch <= 0 || ch >=11) return;
	char cout[OUT_BUFFER_SIZE];
	char dout1[10];
	char dout3[10];
	char dout5[10];
	char dout7[10];
	char dout9[10];
	char dout10[21];
	
	if( motorDrive[ch-1][0] == 255 ) {
		itoa(-1, dout1, 10);
	} else {
		itoa(ppwms[motorDrive[ch-1][0]]->pin, dout1, 10);
		itoa(get_slice(ch), dout5, 10);
		itoa(ppwms[motorDrive[ch-1][0]]->get_pwm_channel(), dout7, 10);
		itoa(reverse_delay, dout9, 10);
		snprintf(dout10, 21, "%lu", (unsigned long)get_on_time_us(ch));
	}
	itoa(motorDrive[ch-1][1], dout3, 10);

	if( motorDrive[ch-1][0] == 255 ) {
		sprintf(cout,"DelayHB-PWM UNINITIALIZED Pin:%s, Dir Pin:%s\r\n\0", dout1, dout3);
	} else {
		sprintf(cout,"DelayHB-PWM Pin:%s, Dir Pin:%s, Slice:%s, PWM Channel:%s, Reverse Delay:%d, On Time:%s\r\n\0", dout1, dout3, dout5, dout7, dout9, dout10);
	}
	
	for(int i=0; i < OUT_BUFFER_SIZE; ++i){
		 outStr[i] = cout[i];
		 if(!outStr[i])
		 break;
	}

}

//DelayHBridgeDriver delayhBridgeDriver;