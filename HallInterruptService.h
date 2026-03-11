/*
 * HallInterruptService.h
 * Service called on each change of hall pins. The combination of the 3 hall sensors determines commutation
 * at most 2 sensors are active and so potentially 2 interrupts per commutation event. We screen out the second by comparing
 * the last hall reading and ejecting if they are the same. We read the 3 bits and check range 1-6, ejecting if range is outside that.
 * As a safety feature we shut down input upon leaving this service to avoid a possible shoot-through condition in the event of
 * various failure modes.
 * Created: 3/12/2014 3:38:25 AM
 * Author: jg
 */ 

#ifndef HALLINTERRUPTSERVICE_H_
#define HALLINTERRUPTSERVICE_H_

#include "WInterruptService.h"
#include "Propulsion/sensor_three_phase_BLDC.h"
#include "pico/stdlib.h"
#include "pins_pico/stdlib.h"

volatile union _fastTemp{
	unsigned int word;
	struct{
		unsigned char LByte;
		unsigned char HByte;  //Hbyte = Zero
	};
} fastTemp;

class HallInterruptService: public InterruptService {
	private:
	const uint8_t wlmask = 0b100000;
	const uint8_t vlmask = 0b010000;
	const uint8_t ulmask = 0b001000;
	const uint8_t whmask = 0b000100;
	const uint8_t vhmask = 0b000010;
	const uint8_t uhmask = 0b000001;
	const uint8_t validHallLo = 0b001;
	const uint8_t validHallHi = 0b110;
	unsigned int lastHall = 0; // last hall reading, multiple pins fire at once so we need to screen the redundant interrupt
	uint8_t commPattern; // the 6 bit value from our commutation tables which are indexed by hall sensor aggregate value
	public:
	unsigned int count = 0; // incremented every valid hall change int. to measure relative speed
	BLDC3PhaseSensor* motor; // main motor object ref
	HallInterruptService(BLDC3PhaseSensor* motor) { this->motor = motor; }
	//Pin Change Interrupt Service Routine. RoboCore provides a virtual base defining the 'service' method for all unified interrupt requests
	void service(void)
	{
		cli();
		fastTemp.word = motor->Read_hall_pins();
		// check for potential garbage
		if( fastTemp.word >= validHallLo && fastTemp.word <= validHallHi  && fastTemp.word != lastHall) {
			lastHall = fastTemp.word;
			++count;    // variable used to measure speed
			// shut down all inputs to prevent possible shoot-through
			motor->Stop_motor();
			motor->Enable_motor();
			if( motor->direction == CLOCKWISE )
				commPattern = motor->drvPatternsCW[lastHall];
			else
				commPattern = motor->drvPatternsCCW[lastHall];
			_delay_ms(1); // dead time for gate shutdown
			if( commPattern & wlmask )
				motor->wl->digitalWrite(HIGH);
			if( commPattern & vlmask )
				motor->vl->digitalWrite(HIGH);
			if( commPattern & ulmask )
				motor->ul->digitalWrite(HIGH);
			_delay_ms(1); // gate ramp up, bootstrap charge
			if( commPattern & whmask )
				motor->wh->pwmWrite(motor->speed,0b10); // Clear compare output mode for fast pwm: OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (inverting mode).
			if( commPattern & vhmask )
				motor->vh->pwmWrite(motor->speed,0b10); // Clear compare output mode for fast pwm: OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (inverting mode).
			if( commPattern & uhmask )
				motor->uh->pwmWrite(motor->speed,0b10); // Clear compare output mode for fast pwm: OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (inverting mode).		
			_delay_ms(20); // discharge
			motor->Stop_motor();
		}
		sei();	
	}

};


#endif /* HALLINTERRUPTSERVICE_H_ */