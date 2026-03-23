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
#include "pico/stdlib.h"
#include "pins_pico/stdlib.h"
#include "AnalogPin.h"
#include "WPWM.h"

volatile union _fastTemp{
	unsigned int word;
	struct{
		unsigned char LByte;
		unsigned char HByte;  //Hbyte = Zero
	};
} fastTemp;

class HallInterruptService: public InterruptService {
	private:
	int validHallLo = 25;
	int validHallHi = 4096;
	unsigned int lastHall = 0; // last hall reading, multiple pins fire at once so we need to screen the redundant interrupt
	uint8_t commPattern; // the 6 bit value from our commutation tables which are indexed by hall sensor aggregate value
	public:
	Analog* apin;
	PWM* ppin;
	unsigned int count = 0; // incremented every valid hall change int. to measure relative speed
	HallInterruptService(Analog* _apin, PWM* _pwm) : apin(_apin), ppin(_ppin), public Analog(_apin) { }
	//Pin Change Interrupt Service Routine. RoboCore provides a virtual base defining the 'service' method for all unified interrupt requests
	void service(void)
	{
		fastTemp.word = apin->analogRead();
		// check for potential garbage
		if( fastTemp.word >= validHallLo && fastTemp.word <= validHallHi  && fastTemp.word != lastHall) {
			lastHall = fastTemp.word;
			++count;    // variable used to measure speed
			// shut down all inputs to prevent possible shoot-through
			if(fastTemp.word == validHallHi)
				ppin->pwmOff();
		}
	}
};


#endif /* HALLINTERRUPTSERVICE_H_ */