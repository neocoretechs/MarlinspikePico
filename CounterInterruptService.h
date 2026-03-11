/*
 * CounterInterruptService.h
 * Interrupt service that increments a counter. It can be attached to any timer or pin change to provide a monotomically
 * increasing counter of the number of overflows/compares performed by the timer.
 * In PWM, this is used to determine the number of PWM 'cycles' performed to provide a dead man switch.
 * Created: 9/9/2016 3:03:02 PM
 *  Author: jg
 */ 


#ifndef COUNTERINTERRUPTSERVICE_H_
#define COUNTERINTERRUPTSERVICE_H_
#include "WPWM.h"
class CounterInterruptService: public InterruptService {
	private:
	volatile int counter;
	int maxcount;
	public:
	CounterInterruptService(int tmax) {
		this->maxcount = tmax;
		counter = 0;
	}
	//Interrupt Service Routine. RoboCore provides a virtual base defining the 'service' method for all unified interrupt requests
	void service(void)
	{	
		if( counter < maxcount ) {
			++counter;
		} 
			
	}
	
	int get_counter() {
		int cntx;
		uint8_t oldSREG = SREG;
		cli();
		cntx = counter;
		SREG = oldSREG;
		return cntx; 
	}
		
	void set_counter(int cntx) {
		uint8_t oldSREG = SREG;
		cli();
		counter = cntx;
		SREG = oldSREG;
	}

};

#endif /* COUNTERINTERRUPTSERVICE_H_ */