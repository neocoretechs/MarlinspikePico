/*
 * WPWM.h
 *
 * Created: 4/7/2014 8:47:26 PM
 *  Author: jg
 */ 
#ifndef WPWM_H_
#define WPWM_H_

#include "WInterruptService.h"
#include <cstdint>
#include <cstddef>
#include <intctrl.h>
#include "pico/stdlib.h"
using namespace std;
class PWM {
	public:
	uint pin;
	uint channel = 0;
	InterruptService* interruptService=NULL;
	static PWM* instances[8];
	PWM(uint spin);
	void init(uint spin);
	void pwmWrite(bool enable, uint power);
	inline void pwmOff() { pwmWrite(0, 0); };
	static void pwm_irq_handler();
	void attachInterrupt(InterruptService* cins, bool overflow = false);
	void detachInterrupt(bool overflow = false);
};


#endif /* WPWM_H_ */