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
#include "pico/stdlib.h"
using namespace std;
class PWM {
	public:
	const uint MAX_PWM_LEVEL = 65535;
	const uint PWM_INCREMENT = 65; // 0 - 1000
	uint pin;
	uint slice;
	uint channel = 0;
	volatile int watchdog = 0;
	bool safeShutdown = false;
	InterruptService* interruptService=NULL;
	static PWM* instances[8];
	PWM(uint spin);
	void init();
	void pwmWrite(bool enable, uint power);
	inline void pwmOff() { pwmWrite(0, 0); };
	static void pwm_irq_handler();
	void attachInterrupt(InterruptService* cins, bool overflow = false);
	void detachInterrupt();
};


#endif /* WPWM_H_ */