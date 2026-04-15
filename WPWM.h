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
#include "hardware/dma.h"
#include "hardware/regs/pwm.h"

using namespace std;
class PWM {
	public:
	//const uint MAX_PWM_LEVEL = 65535;
	//const uint PWM_INCREMENT = 65; // 0 - 1000
	uint pin;
	uint slice;
	uint channel = 0;
	volatile int watchdog = 0;
	volatile int watchdogMax = 10000;
	volatile bool safeShutdown = false;
	volatile bool shutdownRequested = false;
	volatile bool shutdownLogged = false;
	// Constants for the hardware fuse
	const uint32_t PWM_OFF_VAL = 0x0; 
	uint32_t dummy_dest;
	const uint32_t dummy_src = 0xDEADBEEF;
	// Channel handles for monitoring in main loop
	int count_chan;
	int kill_chan;
	InterruptService* interruptService=NULL;
	static PWM* instances[8];
	PWM(uint spin);
	void init();
	void pwmWrite(bool enable, uint power);
	void pwmOff();
	static void pwm_irq_handler();
	void attachInterrupt(InterruptService* cins, bool overflow = false);
	void detachInterrupt();
	void setSafeShutdown(bool enable, int max);
	void setup_slice_dma(volatile uint8_t* active_mask_buffer,const uint8_t *slice_bits);
	uint16_t get_counter();
};


#endif /* WPWM_H_ */