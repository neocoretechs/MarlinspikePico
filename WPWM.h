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
#include <atomic>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/regs/pwm.h"

using namespace std;
extern int dma_chan_per_slice[8];

class PWM {
	public:
	//const uint MAX_PWM_LEVEL = 65535;
	//const uint PWM_INCREMENT = 65; // 0 - 1000
	uint pin;
	uint slice;
	uint channel = 0;
	// Constants for the hardware fuse
	const uint32_t PWM_OFF_VAL = 0x0; 
	InterruptService* interruptService=NULL;
	static PWM* instances[8];
	PWM(uint spin);
	void init();
	void pwmWrite(bool enable, uint power);
	void pwmOff();
	static void pwm_irq_handler();
	void attachInterrupt(InterruptService* cins, bool overflow = false);
	void detachInterrupt();
	int setup_slice_dma(void);
	uint16_t get_counter();
	uint get_slice() { return this->slice;}
	int get_dma_chan() { return dma_chan_per_slice[this->slice];}
	uint get_pwm_channel();
};

#endif /* WPWM_H_ */