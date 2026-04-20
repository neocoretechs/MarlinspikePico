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
static const uint8_t slice_bits[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}; // bit mask for each slice for DMA source	
static uint32_t now_us;
static volatile absolute_time_t last_command_time[8] = {0,0,0,0,0,0,0,0};
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
	int setup_slice_dma(volatile uint8_t* active_mask_buffer);
	uint16_t get_counter();
	int64_t get_on_time_us();
	uint get_slice() { return this->slice;}
	int get_dma_chan() { return dma_chan_per_slice[this->slice]; }
};


#endif /* WPWM_H_ */