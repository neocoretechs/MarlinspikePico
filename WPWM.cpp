/*
 * WPWM.cpp
 * Class to encapsulate Pulse width modulation of a pin via onboard timers.
 * Uses timer objects depending on desired pin. Timers are pre-instantiated at startup as Timer0, Timer1...Timer5
 * init() method must be called before resolution and prescalar as they act on the timer chosen.
 * an option to add an interrupt derived from CounterInterruptService, a simple counter of cycles, is provided.
 * Created: 4/7/2014 9:49:17 PM
 *  Author: jg
 */ 
#include "WPWM.h"
#include <hardware/clocks.h>
#include <hardware/pwm.h>
#include <hardware/irq.h>
// 8 total "slices", each slice has 2 "channels" one interrupt server a slice, so 8 total interrupts possible. We will use the same interrupt handler for all slices and dispatch based on which slice fired.

PWM* PWM::instances[8] = {nullptr};
	/*
	* Constructor 
	*/
	PWM::PWM(uint spin) {
		this->pin = spin;
	}
	/*
	* Init GPIO and set up global interrupt handler
	* We will handle safe shutdown and process any additional interrupt
	* handlers that get attached
	*/
	void PWM::init() {
		gpio_set_function(this->pin, GPIO_FUNC_PWM);
		this->slice = pwm_gpio_to_slice_num(pin);
		PWM::instances[this->slice] = this; // register instance for IRQ dispatch

		pwm_config config = pwm_get_default_config();
		pwm_config_set_clkdiv(&config, 10.0f); // div
		pwm_config_set_wrap(&config, 999); // top 0-1000
		pwm_init(this->slice, &config, false);

		// Clear any pending IRQ
    	pwm_clear_irq(this->slice);
    	// Enable IRQ for this slice
    	pwm_set_irq_enabled(this->slice, true);
		// Register the handler (only once globally)
		irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_irq_handler);
		irq_set_priority(PWM_IRQ_WRAP, 0); // highest prio
		irq_set_enabled(PWM_IRQ_WRAP, true);
	}

	/*
	* Sets up the PWM pin for interrupt service with a counter or its subclass.
	* Sets the counter to 0, attaches the interrupt to be serviced each cycle.
	* One IRQ per slice
	* register a handler for that slice
	* Inside the handler, check which channel fired
	* ASSUMES OVERLOADED INIT FOR PIN ASSIGNMENT HAS BEEN CALLED!
	*/
	void PWM::attachInterrupt(InterruptService* cins, bool overflow) {
    	this->interruptService = cins;
   	 
	}
	void PWM::detachInterrupt() {
   	 	// Clear any pending IRQ
    	//pwm_clear_irq(this->slice);
    	// Enable IRQ for this slice
    	//pwm_set_irq_enabled(this->channel, false);
		this->interruptService = 0;
	}
	uint16_t PWM::get_counter() {
		return pwm_get_counter(this->slice);
	}
	void PWM::pwmOff() {
		uint slice = this->slice;
		uint32_t mask = 1u << slice;
		pwm_clear_irq(mask);
		pwm_set_chan_level(slice, PWM_CHAN_A, 0);
		pwm_set_chan_level(slice, PWM_CHAN_B, 0);
		pwm_set_enabled(slice, false);
		//pwm_set_irq_enabled(slice, false);
		watchdog = 0;
		shutdownRequested = false;
		shutdownLogged = true;		
	}
	
	/*
	* Static IRQ handler that dispatches to the correct instance
	* Because the RP2040 IRQ is per slice, not per pin, you need a static trampoline:
	*/
	void PWM::pwm_irq_handler() {
    	// Read mask of slices that triggered (one call)
    	uint32_t mask = pwm_get_irq_status_mask();
    	if (mask == 0) return;
    	// Clear IRQs for those slices
    	pwm_clear_irq(mask);
		(void)pwm_hw->intr;
    	// Iterate slices that fired
    	for (int slice = 0; slice < 8; ++slice) {
        	if (!(mask & (1u << slice))) continue;
        	PWM* inst = PWM::instances[slice];
        	if (!inst) continue; // safe: skip unregistered slices
        	// Dispatch user ISR if present (must be IRQ-safe)
        	if (inst->interruptService) {
            	inst->interruptService->service(); // ensure service() is IRQ-safe
        	}
        	// Watchdog handling: only modify volatile counters/flags
        	if(inst->safeShutdown) {
            	if (inst->watchdog > 0) {
                	--inst->watchdog;
            	} else {
                	// mark for shutdown
					inst->shutdownRequested = true;
					pwm_set_irq_enabled(inst->slice, false);
					pwm_set_enabled(inst->slice, false);
            	}
        	}
    	}
	}
	void PWM::setSafeShutdown(bool enable, int max) {
		safeShutdown = enable;
		watchdogMax = max;
	}

	void PWM::pwmWrite(bool enable, uint power) {
		// clear any pending IRQ to avoid immediate timeout if we are enabling
		pwm_clear_irq(1u << this->slice);
		pwm_set_gpio_level(this->pin, power);
    	pwm_set_enabled(this->slice, enable);
		// possibly re-enable wrap IRQ for this slice to ensure we get the next cycle interrupt for watchdog handling
		pwm_set_irq_enabled(this->slice, enable);
		watchdog = enable ? watchdogMax : 0;
		shutdownRequested = false;
		shutdownLogged = false;
	}		


