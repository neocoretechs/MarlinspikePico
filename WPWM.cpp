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
#include <intctrl.h>
	/*
	* Constructor 
	*/
	PWM::PWM(uint spin) {
		this->pin = spin;
	}

	void PWM::init(uint spin) {
	 this->pin = spin; 
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
    	uint slice = pwm_gpio_to_slice_num(pin);
   	 	// Clear any pending IRQ
    	pwm_clear_irq(slice);
    	// Enable IRQ for this slice
    	pwm_set_irq_enabled(slice, true);
		// Register the handler (only once globally)
		irq_set_exclusive_handler(IO_IRQ_BANK0, pwm_irq_handler);
			irq_set_enabled(IO_IRQ_BANK0, true);
	}
	/*
	* Static IRQ handler that dispatches to the correct instance
	* Because the RP2040 IRQ is per slice, not per pin, you need a static trampoline:
	*/
	void PWM::pwm_irq_handler() {
		for (int slice = 0; slice < 8; slice++) {
			if (pwm_hw->intr & (1 << slice)) {
				pwm_clear_irq(slice);
				// Dispatch to instance if registered
				if (PWM::instances[slice] && PWM::instances[slice]->interruptService) {
					PWM::instances[slice]->interruptService->service();
				}
			}
		}
	}
	void PWM::pwmWrite(bool enable, uint power) {
		uint32_t wrap = clock_get_hz(clk_sys) / 9999;
    	uint level = (power + 1000) * wrap / 2000;
    	uint slice = pwm_gpio_to_slice_num(pin);
    	uint chan  = pwm_gpio_to_channel(pin);
    	pwm_set_wrap(slice, wrap);
    	pwm_set_chan_level(slice, chan, level);
    	pwm_set_enabled(slice, enable);
	}		


