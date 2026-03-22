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
	/*
	* Constructor 
	*/
	PWM::PWM(uint spin) {
		this->pin = spin;
	}

	void PWM::init() {
	 gpio_set_function(this->pin, GPIO_FUNC_PWM);
	 this->slice = pwm_gpio_to_slice_num(pin);
	 pwm_config config = pwm_get_default_config();
	 pwm_init(this->slice, &config, false);
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
   	 	// Clear any pending IRQ
    	pwm_clear_irq(this->slice);
    	// Enable IRQ for this slice
    	pwm_set_irq_enabled(this->slice, true);
		// Register the handler (only once globally)
		irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_irq_handler);
		irq_set_enabled(PWM_IRQ_WRAP, true);
	}
	void PWM::detachInterrupt() {
   	 	// Clear any pending IRQ
    	pwm_clear_irq(this->slice);
    	// Enable IRQ for this slice
    	pwm_set_irq_enabled(this->channel, false);
	}
	/*
	* Static IRQ handler that dispatches to the correct instance
	* Because the RP2040 IRQ is per slice, not per pin, you need a static trampoline:
	*/
	void PWM::pwm_irq_handler() {
		for (int xslice = 0; xslice < 8; xslice++) {
			if (pwm_hw->intr & (1 << xslice)) {
				pwm_clear_irq(xslice);
				// Dispatch to instance if registered
				if (PWM::instances[xslice] && PWM::instances[xslice]->interruptService) {
					PWM::instances[xslice]->interruptService->service();
				}
			}
			if(PWM::instances[xslice]->safeShutdown) {
				uint xslice = pwm_get_irq_status_mask();
				pwm_clear_irq(xslice);
				if(PWM::instances[xslice]->watchdog > 0) {
					PWM::instances[xslice]->watchdog--;
				} else {
					pwm_set_enabled(xslice,false);
				}
			}
		}
	}
	void PWM::pwmWrite(bool enable, uint power) {
		pwm_set_gpio_level(this->pin, (power*PWM_INCREMENT));
    	pwm_set_enabled(this->slice, enable);
		watchdog = 1000;
	}		


