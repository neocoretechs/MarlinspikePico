
#ifndef DIGITALPIN_H_
#define DIGITALPIN_H_
#include "pico/stdlib.h"
#include <hardware/pwm.h>
#include "Configuration_adv.h"
enum PinMode {
    INPUT ,
    OUTPUT, 
    INPUT_PULLUP ,
    INPUT_PULLDOWN 
};
class Digital {
public:
    uint pin;
    uint mode;
    bool is_output = false;

    Digital(uint pin) : pin(pin) {
        gpio_init(pin);
    }

    void setPin(uint spin) {
        pin = spin;
        gpio_init(pin);
        gpio_set_dir(pin, is_output ? GPIO_OUT : GPIO_IN);
    }

    void pinMode(uint mode) {
        this->mode = mode;
        is_output = (mode == GPIO_OUT);
        gpio_set_dir(pin, is_output ? GPIO_OUT : GPIO_IN);

        if (mode == GPIO_IN) {
            gpio_disable_pulls(pin);
        } else if (mode == INPUT_PULLUP) {
            gpio_pull_up(pin);
        } else if (mode == INPUT_PULLDOWN) {
            gpio_pull_down(pin);
        }
    }

    void digitalWrite(bool value) {
        // If PWM is active on this pin, disable it
        turnOffPWMIfNeeded();
        gpio_put(pin, value);
    }

    bool digitalRead() {
        turnOffPWMIfNeeded();
        return gpio_get(pin);
    }

    // Pico version of pulseIn()
    uint32_t pulseIn(bool state, uint32_t timeout_us = 1000000) {
        uint64_t start = time_us_64();

        // Wait for previous pulse to end
        while (gpio_get(pin) == state) {
            if (time_us_64() - start > timeout_us) return 0;
        }

        // Wait for pulse to start
        while (gpio_get(pin) != state) {
            if (time_us_64() - start > timeout_us) return 0;
        }

        uint64_t pulse_start = time_us_64();

        // Measure pulse
        while (gpio_get(pin) == state) {
            if (time_us_64() - start > timeout_us) return 0;
        }

        return (uint32_t)(time_us_64() - pulse_start);
    }

private:
    void turnOffPWMIfNeeded() {
        uint slice = pwm_gpio_to_slice_num(pin);
        // If pin is not PWM-capable, slice will be valid but channel may not be mapped.
        // Safe to disable anyway.
        pwm_set_enabled(slice, false);
    }
};
#endif