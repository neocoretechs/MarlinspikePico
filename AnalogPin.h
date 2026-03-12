#ifndef ANALOG_H
#define ANALOG_H

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "DigitalPin.h"

class Analog : public virtual Digital {
private:
    uint8_t analog_channel = 0;

public:
    Analog(uint8_t spin) : Digital(spin) {
        adc_init();
    }

    void analogReference(uint8_t mode) {
        // RP2040 does not support selectable analog references.
        // The reference is fixed at 3.3V.
    }

    int analogRead() {
        // RP2040 ADC pins are GPIO 26–29 → ADC channels 0–3
        if (pin >= 26 && pin <= 29) {
            analog_channel = pin - 26;
        } else {
            return -1; // not an ADC-capable pin
        }

        adc_select_input(analog_channel);
        uint16_t result = adc_read();  // 12-bit result (0–4095)

        return result;
    }
};

#endif