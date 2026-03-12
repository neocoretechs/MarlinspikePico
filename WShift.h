/*
 * WShift.h
 *
 * Created: 4/7/2014 1:08:16 PM
 *  Author: jg
 */
#ifndef WSHIFT_H_
#define WSHIFT_H_

#include "pico/stdlib.h"
#include "stdlib.h"
#include "DigitalPin.h"
class Shift : public Digital {
	private:
	uint8_t clockPin, bitOrder;
	Digital* dclockPin;
	public:
	Shift(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) : Digital(dataPin) {
			this->clockPin = clockPin;
			this->bitOrder = bitOrder;
			dclockPin = new Digital(clockPin);
	}
	uint8_t shiftIn();
	void shiftOut(uint8_t val);
};



#endif /* WSHIFT_H_ */