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
/**
 * Talk to a 74HC595. ShiftOut shifts out a byte of data one bit at a time, starting with either the most or least significant bit, depending on the value of the bitOrder parameter.
 * The data is set on the dataPin, and pulsed on the clockPin. Once all the bits have been shifted in, the latchPin is pulsed, which tells the 74HC595 to output the data on its output pins.
 * The 74HC165 is the opposite, it shifts in data on the dataPin, and the clockPin is pulsed to shift the data in. Once all the bits have been shifted in, the latchPin is pulsed, 
 * which tells the 74HC165 to output the data on its output pins.	
 */
class Shift : public Digital {
	private:
	uint8_t clockPin;
	bool bitOrder;
	Digital* dclockPin;
	public:
	bool LSBFIRST = true;
	bool MSBFIRST = false;
	Shift(uint8_t dataPin, uint8_t clockPin, bool bitOrder) : Digital(dataPin) {
			this->clockPin = clockPin;
			this->bitOrder = bitOrder;
			dclockPin = new Digital(clockPin);
	}
	uint8_t shiftIn();
	void shiftOut(uint8_t val);
};



#endif /* WSHIFT_H_ */