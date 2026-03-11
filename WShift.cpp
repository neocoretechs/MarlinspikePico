/*
  wiring_shift.c
  
*/
#include "WShift.h"
	
	uint8_t Shift::shiftIn() {
		uint8_t value = 0;
		uint8_t i;
		for (i = 0; i < 8; ++i) {
			dclockPin->digitalWrite(HIGH);
			if (bitOrder == LSBFIRST)
				value |= digitalRead() << i;
			else
				value |= digitalRead() << (7 - i);
			dclockPin->digitalWrite(LOW);
		}
		return value;
	}

	void Shift::shiftOut(uint8_t val)
	{
		uint8_t i;
		for (i = 0; i < 8; i++)  {
			if (bitOrder == LSBFIRST)
				digitalWrite(!!(val & (1 << i)));
			else	
				digitalWrite(!!(val & (1 << (7 - i))));
			dclockPin->digitalWrite(HIGH);
			dclockPin->digitalWrite(LOW);		
		}
	}
