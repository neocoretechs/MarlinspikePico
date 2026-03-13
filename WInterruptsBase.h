/*
 * WInterruptsBase.h
 *
 * Created: 3/12/2014 11:56:39 PM
 *  Author: jg
 */ 

#ifndef WINTERRUPTSBASE_H_
#define WINTERRUPTSBASE_H_
#include "stdlib.h"
#include "WInterruptService.h"
#include <inttypes.h>

class InterruptsBase {
	private:
	public:
	virtual uint8_t attachInterrupt(int mode)=0;
	virtual void attachInterrupt(uint8_t interruptNum, int mode);
	virtual void detachInterrupt(uint8_t interruptNum);
};

#endif /* WINTERRUPTSBASE_H_ */