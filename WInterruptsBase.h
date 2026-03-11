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

#ifdef __cplusplus
extern "C" {
#endif

class InterruptsBase {
	private:
	public:
	virtual uint attachInterrupt(InterruptService* userFunc, int mode)=0;
	virtual void attachInterrupt(uint interruptNum, InterruptService* userFunc, int mode);
	virtual void detachInterrupt(uint interruptNum);
};

#ifdef __cplusplus
}
#endif

#endif /* WINTERRUPTSBASE_H_ */