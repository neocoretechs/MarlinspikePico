#ifndef __INTERRUPT_SERVICE_
#define __INTERRUPT_SERVICE_
#include "WInterruptsBase.h"
class InterruptService : public InterruptsBase
{
	public:
	InterruptService() : InterruptsBase(){}
	virtual void service(void)=0;
};

#endif