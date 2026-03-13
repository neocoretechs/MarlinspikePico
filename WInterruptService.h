#ifndef __INTERRUPT_SERVICE_
#define __INTERRUPT_SERVICE_
#include "WInterruptsBase.h"
#ifdef __cplusplus
class InterruptService : public InterruptsBase
{
	public:
	virtual void service(void)=0;
};
#endif

#endif