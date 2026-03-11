#ifndef __INTERRUPT_SERVICE_
#define __INTERRUPT_SERVICE_

#ifdef __cplusplus
class InterruptService
{
	public:
	virtual void service(void)=0;
};
#endif

#endif