/*
 * DuplexService.h
 *
 * Created: 3/26/2014 1:40:43 AM
 *  Author: jg
 */ 
#ifndef DUPLEXSERVICE_H_
#define DUPLEXSERVICE_H_
class DuplexService {
    virtual void onTransmit(void)=0;
    virtual void onReceive(void)=0;
};

#endif /* DUPLEXSERVICE_H_ */