
/*
 * TWIService.h
 *
 * Created: 3/26/2014 1:45:08 AM
 *  Author: jg
 */ 
#ifndef TWISERVICE_H_
#define TWISERVICE_H_
#include "DuplexService.h"
#include "pico/stdlib.h"

class TWIService: public DuplexService {
	public:
	uint8_t device;
	uint8_t rxBuffer[32];
	uint8_t rxBufferIndex;
	uint8_t rxBufferLength;
	uint8_t rxBufferPointer;
	uint8_t txBuffer[32];
	uint8_t txBufferIndex;
	uint8_t txBufferLength;
	TWIService(uint8_t device);
	void clearBuffers();
	void clearTxBuffer();
	void clearRxBuffer();
	boolean setRxBufferLength(uint8_t len);
	boolean setTxBufferLength(uint8_t len);
	uint8_t remainingXmit();
	uint8_t remainingRcve();
	boolean addRx(uint8_t trx);
	boolean addTx(uint8_t trx);
	uint8_t getRx();
	uint8_t getTx();	
	virtual void onTransmit( void );// called at TW_ST_SLA_ACK, enter slave transmitter mode
	virtual void onReceive( void ); // called at TW_SR_STOP
};

#endif /* TWISERVICE_H_ */