/*
 * ServoInterruptService.h
 *
 * Created: 4/8/2014 7:07:15 PM
 *  Author: jg
 */ 
#ifndef SERVOINTERRUPTSERVICE_H_
#define SERVOINTERRUPTSERVICE_H_
#include "Servo.h"
#include "WHardwareTimer.h"
#include "WInterruptService.h"

class ServoInterruptService: public InterruptService {
	private:
	volatile int8_t* ServoChannel;
	timer16_Sequence_t Timer;
	public:
	//new ServoInterruptService(&Channel[timer], timer);
	ServoInterruptService(volatile int8_t* servoChan, timer16_Sequence_t timer)	{
		ServoChannel = servoChan;
		Timer = timer;
	}

	virtual void service( void ) {
	//static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn, volatile uint16_t* OCRnA)

		if( *ServoChannel < 0 ) {
			SERVO(Timer,*ServoChannel).Pin.timer->setCounter(0);
			//*TCNTn = 0; 
			// channel set to -1 indicated that refresh interval completed so reset the timer
		} else {
			if( SERVO_INDEX(Timer,*ServoChannel) < ServoCount && SERVO(Timer,*ServoChannel).Pin.isActive == true )
			//if( ServoIndex < *ServoCount && Servot->Pin.isActive)
				//Servot->Pin.nbr->digitalWrite(LOW);
				SERVO(Timer,*ServoChannel).Pin.nbr->digitalWrite(LOW); // pulse this channel low if activated
		}

		//Channel[timer]++;    // increment to the next channel
		*ServoChannel++;
		if( SERVO_INDEX(Timer,*ServoChannel) < ServoCount && *ServoChannel < SERVOS_PER_TIMER) {
		//if( ServoIndex < *ServoCount && *ServoChannel < SERVOS_PER_TIMER) {
			//*OCRnA = *TCNTn + SERVO(timer,Channel[timer]).ticks;
			SERVO(Timer,*ServoChannel).Pin.timer->setOCR(CHANNEL_A, SERVO(Timer,*ServoChannel).Pin.timer->getCounter() + SERVO(Timer,*ServoChannel).ticks);
			if( SERVO(Timer,*ServoChannel).Pin.isActive) {
				SERVO(Timer,*ServoChannel).Pin.nbr->digitalWrite(HIGH);
			}
		} else {
			// finished all channels so wait for the refresh period to expire before starting over
			//if( ((unsigned)*TCNTn) + 4 < usToTicks(REFRESH_INTERVAL) )  // allow a few ticks to ensure the next OCR1A not missed
			if( ((unsigned)SERVO(Timer,*ServoChannel).Pin.timer->getCounter()) + 4 < usToTicks(REFRESH_INTERVAL) )
			//*OCRnA = (unsigned int)usToTicks(REFRESH_INTERVAL);
				SERVO(Timer,*ServoChannel).Pin.timer->setOCR(CHANNEL_A, (unsigned int)usToTicks(REFRESH_INTERVAL));
			else
			//*OCRnA = *TCNTn + 4;  // at least REFRESH_INTERVAL has elapsed
				SERVO(Timer,*ServoChannel).Pin.timer->setOCR(CHANNEL_A,SERVO(Timer,*ServoChannel).Pin.timer->getCounter()+4);
			*ServoChannel = -1; // this will get incremented at the end of the refresh period to start again at the first channel
		}
	//throw std::exception("The method or operation is not implemented.");
	}

};


#endif /* SERVOINTERRUPTSERVICE_H_ */