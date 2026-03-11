/*
 Servo.cpp - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
 */

/*

 A servo is activated by creating an instance of the Servo class passing the desired pin to the set() method.
 The servos are pulsed in the background using the value most recently written using the write() method

 Timers are seized as needed in groups of 12 servos - 24 servos use two timers, 48 servos will use four.

 The methods are:

 Servo - Class for manipulating servo motors connected to Arduino pins.

 set(pin, min, max  ) -  set min and max values in microseconds
 default min is 544, max is 2400

 write()     - Sets the servo angle in degrees.  (invalid angle that is valid as pulse in microseconds is treated as microseconds)
 writeMicroseconds() - Sets the servo pulse width in microseconds
 read()      - Gets the last written servo pulse width as an angle between 0 and 180.
 readMicroseconds()   - Gets the last written servo pulse width in microseconds. (was read_us() in first release)
 attached()  - Returns true if there is a servo attached.
 detach()    - Stops an attached servos from pulsing its i/o pin.

*/
#include "Servo.h"
#include "ServoInterruptService.h"
#include "WMath.h"

static void initISR(timer16_Sequence_t timer)
{
	uint8_t sIndex = SERVO_INDEX(timer,Channel[timer]);
	InterruptService* service = new ServoInterruptService(&Channel[timer], timer);
	servos[sIndex].Pin.timer->clearInterrupt(CHANNEL_A); // TIFR3 = _BV(OCF3A) clear any pending interrupts;
	servos[sIndex].Pin.timer->attachInterrupt(INTERRUPT_COMPARE_MATCH_A, service, true); // also enables output compare interrupt TIMSK3 = _BV(OCIE3A)
	servos[sIndex].Pin.timer->setMode(0); //TCCR3A = 0 normal counting mode
	servos[sIndex].Pin.timer->setClockSource(CLOCK_PRESCALE_8); //TCCR3B = _BV(CS31) set prescaler of 8
	servos[sIndex].Pin.timer->setCounter(0); //TCNT3 = 0 clear the timer count
}

static boolean isTimerActive(timer16_Sequence_t timer)
{
  // returns true if any servo is active on this timer
  for(uint8_t channel=0; channel < SERVOS_PER_TIMER; channel++) {
    if(SERVO(timer,channel).Pin.isActive == true)
      return true;
  }
  return false;
}


Servo::Servo()
{
  if( ServoCount < MAX_SERVOS) {
    this->servoIndex = ServoCount++;                    // assign a servo index to this instance
	servos[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);   // store default values  - 12 Aug 2009
  }
  else
    this->servoIndex = INVALID_SERVO ;  // too many servos
}


uint8_t Servo::set(int pin, HardwareTimer* hTimer, int min, int max)
{
  if(this->servoIndex < MAX_SERVOS ) {
    servos[this->servoIndex].Pin.nbr = new Digital(pin);
	servos[this->servoIndex].Pin.nbr->pinMode(OUTPUT);
	servos[this->servoIndex].Pin.timer = hTimer;
    // todo min/max check: abs(min - MIN_PULSE_WIDTH) /4 < 128
    this->min  = (MIN_PULSE_WIDTH - min)/4; //resolution of min/max is 4 uS
    this->max  = (MAX_PULSE_WIDTH - max)/4;
    // initialize the timer if it has not already been initialized
    timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
    if(isTimerActive(timer) == false) {
      initISR(timer);
	}
    servos[this->servoIndex].Pin.isActive = true;  // this must be set after the check for isTimerActive
  }
  return this->servoIndex ;
}

void Servo::detach()
{
  servos[this->servoIndex].Pin.isActive = false;
  timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
  if(isTimerActive(timer) == false) {
	  servos[this->servoIndex].Pin.timer->detachInterrupt(0);
  }
}

void Servo::write(int value)
{
  if(value < MIN_PULSE_WIDTH)
  {  // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
    if(value < 0) value = 0;
    if(value > 180) value = 180;
    value = map(value, 0, 180, SERVO_MIN(),  SERVO_MAX());
  }
  this->writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value)
{
  // calculate and store the values for the given channel
  byte channel = this->servoIndex;
  if( (channel < MAX_SERVOS) )   // ensure channel is valid
  {
    if( value < SERVO_MIN() )          // ensure pulse width is valid
      value = SERVO_MIN();
    else if( value > SERVO_MAX() )
      value = SERVO_MAX();

  	value = value - TRIM_DURATION;
    value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead - 12 Aug 2009

    uint8_t oldSREG = SREG;
    cli();
    servos[channel].ticks = value;
    SREG = oldSREG;
  }
}

int Servo::read() // return the value as degrees
{
  return  map( this->readMicroseconds()+1, SERVO_MIN(), SERVO_MAX(), 0, 180);
}

int Servo::readMicroseconds()
{
  unsigned int pulsewidth;
  if( this->servoIndex != INVALID_SERVO )
    pulsewidth = ticksToUs(servos[this->servoIndex].ticks)  + TRIM_DURATION ;   // 12 aug 2009
  else
    pulsewidth  = 0;

  return pulsewidth;
}

bool Servo::attached()
{
  return servos[this->servoIndex].Pin.isActive ;
}

