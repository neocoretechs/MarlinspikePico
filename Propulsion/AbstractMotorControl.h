/* 
* AbstractMotorControl.h
* Class to maintain the abstract collection of propulsion channels that comprise the traction power.
* Be they brushed motors driven by H-bridge, brushless DC, or a smart controller that uses a high level AT command protocol
* Controllers are channel-oriented. They manage collection of objects representing channels, or conceptually, wheels.
* IMPORTANT: Channels are referenced by the M and G code as a value from 1-10 but the indexes are from 0-9 in the arrays
* of channels, therefore, always subtract 1 when going from methods that take values from the processing loop, such as:
* commandMotorPower
* getDriverInfo
*
* Structure:
* 1) Since we can drive motors via PWM or switched on/off GPIO pins, with either split inputs with separate enable pins or
* one enable pin on a forward/reverse controller, we delegate those functions to the subclasses.
* 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
* these are indexed by channel. ultrasonicIndex, minMotorDist, etc. here. All PWM has a pin, a prescale, and a resolution. 
* We standardize the resolution to 8 bits typically.
* 3) Optionally, a duration and a minimum motor power. The duration represents a an abstraction of the maximum interval before the device
* is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
* The minimum motor power is an unsigned value that is added to the base power level typically to compensate for differences in
* motor integrity affecting values that represent the same speed on different channels.
* 4) The motorSpeed is indexed by channel and the value is the range that comes from the main controller, before any processing into a timer value.
* 5) the current direction and default direction have different meanings depending on subclass.
*
* Types of low level DC drivers supported:
* HBridge - A low level motor PWM driver that uses 1 enable pin with 2 states (logic high/low), to drive a mortor in the forward or backward direction.
* This is the most common type of low level PWM DC motor driver.
*
* SplitBridge - A low level motor PWM driver that uses 2 separate enable pins, each with one state (logic high),
* such that the driver can be configured to drive 2 motors in a forward direction,
* one motor in a forward or backward direction, or
* the two channels can be ganged together to deliver twice the current to one motor in a forward direction,
* all with a variable speed control via PWM.
*
* SwitchBridge - A low level motor driver that does not use PWM and that uses 2 separate enable pins, each with one state (logic high).
* Instead of PWM, full drive current is delivered via a switched mechanical or electronic device such as
* a mechanical or solid state relay using MOSFET transistor, BJT transistor, or IGBT transistor. By necessity it
* typically uses 2 separate enable pins that enable the operation of a DC motor in the same fashion as a
* SplitBridge but without variable speed control.
*
* Variable PWM Driver - A low level PWM driver that uses a single enable pin with one state(logic high) to switch the driver on or off.
* This type of driver would typically be used for LEDs with variable brightness, a single motor in the forward direction, etc.

* Created: 10/2/2016 12:53:49 PM
* Author: jg
*/

#ifndef __ABSTRACTMOTORCONTROL_H__
#define __ABSTRACTMOTORCONTROL_H__

#include <atomic>
#include "../Ultrasonic.h"
#include "../CounterInterruptService.h"
#include "../Configuration_adv.h"
#define MAXPOWER 1000
class AbstractMotorControl
{
private:
	uint channels = 0;
	// Ultrasonic arrays by channel:
	Ultrasonic* usensor[10]; // pointer to ultrasonic sensor array, set by linkDistanceSensor
	uint32_t minMotorDist[10] = {0,0,0,0,0,0,0,0,0,0}; // Ranging device motor shutdown range, by channel
	// Ultrasonic array by channel:
	// 0-index in 'usensor' to ultrasonic distance sensor for minimum distance safety shutdown.
	// 1-forward or reverse facing ultrasonic (1 forward)
	uint ultrasonicIndex[10][2] = {{255,1},{255,1},{255,1},{255,1},{255,1},{255,1},{255,1},{255,1},{255,1},{255,1}};
	uint32_t maxMotorDuration[10] = {4,4,4,4,4,4,4,4,4,4}; // number of pin change interrupts from wheel encoder before safety interlock
protected:
	// 10 channels of last motor speed
	int motorSpeed[10] = {0,0,0,0,0,0,0,0,0,0};
	uint currentDirection[10] = {0,0,0,0,0,0,0,0,0,0};
	uint defaultDirection[10] = {0,0,0,0,0,0,0,0,0,0};
	uint minMotorPower[10] = {0,0,0,0,0,0,0,0,0,0}; // Offset to add to G5, use with care, meant to compensate for mechanical differences
	CounterInterruptService* wheelEncoderService[10] = {0,0,0,0,0,0,0,0,0,0}; // encoder service
	int MOTORPOWERSCALE = 0; // Motor scale, divisor for motor power to reduce 0-1000 scale if non zero
	uint MOTORSHUTDOWN = 0; // Override of motor controls, puts it up on blocks
	int MAXMOTORPOWER = 1000; // Max motor power in PWM final timer units
	int fault_flag = 0;
	volatile int watchdog = 0;
	volatile int watchdogMax = 500000;
	volatile bool shutdownRequested = false;
	volatile bool shutdownLogged = false;
	volatile uint64_t last_command_time[10] = {0,0,0,0,0,0,0,0,0,0};
public:
  	AbstractMotorControl(int maxPower) : MAXMOTORPOWER(maxPower) {}
	virtual int commandMotorPower(int16_t p[10])=0;//make AbstractMotorControl not instantiable
	virtual int commandEmergencyStop(int status)=0;
	virtual int isConnected(void)=0;
	virtual void getDriverInfo(uint8_t ch, char* outStr) = 0;
	virtual int queryFaultFlag(void)=0;
    virtual int queryStatusFlag(void)=0;
	virtual int queryBrushlessCounter(uint8_t ch);
	virtual bool usesPWM(uint8_t pin) { return false; }
	virtual bool usesDigital(uint8_t pin) { return false; }
	void linkDistanceSensor(Ultrasonic** us, uint8_t upin, uint32_t distance, uint8_t facing=1);
	bool checkUltrasonicShutdown(void);
	bool checkEncoderShutdown(void);
	void createEncoder(uint8_t channel, uint8_t encode_pin);
	void setCurrentDirection(uint8_t ch, uint8_t val) { if(ch <= 0 || ch >=11) return; currentDirection[ch-1] = val; }
	// If the wheel is mirrored to speed commands or commutation, 0 - normal, 1 - mirror
	void setDefaultDirection(uint8_t ch, uint8_t val) { if(ch <= 0 || ch >=11) return;defaultDirection[ch-1] = val; }
	void setDuration(uint8_t ch, uint32_t durx) { if(ch <= 0 || ch >=11) return;maxMotorDuration[ch-1] = durx; }
	virtual void setMinMotorPower(uint8_t ch, int mpow) { if(ch <= 0 || ch >=11) return;minMotorPower[ch-1] = mpow; }
	int  getEncoderCount(uint8_t ch);
	int totalUltrasonics(void) {  int j = 0; for(int i = 0; i < 10; i++) if(ultrasonicIndex[i][0] != 255)++j; return j; }
	uint8_t getUltrasonicFacing(uint8_t ch) { if(ch <= 0 || ch >=11) return 0;return ultrasonicIndex[ch-1][1]; }
	uint32_t getMinMotorDist(uint8_t ch) { if(ch <= 0 || ch >=11) return 0;return minMotorDist[ch-1]; }
	uint8_t getUltrasonicIndex(uint8_t ch) { if(ch <= 0 || ch >=11) return 255;return ultrasonicIndex[ch-1][0]; }
	int getMaxMotorDuration(uint8_t ch) { if(ch <= 0 || ch >=11) return 0;return maxMotorDuration[ch-1]; }
	int getMinMotorPower(uint8_t ch) { if(ch <= 0 || ch >=11) return 0;return minMotorPower[ch-1] ; }
	virtual void setMaxMotorPower(int p) { MAXMOTORPOWER = p; }
	int getMaxMotorPower() { return MAXMOTORPOWER; }
	int getMotorSpeed(uint8_t ch) { if(ch <= 0 || ch >=11) return 0;return motorSpeed[ch-1]; }
	uint8_t getCurrentDirection(uint8_t ch) { if(ch <= 0 || ch >=11) return 0;return currentDirection[ch-1]; }
	uint8_t getDefaultDirection(uint8_t ch) { if(ch <= 0 || ch >=11) return 0;return defaultDirection[ch-1]; }
	InterruptService* getWheelEncoder(uint8_t ch) { if(ch <= 0 || ch >=11) ch=1;return wheelEncoderService[ch-1]; }
	CounterInterruptService* getWheelEncoderService(uint8_t ch) { if(ch <= 0 || ch >=11) ch=1;return wheelEncoderService[ch-1]; }
	void setChannels(uint8_t ch) { channels = ch; }
	uint8_t getChannels(void) { return channels; }
	void resetSpeeds(void);
	void resetEncoders(void);
	void setMotorShutdown(void) { commandEmergencyStop(1); MOTORSHUTDOWN = 1;}
	void setMotorRun(void) { commandEmergencyStop(0); MOTORSHUTDOWN = 0;}
	uint8_t getMotorShutdown(void) { return MOTORSHUTDOWN; }
	virtual void setMotorPowerScale(int p) { MOTORPOWERSCALE = p; }
	int getMotorPowerScale() { return MOTORPOWERSCALE; }
	virtual int checkSafeShutdown()= 0;
	virtual int get_dma_chan(uint8_t channel)=0;
	virtual int get_slice(uint8_t channel)=0;
	int64_t get_on_time_us(int ch);
}; //AbstractMotorControl

#endif //__ABSTRACTMOTORCONTROL_H__
