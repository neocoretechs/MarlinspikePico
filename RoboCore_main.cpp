
/*
 * RoboCore robotic controller platform.
 *
 * Processes a variation of M and G code from CNC and 3D printing to control a range of motor controllers and drivers
 * and GPIO pins for smart machine and robotics hardware functionality.
 *
 * Originally geared toward the Mega2560 microcontroller chip, later ported to Pico.
 * this code unifies the microcontroller platform and allows it to be easily accessed through
 * the standard CNC-derived 'language'. 
 *
 * As an example, it unifies the timers to deliver consistent PWM functions across all available pins, and allows
 * various motor controllers like H-bridge, half bridge, and smart controllers to be controlled through UART, PWM, and GPIO functions.
 *
 * It contains a main processing loop that receives M and G code commands through the UART via USB or TTL, like a CNC machine or 3D printer,
 * then uses a pure C++, rewritten, unified version of the Wiring library for arduino to handle the functions provided in Wiring through the 
 * main CNC-derived M and G code processing loop.
 *
 * Servos, ADCs, Timers, Pin change interrupts, Analog and Digital IO pins, SPI, I2C, PWM, UARTs, and motor controllers are all unified and
 * accessible through the main processing loop. In the primary loop, M and G code commands are processed such that this generic, multifunction, 
 * realtime, robotic smart controller can be attached to a single board computer (SBC), or used standalone and accessed through a UART, 
 * or potentially be attached via SPI, I2C or TWI to function as a realtime hardware controller.
 *
 * Another example is the way in which ultrasonic sensors can be attached to any motor driver through main loop processing commands and used to
 * set up a minimum distance to detect an object before issuing a command to shut down the motor driver.
 *
 * To continue the examples, through the processing loop, hall effect sensors can be attached to any motor driver through pins designated in the main
 * processing loop M and G code commands and used to detect wheel rotations, then set up pin change interrupts for those sensors through other
 * M and G code commands, then perform actions based on the wheel rotations and the interrupts generated and processed through other M and G code commands.
 *
 * A microcontroller running this code can be used to easily construct a robot, a 3D printer, a CNC machine, or any number of devices that need 
 * a generic smart controller capable of low level access to GPIO pins and hardware timers and ADCs etc.
 *
 * Copyright NeoCoreTechs 2020,2026
 * Author: Jonathan Neville Groff
 */

#include "language.h"
#include "RoboCore.h"
#include "new.h"
#include "math.h"
#include "Propulsion/AbstractMotorControl.h"
#include "Propulsion/RoboteqDevice.h"
#include "Propulsion/HBridgeDriver.h"
#include "Propulsion/SplitBridgeDriver.h"
#include "Propulsion/SwitchBridgeDriver.h"
#include "CounterInterruptService.h"
#include "AbstractPWMControl.h"
#include "VariablePWMDriver.h"

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html, protocol here is different but similar
// When 'stopped' is true the Gcodes G0-G5 are ignored as a safety interlock.

//===========================================================================
//=============================private variables=============================
//===========================================================================
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static uint8_t realtime_output = 1; // Determines whether real time data from inactive period is streamed

static char cmdbuffer[MAX_CMD_SIZE];
static char outbuffer[OUT_BUFFER_SIZE];

static char serial_char;
static int serial_read;
static int serial_count = 0;
static bool comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;

unsigned long starttime = 0;
unsigned long stoptime = 0;

bool Stopped=false;

bool target_direction;

byte mcu;
Analog* apin;
Digital* dpin;
PWM* ppin;
int nread = 0;
uint32_t micros = 0;
int* values;
String motorCntrlResp;
int status;
int fault = 0;
// Dynamically defined ultrasonic rangers
Ultrasonic* psonics[10]={0,0,0,0,0,0,0,0,0,0};
// Last distance published per sensor
float sonicDist[10]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
// Dynamically defined analog pins
int analogRanges[2][16];
Analog* panalogs[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// Dynamically defined digital pins
int digitalTarget[32];
Digital* pdigitals[32]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// PWM control block
PWM* ppwms[12]={0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t channel;
  
int digitarg;
static int uspin = 0;
unsigned long t;
int pin_number, pin_numberB;
int dir_pin, dir_default, enable_pin;
int encode_pin = 0;
uint8_t dir_face;
uint32_t dist;
int timer_res = 8; // resolution in bits
int timer_pre = 1; // 1 is no prescale

// &roboteqDevice, new HBridgeDriver, new SplitBridgeDriver...
AbstractMotorControl* motorControl[10]={0,0,0,0,0,0,0,0,0,0};
AbstractPWMControl* pwmControl[10]={0,0,0,0,0,0,0,0,0,0};
	
#define STEPS_PER_TURN 2048 // number of steps in 360deg;	
AccelStepper* accelStepper;
//===========================================================================
//=============================ROUTINES=============================
//===========================================================================


#define POLY 0x8408
int crc_ok = 0x470F;
unsigned short crc16(char *data_p, unsigned short length) {
	unsigned char i;
	unsigned int data;
	unsigned int crc;	
	crc = 0xffff;
	if (length == 0)
	return (~crc);
	do {
		for (i = 0 ,data = (unsigned int)0xff & *data_p++; i < 8; i++, data >>= 1) {
			if ((crc & 0x0001) ^ (data & 0x0001))
			crc = (crc >> 1) ^ POLY;
			else
			crc >>= 1;
		}
	} while (--length);
	crc = ~crc;
	data = crc;
	crc = (crc << 8) | (data >> 8 & 0xFF);
	return (crc);
}
void setup_killpin()
{
  #if defined(KILL_PIN) && KILL_PIN > -1
	Digital kp = new Digital(KILL_PIN);
    kp.pinMode(INPUT);
    kp.digitalWrite(HIGH);
  #endif
}

void setup_photpin()
{
  #if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
    SET_OUTPUT(PHOTOGRAPH_PIN);
    WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}

void setup_powerhold()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if defined(PS_ON_PIN) && PS_ON_PIN > -1
    SET_OUTPUT(PS_ON_PIN);
	#if defined(PS_DEFAULT_OFF)
	  WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
	  WRITE(PS_ON_PIN, PS_ON_AWAKE);
	#endif
  #endif
}

void suicide()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, LOW);
  #endif
}
const char* itoa(int value) {
    static char buf[16];   // enough for -2147483648\0
    char tmp[16];
    int i = 0;
    int neg = 0;
    if (value == 0) {
        buf[0] = '0';
        buf[1] = '\0';
        return buf;
    }
    if (value < 0) {
        neg = 1;
        value = -value;
    }
    while (value > 0) {
        tmp[i++] = '0' + (value % 10);
        value /= 10;
    }
    int pos = 0;
    if (neg)
        buf[pos++] = '-';
    // reverse digits into buf
    while (i > 0)
        buf[pos++] = tmp[--i];
    buf[pos] = '\0';
    return buf;
}

/*--------------------------------------------------
* Main setup a la wiring
*/
void setup()
{
  //setup_killpin();
  //setup_powerhold();
  stdio_init_all();   // enables USB CDC
  tud_init(0);        // initialize TinyUSB
  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();
  //Config_PrintSettings();
  //watchdog_init();
}

/*-------------------------------------------------
* And the primary command parser loop
*/
void loop() {
  get_command();
  if(!comment_mode) {
    process_commands();
  }
  manage_inactivity();
}
  
void get_command() {
	comment_mode = true;
    while (true) {
        tud_task();     // USB housekeeping
        if (tud_cdc_available())
            serial_count = tud_cdc_read(cmdbuffer, sizeof(cmdbuffer));
		else
			continue;
	if(serial_count <= 0) {
		comment_mode = true; //for new command
		continue;
	}
	serial_char = cmdbuffer[serial_count-1];
    if(serial_char == '\n' || serial_char == '\r' || serial_count >= (MAX_CMD_SIZE - 1) ) {
	  comment_mode = false;
      cmdbuffer[serial_count] = 0; //terminate string
      if(strchr(cmdbuffer, 'N') != NULL) {
          strchr_pointer = strchr(cmdbuffer, 'N');
          gcode_N = (strtol(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL, 10));
          if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer, "M110") == NULL) ) {
			tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
            tud_cdc_write(MSG_ERR_LINE_NO, strlen(MSG_ERR_LINE_NO));
            tud_cdc_write(gcode_LastN,strlen(gcode_LastN));
			tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
            //Serial.println(gcode_N);
            FlushSerialRequestResend();
            serial_count = 0;
			comment_mode = true;
            return;
          }
          if(strchr(cmdbuffer, '*') != NULL) {
            unsigned short checksum = 0;
            byte count = 0;
            //while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer, '*');
			checksum = crc16(&cmdbuffer[strchr_pointer - cmdbuffer + 1],(strchr_pointer - cmdbuffer + 1) );
            if( (int)(strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL)) != checksum) {
			  tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
              tud_cdc_write(MSG_ERR_CHECKSUM_MISMATCH,strlen(MSG_ERR_CHECKSUM_MISMATCH));
              tud_cdc_write(gcode_LastN,strlen(gcode_LastN));
			  tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
              FlushSerialRequestResend();
              serial_count = 0;
			  comment_mode = true;
              return;
            }
            //if no errors, continue parsing
          } else {
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
            tud_cdc_write(MSG_ERR_NO_CHECKSUM,strlen(MSG_ERR_NO_CHECKSUM));
            tud_cdc_write(gcode_LastN,strlen(gcode_LastN));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
            FlushSerialRequestResend();
            serial_count = 0;
			comment_mode = true;
            return;
          }
          gcode_LastN = gcode_N;
          //if no errors, continue parsing
      } else { // if we don't receive 'N' but still see '*'
          if((strchr(cmdbuffer, '*') != NULL)) {
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
            tud_cdc_write(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM,strlen(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM));
            tud_cdc_write(gcode_LastN,strlen(gcode_LastN));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
            serial_count = 0;
			comment_mode = true;
            return;
          }
      }
      if(strchr(cmdbuffer, ';') != NULL) {
		  comment_mode = true;
          serial_count = 0; //clear buffer
		  return;
	  }
	  // Determine if an outstanding error caused safety shutdown. If so respond with header
      if((strchr(cmdbuffer, 'G') != NULL)){
          strchr_pointer = strchr(cmdbuffer, 'G');
          switch((int)((strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL)))) {
			case 0:
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
				if(Stopped) { // If robot is stopped by an error the G[0-5] codes are ignored.
					tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
					tud_cdc_write(MSG_ERR_STOPPED,strlen(MSG_ERR_STOPPED));
					tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
					tud_cdc_write_flush();
					comment_mode = true;
					serial_count = 0;
					return;
				}
				break;
			default:
				break;
          }
        }
		// finished processing c/r terminated cmdl
		serial_count = 0;
		return;
      } // if c/r l/f
      cmdbuffer[serial_count++] = serial_char;
  } // while avail

}

float code_value()
{
  return (strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL));
}

long code_value_long()
{
  return (strtol(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer, code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

/*----------------------------------------
*
* Process the command sequence
*-----------------------------------------
*/
void process_commands() { 
  if(code_seen('G')) {
	  int cval = (int)code_value();
	  processGCode(cval);
  } else {
	  if(code_seen('M') ) {
		  int cval = (int)code_value();
		  processMCode(cval);
	  } else { // if neither G nor M code
		   int ibuf = 0;
		   tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
		   tud_cdc_write(MSG_UNKNOWN_COMMAND,strlen(MSG_UNKNOWN_COMMAND));
		   while(cmdbuffer[ibuf]) tud_cdc_write(cmdbuffer[ibuf++], 1);
		   tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
		   tud_cdc_write_flush();
	  }
  }
}
/*--------------------------
* Process the Gcode command sequence
*---------------------------
*/
void processGCode(int cval) {
	  int result;
	  int motorController, PWMDriver, motorChannel, motorPower, PWMLevel;
	  unsigned long codenum;
	  char *starpos = NULL;
    switch(cval)
    {
	    
    case 4: // G4 dwell
      //LCD_MESSAGEPGM(MSG_DWELL);
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      //codenum += millis();  // keep track of when we started waiting
      previous_millis_cmd = 0;//millis();
      while(++previous_millis_cmd  < codenum ){
        manage_inactivity();
		_delay_ms(1);
      }
	  tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
	  tud_cdc_write("G4", strlen("G4"));
	  tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
	  tud_cdc_write_flush();
      break;
	  
	case 5: // G5 - Absolute command motor [Z<controller>] C<Channel> [P<motor power -1000 to 1000>] [X<PWM power -1000 to 1000>(scaled 0-2000)]
	     if(!Stopped) {
			 if(code_seen('Z')) {
				 motorController = code_value();
			 }
		     if(code_seen('C')) {
				motorChannel = code_value(); // channel 1,2
				if(code_seen('P')) {
					motorPower = code_value(); // motor power -1000,1000
					fault = 0; // clear fault flag
					if( (status=motorControl[motorController]->commandMotorPower(motorChannel, motorPower)) ) {
							tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
							tud_cdc_write(MSG_BAD_MOTOR,strlen(MSG_BAD_MOTOR));
							tud_cdc_write(status,strlen(status));
							tud_cdc_write(" ", 1);
							tud_cdc_write(motorChannel, strlen(motorChannel));
							tud_cdc_write(" ", 1);
							tud_cdc_write(motorPower, strlen(motorPower));
							tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
							tud_cdc_write_flush();
					} else {
							tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
							tud_cdc_write("G5", strlen("G5"));
							tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
							tud_cdc_write_flush();
					}
				} else {// code P or X
					if(code_seen('X')) {
						PWMLevel = code_value(); // PWM level -1000,1000, scaled to 0-2000 in PWM controller, as no reverse
						fault = 0; // clear fault flag
						// use motor related index and value, as we have them
						if( (status=pwmControl[motorController]->commandPWMLevel(motorChannel, PWMLevel)) ) {
							tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
							tud_cdc_write(MSG_BAD_PWM,strlen(MSG_BAD_PWM));
							tud_cdc_write(status,strlen(status));
							tud_cdc_write(" ", 1);
							tud_cdc_write(motorChannel, strlen(motorChannel));
							tud_cdc_write(" ", 1);
							tud_cdc_write(PWMLevel, strlen(PWMLevel));
							tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
							tud_cdc_write_flush();
						} else {
							tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
							tud_cdc_write("G5", strlen("G5"));
							tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
							tud_cdc_write_flush();
						}
					} // code X
				}
			 } // code C
	     } // stopped
	     break;
	  
	case 99: // G99 start watchdog timer. G99 T<time_in_millis> values are 15,30,60,120,250,500,1000,4000,8000 default 4000
		if( code_seen('T') ) {
			int time_val = code_value();
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("G99", strlen("G99"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		}
		break;
		
	case 100: // G100 reset watchog timer before time interval is expired, otherwise a reset occurs
		tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
		tud_cdc_write("G100", strlen("G100"));
		tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;
	
	case 200: // G200 set up stepper. G200 W<wires else default 4> P<pin 1 default 22> Q<pin 2 default 24> R<pin 3 default 26> S<pin 4 defualt 28> F<pulse width default 20> M<motor speed default 500> A<motor accel def 400>
		int swire;
		swire = code_seen('W') ? code_value() : 4;
		int p1,q2,r3,s4;
		p1 = code_seen('P') ? code_value() : 22;
		q2 = code_seen('Q') ? code_value() : 24;
		r3 = code_seen('R') ? code_value() : 26;
		s4 = code_seen('S') ? code_value() : 28;
		int pulseWidth;
		int motorSpeed;
		int motorAccel;
		pulseWidth = code_seen('F') ? code_value() : 20;
		motorSpeed = code_seen('M') ? code_value() : 500;
		motorAccel = code_seen('A') ? code_value() : 400;
		tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
		tud_cdc_write("G200", strlen("G200"));
		tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;
		
	case 201: // G201 stepper stop
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("G201", strlen("G201"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		break;
	
	case 202: // G202 S<steps>  clockwise
		int steps;
			if(code_seen('S'))
				steps = code_value();
	        // The two lines that follow allow to send commands in any sequence:
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("G202", strlen("G202"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		break;
		
	case 203: // G203 S<steps>  anti-clockwise
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("G203", strlen("G203"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		break;	
		
	default:
		int ibuf = 0;
		tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
		tud_cdc_write(MSG_UNKNOWN_GCODE, strlen(MSG_UNKNOWN_GCODE));
		while(cmdbuffer[ibuf]) tud_cdc_write(cmdbuffer[ibuf++], strlen(cmdbuffer[ibuf++]));
		tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;
		
    } // switch
  } // if code
  
  /*-------------------------------------
  * M Code processing
  *--------------------------------------
  */
  void processMCode(int cval) {
	  int motorController = 0; 
	  int PWMDriver = 0;
	  bool assigned;
	  bool enable;
	  int power;
	  
    switch( cval ) {
	case 0: // M0 - Set real time output off
		realtime_output = 0;
		tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
		tud_cdc_write("M0", strlen("M0"));
		tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;
		
	case 1: // M1 - Set real time output on
		realtime_output = 1;
		tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
		tud_cdc_write("M1", strlen("M1"));
		tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;
		
	//CHANNEL 1-10, NO CHANNEL ZERO!	
	case 2: // M2 [Z<slot>] [C<channel> W<encoder pin> E<default dir>] - set smart controller (default) with optional encoder pin per channel, can be issued multiple times
		 if(code_seen('Z')) {
			 motorController = code_value();
		 }
		if(code_seen('C')) {
			channel = code_value();
			if(channel <= 0) {
				break;
			}
			if(code_seen('W')) {
				pin_number = code_value();
				motorControl[motorController]->createEncoder(channel, pin_number);
			}
			if(code_seen('E')) {
				motorControl[motorController]->setDefaultDirection(channel, code_value());
			}
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M2", strlen("M2"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		}
		break;
		
	// Set HBridge PWM motor driver, map pin to channel, this will check to prevent free running motors during inactivity
	// For a PWM motor control subsequent G5 commands are affected here.
	// and then D a direction pin that determines forward/backward , then E, the default value of the direction pin.
	// The D and E pins determine whether a HIGH or LOW determines FORWARD/BACK on the given controller channel. This is
	// to compensate for 'backward' motors mounted left/right, or differences in controller design. Using a combination of
	// these 2 parameters you can tune any controller/motor setup properly for forward/back.
	// Finally, W<encoder pin>  to receive hall wheel sensor signals and
	// optionally PWM timer setup [R<resolution 8,9,10 bits>] [X<prescale 0-7>].
	// The Timer mode (0-3) is preset to 2 in the individual driver. Page 129 in datasheet. Technically we are using a 'non PWM'
	// where the 'compare output mode' is defined by 3 operating modes. Since we are unifying all the timers to use all available PWM
	// pins, the common mode among them all is the 'non PWM', within which the 3 available operating modes can be chosen from.
	// There are essentially three main operating modes:
	// 0 - Stop
	// 1 - Toggle on compare match
	// 2 - Clear on match
	// 3 - Set on match
	// For motor operation and general purpose PWM, mode 2 the most universally applicable.
	case 3: // M3 [Z<slot>] P<pin> C<channel> D<direction pin> E<default dir> W<encoder pin> [R<resolution 8,9,10 bits>] [X<prescale 0-7>]
		timer_res = 8; // resolution in bits
		timer_pre = 1; // 1 is no prescale
		pin_number = -1;
		encode_pin = 0;
		if(code_seen('Z')) {
			motorController = code_value();
		}
	 // motorControl = (AbstractMotorControl*)&hBridgeDriver;
	 if(motorControl[motorController]) {
	  ((HBridgeDriver*)motorControl[motorController])->setMotors((PWM**)&ppwms);
	  ((HBridgeDriver*)motorControl[motorController])->setDirectionPins((Digital**)&pdigitals);
	  if(code_seen('P')) {
          pin_number = code_value();
	  } else {
		 break;
	  }
      if(code_seen('C')) {
        channel = code_value();
		if(channel <= 0) {
			break;
		}
		if( code_seen('D')) {
			dir_pin = code_value();
		} else {
			break;
		}
		if( code_seen('E')) {
			dir_default = code_value();
		} else {
			break;
		}
		if( code_seen('W')) {
			encode_pin = code_value();
		}
		if(code_seen('X')) {
			timer_pre = code_value();
		}
		if( code_seen('R')) {
			timer_res = code_value();
		}
		((HBridgeDriver*)motorControl[motorController])->createPWM(channel, pin_number, dir_pin, dir_default, timer_pre, timer_res);
		if(encode_pin) {
			motorControl[motorController]->createEncoder(channel, encode_pin);
		}
		tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
		tud_cdc_write("M3", strlen("M3"));
		tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
	   } // code_seen['C']
      } // if motorControl[motorController]
	  break;
	  
	// Split bridge or 2 half bridge motor controller. Takes 2 inputs: one for forward,called P, one for backward,called Q, then motor channel, 
	// and then D, an enable pin. Finally, W<encoder pin>  to receive hall wheel sensor signals and 
	// optionally PWM timer setup [R<resolution 8,9,10 bits>] [X<prescale 0-7>].
	// Everything derived from HBridgeDriver can be done here.
	case 4:// M4 [Z<slot>] P<pin> Q<pin> C<channel> D<enable pin> E<default dir> [W<encoder pin>] [R<resolution 8,9,10 bits>] [X<prescale 0-7>]
	  timer_res = 8; // resolution in bits
	  timer_pre = 1; // 1 is no prescale
	  pin_number = -1;
	  pin_numberB = -1;
	  encode_pin = 0;
	  if(code_seen('Z')) {
		motorController = code_value();
	  }
	  if(motorControl[motorController]) {
	  //motorControl = (AbstractMotorControl*)&splitBridgeDriver;
	  ((SplitBridgeDriver*)motorControl[motorController])->setMotors((PWM**)&ppwms);
	  ((SplitBridgeDriver*)motorControl[motorController])->setDirectionPins((Digital**)&pdigitals);
	  if(code_seen('P')) {
		pin_number = code_value();
	  } else {
		 break;
	  }
	  if(code_seen('Q')) {
		pin_numberB = code_value();
	 } else {
		break;
	 }
	  if(code_seen('C')) {
		  channel = code_value();
		  if(channel <= 0) {
			break;
		  }
		  if( code_seen('D')) {
			dir_pin = code_value();
		  } else {
			break;
		  }
		  if( code_seen('E')) {
			dir_default = code_value();
		  } else {
			break;
		  }
		  if( code_seen('W')) {
			encode_pin = code_value();
		  }
		  if(code_seen('X')) {
				timer_pre = code_value();
		  }
		  if( code_seen('R')) {
				timer_res = code_value();
		  }
		  ((SplitBridgeDriver*)motorControl[motorController])->createPWM(channel, pin_number, pin_numberB, dir_pin, dir_default, timer_pre, timer_res);
		  if(encode_pin) {
			motorControl[motorController]->createEncoder(channel, encode_pin);
		  }
		  tud_cdc_write(MSG_BEGIN,strlen);
		  tud_cdc_write("M4", strlen("M4"));
		  tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
		  tud_cdc_write_flush();
		} // code C
		} //motorcontrol[motorcontroller]
		break;
		
	// Switch bridge or 2 digital motor controller. Takes 2 inputs: one digital pin for forward,called P, one for backward,called Q, then motor channel,
	// and then D, an enable pin, and E default dir, with optional encoder
	case 5: //M5 Z<slot> P<pin> Q<pin> C<channel> D<enable pin> E<default dir> [W<encoder>]- Create switch bridge Z slot, P forward pin, Q reverse pin, D enable, E default state of enable for dir
		pin_number = -1;
		pin_numberB = -1;
		encode_pin = 0;
		  if(code_seen('Z')) {
			  motorController = code_value();
		  }
		  if(motorControl[motorController]) {
			  ((SwitchBridgeDriver*)motorControl[motorController])->setPins((Digital**)&pdigitals);
			  if(code_seen('P')) {
				  pin_number = code_value();
			  } else {
				  break;
			  }
			  if(code_seen('Q')) {
				  pin_numberB = code_value();
			  } else {
				  break;
			  }
			  if(code_seen('C')) {
				  channel = code_value();
				  if(channel <= 0) {
					  break;
				  }
				  if( code_seen('D')) {
					  dir_pin = code_value();
				  } else {
					  break;
				  }
				  if( code_seen('E')) {
					  dir_default = code_value();
				  } else {
					  break;
				  }
				  if( code_seen('W')) {
					 encode_pin = code_value();
				  }
				  ((SwitchBridgeDriver*)motorControl[motorController])->createDigital(channel, pin_number, pin_numberB, dir_pin, dir_default);
				  if(encode_pin) {
					  motorControl[motorController]->createEncoder(channel, encode_pin);
				  }
				  tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				  tud_cdc_write("M5", strlen("M5"));
				  tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				  tud_cdc_write_flush();
			  } // code C
		  } //motorcontrol[motorcontroller]
		break;
		
	case 6: //M6 [Z<slot>] [S<scale>] [X<scale>] - Set motor or PWM scaling, divisor for final power to limit speed or level, set to 0 to cancel. If X, slot is PWM
		if(code_seen('Z')) {
			motorController = code_value();
		}
		if( code_seen('S') ) {
			if(motorControl[motorController]) {
				motorControl[motorController]->setMotorPowerScale(code_value());
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M6", strlen("M6"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			}
		} else {
			if(code_seen('X')) {
				if(pwmControl[motorController]) {
					pwmControl[motorController]->setPWMPowerScale(code_value());
					tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
					tud_cdc_write("M6", strlen("M6"));
					tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
					tud_cdc_write_flush();
				}
			}
		}
		break;
		
	case 7: // M7 [Z<slot>] [X]- Set motor override to stop motor operation, or optionally PWM operation, if X, slot is PWM
		if(code_seen('Z')) {
			motorController = code_value();
		}
		if(code_seen('X')) {
			if(pwmControl[motorController]) {
				pwmControl[motorController]->setPWMShutdown();
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M7", strlen("M7"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			}
		} else {
			if(motorControl[motorController]) {
				motorControl[motorController]->setMotorShutdown();
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M7", strlen("M7"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			}
		}
		break;
		
	case 8: // M8 [Z<slot>][X] - Set motor override to start motor operation after stop override M7. If X, slot is PWM
		if(code_seen('Z')) {
			motorController = code_value();
		}
		if(code_seen('X')) {
			if(pwmControl[motorController]) {
				pwmControl[motorController]->setPWMRun();
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M8", strlen("M8"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			}
		} else {
			if(motorControl[motorController] ) {
				motorControl[motorController]->setMotorRun();
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M8", strlen("M8"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			}
		}
		break;
		
	// Activate a previously created PWM controller of type AbstractPWMControl - a non propulsion PWM device such as LED or pump
	// Note there is no encoder or direction pin, and no possibility of reverse. What would be reverse in a motor control is the first
	// half of the power scale instead.
	case 9: // M9 [Z<slot>] P<pin> C<channel> D<enable pin> [R<resolution 8,9,10 bits>] [X<prescale 0-7>] - PWM control
		pin_number = -1;
		encode_pin = 0;
		if(code_seen('Z')) {
			  PWMDriver = code_value();
		}
		if(pwmControl[PWMDriver]) {
		 ((VariablePWMDriver*)pwmControl[PWMDriver])->setPWMs((PWM**)&ppwms);
		 ((VariablePWMDriver*)pwmControl[PWMDriver])->setEnablePins((Digital**)&pdigitals);
		 if(code_seen('P')) {
			  pin_number = code_value();
		 } else {
			  break;
		 }
		 if(code_seen('C')) {
			channel = code_value();
			if(channel <= 0) {
				break;
			}
			if( code_seen('D')) {
				enable_pin = code_value();
			} else {
				break;
			}
			((VariablePWMDriver*)pwmControl[PWMDriver])->createPWM(channel, pin_number, enable_pin, timer_pre, timer_res);
			tud_cdc_write(MSG_BEGIN);
			tud_cdc_write("M9",strlen("M9"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		 }
		}
		break;
		
	// Dynamically allocate a controller to a control slot. The slot parameter is used to refer
	// to the dynamically allocated controller in other M codes that relate to motor control functions.
	// The M10 code merely creates the instance of the proper controller and assigns the slot. Other M codes
	// refer to the slot and provide further configuration. when creating new type of controllers, this is the code
	// that can be expanded to instantiate those controllers
	case 10: // M10 Z<controller slot> T<controller type>
		if( code_seen('Z') ) {
			motorController = code_value();
			if( code_seen('T') ) {
				int controllerType = code_value();		 
				switch(controllerType) {
					case 0: // type 0 smart controller
						if( motorControl[motorController] ) {
							delete motorControl[motorController];
							motorControl[motorController] = 0; // in case assignment below fails
						}
						motorControl[motorController] = new RoboteqDevice();
						tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
						tud_cdc_write("M10", strlen("M10"));
						tud_cdc_write(MSG_TERMINATE);
						tud_cdc_write_flush();
						break;
					case 1: // type 1 Hbridge
						// up to 10 channels, each channel has a direction pin (1), and a PWM pin (0)
						if(motorControl[motorController]) {
							// for each channel, delete the direction pin and PWM created in main pin array to prepare new assignment
							// each controller can have up to 10 channels, each with its own PWM and direction pin
							for(uint8_t i = 0; i < motorControl[motorController]->getChannels(); i++) {
									uint8_t pMotor1 = ((HBridgeDriver*)motorControl[motorController])->getMotorPWMPin(i);
									uint8_t pMotor2 = ((HBridgeDriver*)motorControl[motorController])->getMotorEnablePin(i);
									if(pMotor2 != 255 && pdigitals[pMotor2]) {
										delete pdigitals[pMotor2];
										pdigitals[pMotor2] = 0;
									}
									if(pMotor1 != 255 && ppwms[pMotor1]) {
										delete ppwms[pMotor1];
										ppwms[pMotor1] = 0;
									}
							}
							delete motorControl[motorController];
							motorControl[motorController] = 0; // in case assignment below fails
						}
						motorControl[motorController] = new HBridgeDriver();
						tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
						tud_cdc_write("M10", strlen("M10"));
						tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
						tud_cdc_write_flush();
						break;
					case 2: // type 2 Split bridge, each channel has 2 PWM pins and an enable pin, so up to 5 channels
						if(motorControl[motorController]) {
								// for each channel, delete the direction pin and PWM created in main pin array to prepare new assignment
								// each controller can have up to 10 channels, each with its own PWM and direction pin
								for(uint8_t i = 0; i < motorControl[motorController]->getChannels(); i++) {
										uint8_t pMotor1 = ((SplitBridgeDriver*)motorControl[motorController])->getMotorPWMPin(i);
										uint8_t pMotor2 = ((SplitBridgeDriver*)motorControl[motorController])->getMotorEnablePin(i);
										if(pMotor2 != 255 && pdigitals[pMotor2]) {
											delete pdigitals[pMotor2];
											pdigitals[pMotor2] = 0;
										}
										if(pMotor1 != 255 && ppwms[pMotor1]) {
											delete ppwms[pMotor1];
											ppwms[pMotor1] = 0;
										}
										pMotor1 = ((SplitBridgeDriver*)motorControl[motorController])->getMotorPWMPinB(i);
										if(pMotor1 != 255 && ppwms[pMotor1]) {
											delete ppwms[pMotor1];
											ppwms[pMotor1] = 0;
										}
										
								}
								delete motorControl[motorController];
								motorControl[motorController] = 0; // in case assignment below fails
						}
						motorControl[motorController] = new SplitBridgeDriver();
						tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
						tud_cdc_write("M10", strlen("M10"));
						tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
						tud_cdc_write_flush();
						break;
					case 3: // type 3 Switch bridge, each channel has 2 PWM pins and an enable pin, so up to 5 channels
						if(motorControl[motorController]) {
							// for each channel, delete the direction pin and PWM created in main pin array to prepare new assignment
							// each controller can have up to 10 channels, each with its own PWM and direction pin
							for(uint8_t i = 0; i < motorControl[motorController]->getChannels(); i++) {
								uint8_t pMotor1 = ((SwitchBridgeDriver*)motorControl[motorController])->getMotorDigitalPin(i);
								uint8_t pMotor2 = ((SwitchBridgeDriver*)motorControl[motorController])->getMotorEnablePin(i);
								if(pMotor2 != 255 && pdigitals[pMotor2]) {
									delete pdigitals[pMotor2];
									pdigitals[pMotor2] = 0;
								}
								if(pMotor1 != 255 && pdigitals[pMotor1]) {
									delete pdigitals[pMotor1];
									pdigitals[pMotor1] = 0;
								}
								pMotor1 = ((SwitchBridgeDriver*)motorControl[motorController])->getMotorDigitalPinB(i);
								if(pMotor1 != 255 && pdigitals[pMotor1]) {
									delete pdigitals[pMotor1];
									pdigitals[pMotor1] = 0;
								}
							}
							delete motorControl[motorController];
							motorControl[motorController] = 0; // in case assignment below fails
						}
						motorControl[motorController] = new SwitchBridgeDriver();
						tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
						tud_cdc_write("M10", strlen("M10"));
						tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
						tud_cdc_write_flush();
						break;
					case 4: // Type 4 non-propulsion PWM driver 
						if(pwmControl[motorController]) {
							// for each channel, delete the direction pin and PWM created in main pin array to prepare new assignment
							// each controller can have up to 10 channels, each with its own PWM and direction pin
							for(uint8_t i = 0; i < pwmControl[motorController]->getChannels(); i++) {
								uint8_t pMotor1 = ((VariablePWMDriver*)pwmControl[motorController])->getPWMEnablePin(i);
									if(pMotor1 != 255 && pdigitals[pMotor1]) {
										delete pdigitals[pMotor1];
										pdigitals[pMotor1] = 0;
									}
									pMotor1 = ((VariablePWMDriver*)pwmControl[motorController])->getPWMLevelPin(i);
									if(pMotor1 != 255 && ppwms[pMotor1]) {
										delete ppwms[pMotor1];
										ppwms[pMotor1] = 0;
									}
							}
							delete pwmControl[motorController];
							motorControl[motorController] = 0; // in case assignment below fails
						}
						pwmControl[motorController] = new VariablePWMDriver();
						tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
						tud_cdc_write("M10", strlen("M10"));
						tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
						tud_cdc_write_flush();
						break;
					default:
						tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
						tud_cdc_write("BAD CONTROLLER TYPE:", strlen("BAD CONTROLLER TYPE:"));
						tud_cdc_write(controllerType, strlen(controllerType));
						tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
						tud_cdc_write_flush();
						break;
				}
			} else {
				break;
			}
		}
		break;
		
	case 11: // M11 [Z<slot>] C<channel> [D<duration>] [X<duration>] - Set maximum cycle duration for given channel. If X, slot is PWM
		if(code_seen('Z')) {
			motorController = code_value();
		}
		if( code_seen('C') ) {
			channel = code_value();
			if(channel <= 0) {
				break;
			}
			if(code_seen('X')) {
				if(pwmControl[motorController]) {
					pwmControl[motorController]->setDuration(channel, code_value());
					tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
					tud_cdc_write("M11", strlen("M11"));
					tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
					tud_cdc_write_flush();
				}
			} else {
				if(code_seen('D')) {
					if(motorControl[motorController]) {
						motorControl[motorController]->setDuration(channel, code_value());
						tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
						tud_cdc_write("M11", strlen("M11"));
						tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
						tud_cdc_write_flush();
					}
				}
			}
		}
		break;
	
	case 12: // M12 [Z<slot>] C<channel> [P<offset>] [X<offset>] - set amount to add to G5 for min motor power, or X PWM level, If X, slot is PWM
		if(code_seen('Z')) {
			motorController = code_value();
		}
		if( code_seen('C') ) {
			channel = code_value();
			if(channel <= 0) {
				break;
			}
			if(code_seen('X')) {
				if(pwmControl[motorController]) {
					pwmControl[motorController]->setMinPWMLevel(channel, code_value());
					tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
					tud_cdc_write("M12", strlen("M12"));
					tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
					tud_cdc_write_flush();
				}
			} else {
				if( code_seen('P')) {
					if(motorControl[motorController]) {
						motorControl[motorController]->setMinMotorPower(channel, code_value());
						tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
						tud_cdc_write("M12", strlen("M12"));
						tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
						tud_cdc_write_flush();
					}
				}
			}
		}
		break;
		
	  case 13: //M13 [Z<slot>] [P<power>] [X<power>]- Set maximum motor power or optionally with X, a PWM control maximum level. If X, slot is PWM
		if(code_seen('Z')) {
		  motorController = code_value();
		}
		if( code_seen('P') ) {
		  if(motorControl[motorController]) {
			  motorControl[motorController]->setMaxMotorPower(code_value());
			  tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			  tud_cdc_write("M13", strlen("M13"));
			  tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			  tud_cdc_write_flush();
		  }
		  } else {
		  	if(code_seen('X')) {
			  if(pwmControl[motorController]) {
				  pwmControl[motorController]->setMaxPWMLevel(code_value());
				  tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				  tud_cdc_write("M13", strlen("M13"));
				  tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				  tud_cdc_write_flush();
			  }
		  }
		}
	  break; 
	//  
	// M14 [Z<slot>] C<channel> P<pin> L<low range active> H<high range active> N<number of counts before interrupt generated>
	// Create analog encoder for controller at slot and channel.
	// Activate interrupt between L low and H high range.
	// Detect range N times before interrupt
	//
	case 14:
			double analogRangeL, analogRangeH;
			int counts;
			interrupt_pin = 0;
			if(code_seen('Z')) {
				motorController = (int) code_value();
			}
			if( code_seen('C') ) {
				channel = (int) code_value();
				if(channel <= 0) {
					break;
				}
				if(code_seen('P')) {
					int pin = (int) code_value();
					analogRangeL = code_seen('L') ? code_value() : 0;
					analogRangeH = code_seen('H') ? code_value() : 0;
					counts = (int) (code_seen('N') ? code_value() : 1);
					if( code_seen('I')) {
						interrupt_pin = (int) code_value();
					}
					motorControl[motorController].createEncoder(channel, pin, analogRangeL, analogRangeH, counts, interrupt_pin);
					tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			 		tud_cdc_write("M14", strlen("M14"));
			 		tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
					tud_cdc_write_flush();
				}
			}
			break;
			//  
			// M15 [Z<slot>] C<channel> P<pin> S<pin state 0 low, 1 high> N<number of counts before interrupt generated>
			// Create digital encoder for controller at slot and channel.
			// Activate interrupt at S pin state.
			// Detect range N times before interrupt
			//
		case 15:
			int digitalState;
			if(code_seen('Z')) {
				motorController = (int) code_value();
			}
			if( code_seen('C') ) {
				channel = (int) code_value();
				if(channel <= 0) {
					break;
				}
				if(code_seen('P')) {
					int pin = (int) code_value();
					digitalState = (int) (code_seen('S') ? code_value() : 1);
					counts = (int) (code_seen('N') ? code_value() : 1);
					if( code_seen('I')) {
						interrupt_pin = (int) code_value();
					}
					motorControl[motorController].createDigitalEncoder(channel, pin, 1, counts, interrupt_pin);
			 		tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			 		tud_cdc_write("M15", strlen("M15"));
			 		tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			 		tud_cdc_write_flush();
				}
			}
			break;
			//
			// M16 [Z<slot>] P<pin> C<channel> D<direction pin> E<default dir> [W<encoder pin>] 
			// Set Delayed HBridge PWM motor driver, map pin to channel.
			// For a motor control subsequent G5 commands are affected here.
			// and then D a direction pin that determines forward/backward , then E, the default value of the direction pin.
			// The D pin and E direction determine whether a HIGH or LOW determines FORWARD/BACK on the given controller channel. This is
			// to compensate for 'backward' motors mounted left/right, or differences in controller design. Using a combination of
			// these 2 parameters you can tune any controller/motor setup properly for forward/back.
			// Finally, W<encoder pin> to receive hall wheel sensor signals. 
			// Leave out W option and use M14 later to create a more robust encoder. If W is used, an analog input
			// that fires at the maximum input level is created.
			//
		case 16: 
			pin_number = -1;
			encode_pin = 0;
			interrupt_pin = 0;
			if(code_seen('Z')) {
				motorController = (int) code_value();
			}
			if(motorControl[motorController] != null) {
				if(code_seen('P')) {
					pin_number = (int) code_value();
				} else {
					tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
					tud_cdc_write("M16 PIN ERROR", strlen("M16 PIN ERROR"));
					tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
					tud_cdc_write_flush();
					break;
				}
				if(code_seen('C')) {
					channel = (int) code_value();
					if(channel <= 0) {
						tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
						tud_cdc_write("M16 CHANNEL ERROR", strlen("M16 CHANNEL ERROR"));
						tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
						tud_cdc_write_flush();
						break;
					}
					if( code_seen('D')) {
						dir_pin = (int) code_value();
					} else {
						tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
						tud_cdc_write("M16 DIR PIN ERROR", strlen("M16 DIR PIN ERROR"));
						tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
						tud_cdc_write_flush();
						break;
					}
					if( code_seen('E')) {
						dir_default = (int) code_value();
					} else {
						tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
						tud_cdc_write("M16 DIR DEFAULT ERROR", strlen("M16 DIR DEFAULT ERROR"));
						tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));	
						tud_cdc_write_flush();
						break;
					}
					// optional
					if( code_seen('W')) {
						encode_pin = (int) code_value();
					}
					((HBridgeDriver)motorControl[motorController]).createPWM(channel, pin_number, dir_pin, dir_default, pwm_freq, pwm_duty);
					if(encode_pin != 0) {
						motorControl[motorController].createEncoder(channel, encode_pin, interrupt_pin);
					}
				}
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M16", strlen("M16"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			} else {
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M16 CONTROLLER UNASSIGNED ERROR", strlen("M16 CONTROLLER UNASSIGNED ERROR"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			}
			break;
	case 33: // M33 [Z<slot>] P<ultrasonic pin> D<min. distance in cm> [E<direction 1- forward facing, 0 - reverse facing sensor>] 
	// link Motor controller to ultrasonic sensor, the sensor must exist via M301
		if(code_seen('Z')) {
			motorController = code_value();
		}
		if(motorControl[motorController]) {
	  		pin_number = 0;
	  		if(code_seen('P')) {
        		pin_number = code_value();
				if( code_seen('D')) {
					dist = code_value();
				} else {
					tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
					tud_cdc_write("M33 DIST ERROR", strlen("M33 DIST ERROR"));
					tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
					tud_cdc_write_flush();
					break;
				}
				dir_face = 1; // default forward
				if( code_seen('E')) {
					dir_face = code_value(); // optional
				}
				motorControl[motorController]->linkDistanceSensor((Ultrasonic**)psonics, pin_number, dist, dir_face);
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M33", strlen("M33"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			} else {
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M33 PIN ERROR", strlen("M33 PIN ERROR"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			} // code seen 'P'
	  	} else {// motor control exists
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M33 CONTROLLER ERROR", strlen("M33 CONTROLLER ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));	
			tud_cdc_write_flush();
		}
	  break;
	  
	  case 35: //M35 - Clear all digital pins
		for(int i = 0; i < 32; i++) {
			 if(pdigitals[i]) {
				unassignPin(pdigitals[i]->pin);
				delete pdigitals[i];
				pdigitals[i] = 0;
			 }
		}
		tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
		tud_cdc_write("M35", strlen("M35"));
		tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
	  break;
		  
	  case 36: //M36 - Clear all analog pins
		  	 for(int i = 0; i < 16; i++) {
			  	  if(panalogs[i]) {
				  	delete panalogs[i];
					panalogs[i] = 0;
			  	  }
		  	 }
			 tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			 tud_cdc_write("M36", strlen("M36"));
			 tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			 tud_cdc_write_flush();
	  break;
	  
	  case 37: //M37 - Clear all PWM pins, ALL MOTOR AND PWM DISABLED, perhaps not cleanly
		for(int i = 0; i < 12; i++) {
		  if(ppwms[i]) {
			  delete ppwms[i];
			  ppwms[i] = 0;
		  }
		}
	  	tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
	  	tud_cdc_write("M37", strlen("M37"));
	  	tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
	  	tud_cdc_write_flush();
	  break;
	  
	  case 38: //M38  P<pin> - Remove PWM pin, MOTOR AND PWM DISABLED, perhaps not cleanly
	  	  pin_number = -1;
	  	  if (code_seen('P')) {
		  	pin_number = code_value();
			bool found = false;
			for(int i = 0; i < 12; i++) {
				if(ppwms[i] && ppwms[i]->pin == pin_number) {
					delete ppwms[i];
					ppwms[i] = 0;
					found = true;
					break;
				} // pwms == pin_number
			} // i iterate pwm array
			if(found) {
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M38", strlen("M38"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			} else {
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M38 UNASSIGNED ERROR", strlen("M38 UNASSIGNED ERROR"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			}
	  	  } else {// code P
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M38 PIN ERROR", strlen("M38 PIN ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		  }
	  break;
	  
	  case 39: //M39 P<pin> - Remove Persistent Analog pin 
	  	  pin_number = -1;
	  	  if (code_seen('P')) {
		  	pin_number = code_value();
			bool found = false;
			for(int i = 0; i < 16; i++) {
				if(panalogs[i] && panalogs[i]->pin == pin_number) {
					delete panalogs[i];
					panalogs[i] = 0;
					found = true;
					break;
				}
			}
		  	if(found) {
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M39", strlen("M39"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			} else {
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M39 UNASSIGNED ERROR", strlen("M39 UNASSIGNED ERROR"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			}
	  	  } else {// code P
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M39 PIN ERROR", strlen("M39 PIN ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		  }  
	  break;
			
	  case 40: //M40 P<pin> - Remove persistent digital pin 
	       pin_number = -1;
	       if (code_seen('P')) {
		    	pin_number = code_value();
				bool found = false;
				for(int i = 0; i < 32; i++) {
				    if(pdigitals[i] && pdigitals[i]->pin == pin_number) {
					    delete pdigitals[i];
						pdigitals[i] = 0;
						found = true;
					    break;
			       }
				}
				if(found) {
					tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
					tud_cdc_write("M40", strlen("M40"));
					tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
					tud_cdc_write_flush();
				} else {
					tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
					tud_cdc_write("M40 UNASSIGNED ERROR", strlen("M40 UNASSIGNED ERROR"));
					tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
					tud_cdc_write_flush();
				}
	  	  } else {// code P
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M40 PIN ERROR", strlen("M40 PIN ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		   }
	  break;
		
	  case 41: //M41 - Create persistent digital pin, Write digital pin HIGH P<pin> (this gives you a 5v source on pin)
	     pin_number = -1;
		 found = false;
	     if (code_seen('P')) {
		    pin_number = code_value();
			for(int i = 0; i < 32; i++) {
				if(!pdigitals[i]) {
					dpin = new Digital(pin_number);
					dpin->setPin(pin_number);
					dpin->pinMode(OUTPUT);
					dpin->digitalWrite(HIGH);
					pdigitals[i] = dpin;
					found = true
					break;
				}
			}
			if( found ) {
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M41", strlen("M41"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
				break;
			} else {
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M41 PIN ASSIGN ERROR", strlen("M41 PIN ASSIGN ERROR"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
				break;
			}
		} else {
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M41 PIN ERROR", strlen("M41 PIN ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
			break;
		}     
	break;
	     
    case 42: //M42 - Create persistent digital pin, Write digital pin LOW P<pin> (This gives you a grounded pin)
	  pin_number = -1;
	  if (code_seen('P')) {
        pin_number = code_value();
		if( assignPin(pin_number) ) {
			dpin = new Digital(pin_number);
			dpin->pinMode(OUTPUT);
			dpin->digitalWrite(LOW);
			for(int i = 0; i < 32; i++) {
				if(!pdigitals[i]) {
					pdigitals[i] = dpin;
					break;
				}
			}
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M42", strlen("M42"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		} else {
			for(int i = 0; i < 32; i++) {
				if(pdigitals[i] && pdigitals[i]->pin == pin_number) {
					pdigitals[i]->setPin(pin_number);
					pdigitals[i]->pinMode(OUTPUT);
					pdigitals[i]->digitalWrite(LOW);
					break;
				}
			}
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M42", strlen("M42"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		}
	  }
     break;
	 
	
	case 44: // M44 P<pin> [U] - -Read digital pin with optional pullup
        pin_number = -1;
        if (code_seen('P')) {
          pin_number = code_value();
		} else {
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M44 PIN ERROR", strlen("M44 PIN ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
			break;
		}
		dpin = 0;
		for(int i = 0; i < 32; i++) {
			if(pdigitals[i] && pdigitals[i]->pin == pin_number) {
				dpin = pdigitals[i];
				break;
			}
		}
		if(!dpin) {
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M44 PIN UNASSIGNED ERROR", strlen("M44 PIN UNASSIGNED ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
			break;
		}
		if(code_seen('U')) {
			dpin->pinMode(INPUT_PULLUP);
		}
		int res = dpin->digitalRead();
		tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
		tud_cdc_write(digitalPinHdr, strlen(digitalPinHdr));
		tud_cdc_write(MSG_DELIMIT,strlen(MSG_DELIMIT));
		tud_cdc_write("1 ", strlen("1 "));
		tud_cdc_write(itoa(pin_number), strlen(itoa(pin_number)));
		tud_cdc_write("2 ", strlen("2 "));
		tud_cdc_write(itoa(res), strlen(itoa(res)));
		tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
		tud_cdc_write(digitalPinHdr, strlen(digitalPinHdr));
		tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;
		
	 // PWM value between 0 and 1000
	 // Use M445 to disable pin permanently or use timer more 0 to stop pulse without removing pin assignment
     case 45: // M45 - set up PWM P<pin> S<power val 0-1000>
	  pin_number = -1;
	  if(code_seen('P') ) {
          pin_number = code_value();
	  } else {
		tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
		tud_cdc_write("M45 PIN ERROR",strlen("M45 PIN ERROR"));
		tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;
	  }
      if (code_seen('S')) {
        power = code_value();
		if(power < 0 || power > 1000) {
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M45 POWER ERROR",strlen("M45 POWER ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
			break;
		}
		// this is a semi-permanent pin assignment so dont add if its already assigned
		for(int i = 0; i < 12; i++) {
			if(!ppwms[i]) {
				ppin = new PWM(pin_number);
				ppin->init(pin_number);
				ppwms[i] = ppin;
				break;
			}
		}
		enable = false;
		for(int i = 0; i < 12; i++) {
			if(ppwms[i] && ppwms[i]->pin == pin_number) {
				enable = true;
				ppwms[i]->pwmWrite(true, power);
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M45", strlen("M45"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
				break;
			}
		}
		if(!enable) {
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M45 PIN ASSIGN ERROR",strlen("M45 PIN ASSIGN ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		}
	  break;
	  
	  case 46: // M46 -Read analog pin P<pin>
        pin_number = -1;
        if (code_seen('P')) {
          pin_number = code_value();
		  for(int i = 0; i < 16; i++) {
			if(panalogs[i] && panalogs[i]->pin == pin_number) {
				apin = panalogs[i];
				int res = apin->analogRead();
				res = apin->analogRead(); // de-jitter
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write(analogPinHdr, strlen(analogPinHdr));
				tud_cdc_write(MSG_DELIMIT,strlen(MSG_DELIMIT));
				tud_cdc_write("1 ", 2);
				tud_cdc_write(itoa(pin_number), strlen(itoa(pin_number)));
				tud_cdc_write("2 ", 2);
				tud_cdc_write(itoa(res), strlen(itoa(res)));
				tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
				tud_cdc_write(analogPinHdr, strlen(analogPinHdr));
				tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			}
		  }
		} else {
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M46 PIN ERROR",strlen("M46 PIN ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		}
     break;
	 
	 case 47: // M47 -Read analog pin P<pin> T<threshold> compare to battery threshold, if below, print battery message
	   pin_number = -1;
	   if (code_seen('P')) {
		   pin_number = code_value();
		   digitarg = code_seen('T') ? code_value() : 0;
		   	for(int i = 0; i < 16; i++) {
			if(panalogs[i] && panalogs[i]->pin == pin_number) {
				apin = panalogs[i];
				int res = apin->analogRead();
				res = apin->analogRead(); // de-jitter
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write(analogPinHdr, strlen(analogPinHdr));
				tud_cdc_write(MSG_DELIMIT,strlen(MSG_DELIMIT));
				tud_cdc_write("1 ", 2);
				tud_cdc_write(itoa(pin_number), strlen(itoa(pin_number)));
				tud_cdc_write("2 ", 2);
				tud_cdc_write(itoa(res), strlen(itoa(res)));
				tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
				tud_cdc_write(analogPinHdr, strlen(analogPinHdr));
				tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
			}
		}
		} else {
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M47 PIN ERROR",strlen("M47 PIN ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		}
	 break;
	 
     case 80: // M80 - Turn on Power Supply
	  #if defined(PS_ON_PIN) && PS_ON_PIN > -1
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, PS_ON_AWAKE);
        // If you have a switch on suicide pin, this is useful
        #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
            SET_OUTPUT(SUICIDE_PIN);
            WRITE(SUICIDE_PIN, HIGH);
        #endif
      #endif
	  	tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
	  	tud_cdc_write("M80", strlen("M80"));
	  	tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
	  	tud_cdc_write_flush();
	  break;

     case 81: // M81 [Z<slot>] X - Turn off Power Z shut down motorcontroller in slot, X shut down PWM, slot -1 do all
	  int scode;
	  if( code_seen('Z')) {
		scode = code_value();
		if(scode == -1) {
			if(code_seen('X')) {
				for(int k = 0; k < 10; k++) {
					if(pwmControl[k]) {
						if(pwmControl[k]->isConnected()) {
							pwmControl[k]->commandEmergencyStop(81);
						}
					}
				}
			} else {
				for(int k = 0; k < 10; k++) {
					if(motorControl[k]) {
						if(motorControl[k]->isConnected()) {
							motorControl[k]->commandEmergencyStop(81);
						}
					}
				}
			}
		} else {
			motorController = scode; // slot seen
			if(code_seen('X')) {
				if(pwmControl[motorController]) {
					if(pwmControl[motorController]->isConnected()) {
						pwmControl[motorController]->commandEmergencyStop(81);
					}
				}
			} else {			
				if(motorControl[motorController]) {
					if( motorControl[motorController]->isConnected()) {
						motorControl[motorController]->commandEmergencyStop(81);
					}
				}
			}
		}
	  }
      #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
        suicide();
      #elif defined(PS_ON_PIN) && PS_ON_PIN > -1
        SET_OUTPUT(PS_ON_PIN);
        WRITE(PS_ON_PIN, PS_ON_ASLEEP);
      #endif
	    sleep_ms(1000); // Wait 1 sec before switch off
		tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
		tud_cdc_write("M81", strlen("M81"));
		tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
	  break;

	  
    case 115: // M115
	  tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
      tud_cdc_write(MSG_M115_REPORT,strlen(MSG_M115_REPORT));
	  tud_cdc_write(MSG_DELIMIT,strlen(MSG_DELIMIT));
	  tud_cdc_write(MSG_115_REPORT2,strlen(MSG_115_REPORT2));
	  tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
	  tud_cdc_write(MSG_M115_REPORT, strlen(MSG_115_REPORT));
	  tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
	  tud_cdc_write_flush();
      break;
	
    case 300: // M300 - emit ultrasonic pulse on given pin and return duration P<pin number>
      uspin = code_seen('P') ? code_value() : 0;
      if (uspin > 0) {
		Ultrasonic* upin = new Ultrasonic(uspin);
		pin_number = upin->getPin();
		tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
		tud_cdc_write(sonicCntrlHdr,strlen(sonicCntrlHdr));
		tud_cdc_write(MSG_DELIMIT,strlen(MSG_DELIMIT));
		tud_cdc_write("1 ", 2); // pin
		tud_cdc_write(itoa(pin_number), strlen(itoa(pin_number)));
		tud_cdc_write("2 ", 2); // sequence
		tud_cdc_write(itoa(upin->getRange()), strlen(itoa(upin->getRange()))); // range
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(sonicCntrlHdr, strlen(sonicCntrlHdr));
		tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		delete upin;
      }
    break;
		
    case 301: // M301 P<pin> - attach ultrasonic device to pin
		// wont assign pin 0 as its sensitive
		uspin = code_seen('P') ? code_value() : 0;
		// this is a permanent pin assignment so dont add if its already assigned
		for(int i = 0; i < 10; i++) {
			if(!psonics[i]) {
				psonics[i] = new Ultrasonic(uspin);
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M301", strlen("M301"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
				break;
			}
		}
		// make sure its assigned
		assigned = false;
		for(int i = 0; i < 10; i++) {
			if(psonics[i] && psonics[i]->getPin() == uspin) {
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M301", strlen("M301"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
				assigned = true;
				break;
			}
		}
		if(!assigned) {
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M301 ERROR", strlen("M301 ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		}
		break;
	
	case 302: // M302 P<pin> - remove ultrasonic pin
		uspin = code_seen('P') ? code_value() : 0;
		for(int i = 0; i < 10; i++) {
				if(psonics[i] && psonics[i]->getPin() == uspin) {
					delete psonics[i];
					psonics[i] = 0;
					tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
					tud_cdc_write("M302", strlen("M302"));
					tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
					tud_cdc_write_flush();
					break;
				}
		}
      break;
	
	case 303: // M303 - Check the analog inputs for all pins defined by successive M304 directives. Generate a read and output data if in range.
		for(int i = 0 ; i < 16; i++) {
			if( panalogs[i] && panalogs[i]->mode == INPUT) {
				printAnalog(panalogs[i], i);
				tud_cdc_write_flush();
			}
		}
      break;
	  
	case 304:// M304 P<pin> [L<min>] [H<max>] [U] - toggle analog read optional INPUT_PULLUP with optional exclusion range 0-1024 via L<min> H<max>
		// if optional L and H values exclude readings in that range
		uspin = code_seen('P') ? code_value() : 0;
		// this is a permanent pin assignment so dont add if its already assigned
		for(int i = 0; i < 16; i++) {
			if(!panalogs[i]) {
				panalogs[i] = new Analog(uspin);
				if(code_seen('U'))  {
					panalogs[i]->pinMode(INPUT_PULLUP);
				}
				break;
			}
		}
		// make sure its assigned to update the range if needed
		assigned = false;
		for(int i = 0; i < 16; i++) {
			if(panalogs[i] && panalogs[i]->pin == uspin) {
				analogRanges[0][i] = code_seen('L') ? code_value() : 0;
				analogRanges[1][i] = code_seen('H') ? code_value() : 0;
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M304", strlen("M304"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
				assigned = true;
				break;
			}
		}
		if(!assigned) {
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M304 ERROR", strlen("M304 ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		}
      break;
	
	case 305: // M305 - Read the pins defined in M306 and output them if they are of the defined target value
		for(int i = 0 ; i < 32; i++) {
			if( pdigitals[i] && (pdigitals[i]->mode == INPUT || pdigitals[i]->mode == INPUT_PULLUP)) {
				printDigital(pdigitals[i], digitalTarget[i]);
				tud_cdc_write_flush();
			}
		}
      break;
	
	case 306://  M306 P<pin> T<target> [U] - toggle digital read, 0 or 1 for target value, default 0 optional INPUT_PULLUP 
		// Looks for target value, if so publish with <digitalpin> header and 1 - pin 2 - value
		uspin = code_seen('P') ? code_value() : 0;
		if(uspin == 0) {
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M306 PIN ERROR", strlen("M306 PIN ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
			break;
		}
		digitarg = code_seen('T') ? code_value() : 0;
		// this is a permanent pin assignment so dont add if its already assigned
		found = false;
		for(int i = 0; i < 32; i++) {
			if(!pdigitals[i] && pdigitals[i]->pin == uspin) {
				if(code_seen('U')) {
					pdigitals[i]->pinMode(INPUT_PULLUP);
				}
				digitalTarget[i] = digitarg;
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M306", strlen("M306"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
				found = true;
				break;
			}
		}
		if(!found) {
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write("M306 PIN UNASSIGNED ERROR", strlen("M306 PIN UNASSIGNED ERROR"));
			tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		}
		break;
		
	case 445: // M445 P<pin> - Turn off pulsed write pin - disable PWM
      if(code_seen('P')) {
        pin_number = code_value();
		for(int i = 0; i < 12; i++) {
			if(ppwms[i] && ppwms[i]->pin == pin_number) {
				ppwms[i]->pwmWrite(0,0); // default is 2, clear on match. to turn off, use 0 
				delete ppwms[i];
				ppwms[i] = 0;
				tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
				tud_cdc_write("M445", strlen("M445"));
				tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
				tud_cdc_write_flush();
				break;
			}
		}
	  }
	  break;
	  
    case 502: // M502 Revert to default settings
		tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
		tud_cdc_write("M502", strlen("M502"));
		tud_cdc_write(MSG_TERMINATE,strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
    break;
	
	case 700: // return stats
	  tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
	  tud_cdc_write(MSG_STATUS, strlen(MSG_STATUS));
	  tud_cdc_write(VERSION_STRING, strlen(VERSION_STRING));
	  tud_cdc_write(MSG_CONFIGURATION_VER, strlen(MSG_CONFIGURATION_VER));
	  tud_cdc_write(STRING_VERSION_CONFIG_H, strlen(STRING_VERSION_CONFIG_H));
	  tud_cdc_write(MSG_AUTHOR, strlen(MSG_AUTHOR));
	  tud_cdc_write(STRING_CONFIG_H_AUTHOR, strlen(STRING_CONFIG_H_AUTHOR));
	  tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
	  tud_cdc_write_flush();
	  break; 
	  
	case 701: // Report digital pins in use
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(digitalPinSettingHdr, strlen(digitalPinSettingHdr));
		tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
		for(int i = 0; i < 32; i++) {
			if( pdigitals[i] ) {
				tud_cdc_write(itoa(pdigitals[i]->pin), strlen(itoa(pdigitals[i]->pin)));
				switch(pdigitals[i]->mode) {
					case INPUT:
						tud_cdc_write(" INPUT", strlen(" INPUT"));
						break;
					case INPUT_PULLUP:
						tud_cdc_write(" INPUT_PULLUP", strlen(" INPUT_PULLUP"));
						break;
					case OUTPUT:
						tud_cdc_write(" OUTPUT", strlen(" OUTPUT"));
						break;
					default:
						tud_cdc_write(" ERROR - UNKNOWN", strlen(" ERROR - UNKNOWN"));
						break;
				}
			}
		}
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(digitalPinSettingHdr, strlen(digitalPinSettingHdr));
		tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;
		
	case 702: // Report analog pins in use
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(analogPinSettingHdr, strlen(analogPinSettingHdr));
		tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
		for(int i = 0; i < 16; i++) {
			if( panalogs[i]  ) {
				tud_cdc_write(itoa(panalogs[i]->pin), strlen(itoa(panalogs[i]->pin)));
				switch(panalogs[i]->mode) {
					case INPUT:
						tud_cdc_write(" INPUT", strlen(" INPUT"));
						break;
					case INPUT_PULLUP:
						tud_cdc_write(" INPUT_PULLUP", strlen(" INPUT_PULLUP"));
						break;
					case OUTPUT:
						tud_cdc_write(" OUTPUT", strlen(" OUTPUT"));
						break;
					default:
						tud_cdc_write(" ERROR - UNKNOWN", strlen(" ERROR - UNKNOWN"));
						break;
				}
			}
		}
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(analogPinSettingHdr, strlen(analogPinSettingHdr));
		tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;
		
	case 703: // Report ultrasonic pins in use
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(ultrasonicPinSettingHdr, strlen(ultrasonicPinSettingHdr));
		tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
		for(int i = 0; i < 10; i++) {
			if( psonics[i] ) {
				tud_cdc_write("Pin:", strlen("Pin:"));
				tud_cdc_write(itoa(psonics[i]->getPin()), strlen(itoa(psonics[i]->getPin())));
			}
		}
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(ultrasonicPinSettingHdr, strlen(ultrasonicPinSettingHdr));
		tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;
		
	case 704: // Report PWM pins in use
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(pwmPinSettingHdr, strlen(pwmPinSettingHdr));
		tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
		for(int i = 0; i < 12; i++) {
			if( ppwms[i] ) {
				tud_cdc_write("Pin:", strlen("Pin:"));
				tud_cdc_write(itoa(ppwms[i]->pin), strlen(itoa(ppwms[i]->pin)));
				tud_cdc_write(" Timer channel:", strlen(" Timer channel:"));
				tud_cdc_write(itoa(ppwms[i]->channel), strlen(itoa(ppwms[i]->channel)));
				tud_cdc_write();
			}
		}
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(pwmPinSettingHdr, strlen(pwmPinSettingHdr));
		tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;
		
	case 705:
			tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
			tud_cdc_write(motorControlSettingHdr, strlen(motorControlSettingHdr));
			tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
			for(int j = 0; j < 10; j++) {
				if( motorControl[j] ) {
					for(int i = 0 ; i < motorControl[j]->getChannels(); i++) { //per channel
						tud_cdc_write("Motor channel:", strlen("Motor channel:"));
						tud_cdc_write(itoa(i+1), strlen(itoa(i+1)));
						tud_cdc_write(" Min Power:", strlen(" Min Power:"));
						tud_cdc_write(itoa(motorControl[j]->getMinMotorPower(i+1)), strlen(itoa(motorControl[j]->getMinMotorPower(i+1))));
						tud_cdc_write(" Speed:", strlen(" Speed:"));
						tud_cdc_write(itoa(motorControl[j]->getMotorSpeed(i+1)), strlen(itoa(motorControl[j]->getMotorSpeed(i+1))));
						tud_cdc_write(" Curr. Dir:", strlen(" Curr. Dir:"));
						tud_cdc_write(itoa(motorControl[j]->getCurrentDirection(i+1)), strlen(itoa(motorControl[j]->getCurrentDirection(i+1))));
						tud_cdc_write(" Default. Dir:", strlen(" Default. Dir:"));
						tud_cdc_write(itoa(motorControl[j]->getDefaultDirection(i+1)), strlen(itoa(motorControl[j]->getDefaultDirection(i+1))));
						tud_cdc_write(" Encoder Pin:", strlen(" Encoder Pin:"));
						if(motorControl[j]->getWheelEncoder(i+1)) {
							tud_cdc_write(itoa(motorControl[j]->getWheelEncoder(i+1)->pin), strlen(itoa(motorControl[j]->getWheelEncoder(i+1)->pin)));
							tud_cdc_write(" Count:", strlen(" Count:"));
							tud_cdc_write(itoa(motorControl[j]->getEncoderCount(i+1)), strlen(itoa(motorControl[j]->getEncoderCount(i+1))));
							tud_cdc_write(" Duration:", strlen(" Duration:"));
							tud_cdc_write(itoa(motorControl[j]->getMaxMotorDuration(i+1)), strlen(itoa(motorControl[j]->getMaxMotorDuration(i+1))));
							//tud_cdc_write(motorControl[j]->getDriverInfo(i+1));
						} else {
							tud_cdc_write("None.", strlen("None."));
						}
					}
					tud_cdc_write("Ultrasonic pins:", strlen("Ultrasonic pins:"));
					if( motorControl[j]->totalUltrasonics() ) {
						tud_cdc_write(itoa(motorControl[j]->totalUltrasonics()),strlen(itoa(motorControl[j]->totalUltrasonics())));
						for(int k = 0; k < motorControl[j]->totalUltrasonics(); k++) {
							tud_cdc_write("Pin:", strlen("Pin:"));
							tud_cdc_write(itoa(psonics[motorControl[j]->getUltrasonicIndex(k+1)]->getPin()), strlen(itoa(psonics[motorControl[j]->getUltrasonicIndex(k+1)]->getPin())));
							tud_cdc_write(" Facing:", strlen(" Facing:"));
							tud_cdc_write(itoa(motorControl[j]->getUltrasonicFacing(k+1)), strlen(itoa(motorControl[j]->getUltrasonicFacing(k+1))));
							tud_cdc_write(" Shutdown cm:", strlen(" Shutdown cm:"));
							tud_cdc_write(itoa(motorControl[j]->getMinMotorDist(k+1)), strlen(itoa(motorControl[j]->getMinMotorDist(k+1))));
						}
					} else {
						tud_cdc_write("None.", strlen("None."));
					}
				} // if motorControl[j]
			} // j each motor controller
			tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
			tud_cdc_write(motorControlSettingHdr, strlen(motorControlSettingHdr));
			tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
			//
			// PWM control
			//
			tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
			tud_cdc_write(pwmControlSettingHdr, strlen(pwmControlSettingHdr));
			tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
			for(int j = 0; j < 10; j++) {
				if(pwmControl[j]) {
					for(int i = 0 ; i < pwmControl[j]->getChannels(); i++) { //per channel
						tud_cdc_write("PWM channel:", strlen("PWM channel:"));
						tud_cdc_write(itoa(i+1), strlen(itoa(i+1)));
						tud_cdc_write(" Min Level:", strlen(" Min Level:"));
						tud_cdc_write(itoa(pwmControl[j]->getMinPWMLevel(i+1)), strlen(itoa(pwmControl[j]->getMinPWMLevel(i+1))));
						tud_cdc_write(" Duration:", strlen(" Duration:"));
						tud_cdc_write(itoa(pwmControl[j]->getMaxPWMDuration(i+1)), strlen(itoa(pwmControl[j]->getMaxPWMDuration(i+1))));
					}
				}
			} // j each PWM controller
			tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
			tud_cdc_write(pwmControlSettingHdr, strlen(pwmControlSettingHdr));
			tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
			break;
			
	case 706: // Report all pins in use
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(pinSettingHdr, strlen(pinSettingHdr));
		tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
		for(int i = 0; i < 100; i++) {
			if( pinAssignment(i) == PIN_ASSIGNED ) {
				tud_cdc_write(itoa(i), strlen(itoa(i)));
				tud_cdc_write(",", 1);
			}
		}
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(pinSettingHdr, strlen(pinSettingHdr));
		tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;
		
	case 798: // M798 Z<motor control> [X] Report controller status for given controller. If X, slot is PWM
		char* buf;
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(controllerStatusHdr, strlen(controllerStatusHdr));
		tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
		if (code_seen('Z')) {
			motorController = code_value();
		}		
		if(code_seen('X')) {
				if(pwmControl[motorController]) {
					for(int i = 0; i < pwmControl[motorController]->getChannels() ; i++ ) {
						tud_cdc_write("PWM Channel:", strlen("PWM Channel:"));
						tud_cdc_write(itoa(i+1), strlen(itoa(i+1)));
						pwmControl[motorController]->getDriverInfo(i+1,outbuffer);
						tud_cdc_write(outbuffer,strlen(outbuffer));
						tud_cdc_write_flush();
					}
				}
		} else {
			if( motorControl[motorController] && motorControl[motorController]->isConnected() ) {
				for(int i = 0; i < motorControl[motorController]->getChannels() ; i++ ) {
					tud_cdc_write("Motor Channel:", strlen("Motor Channel:"));
					tud_cdc_write(itoa(i+1), strlen(itoa(i+1)));
					motorControl[motorController]->getDriverInfo(i+1, outbuffer);
					tud_cdc_write(outbuffer, strlen(outbuffer));
					tud_cdc_write_flush();
				}
			}
		} // code_seen('X')
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(controllerStatusHdr, strlen(controllerStatusHdr));
		tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;	
		
	case 799: // M799 [Z<controller>][X] Reset controller, if no argument, reset all. If X, slot is PWM
		if (code_seen('Z')) {
			motorController = code_value();
			if(code_seen('X')) {
				if(pwmControl[motorController]) {
					pwmControl[motorController]->commandEmergencyStop(799);
					tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
					tud_cdc_write("M799", strlen("M799"));
					tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
					tud_cdc_write_flush();
				}
			} else {
				if(motorControl[motorController]) {
					motorControl[motorController]->commandEmergencyStop(799);
					tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
					tud_cdc_write("M799", strlen("M799"));
					tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
					tud_cdc_write_flush();
				}
			}
		} else { // no slot defined, do them all if present
			for(int j = 0;j < 10; j++) {
				if(code_seen('X')) {
					if(pwmControl[j]) {
						pwmControl[j]->commandEmergencyStop(-1);
					}
				} else {
					if(motorControl[j]) {
						motorControl[j]->commandEmergencyStop(-1);
					}
				}
			}
			tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
			tud_cdc_write("M799", strlen("M799"));
			tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
			tud_cdc_write_flush();
		}
		break;		
		
			
	case 802: // Acquire analog pin data M802 Pnn Sxxx Mxxx P=Pin number, S=number readings, M=microseconds per reading. X - pullup.
		// Publish <dataset> 1 - pin, 2 - reading
		if( code_seen('P')) {
			apin = new Analog((uint8_t)code_value());
			if( code_seen('X') ) {
				apin->pinMode(INPUT_PULLUP);
			} else {
				apin->pinMode(INPUT);
			}
		}
		nread = 0;
		if( code_seen('S') ) {
			nread = code_value();
		}
		micros = 0;
		if( code_seen('M')) {
			micros = (uint32_t)code_value();
		}
		values = new int(nread);
		for(int i = 0; i < nread; i++) {
			*(values+i) = apin->analogRead();
			for(int j = 0; j < micros; j++) sleep_us(1);
		}
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(analogPinHdr, strlen(analogPinHdr));
		tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
		for(int i = 0; i < nread; i++) {
			tud_cdc_write(itoa(i+1), strlen(itoa(i+1))); // sequence
			tud_cdc_write(" ", 1);
			// 0 element is pin number
			if( i == 0 ) {
				tud_cdc_write(itoa(apin->pin), strlen(itoa(apin->pin)));
			} else {
				tud_cdc_write(itoa(*(values+i)), strlen(itoa(*(values+i)))); // value
			}
		}
		delete values;
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(analogPinHdr, strlen(analogPinHdr));
		tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;	
		
		
    case 999: // M999: Reset
		Stopped = false;
		//lcd_reset_alert_level();
		gcode_LastN = Stopped_gcode_LastN;
		//FlushSerialRequestResend();
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write("M999", strlen("M999"));
		tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;
		
	default:
		int ibuf = 0;
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(MSG_UNKNOWN_MCODE, strlen(MSG_UNKNOWN_MCODE));
		tud_cdc_write(cmdbuffer, strlen(cmdbuffer));
		tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
		tud_cdc_write_flush();
		break;
	
  } // switch m code

} //processMCode

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  tud_cdc_write_flush();
  tud_cdc_write(MSG_RESEND, strlen(MSG_RESEND));
  tud_cdc_write(itoa(gcode_LastN + 1), strlen(itoa(gcode_LastN + 1)));
  tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
  tud_cdc_write_flush();
}


/*---------------------------------------------------
* Arrive here at the end of each command processing iteration to check for status related events
* ---------------------------------------------------
*/
void manage_inactivity() {
  // check motor controllers
  for(int j =0; j < 10; j++) {
	  if(motorControl[j]) {
		if( motorControl[j]->isConnected() ) {
			motorControl[j]->checkEncoderShutdown();
			motorControl[j]->checkUltrasonicShutdown();
			if( motorControl[j]->queryFaultFlag() != fault ) {
				fault = motorControl[j]->queryFaultFlag();
				publishMotorFaultCode(fault);
				tud_cdc_write_flush();
			}
		}
	  }
  }
  
  if( realtime_output ) {		
	// Check the ultrasonic ranging for all devices defined by successive M301 directives
	for(int i = 0 ; i < 10; i++) {
		if( psonics[i] ) {
			printUltrasonic(psonics[i], i);
			tud_cdc_write_flush();
		}
	}  
  }// realtime output
}

void kill() {
#if defined(PS_ON_PIN) && PS_ON_PIN > -1
  Digital psoPin = new Digital(PS_ON_PIN);
  psoPin.pinMode(INPUT);
#endif
  for(int j = 0; j < 10; j++) {
	motorControl[j]->commandEmergencyStop(-2);
  }
  //SERIAL_ERROR_START;
  //SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  suicide();
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

void Stop() {
  if(!Stopped) {
    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
	for(int i = 0; i < 10; i++)
		if(motorControl[i])
			motorControl[i]->commandEmergencyStop(-3);
    //SERIAL_ERROR_START;
    //SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
  }
}
/*
* The fault code is bit-ordered for 8 cases
*/
void publishMotorFaultCode(int fault) {
	uint8_t bfault = 0;
	uint8_t j = 1;
	tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
	tud_cdc_write(motorFaultCntrlHdr, strlen(motorFaultCntrlHdr));
	tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
	for(int i = 0; i < 8 ; i++) {
		bfault = fault & (1<<i);
		switch(bfault) {
			default:
			case 0: // bit not set
				break;
			case 1:
				tud_cdc_write(itoa(j++), strlen(itoa(j)));
				tud_cdc_write(" ", 1);
				tud_cdc_write(MSG_MOTORCONTROL_1, strlen(MSG_MOTORCONTROL_1));
				break;
			case 2:
				tud_cdc_write(itoa(j++), strlen(itoa(j)));
				tud_cdc_write(" ", 1);
				tud_cdc_write(MSG_MOTORCONTROL_2, strlen(MSG_MOTORCONTROL_2));
				break;
			case 4:
				tud_cdc_write(itoa(j++), strlen(itoa(j)));
				tud_cdc_write(" ", 1);
				tud_cdc_write(MSG_MOTORCONTROL_3, strlen(MSG_MOTORCONTROL_3));
				break;
			case 8:
				tud_cdc_write(itoa(j++), strlen(itoa(j)));
				tud_cdc_write(" ", 1);
				tud_cdc_write(MSG_MOTORCONTROL_4, strlen(MSG_MOTORCONTROL_4));
				break;
			case 16:
				tud_cdc_write(itoa(j++), strlen(itoa(j)));
				tud_cdc_write(" ", 1);
				tud_cdc_write(MSG_MOTORCONTROL_5, strlen(MSG_MOTORCONTROL_5));
				break;
			case 32:
				tud_cdc_write(itoa(j++), strlen(itoa(j)));
				tud_cdc_write(" ", 1);
				tud_cdc_write(MSG_MOTORCONTROL_6, strlen(MSG_MOTORCONTROL_6));
				break;
			case 64:
				tud_cdc_write(itoa(j++), strlen(itoa(j)));
				tud_cdc_write(" ", 1);
				tud_cdc_write(MSG_MOTORCONTROL_7, strlen(MSG_MOTORCONTROL_7));
				break;
			case 128:
				tud_cdc_write(itoa(j++), strlen(itoa(j)));
				tud_cdc_write(" ", 1);
				tud_cdc_write(MSG_MOTORCONTROL_8, strlen(MSG_MOTORCONTROL_8));
				break;
		}
	}
	tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
	tud_cdc_write(motorFaultCntrlHdr, strlen(motorFaultCntrlHdr));
	tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
}
/*
* Deliver the battery voltage from smart controller
*/
void publishBatteryVolts(int volts) {
	tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
	tud_cdc_write(batteryCntrlHdr, strlen(batteryCntrlHdr));
	tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
	tud_cdc_write("1 ", 2);
	tud_cdc_write(itoa(volts*10), strlen(itoa(volts*10)));
	tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
	tud_cdc_write(batteryCntrlHdr, strlen(batteryCntrlHdr));
	tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
}
/************************************************************************/
/* only call this if we know code is stall                              */
/************************************************************************/
void publishMotorStatCode(int stat) {
	tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
	tud_cdc_write(motorFaultCntrlHdr,strlen(motorFaultCntrlHdr));
	tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
	tud_cdc_write("1 ", 2);
	tud_cdc_write(MSG_MOTORCONTROL_9, strlen(MSG_MOTORCONTROL_9));
	tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
	tud_cdc_write(motorFaultCntrlHdr, strlen(motorFaultCntrlHdr));
	tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
}

/*
* Print the ultrasonic range
*/
void printUltrasonic(Ultrasonic* us, int index) {
		float range = us->getRange();
		uint8_t ultpin = us->getPin();
		if( range != sonicDist[index] ) {
			sonicDist[index] = range;
			tud_cdc_write(MSG_BEGIN,strlen(MSG_BEGIN));
			tud_cdc_write(sonicCntrlHdr, strlen(sonicCntrlHdr));
			tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
			tud_cdc_write("1 ", 2); // pin
			tud_cdc_write(itoa(ultpin), strlen(itoa(ultpin)));
			tud_cdc_write("2 ", 2); // sequence
			tud_cdc_write(itoa(range*10), strlen(itoa(range*10))); // range
			tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
			tud_cdc_write(sonicCntrlHdr, strlen(sonicCntrlHdr));
			tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
		}
}
void checkSmartController(void)
{
}
/*
 * If we have values in analogRanges for this pin, check the reading and if it is between these ranges
 * reject the reading. This allows us to define a center or rest point for a joystick etc.
 * If no values were specified on the M code invocation, ignore and process regardless of value.
 */
void printAnalog(Analog* apin, int index) {
	//pin = new Analog(upin);
	int nread = apin->analogRead();
	// jitter comp.
	nread = apin->analogRead();
	if( analogRanges[0][index] != 0 && nread >= analogRanges[0][index] && nread <= analogRanges[1][index])
		return;
	tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
	tud_cdc_write(analogPinHdr, strlen(analogPinHdr));
	tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
	tud_cdc_write("1", 1); // sequence
	tud_cdc_write(" ", 1);
	// 0 element is pin number
	tud_cdc_write(itoa(apin->pin), strlen(itoa(apin->pin)));
	tud_cdc_write("2", 1); // sequence
	tud_cdc_write(" ", 1);
	tud_cdc_write(itoa(nread*10), strlen(itoa(nread*10))); // value
	tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
	tud_cdc_write(analogPinHdr, strlen(analogPinHdr));
	tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
	//delete pin;
}
/*
* 'target' represents the expected value. Two elements returned in sequence. 1 - Pin, 2 - reading
*/
void printDigital(Digital* dpin, int target) {
	//dpin = new Digital(upin);
	//dpin->pinMode(INPUT);
	int nread = dpin->digitalRead();
	//delete dpin;
	// look for activated value
	if( !(nread ^ target) ) {
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(digitalPinHdr, strlen(digitalPinHdr));
		tud_cdc_write(MSG_DELIMIT, strlen(MSG_DELIMIT));
		tud_cdc_write("1", 1); // sequence
		tud_cdc_write(" ", 1);
		tud_cdc_write(itoa(dpin->pin), strlen(itoa(dpin->pin)));
		tud_cdc_write("2", 1); // sequence
		tud_cdc_write(" ", 1);
		tud_cdc_write(itoa(nread), strlen(itoa(nread)));
		tud_cdc_write(MSG_BEGIN, strlen(MSG_BEGIN));
		tud_cdc_write(digitalPinHdr, strlen(digitalPinHdr));
		tud_cdc_write(MSG_TERMINATE, strlen(MSG_TERMINATE));
	}
}
