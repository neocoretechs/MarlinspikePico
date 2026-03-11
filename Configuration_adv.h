#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H

/*
MS1	MS2	MS3	Microstep Resolution
Low	Low	Low		Full step
High Low Low	Half step
Low	High Low	Quarter step
High High Low	Eighth step
High High High	Sixteenth step
// MS1 MS2 MS3 Stepper Driver Microstepping mode table
#define MICROSTEP1 LOW,LOW,LOW
#define MICROSTEP2 HIGH,LOW,LOW
#define MICROSTEP4 LOW,HIGH,LOW
#define MICROSTEP8 HIGH,HIGH,LOW
#define MICROSTEP16 HIGH,HIGH,HIGH
// Microstep setting (Only functional when stepper driver microstep pins are connected to MCU.
#define MICROSTEP_MODES {16,16,16,16,16} // [1,2,4,8,16]
*/

// Motor Current setting (Only functional when motor driver current ref pins are connected to a digital trimpot on supported boards)
/*#define DIGIPOT_MOTOR_CURRENT {135,135,135,135,135} // Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)
// uncomment to enable an I2C based DIGIPOT like on the Azteeg X3 Pro
//#define DIGIPOT_I2C
// Number of channels available for I2C digipot, For Azteeg X3 Pro we have 8
#define DIGIPOT_I2C_NUM_CHANNELS 8
// actual motor currents in Amps, need as many here as DIGIPOT_I2C_NUM_CHANNELS
#define DIGIPOT_I2C_MOTOR_CURRENTS {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}
*/
//===========================================================================
//=============================Additional Features===========================
//===========================================================================
/*
#define SD_FINISHED_STEPPERRELEASE true  //if sd support and the file is finished: disable steppers?
#define SD_FINISHED_RELEASECOMMAND "M84 X Y Z E" // You might want to keep the z enabled so your bed stays in place.
#define SDCARD_RATHERRECENTFIRST  //reverse file order of sd card menu display. Its sorted practically after the file system block order.
*/
// if a file is deleted, it frees a block. hence, the order is not purely chronological. To still have auto0.g accessible, there is again the option to do that.
// using:
//#define MENU_ADDAUTOSTART

// The hardware watchdog should reset the microcontroller disabling all outputs, in case the firmware gets stuck and doesn't do temperature regulation.
//#define USE_WATCHDOG

#ifdef USE_WATCHDOG
// If you have a watchdog reboot in an ArduinoMega2560 then the device will hang forever, as a watchdog reset will leave the watchdog on.
// The "WATCHDOG_RESET_MANUAL" goes around this by not using the hardware reset.
//  However, THIS FEATURE IS UNSAFE!, as it will only work if interrupts are disabled. And the code could hang in an interrupt routine with interrupts disabled.
//#define WATCHDOG_RESET_MANUAL
#endif




//===========================================================================
//=============================Buffers           ============================
//===========================================================================

// The number of linear motions that can be in the plan at any give time.
// THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts and ors are used to do the ring-buffering.
#define BLOCK_BUFFER_SIZE 16 // maximize block buffer

// The ASCII buffer for receiving from the serial port:
#define OUT_BUFFER_SIZE 256
// The ASCII buffer for command line processing:
#define MAX_CMD_SIZE 256

#define BUFSIZE 4
//===========================================================================


#endif //__CONFIGURATION_ADV_H
