#ifndef PINS_H
#define PINS_H
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include "arduino-compat.h"
#include "DigitalPin.h"

/****************************************************************************************
* Pico pin assignment
*
****************************************************************************************/
#define KNOWN_BOARD 1

#define LARGE_FLASH true

#define I2C_SCL       16
#define I2C_SDA       17

// M240  Triggers a camera by emulating a Canon RC-1 Remote
// Data from: http://www.doc-diy.net/photo/rc-1_hacked/
#define PHOTOGRAPH_PIN     -1

#define SUICIDE_PIN        -1  //PIN that has to be turned on right after start, to keep power flowing.
#define SAFETY_TRIGGERED_PIN     -1 //PIN to detect the safety circuit has triggered
#define MAIN_VOLTAGE_MEASURE_PIN -1 //Analogue PIN to measure the main voltage, with a 100k - 4k7 resistor divider.

#define DIGIPOTSS_PIN -1
#define DIGIPOT_CHANNELS {4,5,3,0,1} // X Y Z E0 E1 digipot channels to stepper driver mapping

#define LED_PIN            -1
#define FAN_PIN            -1
#define PS_ON_PIN          -1
#define KILL_PIN           -1
#define CONTROLLERFAN_PIN  -1 //Pin used for the fan to cool controller

#define PIN_RESERVED		1 // type of pin for user assignment description
#define PIN_ASSIGNED		2

#define SENSITIVE_PINS {0, 1, PS_ON_PIN, FAN_PIN }
bool assignPin(uint tpin);
int pinAssignment(uint tpin);
bool unassignPin(uint tpin);
#endif

