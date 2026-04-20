#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// This configuration file contains the basic settings.
// Advanced settings can be found in Configuration_adv.h

// User-specified version info of this build to display in response to M code
// inform user that any changes made to this
// build by the user have been successfully uploaded into firmware.
#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__ // build date and time
#define STRING_CONFIG_H_AUTHOR "(Groff, default config)" // Who made the changes.

// SERIAL_PORT selects which serial port should be used for communication with the host.
// This allows the connection of wireless adapters (for instance) to non-default port pins.
// Serial port 0 is still used as default.
#define SERIAL_PORT 0

// This determines the communication speed of the board low
#define BAUDRATELOW 19200
// This determines the communication speed of the board hi
#define BAUDRATE 115200


// Define this to set a unique identifier for this board, (Used by some programs to differentiate between machines)
// You can use an online service to generate a random UUID. (eg http://www.uuidgenerator.net/version4)
// #define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

//I2C PANELS


#include "Configuration_adv.h"

#endif //__CONFIGURATION_H
