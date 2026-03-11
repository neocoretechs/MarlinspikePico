#ifndef LANGUAGE_H
#define LANGUAGE_H

// Languages
// 1  English

#ifndef LANGUAGE_CHOICE
	#define LANGUAGE_CHOICE 1  // Pick your language from the list above
#endif

#define PROTOCOL_VERSION "1.1"

#define MACHINE_NAME "RoboCore"
#define FIRMWARE_URL "http://www.neocoretechs.com"

#ifndef MACHINE_UUID
   #define MACHINE_UUID "08d85ec1-651a-4a2f-a1a7-4afb154bcc81"
#endif

#define VERSION_STRING  "1.3.0"

#define STRINGIFY_(n) #n
#define STRINGIFY(n) STRINGIFY_(n)

	// Messages outputting data
	#define errormagic "Error:"
	#define echomagic "echo:"
	#define datasetHdr "dataset"
	#define motorCntrlHdr "motorcontrol"
	#define sonicCntrlHdr "ultrasonic"
	#define timeCntrlHdr  "time"
	#define posCntrlHdr  "position"
	#define motorFaultCntrlHdr "motorfault"
	#define PWMFaultCntrlHdr "pwmfault"
	#define batteryCntrlHdr "battery"
	#define digitalPinHdr "digitalpin"
	#define analogPinHdr "analogpin"
	#define digitalPinSettingHdr "digitalpinsetting"
	#define analogPinSettingHdr "analogpinsetting"
	#define ultrasonicPinSettingHdr "ultrasonicpinsetting"
	#define pwmPinSettingHdr "pwmpinsetting"
	#define motorControlSettingHdr "motorcontrolsetting"
	#define pwmControlSettingHdr "pwmcontrolsetting"
	#define pinSettingHdr "assignedpins"
	#define controllerStatusHdr "controllerstatus"
	#define eepromHdr "eeprom"
		
	// Message delimiters, quasi XML
	#define MSG_BEGIN "<"
	#define MSG_DELIMIT ">"
	#define MSG_END "</"
	#define MSG_TERMINATE "/>"

	// Serial Console informational Messages
	#define MSG_STATUS "status"
	#define MSG_POWERUP  "PowerUp"
	#define MSG_EXTERNAL_RESET "External Reset"
	#define MSG_BROWNOUT_RESET "Brown out Reset"
	#define MSG_WATCHDOG_RESET "Watchdog Reset"
	#define MSG_SOFTWARE_RESET "Software Reset"
	#define MSG_AUTHOR  " | Author: "
	#define MSG_CONFIGURATION_VER " Last Updated: "
	#define MSG_FREE_MEMORY " Free Memory: "
	#define MSG_ERR_LINE_NO "Line Number is not Last Line Number+1, Last Line: "
	#define MSG_ERR_CHECKSUM_MISMATCH "checksum mismatch, Last Line: "
	#define MSG_ERR_NO_CHECKSUM "No Checksum with line number, Last Line: "
	#define MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM "No Line Number with checksum, Last Line: "
	#define MSG_M115_REPORT "FIRMWARE_NAME:Marlinspike RoboCore"
	#define MSG_115_REPORT2 "FIRMWARE_URL:" FIRMWARE_URL "\r\nPROTOCOL_VERSION:" PROTOCOL_VERSION "\r\nMACHINE_TYPE:" MACHINE_NAME "\r\nUUID:" MACHINE_UUID
	#define MSG_ERR_KILLED "Controller halted. kill() called!"
	#define MSG_ERR_STOPPED "Controller stopped due to errors"
	#define MSG_RESEND "Resend: "
	#define MSG_UNKNOWN_COMMAND "Neither G nor M code found "
	#define MSG_UNKNOWN_GCODE "Unknown G code "
	#define MSG_UNKNOWN_MCODE "Unknown M code "
	#define MSG_BAD_MOTOR "Bad Motor command "
	#define MSG_BAD_PWM "Bad PWM Driver command "
	
	// These correspond to the controller faults return by 'queryFaultCode'
	#define MSG_MOTORCONTROL_1 "Overheat"
	#define MSG_MOTORCONTROL_2 "Overvoltage"
	#define MSG_MOTORCONTROL_3 "Undervoltage"
	#define MSG_MOTORCONTROL_4 "Short circuit"
	#define MSG_MOTORCONTROL_5 "Emergency stop"
	#define MSG_MOTORCONTROL_6 "Sepex excitation fault"
	#define MSG_MOTORCONTROL_7 "MOSFET failure"
	#define MSG_MOTORCONTROL_8 "Startup configuration fault"
	#define MSG_MOTORCONTROL_9 "Stall"

#endif // ifndef LANGUAGE_H
