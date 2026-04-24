#include "pico/stdlib.h"
#include "RoboteqDevice.h"
#include "../HardwareSerial.h"


char* chomp(char* s) {
	int end = strlen(s) - 1;
	if (end >= 0 && (s[end] == '\n' || s[end] == '\r'))
	s[end] = '\0';
	return s;
}


void RoboteqDevice::setTimeout(uint16_t timeout) {
	m_Timeout = timeout;
}

int RoboteqDevice::isConnected() {
	if (this->m_Serial == NULL)
		return 0;
	m_Serial->write(ROBOTEQ_QUERY_CHAR);
	m_Serial->flush();
	uint8_t inByte;
	for(int i = 0; i < 10; i++) {
		if (m_Serial->available() > 0) {
			while(m_Serial->available() > 0) {
				inByte = m_Serial->read();
				if (inByte == ROBOTEQ_ACK_CHAR) {
						return 1;
				}
			}
		}
		sleep_ms(10);
	}
	// timeout
	return 0;
}
/*
* Command the motor to spin. May reset current direction. Encoders reset regardless if present. Checks for ultrasonic shutdown if present.
* ch - channel. array max is controller dependent per channel power level -1000 to 10000
*/
int RoboteqDevice::commandMotorPower(int16_t p[10]) {
	// check shutdown override
	if( MOTORSHUTDOWN || checkUltrasonicShutdown() )
			return ROBOTEQ_OK;
	for(int ch = 1; ch <= getChannels(); ch++) {
		int motorPower = p[ch-1];
		if( motorPower < 0 )  // and we want to go backward
			currentDirection[ch-1] = 0; // set new direction value
		else // dir is 1 forward
			currentDirection[ch-1] = 1;		
		// New command resets encoder count
		resetEncoders();
		// if power 0, we are stopping anyway
		// If this wheel is mirrored, invert power
		if( defaultDirection[ch-1] )
			motorPower = -motorPower;
		if( motorPower != 0 && abs(motorPower) < motorSpeed[ch-1]) {
			if(motorPower > 0 )
				motorPower = minMotorPower[ch-1];
			else
				motorPower = -minMotorPower[ch-1];
		}
		if( abs(motorPower) > MAXMOTORPOWER ) { // cap it at max
			if(motorPower > 0)
				motorPower = MAXMOTORPOWER;
			else
				motorPower = -MAXMOTORPOWER;
		}
		// Scale motor power if necessary and save it in channel speed array with proper sign for later use
		if( MOTORPOWERSCALE != 0 ) {
			motorPower /= MOTORPOWERSCALE;
		}
		motorSpeed[ch-1] = motorPower;
		last_command_time[ch-1] = (motorPower == 0 ? 0 : time_us_64());
		sprintf(command, "!G %02d %d\r", ch, motorPower);
		fault_flag |= this->sendCommand(command);
	}
	return fault_flag;
}
/*
* Institute an all-channels stop
*/
int RoboteqDevice::commandEmergencyStop(int status)
{
	//sprintf(command, "!EX\r");
	for(int ch = 0; ch < getChannels(); ch++) {
		sprintf(command, "!G %02d %d\r", ch+1, 0);
		this->sendCommand(command);
	}
	fault_flag = 16;
	resetSpeeds();
	resetEncoders();
	return status;
}

int RoboteqDevice::commandReset(void) {
	sprintf(command, "%RESET 321654987\r");
	return this->sendCommand(command);
}

int RoboteqDevice::commandBrushlessCounter(void) {
	sprintf(command, "^BLFB 0\r");
	return this->sendCommand(command);
}
/*
*f1 = overheat
*f2 = overvoltage
*f3 = undervoltage
*f4 = short circuit
*f5 = emergency stop
*f6 = Sepex excitation fault
*f7 = MOSFET failure
*f8 = startup configuration fault
*/
int RoboteqDevice::queryFaultFlag() {
	// Query: ?FF
	// Response: FF=<status>
	int fault = -1;
	memset(buffer, 0, ROBOTEQ_BUFFER_SIZE);
	int res = 0;
	if ((res = this->sendQuery("?FF\r", (uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE)) < 0)
		return res;
	if (res < 4)
		return ROBOTEQ_BAD_RESPONSE;
	// Parse Response
	if (sscanf((char*)buffer, "FF=%i", &fault) < 1)
		return ROBOTEQ_BAD_RESPONSE;
	return fault | fault_flag;
}
/*
*f1 = Serial mode
*f2 = Pulse mode
*f3 = Analog mode
*f4 = Power stage off
*f5 = Stall detected
*f6 = At limit
*f7 = Unused
*f8 = MicroBasic script running
*/
int RoboteqDevice::queryStatusFlag() {
	// Query: ?FS
	// Response: FS=<status>
	int status = -1;
	memset(buffer, 0, ROBOTEQ_BUFFER_SIZE);
	int res;
	if ((res = this->sendQuery("?FS\r", (uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE)) < 0)
	return res;
	if (res < 4)
	return ROBOTEQ_BAD_RESPONSE;
	// Parse Response
	if (sscanf((char*)buffer, "FS=%i", &status) < 1)
	return ROBOTEQ_BAD_RESPONSE;
	return status;
}

int RoboteqDevice::queryFirmware(char* buf, size_t bufSize) {
	// Query: ?FID
	// Response: FID=<firmware>
	memset(buf, 0, bufSize);
	return this->sendQuery("?FID\r", (uint8_t*)buf, 100);
	// TODO: Parse response
}

int RoboteqDevice::queryMotorPower(uint8_t ch) {
	// Query: ?M [ch]
	// Response: M=<motor power>
	int p;
	int res;

	// Build Query Command
	sprintf(command, "?M %i\r", ch);

	// Send Query
	if ((res = this->sendQuery("?BA\r", (uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE)) < 0)
	return res;
	if (res < 4)
	return ROBOTEQ_BAD_RESPONSE;

	// Parse Response
	if (sscanf((char*)buffer, "M=%i", &p) < 1) {
		return ROBOTEQ_BAD_RESPONSE;
	}
	return p;
}
/*
* total amperage at both channels totaled
*/
int RoboteqDevice::queryBatteryAmps(void) {
	// Query: ?BA
	// Response: BA=<ch1*10>:<ch2*10>
	int ch1, ch2;
	int res;

	// Send Query
	if ((res = this->sendQuery("?BA\r", (uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE)) < 0)
	return res;
	if (res < 4)
	return ROBOTEQ_BAD_RESPONSE;

	// Parse Response
	if (sscanf((char*)buffer, "BA=%i:%i", &ch1, &ch2) < 2) {
		return ROBOTEQ_BAD_RESPONSE;
	}

	// Return total amps (ch1 + ch2)
	return ch1+ch2;
}

int RoboteqDevice::queryBatteryAmps(uint8_t ch) {
	// Query: ?BA [ch]
	// Response: BA=<ch*10>
	int amps;
	int res;

	// Build Query Command
	sprintf(command, "?BA %i\r", ch);

	// Send Query
	if ((res = this->sendQuery(command, (uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE)) < 0)
	return res;
	if (res < 4)
	return ROBOTEQ_BAD_RESPONSE;

	// Parse Response
	if (sscanf((char*)buffer, "BA=%i", &amps) < 1) {
		return ROBOTEQ_BAD_RESPONSE;
	}

	return amps;
}

int RoboteqDevice::queryBatteryVoltage(void) {
	// Query: ?V 2 (2 = main battery voltage)
	// Response: V=<voltage>*10
	int voltage = -1;
	memset(buffer, 0, ROBOTEQ_BUFFER_SIZE);
	int res;
	if ((res = this->sendQuery("?V 2\r", (uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE)) < 0)
	return res;
	if (res < 4)
	return ROBOTEQ_BAD_RESPONSE;
	// Parse Response
	if (sscanf((char*)buffer, "V=%i", &voltage) != 1)
	return ROBOTEQ_BAD_RESPONSE;
	return voltage;
}

int RoboteqDevice::queryMotorVoltage(void) {
	// Query: ?V 1 (1 = main motor voltage)
	// Response: V=<voltage>*10
	int voltage = -1;
	memset(buffer, 0, ROBOTEQ_BUFFER_SIZE);
	int res;
	if ((res = this->sendQuery("?V 1\r", (uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE)) < 0)
	return res;
	if (res < 4)
	return ROBOTEQ_BAD_RESPONSE;
	// Parse Response
	if (sscanf((char*)buffer, "V=%i", &voltage) != 1)
	return ROBOTEQ_BAD_RESPONSE;
	return voltage;
}
/*
* Encoder speed in RPM
* To report RPM accurately, the correct Pulses per Revolution (PPR) must be stored in the encoder configuration.
*/
int RoboteqDevice::queryEncoderSpeed(uint8_t ch){
	// Query: ?S [ch]
	// Response: S=[speed]
	int speed;
	int res;

	// Build Query Command
	sprintf(command, "?S %i\r", ch);

	// Send Query
	if ((res = this->sendQuery(command, (uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE)) < 0)
	return res;
	if (res < 3)
	return ROBOTEQ_BAD_RESPONSE;

	// Parse Response
	if (sscanf((char*)buffer, "S=%i", &speed) < 1) {
		return ROBOTEQ_BAD_RESPONSE;
	}
	return speed;
}
/*
* Returns the measured motor speed as a ratio of the Max RPM configuration parameter 
*/
int RoboteqDevice::queryEncoderRelativeSpeed(uint8_t ch) {
	// Query: ?SR [ch]
	// Response: SR=[speed]
	int speed;
	int res;

	// Build Query Command
	sprintf(command, "?SR %i\r", ch);

	// Send Query
	if ((res = this->sendQuery(command, (uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE)) < 0)
	return res;
	if (res < 3)
	return ROBOTEQ_BAD_RESPONSE;

	// Parse Response
	if (sscanf((char*)buffer, "SR=%i", &speed) < 1) {
		return ROBOTEQ_BAD_RESPONSE;
	}
	return speed;
}
/*
* On brushless motor controllers, returns the running total of Hall sensor transition value as
* an absolute number. The counter is a 32-bit counter with a range of +/- 2000000000
* counts.
*/
int RoboteqDevice::queryBrushlessCounter(uint8_t ch) {
	// Query: ?CB [ch]
	// Response: CB=[speed]
	int speed;
	int res;

	// Build Query Command
	sprintf(command, "?C %i\r", ch);
	
	// Send Query
	if ((res = this->sendQuery(command, (uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE)) < 0)
	return res;
	if (res < 3)
	return ROBOTEQ_BAD_RESPONSE;

	// Parse Response
	if (sscanf((char*)buffer, "C=%i", &speed) < 1) {
		return ROBOTEQ_BAD_RESPONSE;
	}
	return speed;
}
/*
* On brushless motor controllers, returns the number of Hall sensor transition value that
* have been measured from the last time this query was made.
*/
int RoboteqDevice::queryBrushlessCounterRelative(uint8_t ch) {
	// Query: ?CBR [ch]
	// Response: CBR=[speed]
	int speed;
	int res;

	// Build Query Command
	sprintf(command, "?CR %i\r", ch);

	// Send Query
	if ((res = this->sendQuery(command, (uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE)) < 0)
	return res;
	if (res < 3)
	return ROBOTEQ_BAD_RESPONSE;

	// Parse Response
	if (sscanf((char*)buffer, "CR=%i", &speed) < 1) {
		return ROBOTEQ_BAD_RESPONSE;
	}
	return speed;
}
/*
* To report RPM accurately, the correct number of motor poles must be
* loaded in the BLPOL configuration parameter.
*/
int RoboteqDevice::queryBrushlessSpeed(uint8_t ch) {
	// Query: ?BS [ch]
	// Response: BS=[speed]
	int speed;
	int res;

	// Build Query Command
	sprintf(command, "?BS %i\r", ch);

	// Send Query
	if ((res = this->sendQuery(command, (uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE)) < 0)
	return res;
	if (res < 3)
	return ROBOTEQ_BAD_RESPONSE;

	// Parse Response
	if (sscanf((char*)buffer, "BS=%i", &speed) < 1) {
		return ROBOTEQ_BAD_RESPONSE;
	}
	return speed;
}
/*
* On brushless motor controllers, returns the measured motor speed as a ratio of the Max RPM configuration parameter
*/
int RoboteqDevice::queryBrushlessSpeedRelative(uint8_t ch) {
	// Query: ?BSR [ch]
	// Response: BSR=[speed]
	int speed;
	int res;

	// Build Query Command
	sprintf(command, "?BSR %i\r", ch);

	// Send Query
	if ((res = this->sendQuery(command, (uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE)) < 0)
	return res;
	if (res < 3)
	return ROBOTEQ_BAD_RESPONSE;

	// Parse Response
	if (sscanf((char*)buffer, "BSR=%i", &speed) < 1) {
		return ROBOTEQ_BAD_RESPONSE;
	}
	return speed;
}

/*
* On RTC units, 32 bit seconds
*/
int RoboteqDevice::queryTime() {
	// Query: ?TM
	// Response: TM=[time]
	int speed;
	int res;

	// Build Query Command
	sprintf(command, "?TM\r");

	// Send Query
	if ((res = this->sendQuery(command, (uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE)) < 0)
	return res;
	if (res < 3)
	return ROBOTEQ_BAD_RESPONSE;

	// Parse Response
	if (sscanf((char*)buffer, "TM=%i", &speed) < 1) {
		return ROBOTEQ_BAD_RESPONSE;
	}
	return speed;
}

int RoboteqDevice::setEncoderPulsePerRotation(uint8_t ch, uint16_t ppr) {
	sprintf(command, "^EPPR %02d %d\r", ch, ppr);
	return this->sendCommand(command);
}

int RoboteqDevice::getEncoderPulsePerRotation(uint8_t ch) {
	// TODO: not implmented
	return ROBOTEQ_OK;
}

int RoboteqDevice::setMotorAmpLimit(uint8_t ch, uint16_t a){
	sprintf(command, "^ALIM %i %i", ch, a);
	return this->sendCommand(command);
}

int RoboteqDevice::getMotorAmpLimit(uint8_t ch){
	// TODO: not implmented
	return ROBOTEQ_OK;
}

int RoboteqDevice::loadConfiguration(void) {
	sprintf(command, "%%EELD\r");
	return this->sendCommand(command);
}

int RoboteqDevice::saveConfiguration(void) {
	sprintf(command, "%%EESAV\r");
	return this->sendCommand(command);
}

int RoboteqDevice::sendCommand(const char *command) {
	return this->sendCommand(command, strlen(command));
}

int RoboteqDevice::sendCommand(const char *command, size_t commandSize) {
	if (this->m_Serial == NULL)
		return ROBOTEQ_ERROR;

	if(commandSize <= 0 || commandSize >= ROBOTEQ_COMMAND_BUFFER_SIZE)
	{
		return ROBOTEQ_BAD_COMMAND;
	}
		
	for(int i = 0 ; i < commandSize; i++) {
		this->m_Serial->write(*(command+i));
		this->m_Serial->flush();
		sleep_ms(1);
		this->m_Serial->read(); // absorb echo
	}
	//this->m_Serial->flush();
	
	int res = this->readResponse((uint8_t*)buffer, ROBOTEQ_BUFFER_SIZE);

	if (res < 1)
		return ROBOTEQ_BAD_RESPONSE;

	// Check Command Status
	if (strncmp((char*)buffer, "+", 1) == 0) {
		return ROBOTEQ_OK;
	} else {
		return ROBOTEQ_BAD_COMMAND;
	}
}

int RoboteqDevice::sendQuery(const char *query, uint8_t *buf, size_t bufSize) {
	return this->sendQuery(query, strlen(query), buf, bufSize);
}

int RoboteqDevice::sendQuery(const char *query, size_t querySize, uint8_t *buf, size_t bufSize) {
	if (this->m_Serial == NULL)
	return ROBOTEQ_ERROR;

	// Write Query to Serial
	//this->m_Serial->write((uint8_t*)query, querySize);
	for(int i = 0 ; i < querySize; i++) {
		this->m_Serial->write(*(query+i));
		this->m_Serial->flush();
		sleep_ms(1);
		this->m_Serial->read(); // absorb echo
	}
	//this->m_Serial->flush();

	// Read Serial until timeout or newline
	return (this->readResponse(buf, bufSize));
}
/*
* read the expected response; iterate 10 times, waiting 100 ms per, on the available read buffer > 0
*/
int RoboteqDevice::readResponse(uint8_t *buf, size_t bufferSize) {
	uint8_t inByte;
	size_t index = 0;

	for(int i = 0; i < 10; i++) {
		if (m_Serial->available() > 0) {
			while(m_Serial->available() > 0) {
				inByte = m_Serial->read();
				buf[index++] = inByte;
				if (index > bufferSize) {
					// out of buffer space
					return ROBOTEQ_BUFFER_OVER;
				}
				if (inByte == 0x0D) {
					return index;
				}
		
			}
		}
		sleep_ms(10);
	}
	// timeout
	return ROBOTEQ_TIMEOUT;
}

void RoboteqDevice::setMinMotorPower(uint8_t ch, int mpow) {}

void RoboteqDevice::getDriverInfo(uint8_t ch, char* outStr) {
	char cout[80];
	char dout1[10];
	char dout2[10];
	char dout3[10];
	char dout4[10];
	if( !isConnected() ) {
		itoa(ch, dout1, 10);
		sprintf(cout, "Controller channel %s is not connected.\0", dout1);
	} else {
		itoa(queryBatteryVoltage(), dout1, 10);
		itoa(queryBatteryAmps(), dout2, 10);
		itoa(queryFaultFlag(), dout3, 10);
		itoa(queryStatusFlag(), dout4, 10);
		sprintf(cout,"Voltage:%s Amps:%s Fault:%s Status:%s\0",dout1 , dout2, dout3, dout4);
	}
	 for(int i=0; i < 80; ++i){
		 outStr[i] = cout[i];
		 if(!outStr[i])
			break;
	 }
}
