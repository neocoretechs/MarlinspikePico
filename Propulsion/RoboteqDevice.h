#ifndef ROBOTEQ_H
#define ROBOTEQ_H

#include "HardwareSerial.h"
#include "AbstractMotorControl.h"

#define ROBOTEQ_DEFAULT_TIMEOUT     1000
#define ROBOTEQ_BUFFER_SIZE         64
#define ROBOTEQ_COMMAND_BUFFER_SIZE 20

#define ROBOTEQ_QUERY_CHAR          0x05
#define ROBOTEQ_ACK_CHAR            0x06

#define ROBOTEQ_OK                  0
#define ROBOTEQ_TIMEOUT             -1
#define ROBOTEQ_ERROR               -2
#define ROBOTEQ_BAD_COMMAND         -3
#define ROBOTEQ_BAD_RESPONSE        -4
#define ROBOTEQ_BUFFER_OVER         -5

#define ROBOTEQ_FAULT_OVERHEAT      0x01
#define ROBOTEQ_FAULT_OVERVOLTAGE   0x02
#define ROBOTEQ_FAULT_UNDERVOLTAGE  0x04
#define ROBOTEQ_FAULT_SHORT         0x08
#define ROBOTEQ_FAULT_ESTOP         0x10
#define ROBOTEQ_FAULT_SCRIPT        0x20
#define ROBOTEQ_FAULT_MOSFET        0x40
#define ROBOTEQ_FAULT_CONFIG        0x80

#define ROBOTEQ_STATUS_SERIAL       0x01
#define ROBOTEQ_STATUS_PULSE        0x02
#define ROBOTEQ_STATUS_ANALOG       0x04
#define ROBOTEQ_STATUS_POWER_OFF    0x08
#define ROBOTEQ_STATUS_STALL        0x10
#define ROBOTEQ_STATUS_LIMIT        0x20
#define ROBOTEQ_SCRIPT_RUN          0x80
/*
* Uses HardwareSerial on the designated UART, presumably with an RS232 converter, to communicate with RobotEQ controllers.
* Tested on VBL2360 but should be universal.
*/
class RoboteqDevice : public AbstractMotorControl {
	private:
		char command[ROBOTEQ_COMMAND_BUFFER_SIZE];
		uint8_t buffer[ROBOTEQ_BUFFER_SIZE];
    // Constructors
    public:
	    RoboteqDevice(int maxPower) : AbstractMotorControl(maxPower) {
				m_Timeout = ROBOTEQ_DEFAULT_TIMEOUT;
				setChannels(2);
				m_Serial = new HardwareSerial(uart1, 4, 5); // Example pin numbers, replace with actual pins
	            m_Serial->begin(115200);
		}
        /*
         * check if controller is connected
         *
         * @return ROBOTEQ_OK if connected
         */
        int isConnected(void);

        //*********************************************************************
        // Commands
        //*********************************************************************
        /*
         * send motor power command (!G)
         *
         * @param ch channel
         * @param p power level (-1000, 1000)
         * @return ROBOTEQ_OK if successful 
         */
        int commandMotorPower(uint8_t ch, int16_t p) override;
        /*
         * send emergency stop command (!EX)
         * note: you have to reset the controller after this sending command
         *
         * @return ROBOTEQ_OK if successful
         */
        int commandEmergencyStop(int status) override;	
		int commandReset(void);
		int commandBrushlessCounter(void);
        void setMinMotorPower(uint8_t ch, int mpow) override;
        int checkSafeShutdown(uint slice) override { return 0;}
        void setSafeShutdown(volatile uint32_t* active_mask_buffer) override {return;}
        //*********************************************************************
        // Query
        //*********************************************************************
        /*
         * query controller firmware
         */
        int queryFirmware(char *buf, size_t bufSize);
        /*
         * query motor amps
         * 
         * @param ch channel
         * @return motor amps * 10
         */
        int queryMotorAmps(uint8_t ch);
        /*
         * query battery amps
         * @return battery amps * 10
         */
        int queryBatteryAmps(void);
        /*
         * query battery amps
         * @param ch channel
         * @return battery amps * 10
         */
        int queryBatteryAmps(uint8_t ch);
        /*
         * query battery voltage
         * @return battery voltage * 10
         */ 
        int queryBatteryVoltage(void);
        /*
         * query motor voltage
         * @return motor voltage * 10
         */
        int queryMotorVoltage(void);
        /*
         * query internal temp 
         * @return temp (in degrees C)
         */
        int queryInternalTemp(void);
        /*
         * query channel temp 
         * @param ch channel
         * @return temp (in degrees C)
         */
        int queryTemp(uint8_t ch);
        /*
         * query the motor power command
         * @param ch channel
         * @return motor power
         */
        int queryMotorPower(uint8_t ch);
        int queryFaultFlag(void) override;
        int queryStatusFlag(void) override;
        /*
         * query encoder speed in RPM
         * @param ch channel
         * @return rpm
         */
        int queryEncoderSpeed(uint8_t ch);
        /*
         * query encoder speed in RPM
         * @param ch channel
         * @return rpm
         */
        int queryEncoderRelativeSpeed(uint8_t ch);
		int queryBrushlessCounterRelative(uint8_t ch);
		int queryBrushlessSpeed(uint8_t ch);
		int queryBrushlessSpeedRelative(uint8_t ch);
		int queryTime();
        /*
         * Query user variable
         * @param var
         * @param value
         * @return ROBOTEQ_OK if successful
         */
        int queryUserVariable(uint32_t var, int32_t *value);
        /*
         * Query user variable
         * @param var
         * @param value
         * @return ROBOTEQ_OK if successful
         */
        int queryUserVariable(uint32_t var, bool *value);

        //*********************************************************************
        // Configuration
        //*********************************************************************
        /*
         * set encoder pulse per rotation
         * @param ch channel
         * @param ppr pulese per rotation
         * @return ROBOTEQ_OK if successful
         */
        int setEncoderPulsePerRotation(uint8_t ch, uint16_t ppr);
        int getEncoderPulsePerRotation(uint8_t ch);
        /*
         * set motor amp limit
         * @param ch channel
         * @param a amps level (x10)
         * @return ROBOTEQ_OK if successful
         */
        int setMotorAmpLimit(uint8_t ch, uint16_t a);
        int getMotorAmpLimit(uint8_t ch);
        /*
         * load controller configuration
         * @return ROBOTEQ_OK if successful
         */
        int loadConfiguration(void);
        /*
         * save controller configuration
         * @return ROBOTEQ_OK if successful
         */
        int saveConfiguration(void);
        void setTimeout(uint16_t timeout);		
		void getDriverInfo(uint8_t ch, char* outStr) override;
        int queryBrushlessCounter(uint8_t ch) override;
    // Private Methods
    private:
        int sendQuery(const char *query, uint8_t *buf, size_t bufSize);
        int sendQuery(const char *query, size_t querySize, uint8_t *buf, size_t bufSize);
        int sendCommand(const char *command);
        int sendCommand(const char *command, size_t commandSize);
        int readResponse(uint8_t *buf, size_t bufSize);
    // Private Data
    private:
        uint16_t    m_Timeout;
        HardwareSerial      *m_Serial;
};

// Helper Functions
char* chomp(char* s);

 //extern RoboteqDevice roboteqDevice;
#endif
