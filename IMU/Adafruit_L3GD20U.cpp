/***************************************************
  This is a library for the L3GD20 GYROSCOPE

  Designed specifically to work with the Adafruit L3GD20 Breakout 
  ----> https://www.adafruit.com/products/1032

  These sensors use I2C or SPI to communicate, 2 pins (I2C) 
  or 4 pins (SPI) are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Kevin "KTOWN" Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#include "pico/stdlib.h"
#include <limits.h>
#include "Adafruit_L3GD20U.h"

 
/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool Adafruit_L3GD20_Unified::begin(gyroRange_t rng)
{
  bool idd = false;

  /* Set the range the an appropriate value */  
  _range = rng;

  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
  L3GD20_ADDRESS = L3GD20_ADDRESS_SA0_LOW;   
  L3GD20_ID = wire->read8(L3GD20_ADDRESS, GYRO_REGISTER_WHO_AM_I); 
  if (L3GD20_ID == 0xD4 || L3GD20_ID == 0xD7) {
	idd = true;
  } else {
	L3GD20_ADDRESS = L3GD20_ADDRESS_SA0_HIGH;
	L3GD20_ID = wire->read8(L3GD20_ADDRESS, GYRO_REGISTER_WHO_AM_I); 
	if (L3GD20_ID == 0xD4 || L3GD20_ID == 0xD7) idd = true;
  }


  /* Set CTRL_REG1 (0x20)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   7-6  DR1/0     Output data rate                                   00
   5-4  BW1/0     Bandwidth selection                                00
     3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
     2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
     1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
     0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

  /* Reset then switch to normal mode and enable all three channels */
  //write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x00);
  wire->write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1,  0b00001111);

  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG2 (0x21)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   5-4  HPM1/0    High-pass filter mode selection                    00
   3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG3 (0x22)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
     6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
     5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
     4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
     3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
     2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
     1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
     0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */
  //write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG3, 0x00);
  //write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG3, 0x10000000); // enable interrupts data ready
  /* Set CTRL_REG4 (0x23)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
     6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
   5-4  FS1/0     Full scale selection                               00
                                  00 = 250 dps
                                  01 = 500 dps
                                  10 = 2000 dps
                                  11 = 2000 dps
     0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

  /* Adjust resolution if requested */
  switch(_range)
  {
    case GYRO_RANGE_250DPS:
      wire->write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG4, 0x00);
      break;
    case GYRO_RANGE_500DPS:
      wire->write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG4, 0x10);
      break;
    case GYRO_RANGE_2000DPS:
      wire->write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG4, 0x20);
      break;
  }
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG5 (0x24)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
     6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
     4  HPen      High-pass filter enable (0=disable,1=enable)        0
   3-2  INT1_SEL  INT1 Selection config                              00
   1-0  OUT_SEL   Out selection config                               00 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */
  //Enable Interrupt request for X, y, z, high
  // write8(L3GD20_ADDRESS, GYRO_REGISTER_INT1_CFG, 0b00101010);

  //Set X, Y, Z interrupt threshold to 1. 
  //write8(L3GD20_ADDRESS, GYRO_REGISTER_TSH_XH, 0x01);
  //write8(L3GD20_ADDRESS, GYRO_REGISTER_TSH_YH, 0x01);
  //write8(L3GD20_ADDRESS, GYRO_REGISTER_TSH_ZH, 0x01);
  return idd;
}

/**************************************************************************/
/*! 
    @brief  Enables or disables auto-ranging
*/
/**************************************************************************/
void Adafruit_L3GD20_Unified::enableAutoRange(bool enabled)
{
  _autoRangeEnabled = enabled;
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
void Adafruit_L3GD20_Unified::getEvent(sensors_event_t* event)
{
  bool readingValid = false;
  
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_GYROSCOPE;
  
  while(!readingValid)
  {
    event->timestamp = 0;//millis();
 
    uint8_t xlo = wire->read8(L3GD20_ADDRESS, GYRO_REGISTER_OUT_X_L);
    uint8_t xhi = wire->read8(L3GD20_ADDRESS, GYRO_REGISTER_OUT_X_H);
    uint8_t ylo = wire->read8(L3GD20_ADDRESS, GYRO_REGISTER_OUT_Y_L);
    uint8_t yhi = wire->read8(L3GD20_ADDRESS, GYRO_REGISTER_OUT_Y_H);
    uint8_t zlo = wire->read8(L3GD20_ADDRESS, GYRO_REGISTER_OUT_Z_L);
    uint8_t zhi = wire->read8(L3GD20_ADDRESS, GYRO_REGISTER_OUT_Z_H);
	
	
    /* Shift values to create properly formed integer (low byte first) */
    event->gyro.x = (int16_t)(xlo | (xhi << 8));
    event->gyro.y = (int16_t)(ylo | (yhi << 8));
    event->gyro.z = (int16_t)(zlo | (zhi << 8));
    
    /* Make sure the sensor isn't saturating if auto-ranging is enabled */
    if (!_autoRangeEnabled)
    {
      readingValid = true;
    }
    else
    {
      /* Check if the sensor is saturating or not */
      if ( (event->gyro.x >= 32760) | (event->gyro.x <= -32760) | 
           (event->gyro.y >= 32760) | (event->gyro.y <= -32760) | 
           (event->gyro.z >= 32760) | (event->gyro.z <= -32760) )
      {
        /* Saturating .... increase the range if we can */
        switch(_range)
        {
          case GYRO_RANGE_500DPS:
            /* Push the range up to 2000dps */
            _range = GYRO_RANGE_2000DPS;
            wire->write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x00);
            wire->write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x0F);
            wire->write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG4, 0x20);
            wire->write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG5, 0x80);
            readingValid = false;
            // Serial.println("Changing range to 2000DPS");
            break;
          case GYRO_RANGE_250DPS:
            /* Push the range up to 500dps */
            _range = GYRO_RANGE_500DPS;
            wire->write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x00);
            wire->write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x0F);
            wire->write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG4, 0x10);
            wire->write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG5, 0x80);
            readingValid = false;
            // Serial.println("Changing range to 500DPS");
            break;
          default:
            readingValid = true;
            break;  
        }
      }
      else
      {
        /* All values are withing range */
        readingValid = true;
      }
    }
  }
  
  /* Compensate values depending on the resolution */
  switch(_range)
  {
    case GYRO_RANGE_250DPS:
      event->gyro.x *= GYRO_SENSITIVITY_250DPS;
      event->gyro.y *= GYRO_SENSITIVITY_250DPS;
      event->gyro.z *= GYRO_SENSITIVITY_250DPS;
      break;
    case GYRO_RANGE_500DPS:
      event->gyro.x *= GYRO_SENSITIVITY_500DPS;
      event->gyro.y *= GYRO_SENSITIVITY_500DPS;
      event->gyro.z *= GYRO_SENSITIVITY_500DPS;
      break;
    case GYRO_RANGE_2000DPS:
      event->gyro.x *= GYRO_SENSITIVITY_2000DPS;
      event->gyro.y *= GYRO_SENSITIVITY_2000DPS;
      event->gyro.z *= GYRO_SENSITIVITY_2000DPS;
      break;
  }
  
  /* Convert values to rad/s */
  event->gyro.x *= SENSORS_DPS_TO_RADS;
  event->gyro.y *= SENSORS_DPS_TO_RADS;
  event->gyro.z *= SENSORS_DPS_TO_RADS;
}

/**************************************************************************/
/*! 
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void  Adafruit_L3GD20_Unified::getSensor(sensor_t* sensor)
{  
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "L3GD20", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay   = 0;
  sensor->max_value   = (float)this->_range * SENSORS_DPS_TO_RADS;
  sensor->min_value   = (this->_range * -1.0) * SENSORS_DPS_TO_RADS;
  sensor->resolution  = 0.0F; // TBD
}