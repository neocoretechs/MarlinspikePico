#include "Wire.h"

Wire::Wire(i2c_inst_t* bus, uint sda, uint scl, uint speed) : _bus(bus) {
    i2c_init(_bus, speed);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

void Wire::write(uint8_t addr, const uint8_t* data, size_t len) {
    i2c_write_blocking(_bus, addr, data, len, false);
}

void Wire::read(uint8_t addr, uint8_t* data, size_t len) {
    i2c_read_blocking(_bus, addr, data, len, false);
}

void Wire::writeRead(uint8_t addr, const uint8_t* wbuf, size_t wlen,
                      uint8_t* rbuf, size_t rlen) {
    write(addr, wbuf, wlen);
    read(addr, rbuf, rlen);
}
 /*************************************************************************
  * Write a byte to register at address
  *************************************************************************/
  void Wire::write8(uint8_t address, uint8_t reg, uint8_t value)
  {
    uint8_t buf[2] = { reg, value };
	  i2c_write_blocking(_bus, address, buf, 2, false);
  }

  /**************************************************************************
  * Read a byte from register at address
  *************************************************************************/
  uint8_t Wire::read8(uint8_t address, uint8_t reg)
  {
    i2c_write_blocking(_bus, address, &reg, 1, true); // repeated start
    uint8_t value;
    i2c_read_blocking(_bus, address, &value, 1, false);
    return value;
  }
  /*************************************************************************
    Reads a 16 bit value over I2C
  **************************************************************************/
  uint16_t Wire::read16(uint8_t address, uint8_t reg) {
    uint8_t buf[2] = { reg, 1 };
	  i2c_write_blocking(_bus, address, buf, 2, true);
    i2c_read_blocking(_bus, address, buf, 2, false);
    return (uint16_t(buf[0]) << 8) | buf[1];
}