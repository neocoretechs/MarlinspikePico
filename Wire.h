#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
class Wire {
public:
    Wire(i2c_inst_t* bus, uint sda, uint scl, uint speed = 400000);

    void write(uint8_t addr, const uint8_t* data, size_t len);
    void read(uint8_t addr, uint8_t* data, size_t len);
    void writeRead(uint8_t addr, const uint8_t* wbuf, size_t wlen,
                   uint8_t* rbuf, size_t rlen);
    void write8(uint8_t address, uint8_t reg, uint8_t value);
    uint8_t read8(uint8_t address, uint8_t reg);
    uint16_t read16(uint8_t address, uint8_t reg);

private:
    i2c_inst_t* _bus;
};