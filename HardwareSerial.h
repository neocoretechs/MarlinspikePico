#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "Stream.h"

class HardwareSerial : public Stream {
public:
    HardwareSerial(uart_inst_t* uart, uint tx_pin, uint rx_pin);

    void begin(unsigned long baud);
    void begin(unsigned long baud, uint8_t config);

    void end();

    int available(void) override;
    int peek(void) override;
    int read(void) override;

    int availableForWrite(void);

    void flush(void) override;

    size_t write(uint8_t c) override;
    using Print::write;

    operator bool() { return true; }

private:
    uart_inst_t* _uart;
    uint _tx_pin;
    uint _rx_pin;

    static const int RX_BUF_SIZE = 256;
    uint8_t _rxbuf[RX_BUF_SIZE];
    volatile uint16_t _rx_head = 0;
    volatile uint16_t _rx_tail = 0;

    void _rx_irq();
};

extern HardwareSerial Serial1;

#endif