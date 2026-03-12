#include "HardwareSerial.h"

HardwareSerial Serial1(uart1, 4, 5);  // Roboteq UART

HardwareSerial::HardwareSerial(uart_inst_t* uart, uint tx_pin, uint rx_pin)
    : _uart(uart), _tx_pin(tx_pin), _rx_pin(rx_pin) {}

void HardwareSerial::begin(unsigned long baud) {
    uart_init(_uart, baud);

    gpio_set_function(_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(_rx_pin, GPIO_FUNC_UART);

    uart_set_format(_uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(_uart, true);

    if (_uart == uart1) {
        irq_set_exclusive_handler(UART1_IRQ, [](){ Serial1._rx_irq(); });
        irq_set_enabled(UART1_IRQ, true);
        uart_set_irq_enables(_uart, true, false);
    }
}

void HardwareSerial::begin(unsigned long baud, uint8_t config) {
    begin(baud); // ignore config for now
}

void HardwareSerial::end() {
    uart_deinit(_uart);
}

int HardwareSerial::available() {
    return (_rx_head != _rx_tail);
}

int HardwareSerial::peek() {
    if (_rx_head == _rx_tail) return -1;
    return _rxbuf[_rx_tail];
}

int HardwareSerial::read() {
    if (_rx_head == _rx_tail) return -1;
    uint8_t c = _rxbuf[_rx_tail];
    _rx_tail = (_rx_tail + 1) % RX_BUF_SIZE;
    return c;
}

int HardwareSerial::availableForWrite() {
    return 1; // always writable
}

void HardwareSerial::flush() {
    uart_tx_wait_blocking(_uart);
}

size_t HardwareSerial::write(uint8_t c) {
    uart_putc_raw(_uart, c);
    return 1;
}

void HardwareSerial::_rx_irq() {
    while (uart_is_readable(_uart)) {
        uint8_t c = uart_getc(_uart);
        uint16_t next = (_rx_head + 1) % RX_BUF_SIZE;
        if (next != _rx_tail) {
            _rxbuf[_rx_head] = c;
            _rx_head = next;
        }
    }
}