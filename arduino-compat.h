#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
// Pin modes
#define INPUT 0
#define OUTPUT 1

inline void pinMode(uint pin, int mode) {
    gpio_init(pin);
    gpio_set_dir(pin, mode == OUTPUT);
}

inline void digitalWrite(uint pin, bool val) {
    gpio_put(pin, val);
}

inline int digitalRead(uint pin) {
    return gpio_get(pin);
}

// Timing
inline uint32_t millis() {
    return to_ms_since_boot(get_absolute_time());
}

inline void delay(uint32_t ms) {
    sleep_ms(ms);
}

// Simple Serial shim (USB stdio by default)
struct SerialShim {
    void begin(uint32_t baud) {
        stdio_init_all();
        // If you want hardware UART instead of USB:
        // uart_init(uart0, baud);
        // gpio_set_function(0, GPIO_FUNC_UART);
        // gpio_set_function(1, GPIO_FUNC_UART);
    }
    int available() {
        return (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
    }
    int read() {
        return getchar_timeout_us(0);
    }
    void print(const char* s) {
        printf("%s", s);
    }
    void println(const char* s) {
        printf("%s\n", s);
    }
    void println(int v) {
        printf("%d\n", v);
    }
};

static SerialShim Serial;