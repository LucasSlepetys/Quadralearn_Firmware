#ifndef TEST_ARDUINO_H
#define TEST_ARDUINO_H
#include <stdint.h>
#include <stddef.h>
enum { INPUT = 1, ADC_11db = 3 };
void pinMode(int gpio, int mode);
void analogReadResolution(int bits);
void analogSetPinAttenuation(int gpio, int attenuation);
uint32_t analogReadMilliVolts(int gpio);
void delay(unsigned long ms);
unsigned long millis(void);
struct TestSerial {
    size_t setTxBufferSize(size_t bytes) { return bytes; } //Host replies already use an in-memory buffer.
    void begin(unsigned long baud);
    void printf(const char *format, ...);
    void println(const char *text);
    int available(void);
    int read(void);
};
extern TestSerial Serial; //Fake console and ADC only; no PWM functions exist here.
#endif
