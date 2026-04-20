// No-op Wire (I2C) stub for native tests.
#pragma once
#include "Arduino.h"

class TwoWire {
public:
    void begin() {}
    void begin(uint8_t addr) {}
    void beginTransmission(uint8_t addr) {}
    uint8_t endTransmission() { return 0; }
    uint8_t requestFrom(uint8_t addr, uint8_t qty) { return 0; }
    int available() { return 0; }
    int read() { return -1; }
    size_t write(uint8_t val) { return 1; }
};
extern TwoWire Wire;
