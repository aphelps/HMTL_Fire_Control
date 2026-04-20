// No-op LiquidCrystal stub for native tests.
#pragma once
#include "Arduino.h"

class LiquidCrystal {
public:
    LiquidCrystal(uint8_t addr) {}
    void begin(uint8_t cols, uint8_t rows) {}
    void clear() {}
    void setCursor(uint8_t col, uint8_t row) {}
    void setBacklight(uint8_t val) {}
    template<typename T> void print(T) {}
    template<typename T> void print(T, int) {}
};
