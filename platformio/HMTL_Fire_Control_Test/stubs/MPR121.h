// Controllable MPR121 stub for native tests.
// Tests inject touch/change events directly; no I2C hardware required.
#pragma once
#include "Arduino.h"

#define MAX_MPR121_PINS 12
#define START_ADDRESS   0x5A

class MPR121 {
public:
    static const int MAX_SENSORS = 12;

    MPR121() { _clearAll(); }

    // --- API called by firmware ---

    bool readTouchInputs() { return _any_change; }

    bool touched(uint8_t i) {
        return (i < MAX_MPR121_PINS) && _touched[i];
    }

    bool changed(uint8_t i) {
        return (i < MAX_MPR121_PINS) && _changed[i];
    }

    void setThresholds(byte touch, byte release) {}
    void setThreshold(uint8_t pin, byte touch, byte release) {}

    void init(byte irqpin, boolean interrupt, byte address,
              boolean times, boolean filtered, boolean autoEn) {}

    // --- Test control API ---

    // Simulate touching (or releasing) electrode i.
    void _setTouched(uint8_t i, bool val) {
        if (i >= MAX_MPR121_PINS) return;
        _changed[i] = (_touched[i] != val);
        _touched[i] = val;
        _any_change  = true;
    }

    // Simulate a readTouchInputs() that returned nothing new.
    void _setNoChange() { _any_change = false; }

    // Clear all state — call from setUp() to start each test clean.
    void _clearAll() {
        for (int i = 0; i < MAX_MPR121_PINS; i++) {
            _touched[i] = false;
            _changed[i] = false;
        }
        _any_change = false;
    }

private:
    bool _touched[MAX_MPR121_PINS];
    bool _changed[MAX_MPR121_PINS];
    bool _any_change;
};
