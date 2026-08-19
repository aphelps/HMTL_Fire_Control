// Scriptable Wire (I2C) mock for native tests.
//
// Models one slave as a 256-byte register file with failure injection, enough
// to test register-level drivers: a write transaction of [reg, val] stores
// val; a write of [reg] alone sets the read pointer; requestFrom serves the
// pointed-at register. wire_mock.end_rc / fail_request inject bus failures.
#pragma once
#include "Arduino.h"
#include <string.h>

struct WireMockState {
    uint8_t  regs[256];
    uint8_t  last_addr;      // slave address of the last transaction
    uint8_t  reg_ptr;        // register pointer (first byte of the last write)
    uint8_t  end_rc;         // endTransmission() return; 0 = ack
    bool     fail_request;   // requestFrom() returns 0 bytes
    int      transactions;   // completed write transactions
};
extern WireMockState wire_mock;

inline void wire_mock_reset() {
    extern WireMockState wire_mock;
    memset(&wire_mock, 0, sizeof(wire_mock));
}

class TwoWire {
    uint8_t _buf[8]; int _n = 0;
    uint8_t _read_val = 0; bool _has_read = false;
public:
    void begin() {}
    void begin(int, int) {}
    void begin(uint8_t) {}
    void beginTransmission(uint8_t a) { wire_mock.last_addr = a; _n = 0; }
    size_t write(uint8_t v) { if (_n < 8) _buf[_n++] = v; return 1; }
    uint8_t endTransmission() {
        if (wire_mock.end_rc == 0 && _n >= 1) {
            wire_mock.reg_ptr = _buf[0];
            if (_n >= 2) wire_mock.regs[_buf[0]] = _buf[1];
            wire_mock.transactions++;
        }
        return wire_mock.end_rc;
    }
    uint8_t requestFrom(uint8_t, uint8_t qty) {
        if (wire_mock.fail_request) return 0;
        _read_val = wire_mock.regs[wire_mock.reg_ptr];
        _has_read = true;
        return qty;
    }
    int available() { return _has_read ? 1 : 0; }
    int read() { if (!_has_read) return -1; _has_read = false; return _read_val; }
};
extern TwoWire Wire;
