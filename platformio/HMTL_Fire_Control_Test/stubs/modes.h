// Minimal modes.h stub for native tests.
//
// Replaces the real modes.h (which pulls in HMTLPrograms.h and the full
// HMTL messaging stack).  Implementations live in test_support.cpp so tests
// can assert on what was called.
#pragma once
#include "Arduino.h"

struct Socket;  // forward-declare; RS485Utils.h defines the real class

void    init_modes(Socket **sockets, byte num_sockets);
boolean messages_and_modes(void);
void    setSparkle();
void    setBlink(uint32_t color);
void    setCancel();
boolean followup_actions();

// pixel_color helper used by handle_poof_enable
inline uint32_t pixel_color(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}
