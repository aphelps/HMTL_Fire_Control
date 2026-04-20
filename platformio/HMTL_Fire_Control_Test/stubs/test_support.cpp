/*
 * Global instances and stub implementations for HMTL_Fire_Control native tests.
 *
 * Included once into every test binary via fire_control_sources.cpp.
 * Pattern mirrors HMTL/platformio/HMTL_Test/stubs/test_support.cpp.
 */

#include "Arduino.h"
#include "RS485Utils.h"
#include "TimeSync.h"
#include "FastLED.h"
#include "PixelUtil.h"
#include "EEPROM.h"
#include "Wire.h"
#include "MPR121.h"
#include "LiquidCrystal.h"
#include "HMTLTypes.h"
#include "Debug.h"

#include <vector>
#include <string>
#include <cstdio>
#include <cstring>
#include <cstdlib>

// ---------------------------------------------------------------------------
// Debug log — in-memory line buffer + log file
// (mirrors HMTL_Test/stubs/test_support.cpp exactly)
// ---------------------------------------------------------------------------

#ifndef DEBUG_LOG_PATH
  #define DEBUG_LOG_PATH "/tmp/fire_control_test_debug.log"
#endif

static std::vector<std::string> s_log_lines;
static std::string               s_log_current;
static FILE                     *s_log_file   = NULL;
static std::string               s_log_path   = DEBUG_LOG_PATH;

static void ensure_log_file() {
    if (!s_log_file)
        s_log_file = fopen(s_log_path.c_str(), "a");
}

static void write_separator(const char *label) {
    ensure_log_file();
    if (!s_log_file) return;
    fprintf(s_log_file,
            "\n"
            "================================================================\n"
            "  %s\n"
            "================================================================\n",
            label);
    fflush(s_log_file);
}

extern "C" {

void debug_log_begin_test(const char *name) {
    s_log_lines.clear();
    s_log_current.clear();
    write_separator(name ? name : "(unknown test)");
}

void debug_log_reset() {
    s_log_lines.clear();
    s_log_current.clear();
    ensure_log_file();
    if (s_log_file) { fputs("--- reset ---\n", s_log_file); fflush(s_log_file); }
}

void debug_log_open(const char *path) {
    if (s_log_file) fclose(s_log_file);
    s_log_path = path;
    s_log_file = fopen(path, "a");
}

void debug_log_close() {
    if (s_log_file) { fclose(s_log_file); s_log_file = NULL; }
}

int debug_log_count() { return (int)s_log_lines.size(); }

const char *debug_log_line(int n) {
    if (n < 0 || n >= (int)s_log_lines.size()) return NULL;
    return s_log_lines[n].c_str();
}

int debug_log_contains(const char *substr) {
    for (const std::string &line : s_log_lines)
        if (line.find(substr) != std::string::npos) return 1;
    if (s_log_current.find(substr) != std::string::npos) return 1;
    return 0;
}

void _debug_emit(const char *s, int newline) {
    ensure_log_file();
    s_log_current += s;
    if (s_log_file) fputs(s, s_log_file);
    if (newline) {
        if (s_log_file) { fputc('\n', s_log_file); fflush(s_log_file); }
        s_log_lines.push_back(s_log_current);
        s_log_current.clear();
    }
}

void debug_err_state(int code) {
    char buf[32];
    snprintf(buf, sizeof(buf), "[ERR_STATE:0x%02x]", code);
    _debug_emit(buf, 1);
}

void debug_print_memory()              {}
void print_hex_buffer(const char *, int) {}

} // extern "C"

// ---------------------------------------------------------------------------
// Arduino GPIO mock — controllable pin values for sensor_switches() tests
// ---------------------------------------------------------------------------

uint8_t _mock_pin_values[64] = {};

int  digitalRead(int pin)              { return (pin < 64) ? _mock_pin_values[pin] : 0; }
void digitalWrite(int pin, int val)    { if (pin < 64) _mock_pin_values[pin] = val; }
void pinMode(int pin, int mode)        {}
int  analogRead(int pin)               { return 0; }

extern "C" {
    void set_pin_value(uint8_t pin, uint8_t val) {
        if (pin < 64) _mock_pin_values[pin] = val;
    }
    void clear_all_pins() { memset(_mock_pin_values, 0, sizeof(_mock_pin_values)); }
}

// ---------------------------------------------------------------------------
// Global instances required by Fire_Control_Sensors.cpp and HMTL_Fire_Control.h
// ---------------------------------------------------------------------------

unsigned long _mock_millis = 0;
TimeSync      timesync;
CFastLED      FastLED;
EEPROMClass   EEPROM;
FakeSerial    Serial;
TwoWire       Wire;

MPR121        touch_sensor;
LiquidCrystal lcd(0);
PixelUtil     pixels;
RS485Socket   rs485;

uint32_t       sensor_state  = 0;
uint16_t       my_address    = 0;
byte          *send_buffer   = nullptr;
config_hdr_t   config        = {};
output_hdr_t  *outputs[HMTL_MAX_OUTPUTS] = {};
void          *objects[HMTL_MAX_OUTPUTS] = {};

// ---------------------------------------------------------------------------
// sendHMTL* stubs — capture the last call so tests can assert on it
// ---------------------------------------------------------------------------

struct SendValueCapture {
    bool     called;
    uint16_t address;
    uint8_t  output;
    int      value;
};

struct SendTimedChangeCapture {
    bool     called;
    uint16_t address;
    uint8_t  output;
    uint32_t period;
    uint32_t start_color;
    uint32_t stop_color;
};

static SendValueCapture      s_send_value       = {};
static SendTimedChangeCapture s_send_timed       = {};
static bool                  s_send_cancel_called = false;
static bool                  s_send_blink_called  = false;
static int                   s_send_call_count    = 0;

void sendHMTLValue(uint16_t address, uint8_t output, int value) {
    s_send_value = { true, address, output, value };
    s_send_call_count++;
}

void sendHMTLTimedChange(uint16_t address, uint8_t output,
                          uint32_t change_period,
                          uint32_t start_color, uint32_t stop_color) {
    s_send_timed = { true, address, output, change_period, start_color, stop_color };
    s_send_call_count++;
}

void sendHMTLCancel(uint16_t address, uint8_t output) {
    s_send_cancel_called = true;
    s_send_call_count++;
}

void sendHMTLBlink(uint16_t address, uint8_t output,
                   uint16_t onperiod, uint32_t oncolor,
                   uint16_t offperiod, uint32_t offcolor) {
    s_send_blink_called = true;
    s_send_call_count++;
}

extern "C" {
    void reset_send_captures() {
        s_send_value       = {};
        s_send_timed       = {};
        s_send_cancel_called = false;
        s_send_blink_called  = false;
        s_send_call_count    = 0;
    }
    bool     send_value_was_called()   { return s_send_value.called; }
    uint16_t last_send_address()       { return s_send_value.address; }
    uint8_t  last_send_output()        { return s_send_value.output; }
    int      last_send_value_int()     { return s_send_value.value; }
    bool     send_timed_was_called()   { return s_send_timed.called; }
    uint16_t last_timed_address()      { return s_send_timed.address; }
    uint32_t last_timed_period()       { return s_send_timed.period; }
    bool     send_cancel_was_called()  { return s_send_cancel_called; }
    bool     send_blink_was_called()   { return s_send_blink_called; }
    int      send_call_count()         { return s_send_call_count; }
}

// ---------------------------------------------------------------------------
// modes.h stub implementations
// ---------------------------------------------------------------------------

static bool s_sparkle_called = false;
static bool s_blink_called   = false;
static bool s_cancel_called  = false;

void init_modes(Socket **sockets, byte num_sockets) {}
boolean messages_and_modes(void) { return false; }

void setSparkle() { s_sparkle_called = true; }
void setBlink(uint32_t color) { s_blink_called = true; }
void setCancel() { s_cancel_called = true; }
boolean followup_actions() { return false; }

extern "C" {
    void reset_mode_captures() {
        s_sparkle_called = false;
        s_blink_called   = false;
        s_cancel_called  = false;
    }
    bool sparkle_was_called() { return s_sparkle_called; }
    bool blink_was_called()   { return s_blink_called; }
    bool cancel_was_called()  { return s_cancel_called; }
}

// ---------------------------------------------------------------------------
// GeneralUtils stubs
// ---------------------------------------------------------------------------

void    blink_value(int, int, int, int) {}
boolean pin_is_PWM(int)                 { return false; }
void    print_hex_string(const byte *, int) {}

// ---------------------------------------------------------------------------
// EEPromUtils stubs
// ---------------------------------------------------------------------------

int eeprom_read_objects(int, byte *, int)  { return -1; }
int eeprom_write_objects(int, byte *, int) { return -1; }
