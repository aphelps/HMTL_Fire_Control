/*
 * Native unit tests for HMTL_Fire_Control sensor logic.
 *
 * Tests exercise real firmware code compiled for the host.  No hardware
 * required.  Run with:
 *
 *   cd platformio/HMTL_Fire_Control_Test
 *   pio test -e native
 */

#include <unity.h>
#include "HMTLTypes.h"
#include "RS485Utils.h"
#include "HMTL_Fire_Control.h"
#include "Fire_Control_Sensors.h"

// Functions defined in Fire_Control_Sensors.cpp but not in any public header
void checkPulse(uint8_t sensor, uint16_t address, uint8_t output,
                uint16_t onperiod, uint16_t offperiod);
void sendLEDMode();
void handle_ignition();
void handle_poof_enable();

// Controllable clock
extern unsigned long _mock_millis;

// Controllable GPIO (set_pin_value / clear_all_pins are in test_support.cpp)
extern "C" {
    void set_pin_value(uint8_t pin, uint8_t val);
    void clear_all_pins();

    // sendHMTL* capture API
    void reset_send_captures();
    bool send_value_was_called();
    uint16_t last_send_address();
    uint8_t  last_send_output();
    int      last_send_value_int();
    bool send_timed_was_called();
    uint16_t last_timed_address();
    uint32_t last_timed_period();
    bool send_cancel_was_called();
    bool send_blink_was_called();
    int  send_call_count();

    // mode stub capture API
    void reset_mode_captures();
    bool sparkle_was_called();
    bool blink_was_called();

    // debug log
    void debug_log_begin_test(const char *name);
}

// Internal variables with external linkage in Fire_Control_Sensors.cpp.
// Accessible from tests without modifying production code.
extern uint16_t pulse_bpm_1, pulse_length_1, pulse_delay_1;
extern uint16_t pulse_bpm_2, pulse_length_2, pulse_delay_2;
extern uint16_t pulse_bpm_3, pulse_length_3, pulse_delay_3;
extern uint16_t pulse_bpm_4, pulse_length_4, pulse_delay_4;
extern bool switch_states[];
extern bool switch_changed[];
extern bool lights_on;
extern uint8_t led_mode;
extern uint8_t led_mode_value;
extern uint8_t brightness;

// touch_sensor is declared extern in HMTL_Fire_Control.h
// (MPR121.h stub provides the _setTouched / _clearAll API)

// ============================================================================
// setUp / tearDown
// ============================================================================

void setUp() {
    _mock_millis = 0;
    debug_log_begin_test(Unity.CurrentTestName);
    reset_send_captures();
    reset_mode_captures();
    clear_all_pins();
    touch_sensor._clearAll();

    // Default pulse globals to known values
    pulse_bpm_1 = 60;   pulse_length_1 = 25;
    pulse_bpm_2 = 120;  pulse_length_2 = 25;
    pulse_bpm_3 = 90;   pulse_length_3 = 25;
    pulse_bpm_4 = 200;  pulse_length_4 = 25;

    // Default light state
    lights_on       = false;
    led_mode        = LED_MODE_ON;
    led_mode_value  = 50;
    brightness      = 96;

    // Reset switch state (not touching any switch)
    for (int i = 0; i < 4; i++) {
        switch_states[i]  = false;
        switch_changed[i] = false;
    }
}

void tearDown() {}

// ============================================================================
// calculate_pulse tests — pure math, no hardware
// ============================================================================

void test_calculate_pulse_60bpm() {
    // delay = (60000 / bpm) - length = (60000 / 60) - 25 = 975
    pulse_bpm_1    = 60;
    pulse_length_1 = 25;
    calculate_pulse();
    TEST_ASSERT_EQUAL(975, pulse_delay_1);
}

void test_calculate_pulse_120bpm() {
    pulse_bpm_2    = 120;
    pulse_length_2 = 25;
    calculate_pulse();
    TEST_ASSERT_EQUAL(475, pulse_delay_2);
}

void test_calculate_pulse_all_channels() {
    pulse_bpm_1 = 60;  pulse_length_1 = 0;
    pulse_bpm_2 = 120; pulse_length_2 = 0;
    pulse_bpm_3 = 90;  pulse_length_3 = 0;
    pulse_bpm_4 = 200; pulse_length_4 = 0;
    calculate_pulse();
    TEST_ASSERT_EQUAL(1000, pulse_delay_1);  // 60000/60  = 1000
    TEST_ASSERT_EQUAL(500,  pulse_delay_2);  // 60000/120 = 500
    TEST_ASSERT_EQUAL(666,  pulse_delay_3);  // 60000/90  = 666 (integer)
    TEST_ASSERT_EQUAL(300,  pulse_delay_4);  // 60000/200 = 300
}

void test_calculate_pulse_long_burst_shortens_delay() {
    pulse_bpm_1    = 60;
    pulse_length_1 = 500;
    calculate_pulse();
    // (60000/60) - 500 = 500
    TEST_ASSERT_EQUAL(500, pulse_delay_1);
}

// ============================================================================
// sensor_to_led tests — pure mapping, OBJECT_TYPE=TOUCH_CONTROLLER
// ============================================================================
//
// TOUCH_CONTROLLER mapping (sensor → LED):
//   sensor 11 → LED 0,  sensor 10 → LED 1 ... sensor 6 → LED 5
//   sensor  5 → LED 11, sensor  4 → LED 10 ... sensor 0 → LED 6

void test_sensor_to_led_upper_half() {
    // Sensors 6–11: led = 11 - sensor
    TEST_ASSERT_EQUAL(0,  sensor_to_led(11));
    TEST_ASSERT_EQUAL(1,  sensor_to_led(10));
    TEST_ASSERT_EQUAL(2,  sensor_to_led(9));
    TEST_ASSERT_EQUAL(3,  sensor_to_led(8));
    TEST_ASSERT_EQUAL(4,  sensor_to_led(7));
    TEST_ASSERT_EQUAL(5,  sensor_to_led(6));
}

void test_sensor_to_led_lower_half() {
    // Sensors 0–5: led = sensor + 6
    TEST_ASSERT_EQUAL(6,  sensor_to_led(0));
    TEST_ASSERT_EQUAL(7,  sensor_to_led(1));
    TEST_ASSERT_EQUAL(8,  sensor_to_led(2));
    TEST_ASSERT_EQUAL(9,  sensor_to_led(3));
    TEST_ASSERT_EQUAL(10, sensor_to_led(4));
    TEST_ASSERT_EQUAL(11, sensor_to_led(5));
}

void test_sensor_to_led_boundary_at_6() {
    // Sensor 6 is the boundary: 11 - 6 = 5 (upper-half formula)
    TEST_ASSERT_EQUAL(5, sensor_to_led(6));
    // Sensor 5: 5 + 6 = 11 (lower-half formula)
    TEST_ASSERT_EQUAL(11, sensor_to_led(5));
}

// ============================================================================
// sensor_switches tests — mocked digitalRead
// ============================================================================

void test_sensor_switches_detects_press() {
    // Assert switch 0 pin LOW → switch_states[0] becomes true
    set_pin_value(SWITCH_PIN_1, LOW);
    sensor_switches();
    TEST_ASSERT_TRUE(switch_states[0]);
    TEST_ASSERT_TRUE(switch_changed[0]);
}

void test_sensor_switches_detects_release() {
    // Start pressed
    switch_states[0] = true;
    set_pin_value(SWITCH_PIN_1, HIGH);
    sensor_switches();
    TEST_ASSERT_FALSE(switch_states[0]);
    TEST_ASSERT_TRUE(switch_changed[0]);
}

void test_sensor_switches_stable_high_not_changed() {
    switch_states[0]  = false;  // was already released
    switch_changed[0] = false;
    set_pin_value(SWITCH_PIN_1, HIGH);
    sensor_switches();
    TEST_ASSERT_FALSE(switch_states[0]);
    TEST_ASSERT_FALSE(switch_changed[0]);  // no transition
}

void test_sensor_switches_stable_low_not_changed() {
    switch_states[0]  = true;   // was already pressed
    switch_changed[0] = false;
    set_pin_value(SWITCH_PIN_1, LOW);
    sensor_switches();
    TEST_ASSERT_TRUE(switch_states[0]);
    TEST_ASSERT_FALSE(switch_changed[0]);  // no transition
}

void test_sensor_switches_all_four_independent() {
    set_pin_value(SWITCH_PIN_1, LOW);
    set_pin_value(SWITCH_PIN_2, HIGH);
    set_pin_value(SWITCH_PIN_3, LOW);
    set_pin_value(SWITCH_PIN_4, HIGH);
    sensor_switches();
    TEST_ASSERT_TRUE(switch_states[0]);
    TEST_ASSERT_FALSE(switch_states[1]);
    TEST_ASSERT_TRUE(switch_states[2]);
    TEST_ASSERT_FALSE(switch_states[3]);
}

// ============================================================================
// checkPulse tests — mocked touch_sensor
// ============================================================================

void test_checkPulse_touched_sends_blink() {
    touch_sensor._setTouched(0, true);  // rising edge: changed=true, touched=true
    checkPulse(0, poofer1_address, POOFER2_POOF1, 100, 200);
    TEST_ASSERT_TRUE(send_blink_was_called());
}

void test_checkPulse_released_sends_cancel_and_off() {
    // Start touched so changed=true, touched=false = release event
    touch_sensor._setTouched(0, true);
    touch_sensor._setTouched(0, false);  // now changed=true, touched=false
    checkPulse(0, poofer1_address, POOFER2_POOF1, 100, 200);
    TEST_ASSERT_TRUE(send_cancel_was_called());
    TEST_ASSERT_TRUE(send_value_was_called());  // sendOff sends value=0
    TEST_ASSERT_EQUAL(0, last_send_value_int());
}

void test_checkPulse_no_change_sends_nothing() {
    touch_sensor._setNoChange();
    // touched(0) is false, changed(0) is false → nothing sent
    checkPulse(0, poofer1_address, POOFER2_POOF1, 100, 200);
    TEST_ASSERT_FALSE(send_blink_was_called());
    TEST_ASSERT_FALSE(send_cancel_was_called());
    TEST_ASSERT_EQUAL(0, send_call_count());
}

// ============================================================================
// sendLEDMode tests — light output based on lights_on / led_mode state
// ============================================================================

void test_send_led_mode_off_sends_cancel_and_off() {
    lights_on = false;
    sendLEDMode();
    // sendCancelAndOff → sendCancel + sendOff(value=0)
    TEST_ASSERT_TRUE(send_cancel_was_called());
    TEST_ASSERT_TRUE(send_value_was_called());
    TEST_ASSERT_EQUAL(0, last_send_value_int());
}

void test_send_led_mode_on_steady_sends_brightness() {
    lights_on = true;
    led_mode  = LED_MODE_ON;
    brightness = 96;
    sendLEDMode();
    TEST_ASSERT_TRUE(send_value_was_called());
    TEST_ASSERT_EQUAL(LIGHTS_ADDRESS, last_send_address());
    TEST_ASSERT_EQUAL(96, last_send_value_int());
}

void test_send_led_mode_on_blink_sends_pulse() {
    lights_on     = true;
    led_mode      = LED_MODE_BLINK;
    led_mode_value = 75;
    sendLEDMode();
    TEST_ASSERT_TRUE(send_blink_was_called());
}

// ============================================================================
// handle_ignition rate-limiting tests
// ============================================================================
//
// handle_ignition() uses a function-local static 'last_on' that persists
// across calls.  We reset it by calling the function with switch off +
// switch_changed, which sets last_on = 0.

static void reset_ignition_static() {
    // Trigger the else-if branch: switch off + changed → last_on = 0
    switch_states[POOFER_IGNITER_SWITCH]  = false;
    switch_changed[POOFER_IGNITER_SWITCH] = true;
    handle_ignition();
    switch_changed[POOFER_IGNITER_SWITCH] = false;
    reset_send_captures();
}

void test_ignition_sends_burst_when_switch_on() {
    reset_ignition_static();
    _mock_millis = 20000;  // past 15s window from last_on=0
    switch_states[POOFER_IGNITER_SWITCH]  = true;
    switch_changed[POOFER_IGNITER_SWITCH] = true;
    handle_ignition();
    TEST_ASSERT_TRUE(send_timed_was_called());
    TEST_ASSERT_EQUAL(poofer1_address, last_timed_address());
}

void test_ignition_blocked_within_15s_window() {
    reset_ignition_static();
    // First call at t=20000 sets last_on=20000
    _mock_millis = 20000;
    switch_states[POOFER_IGNITER_SWITCH]  = true;
    switch_changed[POOFER_IGNITER_SWITCH] = true;
    handle_ignition();
    reset_send_captures();

    // Second call at t=25000 (5s later, within 15s window) → blocked
    _mock_millis = 25000;
    switch_changed[POOFER_IGNITER_SWITCH] = false;
    handle_ignition();
    TEST_ASSERT_FALSE(send_timed_was_called());
}

void test_ignition_allowed_after_15s_window() {
    reset_ignition_static();
    // First call at t=20000
    _mock_millis = 20000;
    switch_states[POOFER_IGNITER_SWITCH]  = true;
    switch_changed[POOFER_IGNITER_SWITCH] = true;
    handle_ignition();
    reset_send_captures();

    // Second call at t=36000 (16s later, past 15s window) → allowed
    _mock_millis = 36000;
    handle_ignition();
    TEST_ASSERT_TRUE(send_timed_was_called());
}

void test_ignition_off_sends_value_zero() {
    reset_ignition_static();
    switch_states[POOFER_IGNITER_SWITCH]  = false;
    switch_changed[POOFER_IGNITER_SWITCH] = true;
    handle_ignition();
    TEST_ASSERT_TRUE(send_value_was_called());
    TEST_ASSERT_EQUAL(0, last_send_value_int());
}

// ============================================================================
// handle_poof_enable tests — mode function capture
// ============================================================================

void test_poof_enable_on_calls_blink() {
    switch_states[POOFER_ENABLE_SWITCH]  = true;
    switch_changed[POOFER_ENABLE_SWITCH] = true;
    handle_poof_enable();
    TEST_ASSERT_TRUE(blink_was_called());
}

void test_poof_enable_off_calls_sparkle() {
    switch_states[POOFER_ENABLE_SWITCH]  = false;
    switch_changed[POOFER_ENABLE_SWITCH] = true;
    handle_poof_enable();
    TEST_ASSERT_TRUE(sparkle_was_called());
}

void test_poof_enable_off_sends_cancel_to_all_poofers() {
    switch_states[POOFER_ENABLE_SWITCH]  = false;
    switch_changed[POOFER_ENABLE_SWITCH] = true;
    handle_poof_enable();
    // CONTROL_SINGLE_QUINT: cancels 5 poofers (5 sendCancelAndOff = 10 sends)
    TEST_ASSERT_TRUE(send_call_count() > 0);
}

// ============================================================================
// main
// ============================================================================

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // calculate_pulse
    RUN_TEST(test_calculate_pulse_60bpm);
    RUN_TEST(test_calculate_pulse_120bpm);
    RUN_TEST(test_calculate_pulse_all_channels);
    RUN_TEST(test_calculate_pulse_long_burst_shortens_delay);

    // sensor_to_led
    RUN_TEST(test_sensor_to_led_upper_half);
    RUN_TEST(test_sensor_to_led_lower_half);
    RUN_TEST(test_sensor_to_led_boundary_at_6);

    // sensor_switches
    RUN_TEST(test_sensor_switches_detects_press);
    RUN_TEST(test_sensor_switches_detects_release);
    RUN_TEST(test_sensor_switches_stable_high_not_changed);
    RUN_TEST(test_sensor_switches_stable_low_not_changed);
    RUN_TEST(test_sensor_switches_all_four_independent);

    // checkPulse
    RUN_TEST(test_checkPulse_touched_sends_blink);
    RUN_TEST(test_checkPulse_released_sends_cancel_and_off);
    RUN_TEST(test_checkPulse_no_change_sends_nothing);

    // sendLEDMode
    RUN_TEST(test_send_led_mode_off_sends_cancel_and_off);
    RUN_TEST(test_send_led_mode_on_steady_sends_brightness);
    RUN_TEST(test_send_led_mode_on_blink_sends_pulse);

    // handle_ignition rate-limiting
    RUN_TEST(test_ignition_sends_burst_when_switch_on);
    RUN_TEST(test_ignition_blocked_within_15s_window);
    RUN_TEST(test_ignition_allowed_after_15s_window);
    RUN_TEST(test_ignition_off_sends_value_zero);

    // handle_poof_enable
    RUN_TEST(test_poof_enable_on_calls_blink);
    RUN_TEST(test_poof_enable_off_calls_sparkle);
    RUN_TEST(test_poof_enable_off_sends_cancel_to_all_poofers);

    return UNITY_END();
}
