/*
 * Native unit tests for the long-combo touch gestures (CONTROL_SINGLE_QUINT
 * direct mode): pads 0+1 held -> one large-poofer burst; pads 2+3 held ->
 * one FC-driven sweep of poofer outputs 1-4.
 *
 * Assertions are against the messages EMITTED (address/output/duration and
 * their order in the send log), not internal state.
 */

#include <unity.h>
#include "HMTLTypes.h"
#include "RS485Utils.h"
#include "HMTL_Fire_Control.h"
#include "Fire_Control_Sensors.h"

// Combo API (defined in Fire_Control_Sensors.cpp, sketch-local)
void handle_long_combos();
void run_combo_sequence();
void combo_sequence_abort();
void combo_reset();

extern unsigned long _mock_millis;
extern bool switch_states[];
extern bool switch_changed[];
extern MPR121 touch_sensor;
extern uint16_t poofer1_address;
extern uint16_t poofer2_address;

extern "C" {
    void reset_send_captures();
    void clear_all_pins();
    bool send_timed_was_called();
    uint16_t last_timed_address();
    uint32_t last_timed_period();
    int  send_log_size();
    int  send_log_get(int i, int *type, uint16_t *address, uint8_t *output,
                      int *value);
}
void debug_log_begin_test(const char *name);
#include "unity_internals.h"

// From test_support's SendKind enum ordering
#define KIND_TIMED 1

#define LARGE_OUTPUT 0x2  /* POOFER1_LARGE */

void setUp() {
    _mock_millis = 1000;  // never 0: combo hold tracking uses 0 as "untouched"
    debug_log_begin_test(Unity.CurrentTestName);
    reset_send_captures();
    clear_all_pins();
    touch_sensor._clearAll();
    combo_reset();
    for (int i = 0; i < 4; i++) {
        switch_states[i]  = false;
        switch_changed[i] = false;
    }
    // Armed: enable + pilot
    switch_states[POOFER_ENABLE_SWITCH] = true;
    switch_states[POOFER_PILOT_SWITCH]  = true;
}

void tearDown() {}

// Count timed-change sends to (address, output) in the log
static int timed_count(uint16_t address, uint8_t output) {
    int n = 0;
    for (int i = 0; i < send_log_size(); i++) {
        int t, v; uint16_t a; uint8_t o;
        send_log_get(i, &t, &a, &o, &v);
        if (t == KIND_TIMED && a == address && o == output) n++;
    }
    return n;
}

static void tick(unsigned long advance_ms) {
    _mock_millis += advance_ms;
    handle_long_combos();
}

// ============================================================================

void test_combo01_fires_large_after_hold() {
    touch_sensor._setTouched(0, true);
    touch_sensor._setTouched(1, true);
    tick(0);          // records hold start
    tick(700);        // under threshold
    TEST_ASSERT_FALSE(send_timed_was_called());
    tick(150);        // 850ms held: fires
    TEST_ASSERT_TRUE(send_timed_was_called());
    TEST_ASSERT_EQUAL_UINT16(POOFER1_ADDRESS, last_timed_address());
    TEST_ASSERT_EQUAL(1, timed_count(poofer1_address, LARGE_OUTPUT));
    TEST_ASSERT_EQUAL_UINT32(500, last_timed_period());
}

void test_combo01_latches_until_release() {
    touch_sensor._setTouched(0, true);
    touch_sensor._setTouched(1, true);
    tick(0);
    tick(900);
    TEST_ASSERT_EQUAL(1, timed_count(poofer1_address, LARGE_OUTPUT));
    tick(1000);       // keep holding: no repeat
    tick(1000);
    TEST_ASSERT_EQUAL(1, timed_count(poofer1_address, LARGE_OUTPUT));

    // Releasing only one pad does NOT re-arm
    touch_sensor._setTouched(1, false);
    tick(50);
    touch_sensor._setTouched(1, true);
    tick(900);
    TEST_ASSERT_EQUAL(1, timed_count(poofer1_address, LARGE_OUTPUT));

    // Releasing both re-arms; next qualified hold fires again
    touch_sensor._setTouched(0, false);
    touch_sensor._setTouched(1, false);
    tick(50);
    touch_sensor._setTouched(0, true);
    touch_sensor._setTouched(1, true);
    tick(0);
    tick(900);
    TEST_ASSERT_EQUAL(2, timed_count(poofer1_address, LARGE_OUTPUT));
}

void test_single_pad_hold_does_not_fire() {
    touch_sensor._setTouched(0, true);
    tick(0);
    tick(2000);
    TEST_ASSERT_FALSE(send_timed_was_called());
}

void test_combo23_runs_one_sweep_in_order() {
    touch_sensor._setTouched(2, true);
    touch_sensor._setTouched(3, true);
    tick(0);
    tick(800);        // qualifies; step 0 fires immediately
    tick(200);        // step 1
    tick(200);        // step 2
    tick(200);        // step 3
    tick(200);        // idle — must NOT wrap
    tick(1000);       // still idle
    // Exactly one burst per output, in order 0,1,2,3, each 150ms
    int seen = 0;
    uint8_t expect[4] = {0x0, 0x1, 0x2, 0x3};
    for (int i = 0; i < send_log_size(); i++) {
        int t, v; uint16_t a; uint8_t o;
        send_log_get(i, &t, &a, &o, &v);
        if (t == KIND_TIMED && a == poofer2_address) {
            TEST_ASSERT_TRUE_MESSAGE(seen < 4, "sequence wrapped/extra step");
            TEST_ASSERT_EQUAL_UINT8(expect[seen], o);
            TEST_ASSERT_EQUAL(150, v);
            seen++;
        }
    }
    TEST_ASSERT_EQUAL(4, seen);
}

void test_combo23_disarm_aborts_sweep() {
    touch_sensor._setTouched(2, true);
    touch_sensor._setTouched(3, true);
    tick(0);
    tick(800);        // step 0 sent
    TEST_ASSERT_EQUAL(1, timed_count(poofer2_address, 0x0));
    switch_states[POOFER_PILOT_SWITCH] = false;  // disarm mid-sweep
    tick(200);
    tick(200);
    tick(200);
    TEST_ASSERT_EQUAL(0, timed_count(poofer2_address, 0x1));
    TEST_ASSERT_EQUAL(0, timed_count(poofer2_address, 0x2));
    TEST_ASSERT_EQUAL(0, timed_count(poofer2_address, 0x3));
    // Re-arming does not resume the aborted sweep
    switch_states[POOFER_PILOT_SWITCH] = true;
    run_combo_sequence();
    _mock_millis += 500;
    run_combo_sequence();
    TEST_ASSERT_EQUAL(0, timed_count(poofer2_address, 0x1));
}

void test_program_mode_abort_hook() {
    touch_sensor._setTouched(2, true);
    touch_sensor._setTouched(3, true);
    tick(0);
    tick(800);
    TEST_ASSERT_EQUAL(1, timed_count(poofer2_address, 0x0));
    combo_sequence_abort();   // what program-mode entry calls
    _mock_millis += 1000;
    run_combo_sequence();
    TEST_ASSERT_EQUAL(0, timed_count(poofer2_address, 0x1));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_combo01_fires_large_after_hold);
    RUN_TEST(test_combo01_latches_until_release);
    RUN_TEST(test_single_pad_hold_does_not_fire);
    RUN_TEST(test_combo23_runs_one_sweep_in_order);
    RUN_TEST(test_combo23_disarm_aborts_sweep);
    RUN_TEST(test_program_mode_abort_hook);
    UNITY_END();
    return 0;
}
