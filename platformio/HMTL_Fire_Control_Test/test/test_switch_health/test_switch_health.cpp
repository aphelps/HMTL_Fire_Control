/*
 * Switch read-health tests, compiled with FC_SWITCHES_MCP23017 (env:native_mcp).
 *
 * These cannot live in test_sensors: that env builds the GPIO path, where
 * digitalRead cannot fail and every read is healthy by construction. One binary
 * cannot test both sides of a compile flag.
 *
 * What is under test is not the flag for its own sake but the pair of facts a
 * remote consumer needs to tell apart on a SINGLE poll:
 *
 *   "all four switches are open"   (a disarmed panel, entirely normal)
 *   "I could not read the switches" (a dead bus, reported as all-open by the
 *                                    fail-safe -- identical on the wire)
 *
 * The cumulative error counter does not distinguish them: a monotonic count
 * carries no information on one sample, only across two. So each test asserts
 * the observable switch state AND the health bit together -- a test that
 * checked only the flag would pass against firmware whose fail-safe never ran.
 */
/*
 * NOTE (merge of the OTA branch, 2026-08-21): these cases originally asserted
 * that a SINGLE failed read trips the fail-safe.  The OTA branch added a
 * debounce -- only an outage sustained for SWITCH_READ_FAIL_DEBOUNCE_MS counts
 * -- because this bus glitches roughly once in five hundred transactions and a
 * one-shot trip forced every switch open several times a minute.  The failures
 * below are therefore made SUSTAINED rather than single.  A lone glitch not
 * tripping is now the correct behaviour and is pinned by its own test on the
 * OTA side.
 */
#include <unity.h>
#include "Wire.h"
#include "Debug.h"
#include "HMTLTypes.h"
#include "RS485Utils.h"
#include "HMTL_Fire_Control.h"
#include "Fire_Control_Sensors.h"

extern "C" {
    void debug_log_begin_test(const char *name);
}

#define REG_GPIOA 0x12

extern bool switch_states[];
extern bool switch_changed[];
extern unsigned long _mock_millis;

/* All four switches reading closed on the expander: bits 0-3 low. */
static const uint8_t ALL_CLOSED = 0xF0;
static const uint8_t ALL_OPEN   = 0xFF;

void setUp() {
    debug_log_begin_test(Unity.CurrentTestName);
    wire_mock_reset();
    _mock_millis = 0;
    /*
     * The seen-open qualification is file-scope state in Fire_Control_Sensors.cpp
     * and survives between test cases, so without this every test after the
     * first inherits switches already qualified by an earlier one -- and a test
     * that means "this switch has NEVER proven it can read open" silently tests
     * the opposite.
     */
    fc_reset_switch_interlock();
    for (int i = 0; i < 4; i++) { switch_states[i] = false; switch_changed[i] = false; }
}
void tearDown() {}

/*
 * A switch only counts as closed once it has proven it can read open, so every
 * test has to walk it through that qualification before a close means anything.
 */
static void qualify_all_switches() {
    wire_mock.regs[REG_GPIOA] = ALL_OPEN;
    _mock_millis = 1;    sensor_switches();   /* window opens; 0 is the unset sentinel */
    _mock_millis = 1500; sensor_switches();   /* >= SWITCH_OPEN_LATCH_MS open */
    for (int i = 0; i < 4; i++) switch_changed[i] = false;
}

void test_successful_read_reports_healthy() {
    qualify_all_switches();
    wire_mock.regs[REG_GPIOA] = ALL_CLOSED;
    sensor_switches();
    TEST_ASSERT_TRUE(switch_states[0]);
    TEST_ASSERT_TRUE(fc_switches_read_ok());
}

/*
 * The defect this whole bit exists for. A failed read must report every switch
 * OPEN -- never last-known-state -- and must say so, because the open reading
 * on its own is indistinguishable from a genuinely disarmed panel.
 */
void test_failed_read_reports_all_open_AND_says_it_could_not_read() {
    qualify_all_switches();
    wire_mock.regs[REG_GPIOA] = ALL_CLOSED;
    sensor_switches();
    TEST_ASSERT_TRUE(switch_states[0]);          /* closed, and known good */
    TEST_ASSERT_TRUE(fc_switches_read_ok());

    wire_mock.end_rc = 2;                        /* address NACK: bus gone */
    sensor_switches();                           /* inside the debounce window */
    _mock_millis += SWITCH_READ_FAIL_DEBOUNCE_MS + 10;
    sensor_switches();                           /* sustained -- now a fault */

    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_FALSE(switch_states[i]);     /* the fail-safe effect ... */
    }
    TEST_ASSERT_FALSE(fc_switches_read_ok());    /* ... and it is reported */
}

/*
 * Ordering guard. `switches_read_ok = false` must be the FIRST statement of
 * sensor_switches(), so the MCP early return leaves it false by construction.
 * A merge that puts the `!mcp_ok` return above that assignment -- which is
 * exactly what git's conflict presentation did once before -- leaves the flag
 * stale-TRUE from the previous good read, and this test is what catches it.
 */
void test_health_bit_does_not_go_stale_true_across_a_failure() {
    qualify_all_switches();
    wire_mock.regs[REG_GPIOA] = ALL_CLOSED;
    sensor_switches();
    TEST_ASSERT_TRUE(fc_switches_read_ok());

    wire_mock.fail_request = true;               /* short read, not a NACK */
    sensor_switches();
    _mock_millis += SWITCH_READ_FAIL_DEBOUNCE_MS + 10;
    sensor_switches();                           /* sustained */
    TEST_ASSERT_FALSE(fc_switches_read_ok());
}

void test_health_recovers_when_the_bus_does() {
    qualify_all_switches();
    wire_mock.end_rc = 2;
    sensor_switches();
    _mock_millis += SWITCH_READ_FAIL_DEBOUNCE_MS + 10;
    sensor_switches();
    TEST_ASSERT_FALSE(fc_switches_read_ok());

    wire_mock.end_rc = 0;
    wire_mock.regs[REG_GPIOA] = ALL_OPEN;
    sensor_switches();
    TEST_ASSERT_TRUE(fc_switches_read_ok());
}

/*
 * A dead bus observes nothing, so it must not accrue open-time toward arming.
 * Without this, an outage long enough to span the latch window would qualify
 * every switch on nothing but absence of data, and the first successful read
 * showing a closed switch would arm immediately.
 */
void test_failed_reads_do_not_qualify_switches_toward_arming() {
    wire_mock.end_rc = 2;
    _mock_millis = 1;    sensor_switches();
    _mock_millis = 5000; sensor_switches();      /* far past the latch window */

    wire_mock.end_rc = 0;
    wire_mock.regs[REG_GPIOA] = ALL_CLOSED;
    _mock_millis = 5001; sensor_switches();

    TEST_ASSERT_TRUE(fc_switches_read_ok());
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_FALSE(switch_states[i]);     /* not armed by an outage */
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_successful_read_reports_healthy);
    RUN_TEST(test_failed_read_reports_all_open_AND_says_it_could_not_read);
    RUN_TEST(test_health_bit_does_not_go_stale_true_across_a_failure);
    RUN_TEST(test_health_recovers_when_the_bus_does);
    RUN_TEST(test_failed_reads_do_not_qualify_switches_toward_arming);
    return UNITY_END();
}
