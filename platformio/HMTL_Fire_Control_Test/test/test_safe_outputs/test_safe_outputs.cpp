/*
 * Native unit tests for the safe-state drives.
 *
 * The property under test throughout is CANCEL-THEN-OFF for each output, not
 * merely "something was sent".  sendBurst() starts a TIMED_CHANGE program on
 * the remote module -- 30 seconds for the igniter and pilot -- and a bare
 * value-0 does not stop it; the program re-asserts the output for the rest of
 * its duration.  A test that only checked "a zero was sent" would pass against
 * code that leaves a remote output burning.
 *
 *   cd platformio/HMTL_Fire_Control_Test
 *   pio test -e native --filter test_safe_outputs
 */

#include <unity.h>
#include "HMTLTypes.h"
#include "RS485Utils.h"
#include "HMTL_Fire_Control.h"
#include "Fire_Control_Sensors.h"

extern "C" {
    void reset_send_captures();
    int  send_call_count();
    int  send_log_cancelled_and_off(uint16_t address, uint8_t output);
    int  send_log_size();
    void debug_log_begin_test(const char *name);
}

extern uint16_t poofer1_address;
extern uint16_t poofer2_address;

void setUp(void) {
    debug_log_begin_test(Unity.CurrentTestName);
    reset_send_captures();
}
void tearDown(void) {}

#define ASSERT_CLOSED(addr, output, label)                                    \
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, send_log_cancelled_and_off((addr), (output)), \
                                  label " was not cancelled AND set off")

// ---------------------------------------------------------------------------
// fc_poofers_safe()
// ---------------------------------------------------------------------------

void test_poofers_safe_closes_every_accumulator() {
    fc_poofers_safe();
    /* This build is CONTROL_SINGLE_QUINT: one large poofer on module 1 and
     * four accumulators on module 2. */
    ASSERT_CLOSED(poofer1_address, POOFER1_LARGE,  "POOFER1_LARGE");
    ASSERT_CLOSED(poofer2_address, POOFER2_POOF1, "POOFER2_POOF1");
    ASSERT_CLOSED(poofer2_address, POOFER2_POOF2, "POOFER2_POOF2");
    ASSERT_CLOSED(poofer2_address, POOFER2_POOF3, "POOFER2_POOF3");
    ASSERT_CLOSED(poofer2_address, POOFER2_POOF4, "POOFER2_POOF4");
    /* 5 outputs x (cancel + off) */
    TEST_ASSERT_EQUAL_INT(10, send_call_count());
}

void test_poofers_safe_does_not_touch_igniter_or_pilot() {
    /* Keeping the drives separate is what lets the igniter off-edge close the
     * igniter WITHOUT also killing the poofers. */
    fc_poofers_safe();
    TEST_ASSERT_EQUAL_INT(0, send_log_cancelled_and_off(poofer1_address,
                                                        POOFER1_IGNITER));
    TEST_ASSERT_EQUAL_INT(0, send_log_cancelled_and_off(poofer1_address,
                                                        POOFER1_PILOT));
}

// ---------------------------------------------------------------------------
// fc_igniter_safe() / fc_pilot_safe()
// ---------------------------------------------------------------------------

void test_igniter_safe_cancels_the_burst() {
    /* The regression this locks down: the igniter off-edge used to send a bare
     * sendOff(), which does not stop a 30 s TIMED_CHANGE already running. */
    fc_igniter_safe();
    ASSERT_CLOSED(poofer1_address, POOFER1_IGNITER, "POOFER1_IGNITER");
}

void test_pilot_safe_cancels_the_burst() {
    fc_pilot_safe();
    ASSERT_CLOSED(poofer1_address, POOFER1_PILOT, "POOFER1_PILOT");
}

// ---------------------------------------------------------------------------
// fc_all_outputs_safe()
// ---------------------------------------------------------------------------

void test_all_outputs_safe_closes_every_configured_output() {
    fc_all_outputs_safe();
    ASSERT_CLOSED(poofer1_address, POOFER1_LARGE,   "POOFER1_LARGE");
    ASSERT_CLOSED(poofer2_address, POOFER2_POOF1,   "POOFER2_POOF1");
    ASSERT_CLOSED(poofer2_address, POOFER2_POOF2,   "POOFER2_POOF2");
    ASSERT_CLOSED(poofer2_address, POOFER2_POOF3,   "POOFER2_POOF3");
    ASSERT_CLOSED(poofer2_address, POOFER2_POOF4,   "POOFER2_POOF4");
    ASSERT_CLOSED(poofer1_address, POOFER1_IGNITER, "POOFER1_IGNITER");
    ASSERT_CLOSED(poofer1_address, POOFER1_PILOT,   "POOFER1_PILOT");
    /* 7 outputs x (cancel + off), and nothing else. */
    TEST_ASSERT_EQUAL_INT(14, send_call_count());
}

void test_all_outputs_safe_is_idempotent() {
    /* It is called at boot, and again on every OTA request; calling it twice
     * must be harmless and must still close everything. */
    fc_all_outputs_safe();
    reset_send_captures();
    fc_all_outputs_safe();
    ASSERT_CLOSED(poofer1_address, POOFER1_LARGE,   "POOFER1_LARGE");
    ASSERT_CLOSED(poofer1_address, POOFER1_IGNITER, "POOFER1_IGNITER");
    ASSERT_CLOSED(poofer1_address, POOFER1_PILOT,   "POOFER1_PILOT");
    TEST_ASSERT_EQUAL_INT(14, send_call_count());
}

void test_all_outputs_safe_needs_no_switch_state() {
    /*
     * It must be callable unconditionally -- from setup() before any switch has
     * ever been read, and from the OTA path.  The two sequences it replaced
     * were edge-triggered on a switch transition and so could not simply be
     * invoked; that is why they had to be factored out rather than reused.
     */
    fc_reset_switch_interlock();
    fc_all_outputs_safe();
    TEST_ASSERT_EQUAL_INT(14, send_call_count());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_poofers_safe_closes_every_accumulator);
    RUN_TEST(test_poofers_safe_does_not_touch_igniter_or_pilot);

    RUN_TEST(test_igniter_safe_cancels_the_burst);
    RUN_TEST(test_pilot_safe_cancels_the_burst);

    RUN_TEST(test_all_outputs_safe_closes_every_configured_output);
    RUN_TEST(test_all_outputs_safe_is_idempotent);
    RUN_TEST(test_all_outputs_safe_needs_no_switch_state);

    return UNITY_END();
}
