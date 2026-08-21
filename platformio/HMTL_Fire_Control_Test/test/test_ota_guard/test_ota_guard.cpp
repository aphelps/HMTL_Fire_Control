/*
 * Native unit tests for the OTA admission guard.
 *
 * This guard is the one place in the OTA path where being wrong is DANGEROUS
 * rather than merely broken, and its failure mode is silent: the ignition
 * fail-safe resolves an unreadable switch bank to "all switches open", which
 * this guard's question ("is anything active?") reads as "no, nothing is
 * active" -- a permit on fault.  Several of the tests below exist specifically
 * to fail if someone ever "simplifies" the guard back into that shape.
 *
 *   cd platformio/HMTL_Fire_Control_Test
 *   pio test -e native --filter test_ota_guard
 */

#include <unity.h>
#include "HMTLTypes.h"
#include "RS485Utils.h"
#include "HMTL_Fire_Control.h"
#include "Fire_Control_Sensors.h"
#include "fc_ota_guard.h"

extern "C" {
    void debug_log_begin_test(const char *name);
    void set_pin_value(uint8_t pin, uint8_t val);
    void clear_all_pins();
    /* Compiled in only under -DFC_SWITCHES_TEST_FAULT_INJECTION (native env
     * only).  Makes read_switch_bank() report failure, which is the failure
     * mode the direct-GPIO path does not otherwise have. */
    void fc_test_set_switch_bank_read_fails(bool fails);
}

extern unsigned long _mock_millis;
extern bool switch_states[];
extern bool switch_changed[];
/* The pin each switch is wired to.  Indexed the same way switch_states[] is,
 * so a test can name a switch by role (POOFER_ENABLE_SWITCH) rather than by
 * pin number. */
static const uint8_t test_switch_pins[FC_NUM_SWITCHES] = {
    SWITCH_PIN_1, SWITCH_PIN_2, SWITCH_PIN_3, SWITCH_PIN_4 };

/* Switches are ACTIVE-LOW (closed == LOW), so "all open" is every pin HIGH.
 * clear_all_pins() leaves them at 0, which is every switch CLOSED -- the
 * opposite of an idle panel. */
static void all_switches_open() {
    for (uint8_t i = 0; i < FC_NUM_SWITCHES; i++) {
        set_pin_value(test_switch_pins[i], 1);
    }
}

void setUp(void) {
    debug_log_begin_test(Unity.CurrentTestName);
    fc_test_set_switch_bank_read_fails(false);
    clear_all_pins();
    all_switches_open();
    fc_reset_switch_interlock();
    for (uint8_t i = 0; i < FC_NUM_SWITCHES; i++) {
        switch_states[i]  = false;
        switch_changed[i] = false;
    }
    /* Never 0: switch_open_since[] uses 0 as its "not started" sentinel. */
    _mock_millis = 1;
}
void tearDown(void) {
    /* Never leave the injected fault set: it would silently disarm every later
     * test in this binary. */
    fc_test_set_switch_bank_read_fails(false);
}

/* A guard input that SHOULD be permitted; each test spoils exactly one thing. */
static fc_ota_guard_input_t healthy_input() {
    fc_ota_guard_input_t in;
    for (uint8_t i = 0; i < FC_NUM_SWITCHES; i++) {
        in.switch_active[i] = false;
    }
    in.switch_read_ok  = true;
    in.snapshot_valid  = true;
    in.snapshot_age_ms = 0;
    return in;
}

static fc_ota_verdict_t evaluate(const fc_ota_guard_input_t &in) {
    return fc_ota_evaluate(in, FC_OTA_MAX_SNAPSHOT_AGE_MS, NULL);
}

// ---------------------------------------------------------------------------
// The permit case -- deliberately narrow
// ---------------------------------------------------------------------------

void test_permits_only_when_everything_is_healthy_and_idle() {
    TEST_ASSERT_EQUAL(FC_OTA_PERMIT, evaluate(healthy_input()));
}

// ---------------------------------------------------------------------------
// ANY active switch refuses -- not merely "armed"
// ---------------------------------------------------------------------------

void test_each_switch_individually_blocks() {
    /*
     * Every switch in turn, on its own.  This is the whole point of the "any
     * active switch" rule: POOFER_IGNITER and PROGRAM_MODE do not arm the
     * controller, so a guard written against fc_is_armed() would permit an
     * upload with either of them thrown.
     */
    for (uint8_t sw = 0; sw < FC_NUM_SWITCHES; sw++) {
        fc_ota_guard_input_t in = healthy_input();
        in.switch_active[sw] = true;
        TEST_ASSERT_EQUAL_MESSAGE(FC_OTA_REFUSE_SWITCH_ACTIVE, evaluate(in),
                                  "a single active switch must refuse OTA");
    }
}

void test_refusal_names_the_blocking_switch() {
    fc_ota_guard_input_t in = healthy_input();
    in.switch_active[2] = true;
    int8_t blocking = -1;
    TEST_ASSERT_EQUAL(FC_OTA_REFUSE_SWITCH_ACTIVE,
                      fc_ota_evaluate(in, FC_OTA_MAX_SNAPSHOT_AGE_MS, &blocking));
    /* A refusal has to say what is blocking it, or the operator's only recourse
     * is to flip switches at random until the upload takes. */
    TEST_ASSERT_EQUAL_INT8(2, blocking);
}

void test_non_arming_switch_alone_blocks() {
    /* Explicitly the case fc_is_armed() misses: igniter closed, but neither
     * ENABLE nor PILOT, so the controller is NOT armed -- and OTA must still
     * refuse. */
    fc_ota_guard_input_t in = healthy_input();
    in.switch_active[POOFER_IGNITER_SWITCH] = true;
    TEST_ASSERT_FALSE(in.switch_active[POOFER_ENABLE_SWITCH]);
    TEST_ASSERT_FALSE(in.switch_active[POOFER_PILOT_SWITCH]);
    TEST_ASSERT_EQUAL(FC_OTA_REFUSE_SWITCH_ACTIVE, evaluate(in));
}

void test_all_switches_active_blocks() {
    fc_ota_guard_input_t in = healthy_input();
    for (uint8_t i = 0; i < FC_NUM_SWITCHES; i++) in.switch_active[i] = true;
    TEST_ASSERT_EQUAL(FC_OTA_REFUSE_SWITCH_ACTIVE, evaluate(in));
}

// ---------------------------------------------------------------------------
// Positive evidence of health -- the permit-on-fault cases
// ---------------------------------------------------------------------------

void test_failed_switch_read_refuses_even_though_nothing_looks_active() {
    /*
     * THE INVERTED-FAIL-SAFE CASE.  On a read failure the ignition path forces
     * every switch open, so switch_active[] is all-false here -- which is
     * exactly what a permitted upload looks like.  The read-health flag is the
     * only thing standing between that and flashing a controller whose real
     * state is unknown.
     */
    fc_ota_guard_input_t in = healthy_input();
    in.switch_read_ok = false;
    for (uint8_t i = 0; i < FC_NUM_SWITCHES; i++) {
        TEST_ASSERT_FALSE(in.switch_active[i]);
    }
    TEST_ASSERT_EQUAL_MESSAGE(FC_OTA_REFUSE_SWITCH_READ_FAILED, evaluate(in),
                              "an unreadable switch bank must REFUSE, not permit");
}

void test_no_snapshot_refuses() {
    /* Before core 1 has published anything, "all switches inactive" is a
     * default-initialised struct, not an observation. */
    fc_ota_guard_input_t in = healthy_input();
    in.snapshot_valid = false;
    TEST_ASSERT_EQUAL(FC_OTA_REFUSE_NO_SNAPSHOT, evaluate(in));
}

void test_stale_snapshot_refuses() {
    /* A stalled core 1 keeps reporting its last known state forever.  Without
     * the freshness bound, a controller that stopped running its control loop
     * while disarmed would look permanently ready for an upload. */
    fc_ota_guard_input_t in = healthy_input();
    in.snapshot_age_ms = FC_OTA_MAX_SNAPSHOT_AGE_MS + 1;
    TEST_ASSERT_EQUAL(FC_OTA_REFUSE_STALE_SNAPSHOT, evaluate(in));
}

void test_snapshot_exactly_at_the_bound_is_still_fresh() {
    fc_ota_guard_input_t in = healthy_input();
    in.snapshot_age_ms = FC_OTA_MAX_SNAPSHOT_AGE_MS;
    TEST_ASSERT_EQUAL(FC_OTA_PERMIT, evaluate(in));
}

void test_health_is_reported_before_switch_activity() {
    /* With both wrong, the reason given must be the fault -- switch states read
     * from a failed read are fabrications and must not be quoted as findings. */
    fc_ota_guard_input_t in = healthy_input();
    in.switch_read_ok = false;
    in.switch_active[0] = true;
    TEST_ASSERT_EQUAL(FC_OTA_REFUSE_SWITCH_READ_FAILED, evaluate(in));
}

// ---------------------------------------------------------------------------
// The safe-state acknowledgement
// ---------------------------------------------------------------------------

void test_missing_safe_state_ack_refuses() {
    /*
     * Test Plan: "the safe-state request times out -> the update is refused
     * (not silently allowed)".  Everything else is healthy and idle, so this
     * fails only if the ack is being ignored.
     */
    fc_ota_guard_input_t in = healthy_input();
    TEST_ASSERT_EQUAL(FC_OTA_PERMIT, evaluate(in));
    TEST_ASSERT_EQUAL_MESSAGE(
        FC_OTA_REFUSE_SAFE_STATE_TIMEOUT,
        fc_ota_evaluate_begin(in, FC_OTA_MAX_SNAPSHOT_AGE_MS, false, NULL),
        "no safe-state ack must ABORT the update");
}

void test_safe_state_ack_alone_does_not_override_a_refusal() {
    /* An ack is necessary, never sufficient: core 1 confirming it drove the
     * outputs safe says nothing about a switch the operator is holding. */
    fc_ota_guard_input_t in = healthy_input();
    in.switch_active[1] = true;
    TEST_ASSERT_EQUAL(FC_OTA_REFUSE_SWITCH_ACTIVE,
                      fc_ota_evaluate_begin(in, FC_OTA_MAX_SNAPSHOT_AGE_MS,
                                            true, NULL));
}

void test_begin_permits_only_with_ack_and_health() {
    fc_ota_guard_input_t in = healthy_input();
    TEST_ASSERT_EQUAL(FC_OTA_PERMIT,
                      fc_ota_evaluate_begin(in, FC_OTA_MAX_SNAPSHOT_AGE_MS,
                                            true, NULL));
}

// ---------------------------------------------------------------------------
// Exhaustive: nothing but the fully-healthy, fully-idle, acked case permits
// ---------------------------------------------------------------------------

void test_only_one_input_combination_permits() {
    /*
     * Walk every combination of the four health/activity inputs plus the ack.
     * A single unexpected PERMIT here is the bug this whole file exists to
     * catch, and this test finds it regardless of which condition was dropped.
     */
    int permits = 0;
    for (int valid = 0; valid < 2; valid++) {
      for (int read_ok = 0; read_ok < 2; read_ok++) {
        for (int stale = 0; stale < 2; stale++) {
          for (int sw_mask = 0; sw_mask < (1 << FC_NUM_SWITCHES); sw_mask++) {
            for (int acked = 0; acked < 2; acked++) {
              fc_ota_guard_input_t in;
              in.snapshot_valid  = (valid != 0);
              in.switch_read_ok  = (read_ok != 0);
              in.snapshot_age_ms = stale ? (FC_OTA_MAX_SNAPSHOT_AGE_MS + 1) : 0;
              for (uint8_t i = 0; i < FC_NUM_SWITCHES; i++) {
                in.switch_active[i] = ((sw_mask >> i) & 1) != 0;
              }
              if (fc_ota_evaluate_begin(in, FC_OTA_MAX_SNAPSHOT_AGE_MS,
                                        acked != 0, NULL) == FC_OTA_PERMIT) {
                permits++;
                TEST_ASSERT_TRUE_MESSAGE(valid && read_ok && !stale &&
                                             sw_mask == 0 && acked,
                                         "permitted an unsafe combination");
              }
            }
          }
        }
      }
    }
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, permits,
                                  "exactly one combination may permit OTA");
}

// ---------------------------------------------------------------------------
// Reason strings
// ---------------------------------------------------------------------------

void test_every_verdict_has_a_distinct_reason_string() {
    /* The reason is what an operator sees in the 409 body; an empty or shared
     * string turns a specific refusal into an unactionable one. */
    const fc_ota_verdict_t verdicts[] = {
        FC_OTA_PERMIT,
        FC_OTA_REFUSE_NO_SNAPSHOT,
        FC_OTA_REFUSE_SWITCH_READ_FAILED,
        FC_OTA_REFUSE_STALE_SNAPSHOT,
        FC_OTA_REFUSE_SWITCH_ACTIVE,
        FC_OTA_REFUSE_SAFE_STATE_TIMEOUT,
    };
    const int n = (int)(sizeof(verdicts) / sizeof(verdicts[0]));
    for (int i = 0; i < n; i++) {
        const char *a = fc_ota_verdict_str(verdicts[i]);
        TEST_ASSERT_NOT_NULL(a);
        TEST_ASSERT_TRUE(a[0] != '\0');
        for (int j = i + 1; j < n; j++) {
            TEST_ASSERT_TRUE(strcmp(a, fc_ota_verdict_str(verdicts[j])) != 0);
        }
    }
}

// ---------------------------------------------------------------------------
// End to end: a FAILED switch read must refuse, through the real sensor code
//
// Everything above feeds the guard a hand-built input.  These two go through
// sensor_switches() itself, because the defect this pair exists to catch is not
// in the guard at all -- the guard already refuses when switch_read_ok is false.
// It is in how that flag is MAINTAINED: a flag set only at the end of a
// successful read stays TRUE for the whole of a failed one, so the guard is
// handed "healthy, nothing active" for a bank nobody can see, and permits.
//
// The direct-GPIO path on this branch cannot fail, so the failure is injected
// (see FC_SWITCHES_TEST_FAULT_INJECTION).  The MCP23017 path returns exactly
// this way on an I2C error, so what these tests pin down is precisely the merge
// hazard: they fail against a sensor_switches() that only sets the flag at the
// end, and pass against one that clears it on entry.
// ---------------------------------------------------------------------------

/* Build the guard's input the way HMTL_Fire_Control_API.cpp's
 * fc_ota_guard_input() does -- from fc_switch_raw() and fc_switches_read_ok(),
 * never from fc_switch_state() or fc_is_armed(). */
static fc_ota_guard_input_t input_from_sensors() {
    fc_ota_guard_input_t in;
    for (uint8_t i = 0; i < FC_NUM_SWITCHES; i++) {
        in.switch_active[i] = fc_switch_raw(i);
    }
    in.switch_read_ok  = fc_switches_read_ok();
    in.snapshot_valid  = true;
    in.snapshot_age_ms = 0;
    return in;
}

void test_failed_read_after_a_good_one_refuses_end_to_end() {
    /*
     * The exact sequence that produced the bug: a good read (which sets the
     * health flag), then a failed one.  If the flag is not cleared on entry it
     * is still TRUE here, every switch reads open because that is the ignition
     * fail-safe, and the guard permits an upload on a controller whose switch
     * bank is dead.
     */
    sensor_switches();
    TEST_ASSERT_TRUE_MESSAGE(fc_switches_read_ok(),
                             "a completed read must report healthy");
    TEST_ASSERT_EQUAL_MESSAGE(FC_OTA_PERMIT, evaluate(input_from_sensors()),
                              "idle and healthy should permit");

    fc_test_set_switch_bank_read_fails(true);
    _mock_millis += 10;
    sensor_switches();               /* first failure -- inside the debounce window */
    _mock_millis += SWITCH_READ_FAIL_DEBOUNCE_MS + 10;
    sensor_switches();               /* sustained -- only now is it a fault */

    TEST_ASSERT_FALSE_MESSAGE(fc_switches_read_ok(),
                              "a FAILED read must not leave the previous "
                              "read's health flag standing");
    TEST_ASSERT_FALSE_MESSAGE(fc_any_switch_raw_active(),
                              "the fail-safe reports every switch open -- "
                              "which is why the health flag is the only thing "
                              "standing between this and a permit");
    TEST_ASSERT_EQUAL_MESSAGE(FC_OTA_REFUSE_SWITCH_READ_FAILED,
                              evaluate(input_from_sensors()),
                              "an unreadable switch bank must REFUSE, never "
                              "permit");
}

void test_failed_read_clears_stale_raw_switch_values() {
    /*
     * A switch is closed, then the bank stops reading.  The raw values must not
     * survive the failure either: they are what a refusal reason quotes, and a
     * reason built from the last good sample describes a panel nobody has been
     * able to see since.  (Health is what refuses; this keeps the refusal
     * honest about why.)
     */
    set_pin_value(test_switch_pins[0], 0);   /* active-low: closed */
    sensor_switches();
    TEST_ASSERT_TRUE(fc_switch_raw(0));

    fc_test_set_switch_bank_read_fails(true);
    _mock_millis += 10;
    sensor_switches();               /* first failure -- inside the debounce window */
    _mock_millis += SWITCH_READ_FAIL_DEBOUNCE_MS + 10;
    sensor_switches();               /* sustained -- only now is it a fault */

    TEST_ASSERT_FALSE(fc_switch_raw(0));
    TEST_ASSERT_FALSE(fc_switches_read_ok());
    TEST_ASSERT_EQUAL(FC_OTA_REFUSE_SWITCH_READ_FAILED,
                      evaluate(input_from_sensors()));
}

void test_health_returns_only_after_a_read_completes_again() {
    /* Recovery is not automatic and not sticky-false: the flag tracks THIS
     * call, so a bus that comes back permits again on the next good read. */
    fc_test_set_switch_bank_read_fails(true);
    sensor_switches();
    TEST_ASSERT_FALSE(fc_switches_read_ok());

    fc_test_set_switch_bank_read_fails(false);
    _mock_millis += 10;
    sensor_switches();
    TEST_ASSERT_TRUE(fc_switches_read_ok());
    TEST_ASSERT_EQUAL(FC_OTA_PERMIT, evaluate(input_from_sensors()));
}

void test_failed_read_does_not_arm() {
    /* The ignition fail-safe still points the way it should: unreadable means
     * not armed.  Fixing the OTA direction must not disturb this one. */
    sensor_switches();               /* all open; starts the qualification */
    _mock_millis += 2000;
    sensor_switches();               /* every switch now counts as seen-open */
    set_pin_value(test_switch_pins[POOFER_ENABLE_SWITCH], 0);
    set_pin_value(test_switch_pins[POOFER_PILOT_SWITCH], 0);
    _mock_millis += 10;
    sensor_switches();
    TEST_ASSERT_TRUE_MESSAGE(fc_is_armed(), "precondition: armed");

    fc_test_set_switch_bank_read_fails(true);
    _mock_millis += 10;
    sensor_switches();               /* first failure -- inside the debounce window */
    _mock_millis += SWITCH_READ_FAIL_DEBOUNCE_MS + 10;
    sensor_switches();               /* sustained -- only now is it a fault */
    TEST_ASSERT_FALSE_MESSAGE(fc_is_armed(),
                              "an unreadable bank must drop the armed state");
}


/*
 * The debounce, and why it exists.
 *
 * Measured on the bench 2026-08-21: the expander bus glitches at random -- one
 * failed transaction in roughly five hundred, recovering on the very next read.
 * Tripping the fail-safe on one of those forced every switch open several times
 * a minute and could refuse an OTA at random. Milliseconds do not matter for
 * switches, so a lone glitch must change nothing.
 */
void test_a_single_failed_read_does_not_refuse_an_upload() {
    _mock_millis += 10;
    sensor_switches();                          /* one good read to start from */
    TEST_ASSERT_TRUE(fc_switches_read_ok());

    fc_test_set_switch_bank_read_fails(true);
    _mock_millis += 10;
    sensor_switches();                          /* one glitch, inside the window */

    TEST_ASSERT_TRUE_MESSAGE(fc_switches_read_ok(),
        "a lone glitch must not refuse an upload");

    fc_test_set_switch_bank_read_fails(false);  /* bus recovers */
    _mock_millis += 10;
    sensor_switches();
    TEST_ASSERT_TRUE(fc_switches_read_ok());
}

/*
 * The fail-safe is driven by OBSERVATIONS, not by the wall clock.  One failed
 * read can never trip it however long ago it happened, because the failure
 * clock is stamped on that first failure and only a LATER failed read can cross
 * the threshold.  This is what makes a separate "at least N failures" counter
 * unnecessary -- asserted here rather than only argued in a comment, because an
 * inert guard and a working one look identical from the source.
 */
void test_elapsed_time_alone_never_trips_the_fail_safe() {
    _mock_millis += 10;
    sensor_switches();                          /* one good read to start from */

    fc_test_set_switch_bank_read_fails(true);
    _mock_millis += 10;
    sensor_switches();                          /* the one and only failed read */
    TEST_ASSERT_TRUE(fc_switches_read_ok());

    /* Time passes far beyond the window, but nothing READS the bank. */
    _mock_millis += 10 * SWITCH_READ_FAIL_DEBOUNCE_MS;
    TEST_ASSERT_TRUE_MESSAGE(fc_switches_read_ok(),
        "elapsed time alone must not trip a fail-safe; only an observation can");

    sensor_switches();                          /* the SECOND failure trips it */
    TEST_ASSERT_FALSE_MESSAGE(fc_switches_read_ok(),
        "a second failed read past the window is a fault");
}


/*
 * The window measures CONTINUOUS failure.  A good read in between must reset
 * it, or a slow drip of unrelated glitches -- which is exactly what this bus
 * produces -- would eventually accumulate into a spurious fault.
 */
void test_glitches_separated_by_good_reads_never_accumulate() {
    _mock_millis += 10;
    sensor_switches();

    for (int i = 0; i < 6; i++) {
        fc_test_set_switch_bank_read_fails(true);
        _mock_millis += 900;                    /* just under the window ... */
        sensor_switches();
        /*
         * Asserted HERE, on the failed read itself.  Checked after the good
         * read below it would prove nothing: that read sets the flag true
         * unconditionally and would mask a trip entirely.
         */
        TEST_ASSERT_TRUE_MESSAGE(fc_switches_read_ok(),
            "intermittent glitches must never accumulate into a fault");

        fc_test_set_switch_bank_read_fails(false);
        _mock_millis += 10;
        sensor_switches();                      /* ... then a good read */
    }
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_permits_only_when_everything_is_healthy_and_idle);

    RUN_TEST(test_each_switch_individually_blocks);
    RUN_TEST(test_a_single_failed_read_does_not_refuse_an_upload);
    RUN_TEST(test_elapsed_time_alone_never_trips_the_fail_safe);
    RUN_TEST(test_glitches_separated_by_good_reads_never_accumulate);
    RUN_TEST(test_refusal_names_the_blocking_switch);
    RUN_TEST(test_non_arming_switch_alone_blocks);
    RUN_TEST(test_all_switches_active_blocks);

    RUN_TEST(test_failed_switch_read_refuses_even_though_nothing_looks_active);
    RUN_TEST(test_no_snapshot_refuses);
    RUN_TEST(test_stale_snapshot_refuses);
    RUN_TEST(test_snapshot_exactly_at_the_bound_is_still_fresh);
    RUN_TEST(test_health_is_reported_before_switch_activity);

    RUN_TEST(test_missing_safe_state_ack_refuses);
    RUN_TEST(test_safe_state_ack_alone_does_not_override_a_refusal);
    RUN_TEST(test_begin_permits_only_with_ack_and_health);

    RUN_TEST(test_only_one_input_combination_permits);
    RUN_TEST(test_every_verdict_has_a_distinct_reason_string);

    /* End-to-end through sensor_switches() -- the permit-on-fault regression */
    RUN_TEST(test_failed_read_after_a_good_one_refuses_end_to_end);
    RUN_TEST(test_failed_read_clears_stale_raw_switch_values);
    RUN_TEST(test_health_returns_only_after_a_read_completes_again);
    RUN_TEST(test_failed_read_does_not_arm);

    return UNITY_END();
}
