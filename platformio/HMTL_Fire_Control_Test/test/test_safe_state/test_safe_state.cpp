/*
 * Native unit tests for the core-0 -> core-1 safe-state handshake.
 *
 * The OTA guard consumes only this handshake's boolean result.  That means a
 * bug here is INVISIBLE to the guard: an ack that satisfied the wrong request
 * would hand it a confident "the outputs were driven safe" for a drive that
 * never happened, and every guard test would still pass.  It is also the one
 * part of the OTA path that cannot be reasoned about by reading, because its
 * whole subject is two cores racing.
 *
 * fc_safe_state.cpp takes its lock, its clock and its yield from an ops table
 * for exactly this reason: the mock below can make an ack arrive early, late,
 * out of order, or never, and can assert that the outputs are never driven with
 * the lock held.
 *
 *   cd platformio/HMTL_Fire_Control_Test
 *   pio test -e native --filter test_safe_state
 */

#include <unity.h>
#include <string.h>

#include "fc_safe_state.h"

extern "C" {
    void debug_log_begin_test(const char *name);
}

// ---------------------------------------------------------------------------
// Mock platform: counting lock, virtual clock
// ---------------------------------------------------------------------------

struct MockPlatform {
    int      lock_depth;      /* >0 while the lock is held */
    int      max_lock_depth;  /* must never exceed 1 */
    int      lock_count;
    int      unlock_count;
    uint32_t now;
    /* Advanced by wait_ms.  A test that wants an ack to arrive DURING the wait
     * installs a hook here, which is how "core 1 answered while core 0 was
     * polling" is staged without threads. */
    void   (*on_wait)(struct MockPlatform *m);
    fc_safe_state_t *state;
    int      drives;          /* times the drive callback ran */
    int      drive_lock_depth;/* lock depth observed inside the drive */
};

static MockPlatform g_mock;

static void mock_lock(void *ctx) {
    MockPlatform *m = (MockPlatform *)ctx;
    m->lock_depth++;
    if (m->lock_depth > m->max_lock_depth) m->max_lock_depth = m->lock_depth;
    m->lock_count++;
}
static void mock_unlock(void *ctx) {
    MockPlatform *m = (MockPlatform *)ctx;
    m->lock_depth--;
    m->unlock_count++;
}
static uint32_t mock_now(void *ctx) { return ((MockPlatform *)ctx)->now; }
static void mock_wait(void *ctx, uint32_t ms) {
    MockPlatform *m = (MockPlatform *)ctx;
    m->now += ms;
    if (m->on_wait) m->on_wait(m);
}

static fc_safe_state_ops_t g_ops;
static fc_safe_state_t     g_state;

static void mock_drive(void *ctx) {
    MockPlatform *m = (MockPlatform *)ctx;
    m->drives++;
    m->drive_lock_depth = m->lock_depth;
}

void setUp(void) {
    debug_log_begin_test(Unity.CurrentTestName);
    memset(&g_mock, 0, sizeof(g_mock));
    g_mock.state = &g_state;
    g_ops.lock    = mock_lock;
    g_ops.unlock  = mock_unlock;
    g_ops.now_ms  = mock_now;
    g_ops.wait_ms = mock_wait;
    g_ops.ctx     = &g_mock;
    fc_safe_state_init(&g_state, &g_ops);
}

void tearDown(void) {
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, g_mock.lock_depth,
                                  "every lock must be released");
    TEST_ASSERT_EQUAL_INT_MESSAGE(g_mock.lock_count, g_mock.unlock_count,
                                  "lock/unlock must be balanced");
    TEST_ASSERT_TRUE_MESSAGE(g_mock.max_lock_depth <= 1,
                             "the lock must never be taken recursively -- a "
                             "portMUX spinlock deadlocks if it is");
}

// ---------------------------------------------------------------------------
// The happy path
// ---------------------------------------------------------------------------

/* Core 1 services the request during core 0's poll wait. */
static void service_on_wait(MockPlatform *m) {
    fc_safe_state_service(m->state, mock_drive, m);
}

void test_request_serviced_during_the_wait_is_acked() {
    g_mock.on_wait = service_on_wait;
    TEST_ASSERT_TRUE(fc_safe_state_request(&g_state, 2000));
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, g_mock.drives,
                                  "exactly one drive for one request");
    TEST_ASSERT_EQUAL_UINT32(1, fc_safe_state_ack_seq(&g_state));
}

void test_service_does_nothing_without_a_request() {
    TEST_ASSERT_FALSE(fc_safe_state_service(&g_state, mock_drive, &g_mock));
    TEST_ASSERT_EQUAL_INT(0, g_mock.drives);
}

void test_service_is_idempotent_for_one_request() {
    g_mock.on_wait = service_on_wait;
    TEST_ASSERT_TRUE(fc_safe_state_request(&g_state, 2000));
    /* A second pass has nothing outstanding and must NOT re-drive: on the
     * device the drive is RS485 traffic on the flame path. */
    TEST_ASSERT_FALSE(fc_safe_state_service(&g_state, mock_drive, &g_mock));
    TEST_ASSERT_EQUAL_INT(1, g_mock.drives);
}

void test_drive_runs_outside_the_lock() {
    /*
     * The drive sends RS485.  Holding a portMUX spinlock across a bus
     * transaction would block the other core for its whole duration -- and the
     * other core is the one running the flame path.
     */
    g_mock.on_wait = service_on_wait;
    fc_safe_state_request(&g_state, 2000);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, g_mock.drive_lock_depth,
                                  "fc_all_outputs_safe() must not be called "
                                  "with the spinlock held");
}

// ---------------------------------------------------------------------------
// An ack satisfies only the request it answers
// ---------------------------------------------------------------------------

void test_stale_ack_from_a_prior_request_does_not_satisfy_a_newer_one() {
    /*
     * The reason this is a counter and not a flag.  Request 1 is serviced and
     * acked.  Request 2 is raised and core 1 never runs again.  The standing
     * ack (1) describes a safe state established BEFORE request 2 asked for
     * one -- switches may have moved since -- so it must not satisfy it.
     */
    g_mock.on_wait = service_on_wait;
    TEST_ASSERT_TRUE(fc_safe_state_request(&g_state, 2000));

    g_mock.on_wait = NULL;                 /* core 1 stops */
    TEST_ASSERT_FALSE_MESSAGE(fc_safe_state_request(&g_state, 100),
                              "a prior request's ack must not answer a new "
                              "request");
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, g_mock.drives,
                                  "no second drive ever ran");
}

void test_ack_of_a_request_raised_during_a_drive_is_not_granted_by_it() {
    /*
     * Core 0 raises request 2 while core 1's drive for request 1 is still
     * running.  The drive that is already in flight started from a state
     * reading taken before request 2 existed, so it cannot be its answer: the
     * service must ack only the sequence it began with, leaving request 2
     * outstanding.
     */
    fc_safe_state_request(&g_state, 0);    /* seq 1, times out immediately */
    TEST_ASSERT_EQUAL_UINT32(1, fc_safe_state_req_seq(&g_state));

    struct Raiser {
        static void drive(void *ctx) {
            MockPlatform *m = (MockPlatform *)ctx;
            m->drives++;
            m->drive_lock_depth = m->lock_depth;
            /* Core 0 asks again, mid-drive. */
            fc_safe_state_request(m->state, 0);
        }
    };
    TEST_ASSERT_TRUE(fc_safe_state_service(&g_state, Raiser::drive, &g_mock));

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(2, fc_safe_state_req_seq(&g_state),
                                     "the mid-drive request was recorded");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1, fc_safe_state_ack_seq(&g_state),
                                     "the in-flight drive acks only the "
                                     "request it started from");
    /* And that outstanding request still gets its own drive. */
    TEST_ASSERT_TRUE(fc_safe_state_service(&g_state, mock_drive, &g_mock));
    TEST_ASSERT_EQUAL_INT(2, g_mock.drives);
}

/* Core 1 acks a sequence AHEAD of the one core 0 is waiting on -- i.e. another
 * request overtook ours and its drive has already completed. */
static void overtaking_ack_on_wait(MockPlatform *m) {
    m->state->req_seq++;                       /* someone else asked */
    fc_safe_state_service(m->state, mock_drive, m);
}

void test_an_ack_ahead_of_our_sequence_satisfies_us() {
    /*
     * The one direction that IS safe, and the reason the comparison is >= and
     * not ==.  A drive that completed AFTER our request was raised has
     * established the state we asked for, whatever sequence it carries; an
     * equality test would sit there until the timeout even though the outputs
     * are demonstrably safe.  Refusing there would be merely annoying, but the
     * fix people reach for when a guard is annoying is to weaken it.
     */
    g_mock.on_wait = overtaking_ack_on_wait;
    TEST_ASSERT_TRUE(fc_safe_state_request(&g_state, 2000));   /* our seq is 1 */
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(2, fc_safe_state_ack_seq(&g_state),
                                     "the ack we accepted was for a LATER "
                                     "request than ours");
}

void test_an_abandoned_request_does_not_block_the_next_one() {
    /* A timed-out request leaves req_seq raised.  The next request must still
     * get a drive of its own rather than inheriting the abandoned one. */
    g_mock.on_wait = NULL;
    TEST_ASSERT_FALSE(fc_safe_state_request(&g_state, 0));      /* seq 1 */
    g_mock.on_wait = service_on_wait;
    TEST_ASSERT_TRUE(fc_safe_state_request(&g_state, 2000));    /* seq 2 */
    TEST_ASSERT_EQUAL_INT(1, g_mock.drives);
    TEST_ASSERT_EQUAL_UINT32(2, fc_safe_state_ack_seq(&g_state));
}

// ---------------------------------------------------------------------------
// Timeout: refuse, and latch nothing
// ---------------------------------------------------------------------------

void test_timeout_returns_false_when_core_1_never_answers() {
    g_mock.on_wait = NULL;
    TEST_ASSERT_FALSE(fc_safe_state_request(&g_state, 100));
    TEST_ASSERT_EQUAL_INT(0, g_mock.drives);
    TEST_ASSERT_TRUE_MESSAGE(g_mock.now > 100,
                             "it must actually have waited the bound out");
}

void test_timeout_latches_nothing_and_a_retry_succeeds() {
    /*
     * The plan's rule: a failed ack ABORTS the upload, it does not latch OTA
     * off until reboot.  So the very next request must be able to succeed once
     * core 1 is running again -- nothing is left disabled or stuck.
     */
    g_mock.on_wait = NULL;
    TEST_ASSERT_FALSE(fc_safe_state_request(&g_state, 100));

    g_mock.on_wait = service_on_wait;
    TEST_ASSERT_TRUE(fc_safe_state_request(&g_state, 2000));
    TEST_ASSERT_EQUAL_INT(1, g_mock.drives);
}

void test_zero_timeout_still_refuses_rather_than_permitting() {
    /* A degenerate bound must fail closed, not fall through the loop. */
    g_mock.on_wait = NULL;
    TEST_ASSERT_FALSE(fc_safe_state_request(&g_state, 0));
}

// ---------------------------------------------------------------------------
// Counter wrap
// ---------------------------------------------------------------------------

void test_sequence_wrap_still_compares_correctly() {
    /*
     * 2^32 requests is not reachable in practice, but the signed-difference
     * comparison is the kind of thing a later simplification turns into
     * `ack >= seq`, which breaks exactly here and only here.
     */
    g_state.req_seq = 0xFFFFFFFFUL;
    g_state.ack_seq = 0xFFFFFFFFUL;
    g_mock.on_wait = service_on_wait;
    TEST_ASSERT_TRUE(fc_safe_state_request(&g_state, 2000));  /* seq wraps to 0 */
    TEST_ASSERT_EQUAL_UINT32(0, fc_safe_state_ack_seq(&g_state));
    TEST_ASSERT_EQUAL_INT(1, g_mock.drives);
}

void test_stale_ack_across_the_wrap_does_not_satisfy() {
    g_state.req_seq = 0xFFFFFFFFUL;
    g_state.ack_seq = 0xFFFFFFFEUL;   /* an older, unanswered request */
    g_mock.on_wait = NULL;
    TEST_ASSERT_FALSE(fc_safe_state_request(&g_state, 100));  /* seq 0 */
}

// ---------------------------------------------------------------------------
// Defensive
// ---------------------------------------------------------------------------

void test_null_state_refuses() {
    /* Fail closed, not crash and not permit. */
    TEST_ASSERT_FALSE(fc_safe_state_request(NULL, 1000));
    TEST_ASSERT_FALSE(fc_safe_state_service(NULL, mock_drive, &g_mock));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_request_serviced_during_the_wait_is_acked);
    RUN_TEST(test_service_does_nothing_without_a_request);
    RUN_TEST(test_service_is_idempotent_for_one_request);
    RUN_TEST(test_drive_runs_outside_the_lock);

    RUN_TEST(test_stale_ack_from_a_prior_request_does_not_satisfy_a_newer_one);
    RUN_TEST(test_ack_of_a_request_raised_during_a_drive_is_not_granted_by_it);
    RUN_TEST(test_an_ack_ahead_of_our_sequence_satisfies_us);
    RUN_TEST(test_an_abandoned_request_does_not_block_the_next_one);

    RUN_TEST(test_timeout_returns_false_when_core_1_never_answers);
    RUN_TEST(test_timeout_latches_nothing_and_a_retry_succeeds);
    RUN_TEST(test_zero_timeout_still_refuses_rather_than_permitting);

    RUN_TEST(test_sequence_wrap_still_compares_correctly);
    RUN_TEST(test_stale_ack_across_the_wrap_does_not_satisfy);

    RUN_TEST(test_null_state_refuses);

    return UNITY_END();
}
