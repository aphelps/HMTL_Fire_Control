/*******************************************************************************
 * Author: Adam Phelps
 * License: Create Commons Attribution-Non-Commercial
 * Copyright: 2026
 *
 * The admission decision for an over-the-air firmware upload.
 *
 * This is deliberately a SEPARATE, dependency-free translation unit rather than
 * a few conditions inlined into the HTTP handler.  The handler lives in
 * ESP32-only code that cannot be compiled on the host, and this decision is the
 * one piece of the OTA path where being wrong is dangerous rather than merely
 * broken -- so it is kept where a native unit test can exercise every branch,
 * including the ones that are hard to stage on real hardware.
 *
 * ---------------------------------------------------------------------------
 * THE FAIL-SAFE DIRECTION IS INVERTED HERE RELATIVE TO IGNITION.  READ THIS.
 * ---------------------------------------------------------------------------
 * Everywhere else in this firmware, "we could not read the switches" resolves
 * to "all switches open, do not arm".  That is correct for ignition: an
 * unreadable switch bank must not light a fire.
 *
 * For OTA that same convention is a silent permit-on-fault.  This guard's
 * question is "is anything active?", and a fault that reports every switch as
 * inactive answers "no, nothing is active" -- so an unreadable switch bank,
 * or a core 1 that has stopped publishing, would PERMIT an upload on a
 * controller whose real state is unknown.
 *
 * The guard therefore requires POSITIVE EVIDENCE OF HEALTH and refuses on its
 * absence.  Permitting requires ALL of:
 *   - a snapshot has actually been published,          (snapshot_valid)
 *   - the last switch read succeeded,                  (switch_read_ok)
 *   - that snapshot is recent,                         (snapshot_age_ms)
 *   - every switch is inactive.                        (switch_active[])
 * Any of these missing is a refusal.  "Nothing looked active" is NOT sufficient
 * on its own and must never become sufficient.
 *
 * Note also that switch_active[] must be filled from the RAW switch state
 * (fc_switch_raw()), not the interlock-qualified state (fc_switch_state()) and
 * not fc_is_armed().  Two reasons:
 *   - fc_is_armed() is only ENABLE && PILOT, so it ignores the igniter and
 *     program-mode switches entirely; the requirement is that ANY active
 *     switch blocks, which is strictly stronger than "armed blocks".
 *   - the qualified state reports a switch that has been closed since boot as
 *     inactive (it has not yet been seen open), which is again the ignition
 *     fail-safe pointing the wrong way for this question.
 ******************************************************************************/

#ifndef FC_OTA_GUARD_H
#define FC_OTA_GUARD_H

#include <stdint.h>

#include "Fire_Control_Sensors.h"  /* FC_NUM_SWITCHES */

/*
 * How stale the cross-core snapshot may be and still count as evidence that
 * core 1 is alive.
 *
 * The snapshot is republished once per loop() iteration, which normally runs
 * far faster than this; the bound exists to catch a core 1 that has STOPPED,
 * not to measure jitter.  It is loose enough (one second) that ordinary loop
 * variation -- an LCD I2C update, a burst of RS485 traffic -- can never cause a
 * spurious refusal, because a guard that cries wolf gets worked around.
 */
#ifndef FC_OTA_MAX_SNAPSHOT_AGE_MS
  #define FC_OTA_MAX_SNAPSHOT_AGE_MS 1000
#endif

/*
 * How long core 0 waits for core 1 to acknowledge the safe-state request.
 * loop() services the request every iteration, so this is orders of magnitude
 * more than needed; exceeding it means core 1 is not running, which is a
 * refusal, not something to flash through.
 */
#ifndef FC_OTA_SAFE_STATE_TIMEOUT_MS
  #define FC_OTA_SAFE_STATE_TIMEOUT_MS 2000
#endif

/*
 * How long an upload may go with NO data arriving before the connection is
 * closed underneath it.
 *
 * This is an INACTIVITY bound, not a total-duration one, and the distinction is
 * the point: a large image over a weak link may legitimately take a long time
 * overall, and cutting that off is the failure this exists to avoid.  What is
 * never legitimate is five minutes of complete silence on a connection that is
 * holding the API task.
 *
 * Without it the wait is UNBOUNDED.  Arduino's WebServer reads the body with
 *   while (!client.available() && client.connected()) delay(2);
 * (`WebServer/src/Parsing.cpp`, `_uploadReadByte`), which spins forever against
 * a peer that is still connected but sending nothing.  It yields, so the task
 * watchdog never fires -- the API simply stops serving, with nothing to notice.
 * A firmware upload is the one endpoint that holds a connection open by design,
 * so it is the one that must bound the wait itself.
 */
#ifndef FC_OTA_UPLOAD_IDLE_TIMEOUT_MS
  #define FC_OTA_UPLOAD_IDLE_TIMEOUT_MS (5 * 60 * 1000)
#endif

enum fc_ota_verdict_t {
  FC_OTA_PERMIT = 0,
  /* No snapshot has ever been published -- core 1 has not got that far. */
  FC_OTA_REFUSE_NO_SNAPSHOT,
  /* The switch bank could not be read.  The permit-on-fault case. */
  FC_OTA_REFUSE_SWITCH_READ_FAILED,
  /* Core 1 has stopped publishing; its reported state may be arbitrarily old. */
  FC_OTA_REFUSE_STALE_SNAPSHOT,
  /* At least one switch is physically active. */
  FC_OTA_REFUSE_SWITCH_ACTIVE,
  /* Core 1 did not acknowledge the safe-state request in time. */
  FC_OTA_REFUSE_SAFE_STATE_TIMEOUT,
};

/*
 * What core 0 knows about core 1's state.  Filled from the portMUX-protected
 * snapshot -- never by reading the sensor globals across cores, which have no
 * synchronisation at all.
 */
struct fc_ota_guard_input_t {
  /* RAW (pre-interlock) switch states.  See the header comment. */
  bool     switch_active[FC_NUM_SWITCHES];
  /* The last switch read succeeded. */
  bool     switch_read_ok;
  /* A snapshot has been published at least once. */
  bool     snapshot_valid;
  /* Age of that snapshot, in milliseconds. */
  uint32_t snapshot_age_ms;
};

/*
 * Evaluate the controller-state half of the decision.
 *
 * Returns FC_OTA_PERMIT only when every health condition holds AND no switch is
 * active.  When the verdict is FC_OTA_REFUSE_SWITCH_ACTIVE and blocking_switch
 * is non-null, it receives the index of the lowest-numbered active switch, so a
 * refusal can name what is blocking it rather than being an opaque "no".
 * blocking_switch is set to -1 for every other verdict.
 */
fc_ota_verdict_t fc_ota_evaluate(const fc_ota_guard_input_t &in,
                                 uint32_t max_age_ms,
                                 int8_t *blocking_switch);

/*
 * The complete admission decision for actually beginning a flash: everything
 * fc_ota_evaluate() checks, plus core 1's acknowledgement that it has driven
 * the outputs safe.
 *
 * A missing acknowledgement ABORTS the upload -- it does not latch OTA off
 * until reboot.  The upload simply fails and can be retried, which is the right
 * trade for a controller that may be in a field with no serial access.
 */
fc_ota_verdict_t fc_ota_evaluate_begin(const fc_ota_guard_input_t &in,
                                       uint32_t max_age_ms,
                                       bool safe_state_acked,
                                       int8_t *blocking_switch);

/* Stable, human-readable reason string; safe to embed in an HTTP response. */
const char *fc_ota_verdict_str(fc_ota_verdict_t verdict);

#endif // FC_OTA_GUARD_H
