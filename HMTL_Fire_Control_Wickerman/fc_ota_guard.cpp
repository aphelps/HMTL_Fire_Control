/*******************************************************************************
 * Author: Adam Phelps
 * License: Create Commons Attribution-Non-Commercial
 * Copyright: 2026
 *
 * OTA admission decision.  See fc_ota_guard.h -- in particular the note that
 * this guard's fail-safe direction is the INVERSE of the ignition path's, and
 * that it must refuse on absence of evidence rather than permit on it.
 ******************************************************************************/

#include "fc_ota_guard.h"

fc_ota_verdict_t fc_ota_evaluate(const fc_ota_guard_input_t &in,
                                 uint32_t max_age_ms,
                                 int8_t *blocking_switch) {
  if (blocking_switch) {
    *blocking_switch = -1;
  }

  /*
   * Health first, activity second.  The order matters for the REASON reported,
   * not for the verdict: an unhealthy read produces switch states that are
   * fabrications, so reporting "no switches active" from them -- even as an
   * explanation -- would be misleading.  Refuse citing the fault instead.
   */
  if (!in.snapshot_valid) {
    return FC_OTA_REFUSE_NO_SNAPSHOT;
  }
  if (!in.switch_read_ok) {
    return FC_OTA_REFUSE_SWITCH_READ_FAILED;
  }
  if (in.snapshot_age_ms > max_age_ms) {
    return FC_OTA_REFUSE_STALE_SNAPSHOT;
  }

  /*
   * ANY active switch refuses -- not just the ENABLE+PILOT pair that
   * fc_is_armed() tests.  A switch that does not by itself arm the controller
   * still means an operator has their hands on the box, which is reason enough
   * not to reboot it into a new image.
   */
  for (uint8_t i = 0; i < FC_NUM_SWITCHES; i++) {
    if (in.switch_active[i]) {
      if (blocking_switch) {
        *blocking_switch = (int8_t)i;
      }
      return FC_OTA_REFUSE_SWITCH_ACTIVE;
    }
  }

  return FC_OTA_PERMIT;
}

fc_ota_verdict_t fc_ota_evaluate_begin(const fc_ota_guard_input_t &in,
                                       uint32_t max_age_ms,
                                       bool safe_state_acked,
                                       int8_t *blocking_switch) {
  fc_ota_verdict_t verdict = fc_ota_evaluate(in, max_age_ms, blocking_switch);
  if (verdict != FC_OTA_PERMIT) {
    return verdict;
  }
  /*
   * Checked LAST, and checked separately from everything above, because it is
   * the only condition whose evidence is produced by asking rather than by
   * observing: core 1 must have actually run fc_all_outputs_safe() and said so.
   */
  if (!safe_state_acked) {
    return FC_OTA_REFUSE_SAFE_STATE_TIMEOUT;
  }
  return FC_OTA_PERMIT;
}

const char *fc_ota_verdict_str(fc_ota_verdict_t verdict) {
  switch (verdict) {
    case FC_OTA_PERMIT:
      return "permitted";
    case FC_OTA_REFUSE_NO_SNAPSHOT:
      return "no controller status has been published yet";
    case FC_OTA_REFUSE_SWITCH_READ_FAILED:
      return "the switch bank could not be read";
    case FC_OTA_REFUSE_STALE_SNAPSHOT:
      return "controller status is stale; the control loop may have stopped";
    case FC_OTA_REFUSE_SWITCH_ACTIVE:
      return "a switch is active";
    case FC_OTA_REFUSE_SAFE_STATE_TIMEOUT:
      return "the control loop did not confirm outputs were driven safe";
  }
  /* Unreachable for a valid enum value; refuse-shaped text by construction. */
  return "unknown refusal";
}
