/*******************************************************************************
 * Author: Adam Phelps
 * License: Create Commons Attribution-Non-Commercial
 * Copyright: 2026
 *
 * See fc_safe_state.h.  No Arduino, no FreeRTOS, no ESP32 -- everything the
 * platform provides arrives through fc_safe_state_ops_t.
 ******************************************************************************/

#include "fc_safe_state.h"

void fc_safe_state_init(fc_safe_state_t *s, const fc_safe_state_ops_t *ops) {
  if (!s) return;
  s->req_seq = 0;
  s->ack_seq = 0;
  s->ops     = ops;
}

bool fc_safe_state_request(fc_safe_state_t *s, uint32_t timeout_ms) {
  if (!s || !s->ops) return false;
  const fc_safe_state_ops_t *o = s->ops;

  uint32_t seq;
  o->lock(o->ctx);
  seq = ++s->req_seq;
  o->unlock(o->ctx);

  uint32_t start = o->now_ms(o->ctx);
  for (;;) {
    uint32_t ack;
    o->lock(o->ctx);
    ack = s->ack_seq;
    o->unlock(o->ctx);

    /*
     * Signed difference, so a wrapped counter still compares.  >= rather than
     * == so an ack for a LATER request also satisfies this one: that drive
     * necessarily happened after ours was raised, so the outputs are safe.
     * The converse -- an ack from BEFORE our request -- is strictly less than
     * seq and correctly does not satisfy it.
     */
    if ((int32_t)(ack - seq) >= 0) {
      return true;
    }
    if ((uint32_t)(o->now_ms(o->ctx) - start) > timeout_ms) {
      return false;
    }
    o->wait_ms(o->ctx, 2);
  }
}

bool fc_safe_state_service(fc_safe_state_t *s,
                           void (*drive)(void *), void *drive_ctx) {
  if (!s || !s->ops) return false;
  const fc_safe_state_ops_t *o = s->ops;

  uint32_t req, ack;
  o->lock(o->ctx);
  req = s->req_seq;
  ack = s->ack_seq;
  o->unlock(o->ctx);

  if (req == ack) {
    return false;
  }

  /* Outside the lock -- see the header. */
  if (drive) drive(drive_ctx);

  /*
   * Acknowledge the request as it stood when the drive STARTED, not the
   * current req_seq: a request raised while the drive was running has not been
   * satisfied by it and must get its own drive on the next service pass.
   */
  o->lock(o->ctx);
  s->ack_seq = req;
  o->unlock(o->ctx);
  return true;
}

uint32_t fc_safe_state_req_seq(fc_safe_state_t *s) {
  if (!s || !s->ops) return 0;
  uint32_t v;
  s->ops->lock(s->ops->ctx);
  v = s->req_seq;
  s->ops->unlock(s->ops->ctx);
  return v;
}

uint32_t fc_safe_state_ack_seq(fc_safe_state_t *s) {
  if (!s || !s->ops) return 0;
  uint32_t v;
  s->ops->lock(s->ops->ctx);
  v = s->ack_seq;
  s->ops->unlock(s->ops->ctx);
  return v;
}
