/*******************************************************************************
 * Author: Adam Phelps
 * License: Create Commons Attribution-Non-Commercial
 * Copyright: 2026
 *
 * The core-0 -> core-1 safe-state handshake, as a dependency-free unit.
 *
 * This is the piece that turns "core 0 asked for the outputs to be driven safe"
 * into "core 1 actually ran fc_all_outputs_safe()".  The OTA guard consumes
 * only its boolean result, so if this logic is wrong the guard cannot tell --
 * it would be handed a confident `true` for a drive that never happened.  That
 * makes it worth testing directly, and on the host, where an ack can be made to
 * arrive late, early, or not at all.
 *
 * Hence the ops table: the ESP32 caller supplies portENTER_CRITICAL /
 * portEXIT_CRITICAL / millis() / vTaskDelay(), and a native test supplies a
 * counting mock lock and a virtual clock.  Nothing in this file includes
 * Arduino.h or FreeRTOS, so it compiles anywhere.
 *
 * Why a sequence counter and not a flag: a flag raised while a previous drive
 * is still in progress can be satisfied by that older drive completing, which
 * would report a safe state established BEFORE the request that asked for it.
 * Signed comparison of (ack - seq) is used throughout so a counter that wraps
 * at 2^32 still compares correctly.
 ******************************************************************************/

#ifndef FC_SAFE_STATE_H
#define FC_SAFE_STATE_H

#include <stdint.h>

/*
 * Platform services.  Deliberately a tiny table rather than #ifdef ESP32
 * inside the logic: the point is that the logic below has no platform in it
 * at all, so what a native test exercises is the same code the device runs.
 *
 * lock/unlock must be a real mutual exclusion between the requesting core and
 * the servicing core.  now_ms must be monotonic; wrap is handled by unsigned
 * subtraction at the use site.  wait_ms is called only while polling and may
 * be a no-op, but on a real scheduler it must actually yield.
 */
struct fc_safe_state_ops_t {
  void     (*lock)(void *ctx);
  void     (*unlock)(void *ctx);
  uint32_t (*now_ms)(void *ctx);
  void     (*wait_ms)(void *ctx, uint32_t ms);
  void      *ctx;
};

struct fc_safe_state_t {
  /* Bumped by the requester (core 0). */
  uint32_t req_seq;
  /* Published by the servicer (core 1) once a drive has COMPLETED. */
  uint32_t ack_seq;
  const fc_safe_state_ops_t *ops;
};

void fc_safe_state_init(fc_safe_state_t *s, const fc_safe_state_ops_t *ops);

/*
 * Requester side.  Raises a new request and polls until it is acknowledged or
 * timeout_ms elapses.
 *
 * Returns false on timeout, which the caller must treat as a hard refusal:
 * core 1 not answering means it is stopped or busy in a way we cannot account
 * for, and neither is a state to start rewriting flash in.  NOTHING IS LATCHED
 * on failure -- the next request starts clean, so a refused upload is simply
 * retried.  (The abandoned request stays in req_seq, so a late ack for it can
 * never be mistaken for an answer to a later request.)
 */
bool fc_safe_state_request(fc_safe_state_t *s, uint32_t timeout_ms);

/*
 * Servicer side.  If a request is outstanding, calls drive(drive_ctx) and then
 * publishes the acknowledgement.  Returns true if a drive was run.
 *
 * drive() is invoked with the lock NOT held: on the device it sends RS485
 * traffic, and holding a portMUX spinlock across that would block the other
 * core for the length of a bus transaction.
 */
bool fc_safe_state_service(fc_safe_state_t *s,
                           void (*drive)(void *), void *drive_ctx);

/* Test/diagnostic accessors; take the lock like everything else. */
uint32_t fc_safe_state_req_seq(fc_safe_state_t *s);
uint32_t fc_safe_state_ack_seq(fc_safe_state_t *s);

#endif // FC_SAFE_STATE_H
