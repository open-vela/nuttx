/****************************************************************************
 * arch/arm/src/bk7258/include/bk7258_heartbeat.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_BK7258_INCLUDE_BK7258_HEARTBEAT_H
#define __ARCH_ARM_SRC_BK7258_INCLUDE_BK7258_HEARTBEAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

struct bk7258_heartbeat_status
{
  bool started;
  bool sending;
  uint32_t attempts;
  uint32_t acknowledgements;
  uint32_t failures;
  int last_result;
  uint64_t since_attempt_ms;
  uint64_t since_ack_ms;       /* Meaningful only when acknowledgements > 0. */
  uint64_t max_attempt_gap_ms;
  uint64_t max_send_ms;
  uint32_t mailbox_tx;
  uint32_t mailbox_rx;
  uint32_t mailbox_timeouts;
  uint32_t mailbox_bad_ack;
  uint32_t mailbox_recoveries;
  uint32_t mailbox_down;
  uint8_t mailbox_state;
};

/* Copies counters only: no console output, allocation, or mailbox request. */

void bk7258_heartbeat_get_status(struct bk7258_heartbeat_status *status);

#endif
