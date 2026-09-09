/****************************************************************************
 * arch/arm/src/bk7258/include/bk7258_netstats.h
 *
 * One readable snapshot of everything a frame passes through between the
 * socket layer and the antenna, so that a throughput measurement can say
 * where the ceiling is instead of only that there is one.
 *
 * The AP core owns no radio.  Every frame in either direction crosses the
 * inter-core mailbox, which carries one transaction at a time across all of
 * its channels, and the console is one of those channels.  That last detail
 * is why these counters exist as a structure rather than a print: anything
 * that logs while traffic flows is measuring its own interference.  Read the
 * structure before and after a transfer and subtract.
 *
 * Reading the numbers:
 *
 *   wifi_rx_frames / wifi_rx_lists is how many frames the CP managed to
 *   chain into one mailbox transaction.  The ceiling is
 *   BK7258_WIFI_MAX_LIST (60).  Near the ceiling means the mailbox is
 *   working at full batch and is a genuine constraint; near 1 means it is
 *   idle between arrivals and the limit is upstream of it.
 *
 *   mb_timeout, mb_fifo_full and mb0_desc_full are the mailbox actually
 *   failing rather than merely being busy.  A transfer that is slow while
 *   all three stay flat was not slowed by the mailbox.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __VENDOR_BEKEN_CHIPS_BK7258_INCLUDE_BK7258_NETSTATS_H
#define __VENDOR_BEKEN_CHIPS_BK7258_INCLUDE_BK7258_NETSTATS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct bk7258_net_counters
{
  /* Mailbox transport layer: logical channels, acknowledgements and the
   * single in-flight transaction.  tx and rx count transactions, not bytes.
   */

  uint32_t mb_tx;
  uint32_t mb_rx;
  uint32_t mb_timeout;           /* MB_TIMEOUT (200 ms) expired            */
  uint32_t mb_fifo_full;         /* doorbell refused, sender must retry    */
  uint32_t mb_bad_ack;
  uint32_t mb_bad_header;
  uint32_t mb_ack_overflow;      /* MB_ACK_SLOT_COUNT exhausted            */
  uint32_t mb_deferred;          /* command waited behind the active one   */
  uint32_t mb_recovery_cycle;
  uint32_t mb_recovery_replay;
  uint32_t mb_link_ready;
  uint32_t mb_link_down;
  uint8_t  mb_link_state;
  uint8_t  mb_ack_slots_used;
  bool     mb_busy;              /* a transaction in flight right now      */

  /* Mailbox physical layer: the hardware FIFO and the receive descriptors.
   * mb0_desc_full is a message the AP could not even take delivery of.
   */

  uint32_t mb0_rx;
  uint32_t mb0_write_full;
  uint32_t mb0_write_error;
  uint32_t mb0_read_error;
  uint32_t mb0_desc_full;
  uint32_t mb0_desc_deferred;
  uint32_t mb0_bad_source;
  uint32_t mb0_bad_length;
  uint32_t mb0_bad_address;

  /* Wi-Fi outbound.  batches are mailbox transactions and frames are what
   * they carried, so frames/batches is the batching factor.  The three
   * refusal counters are frames the stack handed down that never reached
   * the air, each of which is otherwise silent.
   */

  uint32_t wifi_tx_batches;
  uint32_t wifi_tx_frames;
  uint32_t wifi_tx_enetdown;     /* link not ready, refused outright       */
  uint32_t wifi_tx_slots_full;   /* every TX slot still owned              */
  uint32_t wifi_tx_rejected;     /* CP answered -EREMOTEIO                 */
  uint8_t  wifi_tx_batch_max;

  /* Wi-Fi inbound.  lists are mailbox transactions carrying frame chains;
   * the per-transaction ceiling is BK7258_WIFI_MAX_LIST.
   */

  uint32_t wifi_rx_lists;
  uint32_t wifi_rx_frames;       /* frames the peer sent, only             */
  uint32_t wifi_tx_completions;  /* our own buffers coming back            */
  uint32_t wifi_rx_queue_full;   /* delivered but no room to queue it      */
  uint32_t wifi_rx_alloc_fail;   /* no read-ahead buffer or RX quota left  */
  uint8_t  wifi_rx_list_max;
  uint8_t  wifi_rx_list_ceiling; /* BK7258_WIFI_MAX_LIST, for scale        */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bk7258_net_get_counters
 *
 * Description:
 *   Fill in a snapshot of the mailbox and Wi-Fi counters.  Safe to call
 *   from a task; individual fields are read without a lock because every
 *   one of them is a word-sized monotonic counter and a torn read would at
 *   worst misplace a single event.
 *
 ****************************************************************************/

void bk7258_net_get_counters(struct bk7258_net_counters *counters);

/****************************************************************************
 * Name: bk7258_mailbox_fill_counters
 *
 * Description:
 *   Contribute the mb_* fields.  Declared here rather than in the mailbox
 *   header because the structure above is what it fills; it is an internal
 *   contributor to bk7258_net_get_counters() and not meant for callers.
 *
 ****************************************************************************/

void bk7258_mailbox_fill_counters(struct bk7258_net_counters *counters);

#endif /* __VENDOR_BEKEN_CHIPS_BK7258_INCLUDE_BK7258_NETSTATS_H */
