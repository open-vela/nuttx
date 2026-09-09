/****************************************************************************
 * arch/arm/src/bk7258/bk7258_mb_uart.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * BK7258 full-duplex mailbox UART0.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>

#include "hardware/bk7258_mbox.h"

#ifndef CONFIG_BK7258_MB_UART0_TXBUFSIZE
#  define CONFIG_BK7258_MB_UART0_TXBUFSIZE 8192
#endif

#ifndef CONFIG_BK7258_MB_UART0_RXBUFSIZE
#  define CONFIG_BK7258_MB_UART0_RXBUFSIZE 256
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MB_UART_TX_SIZE       CONFIG_BK7258_MB_UART0_TXBUFSIZE

/* Bounds for bk7258_mbox_uart_drain_polled().  Every loop it runs has to
 * terminate on its own, because the caller is a crash path: there is no
 * scheduler to preempt it and nothing left to recover if it spins forever.
 *
 * CHUNKS covers a whole ring at BK7258_MB_UART_CHUNK_SIZE per frame with
 * room
 * to spare.  FIFO_SPINS is generous because a full peer FIFO drains on the
 * CP's own interrupt.  SETTLE_SPINS is the gap between frames, discussed at
 * the function.
 */

#define MB_UART_POLLED_CHUNKS       64u
#define MB_UART_POLLED_FIFO_SPINS   200000u
#define MB_UART_POLLED_SETTLE_SPINS 20000u
#define MB_UART_RX_CAPACITY   CONFIG_BK7258_MB_UART0_RXBUFSIZE
#define MB_UART_RX_SIZE       (MB_UART_RX_CAPACITY + 1u)
#define MB_UART_RTS_ASSERT    BK7258_MB_UART_CHUNK_SIZE
#define MB_UART_RTS_RELEASE   ((MB_UART_RX_CAPACITY * 40u) / 100u)
#define MB_UART_WORK_INTERVAL MSEC2TICK(10)
#define MB_UART_STATUS_OVF    0x01u
#define MB_UART_STATUS_PARITY 0x02u
#define MB_UART_PRIORITY      106

struct mb_uart_stats
{
  uint32_t write_calls;
  uint32_t write_bytes;
  uint32_t short_writes;
  uint32_t data_tx;
  uint32_t data_rx;
  uint32_t state_tx;
  uint32_t state_rx;
  uint32_t rx_bytes;
  uint32_t rx_overflow;
  uint32_t bad_pointer;
  uint32_t bad_length;
  uint32_t bad_flags;
  uint32_t bad_crc;
  uint32_t bad_reserved;
  uint32_t remote_fail;
  uint32_t tx_retry;
  uint32_t tx_retry_drop;
  uint32_t tx_unknown;
  uint32_t resets;
  uint32_t rts_assert;
  uint32_t rts_release;
};

struct mb_uart_control
{
  uint8_t tx_ring[MB_UART_TX_SIZE];
  uint8_t rx_ring[MB_UART_RX_SIZE];
  uint16_t tx_read;
  uint16_t tx_write;
  uint16_t rx_read;
  uint16_t rx_write;
  uint16_t tx_inflight;
  uint8_t tx_retry_count;
  uint8_t sticky_status;
  bool initialized;
  bool transaction_busy;
  uint8_t transaction_command;
  bool state_pending;
  bool local_rts;
  bool upper_rts;
  bool peer_rts;
  bool peer_rts_known;
  sem_t worker_sem;
  bk7258_mb_uart_callback_t callback;
  void *callback_arg;
  struct mb_uart_stats stats;
};

static struct mb_uart_control g_uart;

/* The frame bk7258_mbox_uart_drain_polled() submits, and its sequence
 * counter.  A file static rather than a local because the peer reads the
 * message out of this memory after the call that queued it has returned --
 * the same reason the transport's own slots are statics.  Aligned to 32 to
 * match them.
 */

static struct bk7258_mb_wire_message g_polled_message
  __attribute__((aligned(32)));
static uint8_t g_polled_seq;

/* Set once, by bk7258_mbox_uart_crash_mode(), and never cleared. */

static volatile bool g_crash_mode;

static_assert(MB_UART_TX_SIZE > BK7258_MB_UART_CHUNK_SIZE &&
               MB_UART_TX_SIZE <= UINT16_MAX,
               "mailbox UART TX ring index does not fit uint16_t");
static_assert(MB_UART_RX_SIZE > BK7258_MB_UART_CHUNK_SIZE &&
               MB_UART_RX_SIZE <= UINT16_MAX,
               "mailbox UART RX ring index does not fit uint16_t");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t ring_count(uint16_t read_index, uint16_t write_index,
                           uint16_t size)
{
  return write_index >= read_index ? write_index - read_index :
         size - read_index + write_index;
}

static uint16_t tx_count(void)
{
  return ring_count(g_uart.tx_read, g_uart.tx_write, MB_UART_TX_SIZE);
}

static uint16_t rx_count(void)
{
  return ring_count(g_uart.rx_read, g_uart.rx_write, MB_UART_RX_SIZE);
}

static uint16_t tx_free(void)
{
  return MB_UART_TX_SIZE - tx_count() - 1u;
}

static uint16_t rx_free(void)
{
  return MB_UART_RX_CAPACITY - rx_count();
}

static void update_local_rts(void)
{
  bool new_rts = g_uart.upper_rts || rx_free() <= MB_UART_RTS_ASSERT;

  if (new_rts != g_uart.local_rts)
    {
      g_uart.local_rts = new_rts;
      g_uart.state_pending = true;
      if (new_rts)
        {
          g_uart.stats.rts_assert++;
        }
      else
        {
          g_uart.stats.rts_release++;
        }
    }
}

static bool valid_ack(const struct bk7258_mb_wire_message *ack)
{
  /* Armino's mb_uart response defines only the low byte of ack_data1:
   * uart_rts and uart_tx_fail.  The remaining bytes are remnants of the
   * command buffer and are not initialized by the CP response handler.
   * Header matching and the transport state checks already validate the
   * response ownership, so do not interpret those bytes as a wire ACK shape.
   */

  return ack != NULL;
}

static void mb_uart_tx_complete(const struct bk7258_mb_wire_message *ack,
                                int result, void *arg)
{
  uint8_t command;
  bool failed;

  (void)arg;
  /* The mailbox transport invokes completion callbacks with interrupts
   * disabled, so this callback updates UART state directly.
   */

  command = ack == NULL ? g_uart.transaction_command :
            bk7258_mb_header_cmd(ack);
  failed = result < 0 || !valid_ack(ack) ||
           (ack != NULL && (ack->payload_address & 2u) != 0);

  if (ack != NULL && valid_ack(ack))
    {
      g_uart.peer_rts = (ack->payload_address & 1u) != 0;
      g_uart.peer_rts_known = true;
    }

  if (result == -ETIMEDOUT || result == -ECONNRESET)
    {
      g_uart.peer_rts_known = false;
      g_uart.state_pending = true;
    }

  if (command == BK7258_MB_UART_DATA && g_uart.tx_inflight != 0)
    {
      if (!failed)
        {
          g_uart.tx_read = (g_uart.tx_read + g_uart.tx_inflight) %
                           MB_UART_TX_SIZE;
          g_uart.tx_retry_count = 0;
        }
      else if (result == -ETIMEDOUT || result == -ECONNRESET)
        {
          /* A failed/unknown ACK cannot tell whether CP consumed the bytes.
           * Drop this local chunk rather than replaying terminal characters.
           */

          g_uart.tx_read = (g_uart.tx_read + g_uart.tx_inflight) %
                           MB_UART_TX_SIZE;
          g_uart.stats.tx_unknown++;
          g_uart.tx_retry_count = 0;
          g_uart.stats.resets++;
        }
      else
        {
          g_uart.stats.remote_fail++;
          g_uart.state_pending = true;
          if (g_uart.tx_retry_count == 0)
            {
              /* Keep tx_read unchanged.  The same chunk may be copied again
               * once STATE recovery succeeds, but only once.
               */

              g_uart.tx_retry_count = 1;
              g_uart.stats.tx_retry++;
            }
          else
            {
              g_uart.tx_read = (g_uart.tx_read + g_uart.tx_inflight) %
                               MB_UART_TX_SIZE;
              g_uart.tx_retry_count = 0;
              g_uart.stats.tx_retry_drop++;
            }
        }

      g_uart.tx_inflight = 0;
    }
  else if (command == BK7258_MB_UART_STATE && failed)
    {
      g_uart.stats.remote_fail++;
      /* The transport invokes this completion before it changes READY to
       * ABORTING.  Always retain a STATE request so RESET recovery has a
       * probe to send after it reaches PROBING.
       */

      g_uart.state_pending = true;
    }

  g_uart.transaction_busy = false;
  nxsem_post(&g_uart.worker_sem);
}

static int mb_uart_receive(const struct bk7258_mb_wire_message *message,
                           uint8_t *ack_flags, void *arg)
{
  uint8_t command = bk7258_mb_header_cmd(message);
  uint16_t length = message->payload_length;
  uint16_t first;
  bool failed = false;
  bool malformed = false;

  (void)arg;
  if (ack_flags == NULL)
    {
      return -EINVAL;
    }

  if (command != BK7258_MB_UART_DATA && command != BK7258_MB_UART_STATE)
    {
      g_uart.stats.bad_flags++;
      failed = true;
      malformed = true;
    }
  else if ((message->flags & ~1u) != 0)
    {
      g_uart.stats.bad_flags++;
      failed = true;
      malformed = true;
    }
  else if (message->crc8 != 0)
    {
      g_uart.stats.bad_crc++;
      g_uart.sticky_status |= MB_UART_STATUS_PARITY;
      failed = true;
      malformed = true;
    }

  /* The CP command occupies only the first 12 bytes of the 16-byte mailbox
   * slot.  Bytes 12..15 are not part of mb_uart_cmd_t and may be stale.
   */

  if (!failed && command == BK7258_MB_UART_DATA)
    {
      if (message->payload_address != BK7258_MB_UART_RX_ADDRESS)
        {
          g_uart.stats.bad_pointer++;
          failed = true;
          malformed = true;
        }
      else if (length == 0 || length > BK7258_MB_UART_CHUNK_SIZE)
        {
          g_uart.stats.bad_length++;
          failed = true;
          malformed = true;
        }
      else if (!bk7258_mailbox_link_ready())
        {
          failed = true;
        }
      else if (rx_free() < length)
        {
          g_uart.stats.rx_overflow++;
          g_uart.sticky_status |= MB_UART_STATUS_OVF;
          failed = true;
        }
      else
        {
          const uint8_t *source =
            (const uint8_t *)(uintptr_t)BK7258_MB_UART_RX_ADDRESS;

          __asm__ volatile("dmb sy" ::: "memory");
          first = MB_UART_RX_SIZE - g_uart.rx_write;
          if (first > length)
            {
              first = length;
            }

          memcpy(&g_uart.rx_ring[g_uart.rx_write], source, first);
          if (first != length)
            {
              memcpy(g_uart.rx_ring, source + first, length - first);
            }

          g_uart.rx_write = (g_uart.rx_write + length) % MB_UART_RX_SIZE;
          g_uart.stats.data_rx++;
          g_uart.stats.rx_bytes += length;
        }
    }
  else if (!failed)
    {
      if (message->payload_address != 0 || message->payload_length != 0)
        {
          g_uart.stats.bad_length++;
          failed = true;
          malformed = true;
        }
      else
        {
          g_uart.stats.state_rx++;
        }
    }

  if (!malformed)
    {
      bool old_peer_rts = g_uart.peer_rts;

      g_uart.peer_rts = (message->flags & 1u) != 0;
      g_uart.peer_rts_known = true;
      if (old_peer_rts && !g_uart.peer_rts)
        {
          nxsem_post(&g_uart.worker_sem);
        }
    }

  update_local_rts();
  *ack_flags = (g_uart.local_rts ? 1u : 0u) | (failed ? 2u : 0u);
  nxsem_post(&g_uart.worker_sem);
  return malformed ? -EPROTO : OK;
}

static int send_state_locked(void)
{
  struct bk7258_mb_wire_message message;
  int ret;

  memset(&message, 0, sizeof(message));
  message.header = bk7258_mb_make_header(BK7258_MB_UART_STATE, 0, 0, 0,
                                         BK7258_MB_CHAN_UART0_TX);
  message.flags = g_uart.local_rts ? 1u : 0u;
  ret = bk7258_mailbox_send_wire(BK7258_MB_CHAN_UART0_TX, &message,
                                  mb_uart_tx_complete, NULL);
  if (ret == OK)
    {
      g_uart.transaction_busy = true;
      g_uart.transaction_command = BK7258_MB_UART_STATE;
      g_uart.state_pending = false;
      g_uart.stats.state_tx++;
    }

  return ret;
}

static int send_data_locked(void)
{
  struct bk7258_mb_wire_message message;
  uint8_t *destination =
    (uint8_t *)(uintptr_t)BK7258_MB_UART_TX_ADDRESS;
  uint16_t length = tx_count();
  uint16_t first;
  int ret;

  if (length > BK7258_MB_UART_CHUNK_SIZE)
    {
      length = BK7258_MB_UART_CHUNK_SIZE;
    }

  if (length == 0 || g_uart.peer_rts)
    {
      return -EAGAIN;
    }

  first = MB_UART_TX_SIZE - g_uart.tx_read;
  if (first > length)
    {
      first = length;
    }

  memcpy(destination, &g_uart.tx_ring[g_uart.tx_read], first);
  if (first != length)
    {
      memcpy(destination + first, g_uart.tx_ring, length - first);
    }

  memset(&message, 0, sizeof(message));
  message.header = bk7258_mb_make_header(BK7258_MB_UART_DATA, 0, 0, 0,
                                         BK7258_MB_CHAN_UART0_TX);
  message.payload_address = BK7258_MB_UART_TX_ADDRESS;
  message.payload_length = length;
  message.flags = g_uart.local_rts ? 1u : 0u;
  __asm__ volatile("dmb sy" ::: "memory");
  ret = bk7258_mailbox_send_wire(BK7258_MB_CHAN_UART0_TX, &message,
                                  mb_uart_tx_complete, NULL);
  if (ret == OK)
    {
      g_uart.transaction_busy = true;
      g_uart.transaction_command = BK7258_MB_UART_DATA;
      g_uart.tx_inflight = length;
      g_uart.stats.data_tx++;
    }

  return ret;
}

static int mb_uart_worker(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  for (; ; )
    {
      bk7258_mb_uart_callback_t callback;
      void *callback_arg;
      irqstate_t flags;
      enum bk7258_mb_link_state link;

      (void)nxsem_tickwait_uninterruptible(&g_uart.worker_sem,
                                           MB_UART_WORK_INTERVAL);
      nxsem_reset(&g_uart.worker_sem, 0);
      link = bk7258_mailbox_link_state();
      flags = up_irq_save();

      if (!g_uart.transaction_busy)
        {
          if (link == BK7258_MB_LINK_PROBING && g_uart.state_pending)
            {
              (void)send_state_locked();
            }
          else if (link == BK7258_MB_LINK_READY)
            {
              if (g_uart.state_pending)
                {
                  (void)send_state_locked();
                }
              else
                {
                  (void)send_data_locked();
                }
            }
        }

      callback = g_uart.callback;
      callback_arg = g_uart.callback_arg;
      up_irq_restore(flags);

      if (callback != NULL)
        {
          callback(callback_arg);
        }
    }

  return OK;
}

void bk7258_mbox_uart_early_init(void)
{
  /* The zero-initialized TX ring accepts bounded early logs before MBOX0 is
   * available.  Full rings return an accurate short write.
   */
}

int bk7258_mb_uart_init(void)
{
  int ret;

  if (g_uart.initialized)
    {
      return OK;
    }

  /* Keep early TX data and its indices. */

  g_uart.rx_read = 0;
  g_uart.rx_write = 0;
  g_uart.tx_inflight = 0;
  g_uart.tx_retry_count = 0;
  g_uart.sticky_status = 0;
  g_uart.transaction_busy = false;
  g_uart.transaction_command = BK7258_MB_UART_STATE;
  g_uart.state_pending = true;
  g_uart.local_rts = false;
  g_uart.upper_rts = false;
  g_uart.peer_rts = false;
  g_uart.peer_rts_known = false;
  memset(&g_uart.stats, 0, sizeof(g_uart.stats));
  nxsem_init(&g_uart.worker_sem, 0, 0);

  ret = bk7258_mailbox_register_rx(BK7258_MB_CHAN_UART0_RX,
                                    mb_uart_receive, NULL);
  if (ret < 0)
    {
      return ret;
    }

  ret = kthread_create("mb-uart0", MB_UART_PRIORITY, 2048,
                       mb_uart_worker, NULL);
  if (ret < 0)
    {
      return ret;
    }

  g_uart.initialized = true;
  return OK;
}

void bk7258_mb_uart_start(void)
{
  irqstate_t flags;

  if (!g_uart.initialized)
    {
      return;
    }

  flags = up_irq_save();
  g_uart.state_pending = true;
  up_irq_restore(flags);
  (void)bk7258_mailbox_start_probe();
  nxsem_post(&g_uart.worker_sem);
}

void bk7258_mb_uart_request_state(void)
{
  irqstate_t flags;

  if (!g_uart.initialized)
    {
      return;
    }

  flags = up_irq_save();
  g_uart.state_pending = true;
  up_irq_restore(flags);
  nxsem_post(&g_uart.worker_sem);
}

ssize_t bk7258_mbox_uart_write(const uint8_t *data, size_t length)
{
  irqstate_t flags;
  uint16_t available;
  uint16_t written;
  uint16_t first;

  if (data == NULL && length != 0)
    {
      return -EINVAL;
    }

  if (length == 0)
    {
      return 0;
    }

  flags = up_irq_save();
  available = tx_free();
  written = length > available ? available : length;
  first = MB_UART_TX_SIZE - g_uart.tx_write;
  if (first > written)
    {
      first = written;
    }

  memcpy(&g_uart.tx_ring[g_uart.tx_write], data, first);
  if (first != written)
    {
      memcpy(g_uart.tx_ring, data + first, written - first);
    }

  g_uart.tx_write = (g_uart.tx_write + written) % MB_UART_TX_SIZE;
  g_uart.stats.write_calls++;
  g_uart.stats.write_bytes += written;
  if (written < length)
    {
      g_uart.stats.short_writes++;
    }

  up_irq_restore(flags);
  if (g_uart.initialized && written != 0)
    {
      nxsem_post(&g_uart.worker_sem);
    }

  return written;
}

ssize_t bk7258_mbox_uart_read(uint8_t *data, size_t length,
                              unsigned int *status)
{
  irqstate_t flags;
  uint16_t available;
  uint16_t count;
  uint16_t first;

  if (data == NULL || length == 0)
    {
      return -EINVAL;
    }

  flags = up_irq_save();
  available = rx_count();
  count = length > available ? available : length;
  first = MB_UART_RX_SIZE - g_uart.rx_read;
  if (first > count)
    {
      first = count;
    }

  memcpy(data, &g_uart.rx_ring[g_uart.rx_read], first);
  if (first != count)
    {
      memcpy(data + first, g_uart.rx_ring, count - first);
    }

  g_uart.rx_read = (g_uart.rx_read + count) % MB_UART_RX_SIZE;
  if (status != NULL)
    {
      *status = g_uart.sticky_status;
      g_uart.sticky_status = 0;
    }

  if (!g_uart.upper_rts && g_uart.local_rts &&
      rx_count() < MB_UART_RTS_RELEASE)
    {
      update_local_rts();
    }

  up_irq_restore(flags);
  if (g_uart.initialized && count != 0)
    {
      nxsem_post(&g_uart.worker_sem);
    }

  return count;
}

bool bk7258_mbox_uart_rxavailable(void)
{
  irqstate_t flags = up_irq_save();
  bool available = rx_count() != 0;
  up_irq_restore(flags);
  return available;
}

bool bk7258_mbox_uart_txready(void)
{
  bool link_ready = bk7258_mailbox_link_ready();
  irqstate_t flags = up_irq_save();
  bool ready = g_uart.initialized && link_ready && tx_free() != 0;
  up_irq_restore(flags);
  return ready;
}

bool bk7258_mbox_uart_txempty(void)
{
  irqstate_t flags = up_irq_save();
  bool empty = tx_count() == 0 && g_uart.tx_inflight == 0 &&
               !g_uart.transaction_busy;
  up_irq_restore(flags);
  return empty;
}

void bk7258_mbox_uart_rxflowcontrol(bool upper)
{
  irqstate_t flags = up_irq_save();

  g_uart.upper_rts = upper;
  update_local_rts();
  up_irq_restore(flags);
  if (g_uart.initialized)
    {
      nxsem_post(&g_uart.worker_sem);
    }
}

void bk7258_mbox_uart_set_callback(bk7258_mb_uart_callback_t callback,
                                   void *arg)
{
  irqstate_t flags = up_irq_save();

  g_uart.callback = callback;
  g_uart.callback_arg = arg;
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: bk7258_mbox_uart_drain_polled
 *
 * Description:
 *   Push whatever is in the TX ring straight at the mailbox hardware, using
 *   nothing but register writes and bounded spins.
 *
 *   This exists because the ordinary path cannot report a crash.  Console
 *   bytes reach the CP only when mb_uart_worker() moves them, and that is a
 *   kthread woken by a semaphore: it needs interrupts and a scheduler.  An
 *   assertion has neither -- a stack-limit violation arrives as a HardFault,
 *   so _assert() runs in exception context with interrupts off -- and
 *   bk7258_mbox_uart_flush() cannot help either, because it waits by calling
 *   nxsig_usleep().  So the register dump and stack trace were written into
 *   the ring and left there, and the only evidence an assertion had happened
 *   at all was the red LED plus the CP resetting the part eight seconds
 *   later
 *   for a missing heartbeat.  Three separate faults were diagnosed by
 *   inference from timestamps because of this.
 *
 *   The CP is alive and printing throughout that window, and eight seconds
 *   is
 *   an enormous budget for a few kilobytes, so there is nothing wrong with
 *   the
 *   transport -- only with needing a scheduler to use it.
 *
 *   Deliberately best-effort, and the compromises are worth naming:
 *
 *   - No ACK is read, so tx_read advances on submission rather than on
 *     completion, and the staging area is reused after a fixed settle spin
 *     instead of after the CP confirms it copied the payload.  A chunk can
 *     therefore be lost or torn.  Losing some of a dump beats losing all of
 *     it, which is what happens today.
 *   - Peer RTS is ignored.  Flow control protects a running system from
 *     overrunning the CP's log queue; a system that is about to be reset has
 *     no later output to protect.
 *   - It reuses BK7258_MB_UART_TX_ADDRESS, so it will corrupt a normal
 *     UART_DATA transaction that was in flight.  That transaction's ACK is
 *     never coming.
 *
 *   Safe to call from interrupt context and with interrupts already
 *   disabled;
 *   it does not block, allocate or print.
 *
 ****************************************************************************/

void bk7258_mbox_uart_crash_mode(void)
{
  g_crash_mode = true;
}

void bk7258_mbox_uart_drain_polled(void)
{
  FAR uint8_t *destination =
    (FAR uint8_t *)(uintptr_t)BK7258_MB_UART_TX_ADDRESS;
  unsigned int chunk;

  /* The gate, and the reason it is here rather than at the call site.
   *
   * Everything below bypasses the transport: it stamps its own sequence
   * numbers, does not wait for acknowledgements, ignores the peer's flow
   * control and reuses the staging buffer a live transaction may still own.
   * Those are all defensible once nothing is going to run again, and all
   * corrupting when something is.
   *
   * Measured 2026-08-31: hooked into every interrupt-context syslog rather
   * than into crash paths only, this moved the CP's console link-down during
   * social mode from 1.75 s after the camera opened to 378 ms.  The hook was
   * too broad, and putting the check at that one call site would have left
   * the
   * next caller free to make the same mistake.  Enforcing it here means the
   * only way to arm this is to declare a crash.
   */

  if (!g_crash_mode || !g_uart.initialized)
    {
      return;
    }

  for (chunk = 0; chunk < MB_UART_POLLED_CHUNKS; chunk++)
    {
      irqstate_t flags;
      uint32_t wire[2];
      uint16_t length;
      uint16_t first;
      unsigned int spin;
      int ret = -EAGAIN;

      flags = up_irq_save();

      length = tx_count();
      if (length == 0)
        {
          up_irq_restore(flags);
          return;
        }

      if (length > BK7258_MB_UART_CHUNK_SIZE)
        {
          length = BK7258_MB_UART_CHUNK_SIZE;
        }

      first = MB_UART_TX_SIZE - g_uart.tx_read;
      if (first > length)
        {
          first = length;
        }

      memcpy(destination, &g_uart.tx_ring[g_uart.tx_read], first);
      if (first != length)
        {
          memcpy(destination + first, g_uart.tx_ring, length - first);
        }

      /* A fresh sequence number per chunk, for the same reason the transport
       * stamps one: consecutive frames that carry the same one look like a
       * retransmission to the peer.
       */

      memset(&g_polled_message, 0, sizeof(g_polled_message));
      g_polled_message.header =
        bk7258_mb_make_header(BK7258_MB_UART_DATA, 0, 0, ++g_polled_seq,
                              BK7258_MB_CHAN_UART0_TX);
      g_polled_message.payload_address = BK7258_MB_UART_TX_ADDRESS;
      g_polled_message.payload_length = length;

      wire[0] = (uint32_t)(uintptr_t)&g_polled_message;
      wire[1] = sizeof(g_polled_message);

      __asm__ volatile("dmb sy" ::: "memory");

      /* -EAGAIN is the peer's receive FIFO being full, which is transient
       * and
       * the one failure worth waiting out.  Anything else means the hardware
       * will not take this frame however long we spin.
       */

      for (spin = 0; spin < MB_UART_POLLED_FIFO_SPINS; spin++)
        {
          ret = bk7258_mbox_send(0, wire);
          if (ret != -EAGAIN)
            {
              break;
            }
        }

      if (ret < 0)
        {
          up_irq_restore(flags);
          return;
        }

      g_uart.tx_read = (g_uart.tx_read + length) % MB_UART_TX_SIZE;
      g_uart.stats.data_tx++;
      length = tx_count();
      up_irq_restore(flags);

      if (length == 0)
        {
          return;
        }

      /* Only between chunks, never after the last one.  The wait is to stop
       * the next chunk overwriting a payload the CP has not copied yet, so
       * with no next chunk there is nothing to wait for -- and this function
       * is called once per line during a dump, so paying it on every call
       * would be most of the cost for none of the benefit.
       *
       * The CP's receive handler takes microseconds; this is slack rather
       * than
       * a measured requirement.
       */

      for (spin = 0; spin < MB_UART_POLLED_SETTLE_SPINS; spin++)
        {
          __asm__ volatile("nop");
        }
    }
}

int bk7258_mbox_uart_flush(unsigned int timeout_ms)
{
  clock_t deadline = clock_systime_ticks() + MSEC2TICK(timeout_ms);

  if (!g_uart.initialized || !bk7258_mailbox_link_ready())
    {
      return -EAGAIN;
    }

  while (!bk7258_mbox_uart_txempty())
    {
      if ((int32_t)(clock_systime_ticks() - deadline) >= 0)
        {
          return -ETIMEDOUT;
        }

      nxsig_usleep(1000);
    }

  return OK;
}

void bk7258_mbox_uart_dump_stats(void)
{
  irqstate_t flags;
  struct mb_uart_stats stats;
  uint16_t tx;
  uint16_t rx;
  bool local_rts;
  bool peer_rts;

  flags = up_irq_save();
  stats = g_uart.stats;
  tx = tx_count();
  rx = rx_count();
  local_rts = g_uart.local_rts;
  peer_rts = g_uart.peer_rts;
  up_irq_restore(flags);

  printf("mb-uart0: txq=%u rxq=%u rts=%u cts=%u data=%lu/%lu "
         "state=%lu/%lu short=%lu overflow=%lu bad=%lu/%lu/%lu/%lu/%lu "
         "remote=%lu retry=%lu/%lu unknown=%lu reset=%lu\n",
         tx, rx, local_rts, peer_rts,
         (unsigned long)stats.data_tx, (unsigned long)stats.data_rx,
         (unsigned long)stats.state_tx, (unsigned long)stats.state_rx,
         (unsigned long)stats.short_writes,
         (unsigned long)stats.rx_overflow,
         (unsigned long)stats.bad_pointer,
         (unsigned long)stats.bad_length,
         (unsigned long)stats.bad_flags,
         (unsigned long)stats.bad_crc,
         (unsigned long)stats.bad_reserved,
         (unsigned long)stats.remote_fail,
         (unsigned long)stats.tx_retry,
         (unsigned long)stats.tx_retry_drop,
         (unsigned long)stats.tx_unknown,
         (unsigned long)stats.resets);
}
