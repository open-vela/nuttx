/****************************************************************************
 * arch/arm/src/bk7258/bk7258_mailbox_channel.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * BK7258 Mailbox V2 logical transport.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <syslog.h>
#include <string.h>

#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>

#include "bk7258_driver.h"
#include "bk7258_netstats.h"
#include "hardware/bk7258_mbox.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MB_CHANNEL_COUNT       7u
#define MB_RX_CHANNEL_COUNT    6u
#define MB_ACK_SLOT_COUNT      8u
#define MB_ACK_HIGH_WATER      3u
#define MB_TIMEOUT             MSEC2TICK(200)
#define MB_WORK_INTERVAL       MSEC2TICK(10)
#define MB_PROBE_MAX_ATTEMPTS  3u
#define MB_RECOVERY_SLOT_COUNT 4u
#define MB_DOWN_RETRY          MSEC2TICK(100)
#define MB_TX_PRIORITY         105

/* The CP waits for this AP boot notification before it marks CPU1 ready.
 * It must be allowed through while the normal UART/mailbox link is still
 * probing; otherwise board_late_initialize() and the CP wait on each other.
 */
#define MB_PWC_CPU1_BOOT_READY 0x05u

/* Consecutive UART0_TX failures before the link is treated as broken rather
 * than busy.  See handle_ack() for why a single one must not count.
 */

#define MB_UART_TX_FAIL_ABORT  4u

/* The CP's flash operation notification (its flash_notify.c): the header
 * command carries the edge, param1 the request/acknowledge state.  Values
 * come from the IPC_FLASH_OP_* enumerations there and cross cores, so they
 * are fixed by that side.
 */

#define FLASH_OP_START         0u
#define FLASH_OP_END           1u
#define FLASH_OP_STATE_REQ     1u
#define FLASH_OP_STATE_ACK     2u

enum ack_slot_state
{
  ACK_SLOT_FREE = 0,
  ACK_SLOT_QUEUED,
  ACK_SLOT_SENT
};

struct logical_channel
{
  struct bk7258_mb_wire_message pending;
  bk7258_mb_tx_complete_t callback;
  void *arg;
  int result;
  bool queued;
};

struct active_transaction
{
  struct bk7258_mb_wire_message message __attribute__((aligned(32)));
  bk7258_mb_tx_complete_t callback;
  void *arg;
  clock_t started;
  uint32_t order;
  uint8_t channel_index;
  bool busy;
  bool quarantine;
};

struct ack_slot
{
  struct bk7258_mb_wire_message message __attribute__((aligned(32)));
  uint32_t order;
  enum ack_slot_state state;
};

struct recovery_epoch
{
  struct bk7258_mb_wire_message reset __attribute__((aligned(32)));
  struct active_transaction probe;
  uint32_t reset_order;
  uint32_t first_probe_order;
  bool reset_sent;
  bool replay;
};

struct transport_stats
{
  uint32_t tx;
  uint32_t rx;
  uint32_t fifo_full;
  uint32_t timeout;
  uint32_t bad_ack;
  uint32_t bad_header;
  uint32_t unsupported;
  uint32_t ack_overflow;
  uint32_t resets;
  uint32_t probes;
  uint32_t probe_fail;
  uint32_t recovery_cycle;
  uint32_t recovery_replay;
  uint32_t link_ready;
  uint32_t link_down;
  uint32_t deferred_command;
  uint32_t bad_ack_reason;
  uint32_t last_ack_header;
  uint32_t last_active_header;
  uint32_t last_command_header;
  uint32_t last_tx_error;
};

static const uint8_t g_channel_ids[MB_CHANNEL_COUNT] =
{
  BK7258_MB_CHAN_HW_CTRL_TX,
  BK7258_MB_CHAN_IPC_TX,
  BK7258_MB_CHAN_PWC_TX,
  BK7258_MB_CHAN_BT_TX,
  BK7258_MB_CHAN_WIFI_CMD_TX,
  BK7258_MB_CHAN_WIFI_DATA_TX,
  BK7258_MB_CHAN_UART0_TX
};

static const uint8_t g_rx_channel_ids[MB_RX_CHANNEL_COUNT] =
{
  BK7258_MB_CHAN_IPC_RX,
  BK7258_MB_CHAN_BT_RX,
  BK7258_MB_CHAN_WIFI_CMD_RX,
  BK7258_MB_CHAN_WIFI_DATA_RX,
  BK7258_MB_CHAN_UART0_RX,
  BK7258_MB_CHAN_SARADC_RX
};

static struct logical_channel g_channels[MB_CHANNEL_COUNT];
static struct active_transaction g_active;
static struct recovery_epoch g_recovery[MB_RECOVERY_SLOT_COUNT];
static struct ack_slot g_ack_slots[MB_ACK_SLOT_COUNT];

static struct transport_stats g_stats;
static bk7258_mb_channel_rx_t g_rx_callbacks[MB_RX_CHANNEL_COUNT];
static void *g_rx_args[MB_RX_CHANNEL_COUNT];
static int (*g_pwc_rx)(const void *message);
static bk7258_mb_link_callback_t g_link_callback;
static void *g_link_arg;
static sem_t g_tx_sem;
static sem_t g_diag_sem;
static uint32_t g_send_order;
static uint8_t g_tx_seq;
static enum bk7258_mb_link_state g_link_state;
static bool g_initialized;
static bool g_physical_started;
static bool g_state_initialized;
static bool g_tx_worker_created;
static bool g_diag_worker_created;
static bool g_reset_pending;
static bool g_probe_requested;
static uint8_t g_probe_attempts;
static uint8_t g_recovery_index;

/* Consecutive UART0_TX failures, and where the next dispatch scan starts
 * among
 * the channels that take turns.  Both are explained where they are used:
 * complete_active() and dispatch_locked().
 */

static uint8_t g_uart_tx_failures;
static uint8_t g_dispatch_rotor;
static clock_t g_down_since;
static enum bk7258_mb_link_state g_diag_state;
static uint32_t g_peer_reset_generation;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool
is_boot_ready_message(uint8_t logical_channel,
                      const struct bk7258_mb_wire_message *message)
{
  return logical_channel == BK7258_MB_CHAN_PWC_TX &&
         bk7258_mb_header_cmd(message) == MB_PWC_CPU1_BOOT_READY;
}

  /* HW_CTRL power-up and heartbeat indications provide the CP-side readiness
   * signal for the AP console. They must be allowed while the UART channel
   * is
   * still probing, otherwise startup waits on a circular dependency.
   */

static bool is_boot_ipc_message(uint8_t logical_channel,
                                const struct bk7258_mb_wire_message *message)
{
  return logical_channel == BK7258_MB_CHAN_HW_CTRL_TX &&
         (bk7258_mb_header_cmd(message) == 1u ||
          bk7258_mb_header_cmd(message) == 2u);
}

rspinlock_t g_bk7258_driver_lock = RSPINLOCK_INITIALIZER;

static_assert(MB_ACK_SLOT_COUNT >= 2u * 3u + 1u,
               "ACK pool must exceed two hardware FIFO depths");
static_assert(MB_RECOVERY_SLOT_COUNT >= 3u,
               "at least three stable recovery epochs are required");

static struct recovery_epoch *current_recovery(void)
{
  return &g_recovery[g_recovery_index];
}

static unsigned int channel_index(uint8_t channel)
{
  unsigned int i;

  for (i = 0; i < MB_CHANNEL_COUNT; i++)
    {
      if (g_channel_ids[i] == channel)
        {
          return i;
        }
    }

  return MB_CHANNEL_COUNT;
}

static unsigned int rx_channel_index(uint8_t channel)
{
  unsigned int i;

  for (i = 0; i < MB_RX_CHANNEL_COUNT; i++)
    {
      if (g_rx_channel_ids[i] == channel)
        {
          return i;
        }
    }

  return MB_RX_CHANNEL_COUNT;
}

static unsigned int ack_slots_used(void)
{
  unsigned int count = 0;
  unsigned int i;

  for (i = 0; i < MB_ACK_SLOT_COUNT; i++)
    {
      if (g_ack_slots[i].state != ACK_SLOT_FREE)
        {
          count++;
        }
    }

  return count;
}

static struct active_transaction *busy_probe(void)
{
  unsigned int i;

  for (i = 0; i < MB_RECOVERY_SLOT_COUNT; i++)
    {
      if (g_recovery[i].probe.busy)
        {
          return &g_recovery[i].probe;
        }
    }

  return NULL;
}

static int send_slot(struct bk7258_mb_wire_message *message,
                     uint32_t *order)
{
  uint32_t wire[2];
  int ret;

  *order = ++g_send_order;
  wire[0] = (uint32_t)(uintptr_t)message;
  wire[1] = sizeof(*message);
  __asm__ volatile("dmb sy" ::: "memory");
  ret = bk7258_mbox_send(0, wire);
  if (ret == OK)
    {
      g_stats.tx++;
    }
  else if (ret == -EAGAIN)
    {
      g_stats.fifo_full++;
    }

  if (ret < 0)
    {
      g_stats.last_tx_error = (uint32_t)(-ret);
    }

  return ret;
}

static void release_acks_before(uint32_t order)
{
  unsigned int i;

  for (i = 0; i < MB_ACK_SLOT_COUNT; i++)
    {
      if (g_ack_slots[i].state == ACK_SLOT_SENT &&
          g_ack_slots[i].order < order)
        {
          g_ack_slots[i].state = ACK_SLOT_FREE;
        }
    }
}

static void set_link_state(enum bk7258_mb_link_state state)
{
  if (g_link_state != state)
    {
      if (state == BK7258_MB_LINK_READY)
        {
          g_stats.link_ready++;
        }
      else if (state == BK7258_MB_LINK_DOWN)
        {
          g_stats.link_down++;
          g_down_since = clock_systime_ticks();
        }

      g_link_state = state;
      nxsem_post(&g_tx_sem);
      g_diag_state = state;
      nxsem_post(&g_diag_sem);
    }
}

static void complete_transaction(struct active_transaction *active,
                                 const struct bk7258_mb_wire_message *ack,
                                 int result)
{
  struct logical_channel *channel = NULL;

  if (active->channel_index < MB_CHANNEL_COUNT)
    {
      channel = &g_channels[active->channel_index];
      channel->queued = false;
      channel->result = result;
    }

  active->busy = false;
  if (result == OK)
    {
      release_acks_before(active->order);
    }

  if (active->callback != NULL)
    {
      active->callback(ack, result, active->arg);
    }
}

static void fail_pending(int result)
{
  unsigned int i;

  for (i = 0; i < MB_CHANNEL_COUNT; i++)
    {
      struct logical_channel *channel = &g_channels[i];

      if (channel->queued)
        {
          channel->queued = false;
          channel->result = result;
          if (channel->callback != NULL)
            {
              channel->callback(NULL, result, channel->arg);
            }
        }
    }
}

static void release_recovery_slots(void)
{
  unsigned int i;

  g_active.quarantine = false;
  for (i = 0; i < MB_RECOVERY_SLOT_COUNT; i++)
    {
      memset(&g_recovery[i], 0, sizeof(g_recovery[i]));
    }
}

static void select_recovery_epoch(void)
{
  unsigned int i;
  struct recovery_epoch *epoch = NULL;

  for (i = 0; i < MB_RECOVERY_SLOT_COUNT; i++)
    {
      if (!g_recovery[i].probe.quarantine &&
          !g_recovery[i].reset_sent && !g_recovery[i].probe.busy)
        {
          epoch = &g_recovery[i];
          g_recovery_index = i;
          break;
        }
    }

  if (epoch == NULL)
    {
      uint32_t newest = 0;

      for (i = 0; i < MB_RECOVERY_SLOT_COUNT; i++)
        {
          if (g_recovery[i].probe.order >= newest)
            {
              newest = g_recovery[i].probe.order;
              epoch = &g_recovery[i];
              g_recovery_index = i;
            }
        }

      epoch->replay = true;
      g_stats.recovery_replay++;
    }
  else
    {
      memset(epoch, 0, sizeof(*epoch));
      epoch->reset.header = bk7258_mb_make_header(
        0, 0, BK7258_MB_CTRL_ACK_BOX | BK7258_MB_CTRL_RESET, 0,
        BK7258_MB_CHAN_HW_CTRL_TX);
    }

  g_probe_attempts = 0;
  g_reset_pending = !epoch->replay;
  g_stats.recovery_cycle++;
  set_link_state(epoch->replay ? BK7258_MB_LINK_PROBING :
                                 BK7258_MB_LINK_ABORTING);
}

static void begin_abort(int result)
{
  struct active_transaction *probe = busy_probe();

  if (g_active.busy)
    {
      g_active.quarantine = true;
      complete_transaction(&g_active, NULL, result);
    }

  if (probe != NULL)
    {
      probe->quarantine = true;
      complete_transaction(probe, NULL, result);
    }

  fail_pending(result);
  g_probe_requested = false;
  select_recovery_epoch();
}

static struct ack_slot *alloc_ack_slot(void)
{
  unsigned int i;

  for (i = 0; i < MB_ACK_SLOT_COUNT; i++)
    {
      if (g_ack_slots[i].state == ACK_SLOT_FREE)
        {
          g_ack_slots[i].state = ACK_SLOT_QUEUED;
          return &g_ack_slots[i];
        }
    }

  g_stats.ack_overflow++;
  return NULL;
}

static int dispatch_locked(void)
{
  struct recovery_epoch *epoch = current_recovery();
  struct active_transaction *active;
  struct logical_channel *channel;
  unsigned int i;
  unsigned int n;
  int ret;

  /* ACKs always outrank RESET and ordinary commands. */

  for (i = 0; i < MB_ACK_SLOT_COUNT; i++)
    {
      if (g_ack_slots[i].state != ACK_SLOT_QUEUED)
        {
          continue;
        }

      ret = send_slot(&g_ack_slots[i].message, &g_ack_slots[i].order);
      if (ret < 0)
        {
          return ret;
        }

      g_ack_slots[i].state = ACK_SLOT_SENT;
    }

  if (g_reset_pending)
    {
      ret = send_slot(&epoch->reset, &epoch->reset_order);
      if (ret < 0)
        {
          return ret;
        }

      g_reset_pending = false;
      epoch->reset_sent = true;
      set_link_state(BK7258_MB_LINK_PROBING);
    }

  if (g_active.busy || busy_probe() != NULL)
    {
      return OK;
    }

  /* One transaction is in flight at a time, so the order this loop visits
   * channels in is a priority order, and a strict scan from 0 makes it a
   * starvation order too: whatever sits at a lower index and always has
   * something queued keeps everything below it off the link indefinitely.
   *
   * That is not hypothetical.  WIFI_DATA_TX is index 5 and UART0_TX is index
   * 6, so any sustained upload -- social mode is the first workload that
   * produces one -- stops the console from ever being dispatched.  Starved
   * past MB_TIMEOUT it fails, and handle_ack() used to answer that by
   * aborting the link, which stops the heartbeat, which resets the part.
   *
   * Index 0 keeps absolute precedence because that is HW_CTRL_TX, the
   * heartbeat: it is answering an 8 s deadline and is one small frame every
   * 2 s, so it costs the others nothing to always let it go first.  The rest
   * take turns, which bounds how long any of them can be held off by the
   * others rather than leaving it open-ended.
   *
   * Rotating is not enough on its own -- see handle_ack(), which is where a
   * starved console frame turned into a link abort.
   */

  for (n = 0; n < MB_CHANNEL_COUNT; n++)
    {
      i = n == 0 ? 0 :
          1u + (g_dispatch_rotor + (n - 1u)) % (MB_CHANNEL_COUNT - 1u);

      channel = &g_channels[i];
      if (!channel->queued)
        {
          continue;
        }

      if (g_link_state == BK7258_MB_LINK_PROBING)
        {
          if (g_channel_ids[i] == BK7258_MB_CHAN_UART0_TX &&
              bk7258_mb_header_cmd(&channel->pending) ==
              BK7258_MB_UART_STATE)
            {
              active = &epoch->probe;
            }
          else if (is_boot_ready_message(g_channel_ids[i],
                                         &channel->pending) ||
                   is_boot_ipc_message(g_channel_ids[i],
                                       &channel->pending))
            {
              /* CPU1 boot-ready is the one PWC frame that must be sent
               * before the ordinary link reaches READY.
               */

              active = &g_active;
            }
          else
            {
              continue;
            }

          if (active == &epoch->probe && active->quarantine &&
              !epoch->replay)
            {
              return -ENOLINK;
            }
        }
      else if (g_link_state == BK7258_MB_LINK_READY)
        {
          active = &g_active;
          if (active->quarantine)
            {
              return -ENOLINK;
            }
        }
      else
        {
          return OK;
        }

      if (!epoch->replay || active != &epoch->probe)
        {
          memset(&active->message, 0, sizeof(active->message));
          memcpy(&active->message, &channel->pending,
                 sizeof(active->message));
          active->message.header = bk7258_mb_make_header(
            bk7258_mb_header_cmd(&channel->pending),
            bk7258_mb_header_state(&channel->pending), 0, ++g_tx_seq,
            g_channel_ids[i]);
        }

      active->callback = channel->callback;
      active->arg = channel->arg;
      active->channel_index = i;
      active->started = clock_systime_ticks();
      active->busy = true;
      ret = send_slot(&active->message, &active->order);
      if (ret < 0)
        {
          active->busy = false;
          return ret;
        }

      if (active == &epoch->probe)
        {
          if (epoch->first_probe_order == 0)
            {
              epoch->first_probe_order = active->order;
            }

          g_probe_attempts++;
          g_stats.probes++;
        }

      /* Hand the turn to the next channel, so the one that just sent goes to
       * the back of the queue instead of winning again.  Index 0 is outside
       * the rotation and does not move it: the heartbeat keeps its
       * precedence
       * without consuming anyone else's turn.
       */

      if (i != 0)
        {
          g_dispatch_rotor = (uint8_t)(i % (MB_CHANNEL_COUNT - 1u));
        }

      return OK;
    }

  return OK;
}

static uint32_t ack_header_reason(
  const struct bk7258_mb_wire_message *message,
  const struct active_transaction *active)
{
  uint8_t control = bk7258_mb_header_ctrl(message);
  uint32_t reason = 0;

  if (!active->busy)
    {
      return 1u;
    }

  if (control != BK7258_MB_CTRL_ACK_BOX)
    {
      reason |= 2u;
    }

  if (bk7258_mb_header_channel(message) !=
      bk7258_mb_header_channel(&active->message))
    {
      reason |= 4u;
    }

  if (bk7258_mb_header_seq(message) !=
      bk7258_mb_header_seq(&active->message))
    {
      reason |= 8u;
    }

  if (bk7258_mb_header_cmd(message) !=
      bk7258_mb_header_cmd(&active->message))
    {
      reason |= 16u;
    }

  return reason;
}

static bool valid_uart_ack(const struct bk7258_mb_wire_message *message)
{
  /* Armino reuses the transmitted UART command buffer as its response.  It
   * overwrites only the low byte of ack_data1 with uart_rts/uart_tx_fail;
   * payload_address, length, flags and the trailing words therefore retain
   * request data.  Header matching is the ownership check for this ACK.
   */

  return message != NULL;
}

static void handle_ack(const struct bk7258_mb_wire_message *message)
{
  struct recovery_epoch *epoch = current_recovery();
  struct active_transaction *active;
  bool replay;
  int result;
  uint32_t reason;

  active = busy_probe();
  if (active == NULL)
    {
      active = &g_active;
    }

  g_stats.last_ack_header = message->header;
  g_stats.last_active_header = active->message.header;
  reason = ack_header_reason(message, active);
  if (reason != 0)
    {
      g_stats.bad_ack++;
      g_stats.bad_ack_reason |= reason;
      return;
    }

  if (bk7258_mb_header_channel(message) == BK7258_MB_CHAN_UART0_TX &&
      !valid_uart_ack(message))
    {
      /* Keep active stable.  A malformed ACK is not an ownership proof. */

      g_stats.bad_ack++;
      g_stats.bad_ack_reason |= 32u;
      return;
    }

  if ((bk7258_mb_header_state(message) & ~BK7258_MB_STATE_COM_FAIL) != 0)
    {
      g_stats.bad_ack++;
      g_stats.bad_ack_reason |= 64u;
      return;
    }

  result =
    (bk7258_mb_header_state(message) & BK7258_MB_STATE_COM_FAIL) != 0 ?
    -EREMOTEIO : OK;

  if (bk7258_mb_header_channel(message) == BK7258_MB_CHAN_UART0_TX &&
      result == OK && (message->payload_address & 2u) != 0)
    {
      result = -EREMOTEIO;
    }

  replay = active == &epoch->probe && epoch->replay;
  release_acks_before(replay ? epoch->first_probe_order : active->order);
  if (replay)
    {
      unsigned int replay_index = g_recovery_index;
      unsigned int i;

      /* This ACK may be a late ACK for the original copy.  It proves every
       * older epoch was copied, but not that the new replay envelope was
       * copied.  Preserve the replay epoch, free only older epochs, and use
       * one of them for a fresh-sequence confirmation probe.
       */

      complete_transaction(active, message, -EAGAIN);
      g_active.quarantine = false;
      for (i = 0; i < MB_RECOVERY_SLOT_COUNT; i++)
        {
          if (i != replay_index)
            {
              memset(&g_recovery[i], 0, sizeof(g_recovery[i]));
            }
        }

      select_recovery_epoch();
      (void)dispatch_locked();
      return;
    }

  complete_transaction(active, message, result);

  if (active == &epoch->probe)
    {
      if (result == OK && (message->flags & 2u) == 0)
        {
          release_recovery_slots();
          g_recovery_index = 0;
          g_probe_attempts = 0;
          set_link_state(BK7258_MB_LINK_READY);
        }
      else if (result == -EREMOTEIO &&
               g_probe_attempts < MB_PROBE_MAX_ATTEMPTS)
        {
          /* CP may not have opened MB_UART0 yet.  The matched ACK proves
           * that this probe slot is no longer referenced by its FIFO.
           */
        }
      else
        {
          g_stats.probe_fail++;
          epoch->probe.quarantine = result == -ETIMEDOUT;
          set_link_state(BK7258_MB_LINK_DOWN);
        }
    }

  else if (bk7258_mb_header_channel(message) == BK7258_MB_CHAN_UART0_TX)
    {
      /* One unanswered console frame is not evidence of a broken link, for
       * the
       * same reason an ordinary message is not (see mailbox_tx_worker): the
       * heartbeat and everything else were working up to that moment, and a
       * recovery is what would destroy them.  begin_abort() here puts the
       * link
       * into PROBING, and while it is there bk7258_mailbox_send_wire()
       * refuses
       * every heartbeat -- so the CP resets the part unless recovery
       * finishes
       * inside its 8 s budget, which nothing here guarantees.
       *
       * Measured 2026-08-31: entering social mode starts sustained uploads,
       * UART0_TX is last in the dispatch order and starved past MB_TIMEOUT,
       * and this line turned that into "IPC[1]heartbeat timeout" and a
       * reboot
       * loop with the AP running normally throughout.  The dispatch rotation
       * below is the other half of that fix; this half makes a console frame
       * that is merely late cost a log line instead of the chip.
       *
       * A link that really is down fails these consecutively, which is what
       * the threshold distinguishes.  At MB_TIMEOUT per failure it takes
       * ~800 ms to decide, an order of magnitude inside the CP's budget.
       */

      if (result >= 0)
        {
          g_uart_tx_failures = 0;
        }
      else if (++g_uart_tx_failures >= MB_UART_TX_FAIL_ABORT)
        {
          g_uart_tx_failures = 0;
          begin_abort(result);
        }
    }

  (void)dispatch_locked();
}

static int handle_command(const struct bk7258_mb_wire_message *message)
{
  struct ack_slot *slot;
  unsigned int rx_index;
  uint8_t channel = bk7258_mb_header_channel(message);
  uint8_t control = bk7258_mb_header_ctrl(message);
  uint8_t ack_flags = 0;
  uint8_t state = 0;
  int ret = -ENOSYS;

  g_stats.last_command_header = message->header;

  if ((control & BK7258_MB_CTRL_SYNC_TX) == 0)
    {
      slot = alloc_ack_slot();
      if (slot == NULL)
        {
          g_stats.deferred_command++;
          return -EAGAIN;
        }
    }
  else
    {
      slot = NULL;
    }

  if ((control & ~(BK7258_MB_CTRL_SYNC_TX | BK7258_MB_CTRL_RESET)) != 0 ||
      bk7258_mb_header_state(message) != 0 ||
      (channel & 0xf0u) != 0x40u)
    {
      g_stats.bad_header++;
      state = BK7258_MB_STATE_COM_FAIL;
    }
  else if ((rx_index = rx_channel_index(channel)) < MB_RX_CHANNEL_COUNT &&
           g_rx_callbacks[rx_index] != NULL)
    {
      ret = g_rx_callbacks[rx_index](message, &ack_flags,
                                      g_rx_args[rx_index]);
      if (ret == -EAGAIN)
        {
          if (slot != NULL)
            {
              slot->state = ACK_SLOT_FREE;
            }

          g_stats.deferred_command++;
          return -EAGAIN;
        }
      else if (ret < 0)
        {
          state = BK7258_MB_STATE_COM_FAIL;
        }
    }
  else if (channel == BK7258_MB_CHAN_PWC_RX && g_pwc_rx != NULL)
    {
      ret = g_pwc_rx(message);
      if (ret == -EAGAIN)
        {
          if (slot != NULL)
            {
              slot->state = ACK_SLOT_FREE;
            }

          g_stats.deferred_command++;
          return -EAGAIN;
        }
      else if (ret < 0)
        {
          state = BK7258_MB_STATE_COM_FAIL;
        }
    }
  else if (channel == BK7258_MB_CHAN_SARADC_RX &&
           (bk7258_mb_header_cmd(message) == 0u ||
            bk7258_mb_header_cmd(message) == 1u) &&
           message->payload_address == 1u &&
           message->payload_length == 0u && message->flags == 0u &&
           message->crc8 == 0u && message->reserved == 0u)
    {
      /* Match Armino's SARADC operation-notification ABI. */

      ack_flags = 2u;
      ret = OK;
    }
  else if (channel == BK7258_MB_CHAN_FLASH_RX &&
           (bk7258_mb_header_cmd(message) == FLASH_OP_START ||
            bk7258_mb_header_cmd(message) == FLASH_OP_END) &&
           message->payload_address == FLASH_OP_STATE_REQ)
    {
      /* The CP announces every flash access it is about to make, and its
       * end, on this channel -- for its own writes as much as for anyone
       * else's.  flash_lock() in its flash_driver.c calls
       * mb_flash_op_prepare() with the scheduler already suspended, and
       * send_flash_op_state() then spins until the acknowledgement carries
       * IPC_FLASH_OP_ACK, giving up after FLASH_WAIT_ACK_TIMEOUT (5ms).
       * Its tx-complete ISR only adopts ack_data1 when COM_FAIL is clear,
       * so leaving this channel unmatched does not merely lose the
       * notification: the CP burns that 5ms twice around every flash
       * operation with its tasks suspended.
       *
       * Match the active vendor AP cpu1_pause_handle START/END path.
       * Both cores execute from the same Flash. The vendor path does not
       * park the AP; it pauses registered streaming peripherals then ACKs.
       * This port has no such DMA streams enabled. A future streaming
       * peripheral must implement equivalent pause/resume coordination.
       *
       * Only the header command and param1 are checked.  The CP fills a
       * stack-local mb_chnl_cmd_t and assigns just hdr.data and param1, so
       * param2 and param3 -- payload_length, flags, crc8 and reserved --
       * hold whatever was on its stack.  The SARADC branch above can
       * require them to be zero because its sender clears them; this one
       * must not.
       */

      ack_flags = FLASH_OP_STATE_ACK;
      ret = OK;
    }
  else
    {
      g_stats.unsupported++;
      state = BK7258_MB_STATE_COM_FAIL;
    }

  if ((control & BK7258_MB_CTRL_SYNC_TX) != 0)
    {
      return OK;
    }

  memset(&slot->message, 0, sizeof(slot->message));
  slot->message.header = bk7258_mb_make_ack_header(message,
                                                   state != 0);
  /* On the socket channels the peer reinterprets the whole ack box as the
   * command it sent -- ipc_router_tx_cmpl_isr() in the CP's mb_ipc.c does a
   * plain cast, `ipc_cmd = (mb_ipc_cmd_t *)ack_buf`, and then checks that
   * param1 still matches what it queued (masked with IPC_PARAM1_MASK,
   * 0x00FFFFFF).  So param1 has to be echoed, not replaced by our ack flags.
   *
   * Sending our own value instead made that check fail as "tx2 error @440!
   * 150 != 0": the CP had queued 0x150 and got back 0.  A failed check makes
   * the CP drop the frame without dequeuing it, and the router then takes
   * the
   * link down -- 200 ms after the request, every time.  That also destroys
   * the only way to watch this from the AP, since the console is tunnelled
   * over the same mailbox; the diagnosis had to come from the CP's own log
   * plus its sources.
   *
   * param2 (payload_length and crc8) is echoed for the same reason.  The CP
   * only compares it when built with DEBUG_MB_IPC, but echoing costs nothing
   * and keeps the ack a faithful copy.
   */

  if (channel == BK7258_MB_CHAN_IPC_RX)
    {
      slot->message.payload_address = message->payload_address;
      slot->message.payload_length = message->payload_length;
      slot->message.crc8 = message->crc8;
    }
  else
    {
      slot->message.payload_address = ack_flags;
    }

  /* ack_state as well, not just ack_data1.  mb_chnl_ack_t's third word is
   * the
   * state field (its union names it ack_state), and that is the one the peer
   * tests -- this port's own heartbeat does the same when it checks an ack
   * (bk7258_ipc_heartbeat.c ipc_ack_result() reads ->reserved).  Leaving it
   * zero while answering a command made the CP's mb_ipc socket stay in
   * "receive in process" forever: every later request to its flash server
   * came back with route and api status both RX_BUSY (ack byte 0x52) and no
   * reply.  Callers that pass 0 are unaffected.
   */

  slot->message.reserved = ack_flags;

  if (ack_slots_used() >= MB_ACK_HIGH_WATER)
    {
      g_probe_requested = true;
    }

  (void)dispatch_locked();
  return OK;
}

static int mailbox_rx(const struct bk7258_mb_wire_message *message)
{
  irqstate_t flags;
  uint8_t control;

  flags = up_irq_save();
  g_stats.rx++;
  control = bk7258_mb_header_ctrl(message);

  if ((control & BK7258_MB_CTRL_RESET) != 0)
    {
      if ((control & BK7258_MB_CTRL_ACK_BOX) == 0 ||
          (bk7258_mb_header_channel(message) & 0xf0u) != 0x40u ||
          (control & ~(BK7258_MB_CTRL_ACK_BOX | BK7258_MB_CTRL_RESET)) != 0)
        {
          g_stats.bad_header++;
        }
      else
        {
          g_stats.resets++;
          g_peer_reset_generation++;
          bk7258_mbox_discard_deferred();
          begin_abort(-ECONNRESET);
        }
    }
  else if ((control & BK7258_MB_CTRL_ACK_BOX) != 0)
    {
      handle_ack(message);
    }
  else
    {
      int ret = handle_command(message);

      if (ret == -EAGAIN)
        {
          up_irq_restore(flags);
          return ret;
        }
    }

  up_irq_restore(flags);
  nxsem_post(&g_tx_sem);
  return OK;
}

static int mailbox_diag_worker(int argc, char **argv)
{
  enum bk7258_mb_link_state reported = BK7258_MB_LINK_QUIESCING;

  (void)argc;
  (void)argv;

  for (; ; )
    {
      nxsem_wait_uninterruptible(&g_diag_sem);
      nxsem_reset(&g_diag_sem, 0);
      irqstate_t flags = up_irq_save();
      enum bk7258_mb_link_state state = g_diag_state;
      bk7258_mb_link_callback_t callback = g_link_callback;
      void *callback_arg = g_link_arg;
      uint32_t timeout = g_stats.timeout;
      uint32_t bad_ack = g_stats.bad_ack;
      uint8_t epoch = g_recovery_index;

      up_irq_restore(flags);
      if (reported == state)
        {
          continue;
        }

      reported = state;
      if (reported != BK7258_MB_LINK_PROBING &&
          reported != BK7258_MB_LINK_READY)
        {
          syslog(LOG_WARNING,
                 "mailbox: link state=%u epoch=%u timeout=%lu bad_ack=%lu\n",
                 reported, epoch, (unsigned long)timeout,
                 (unsigned long)bad_ack);
        }

      if (callback != NULL)
        {
          callback(reported, callback_arg);
        }
    }

  return OK;
}

static int mailbox_tx_worker(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  for (; ; )
    {
      struct active_transaction *probe;
      unsigned int uart_index;
      irqstate_t flags;
      bool request_probe;

      (void)nxsem_tickwait_uninterruptible(&g_tx_sem, MB_WORK_INTERVAL);
      nxsem_reset(&g_tx_sem, 0);
      flags = up_irq_save();

      probe = busy_probe();
      if (probe != NULL &&
          clock_systime_ticks() - probe->started >= MB_TIMEOUT)
        {
          /* A probe going unanswered really does mean the link is down: the
           * probe is the thing that establishes it.  Recovery is right here.
           */

          g_stats.timeout++;
          probe->quarantine = true;
          complete_transaction(probe, NULL, -ETIMEDOUT);
          g_stats.probe_fail++;
          set_link_state(BK7258_MB_LINK_DOWN);
        }
      else if (g_active.busy &&
               clock_systime_ticks() - g_active.started >= MB_TIMEOUT)
        {
          /* An ordinary message going unanswered does not.  Fail that one
           * transaction and carry on.
           *
           * This used to call begin_abort(), which quarantined the link and
           * started a recovery epoch -- and while the link is PROBING,
           * dispatch
           * only lets probe frames out, so the console and the CP heartbeat
           * were both blocked.  The CP's 8-second watchdog then reset the
           * chip.
           * Measured 2026-08-18: one unanswered flash read turned into an
           * endless boot loop ("IPC[1]heartbeat timeout", "Assert at:
           * mb_ipc_task:297"), because the transport treated a single
           * application-level non-reply as evidence that the whole link was
           * broken.  It is not: the console and the heartbeat were working
           * up
           * to that moment, and they are what recovery then destroyed.
           */

          g_stats.timeout++;
          complete_transaction(&g_active, NULL, -ETIMEDOUT);
        }

      if (g_link_state == BK7258_MB_LINK_DOWN &&
          clock_systime_ticks() - g_down_since >= MB_DOWN_RETRY)
        {
          select_recovery_epoch();
        }

      (void)dispatch_locked();
      if (g_probe_requested && ack_slots_used() < MB_ACK_HIGH_WATER)
        {
          g_probe_requested = false;
        }

      uart_index = channel_index(BK7258_MB_CHAN_UART0_TX);
      request_probe = g_probe_requested &&
                      g_link_state == BK7258_MB_LINK_READY &&
                      uart_index < MB_CHANNEL_COUNT &&
                      !g_channels[uart_index].queued && !g_active.busy;
      if (request_probe)
        {
          g_probe_requested = false;
        }

      up_irq_restore(flags);

      if (request_probe)
        {
          bk7258_mb_uart_request_state();
        }
    }

  return OK;
}

int bk7258_mailbox_init(void)
{
  int ret;

  if (g_initialized)
    {
      return OK;
    }

  if (!g_state_initialized)
    {
      memset(g_channels, 0, sizeof(g_channels));
      memset(&g_active, 0, sizeof(g_active));
      memset(g_recovery, 0, sizeof(g_recovery));
      memset(g_ack_slots, 0, sizeof(g_ack_slots));
      memset(&g_stats, 0, sizeof(g_stats));
      g_link_state = BK7258_MB_LINK_DOWN;
      g_diag_state = BK7258_MB_LINK_DOWN;
      g_recovery_index = 0;
      g_probe_attempts = 0;
      g_peer_reset_generation = 0;
      g_uart_tx_failures = 0;
      g_dispatch_rotor = 0;
      nxsem_init(&g_tx_sem, 0, 0);
      nxsem_init(&g_diag_sem, 0, 0);
      g_state_initialized = true;
    }

  if (!g_tx_worker_created)
    {
      ret = kthread_create("mbox-v2", MB_TX_PRIORITY, 2048,
                           mailbox_tx_worker, NULL);
      if (ret < 0)
        {
          return ret;
        }

      g_tx_worker_created = true;
    }

  if (!g_diag_worker_created)
    {
      ret = kthread_create("mbox-diag", 80, 1024,
                           mailbox_diag_worker, NULL);
      if (ret < 0)
        {
          return ret;
        }

      g_diag_worker_created = true;
    }

  g_initialized = true;
  g_physical_started = false;
  return OK;
}

int bk7258_mailbox_start(void)
{
  int ret;

  if (!g_initialized)
    {
      return -EAGAIN;
    }

  if (g_physical_started)
    {
      return OK;
    }

  bk7258_mbox_set_callback(mailbox_rx);
  ret = bk7258_mbox_init();
  if (ret == OK)
    {
      g_physical_started = true;
    }

  return ret;
}

int bk7258_mailbox_send_wire(uint8_t logical_channel,
                             const struct bk7258_mb_wire_message *message,
                             bk7258_mb_tx_complete_t callback, void *arg)
{
  struct logical_channel *channel;
  unsigned int index;
  irqstate_t flags;

  if (!g_initialized || !g_physical_started || message == NULL)
    {
      return -EAGAIN;
    }

  index = channel_index(logical_channel);
  if (index >= MB_CHANNEL_COUNT)
    {
      return -EINVAL;
    }

  flags = up_irq_save();
  channel = &g_channels[index];

  if (channel->queued)
    {
      up_irq_restore(flags);
      return -EBUSY;
    }

  /* Not READY means not sendable, with one exemption: the UART0 state frame
   * that *is* the probe recovery runs on.
   *
   * The heartbeat is deliberately not exempted here even though it is the
   * one
   * thing the CP's 8 s watchdog measures, because the slot it would have to
   * use (g_active) is quarantined for the duration of a recovery -- the
   * peer's
   * FIFO may still reference it, which is what the quarantine exists to
   * prevent.  Letting it queue instead would be worse than refusing: the
   * caller latches g_heartbeat_pending on a successful submission and would
   * then stop retrying altogether, where -ENOLINK makes it retry every
   * IPC_HEARTBEAT_RETRY until the link is back.
   *
   * So the cost of a recovery that outlasts eight seconds is a chip reset,
   * and
   * the defence is to not enter one for a reason that is not a broken link
   * --
   * see handle_ack(), where a starved console frame used to do exactly that,
   * and dispatch_locked(), where the starvation came from.
   */

  if (g_link_state != BK7258_MB_LINK_READY &&
      !(g_link_state == BK7258_MB_LINK_PROBING &&
        (logical_channel == BK7258_MB_CHAN_UART0_TX &&
         bk7258_mb_header_cmd(message) == BK7258_MB_UART_STATE)) &&
      !is_boot_ready_message(logical_channel, message) &&
      !is_boot_ipc_message(logical_channel, message))
    {
      up_irq_restore(flags);
      return -ENOLINK;
    }

  if (g_link_state == BK7258_MB_LINK_READY && g_active.quarantine)
    {
      up_irq_restore(flags);
      return -ENOLINK;
    }

  memset(&channel->pending, 0, sizeof(channel->pending));
  memcpy(&channel->pending, message, sizeof(channel->pending));
  channel->callback = callback;
  channel->arg = arg;
  channel->result = -EINPROGRESS;
  channel->queued = true;

  (void)dispatch_locked();

  up_irq_restore(flags);

  nxsem_post(&g_tx_sem);
  return OK;
}

int bk7258_mailbox_send_raw(uint8_t logical_channel,
                            const uint8_t frame[BK7258_MB_MESSAGE_SIZE],
                            bk7258_mb_tx_complete_t callback, void *arg)
{
  struct bk7258_mb_wire_message message;

  if (frame == NULL)
    {
      return -EINVAL;
    }

  memcpy(&message, frame, sizeof(message));
  return bk7258_mailbox_send_wire(logical_channel, &message, callback, arg);
}

int bk7258_mailbox_register_rx(uint8_t logical_channel,
                               bk7258_mb_channel_rx_t callback, void *arg)
{
  unsigned int index = rx_channel_index(logical_channel);

  if (index >= MB_RX_CHANNEL_COUNT)
    {
      return -EINVAL;
    }

  g_rx_callbacks[index] = callback;
  g_rx_args[index] = arg;
  return OK;
}

int bk7258_mailbox_start_probe(void)
{
  irqstate_t flags;

  if (!g_initialized)
    {
      return -EAGAIN;
    }

  flags = up_irq_save();
  if (g_link_state == BK7258_MB_LINK_DOWN)
    {
      select_recovery_epoch();
    }

  /* A probe request is idempotent once the transport is ready.  The boot
   * wait loop and the UART worker can request a probe concurrently;
   * reopening
   * a READY link here would abort a valid state ACK and race the first PWC
   * transaction.
   */

  up_irq_restore(flags);
  return OK;
}

void bk7258_mailbox_probe_complete(bool ready)
{
  if (!ready)
    {
      irqstate_t flags = up_irq_save();

      set_link_state(BK7258_MB_LINK_DOWN);
      up_irq_restore(flags);
    }
}

void bk7258_mailbox_force_reset(void)
{
  irqstate_t flags;

  if (!g_initialized)
    {
      return;
    }

  flags = up_irq_save();
  g_stats.resets++;
  begin_abort(-ECONNRESET);
  up_irq_restore(flags);
  nxsem_post(&g_tx_sem);
}

enum bk7258_mb_link_state bk7258_mailbox_link_state(void)
{
  irqstate_t flags = up_irq_save();
  enum bk7258_mb_link_state state = g_link_state;

  up_irq_restore(flags);
  return state;
}

bool bk7258_mailbox_link_ready(void)
{
  return bk7258_mailbox_link_state() == BK7258_MB_LINK_READY;
}

int bk7258_mailbox_wait_link_ready(unsigned int timeout_ms)
{
  clock_t deadline = clock_systime_ticks() + MSEC2TICK(timeout_ms);
  clock_t next_probe = clock_systime_ticks();

  while (!bk7258_mailbox_link_ready())
    {
      clock_t now = clock_systime_ticks();

      if ((int32_t)(clock_systime_ticks() - deadline) >= 0)
        {
          return -ETIMEDOUT;
        }

      /* Keep the boot-time probe self-starting even if the initial
       * ABORTING -> PROBING worker wakeup was consumed before RESET was
       * dispatched.  This only marks STATE pending; normal transaction
       * serialization remains in the mailbox workers.
       */

      if ((int32_t)(now - next_probe) >= 0)
        {
          /* The TX worker can complete STATE between the loop condition and
           * this retry point.  Do not reopen a link that has just become
           * ready.
           */

          if (!bk7258_mailbox_link_ready())
            {
              bk7258_mailbox_start_probe();
              bk7258_mb_uart_request_state();
            }

          next_probe = now + MSEC2TICK(100);
        }

      bk7258_mbox_kick_rx();
      nxsig_usleep(1000);
    }

  return OK;
}

void bk7258_mailbox_set_link_callback(bk7258_mb_link_callback_t callback,
                                      void *arg)
{
  irqstate_t flags = up_irq_save();

  g_link_callback = callback;
  g_link_arg = arg;
  up_irq_restore(flags);
}

uint32_t bk7258_mailbox_peer_reset_generation(void)
{
  irqstate_t flags = up_irq_save();
  uint32_t generation = g_peer_reset_generation;

  up_irq_restore(flags);
  return generation;
}

int bk7258_mbox_send_message(uint8_t command, uint8_t logical_channel,
                             uint32_t param1, uint32_t param2,
                             uint32_t param3)
{
  struct bk7258_mb_wire_message message;
  uint32_t words[4];

  memset(words, 0, sizeof(words));
  words[0] = command;
  words[1] = param1;
  words[2] = param2;
  words[3] = param3;
  memcpy(&message, words, sizeof(message));
  return bk7258_mailbox_send_wire(logical_channel, &message, NULL, NULL);
}

int bk7258_mailbox_send_pwc(uint8_t command, uint32_t p1, uint32_t p2,
                            uint32_t p3)
{
  return bk7258_mbox_send_message(command, BK7258_MB_CHAN_PWC_TX,
                                  p1, p2, p3);
}

void bk7258_mailbox_set_pwc_rx(int (*callback)(const void *message))
{
  g_pwc_rx = callback;
}

static int wait_channel(uint8_t logical_channel, unsigned int timeout_ms)
{
  unsigned int index = channel_index(logical_channel);
  clock_t deadline = clock_systime_ticks() + MSEC2TICK(timeout_ms);

  if (index >= MB_CHANNEL_COUNT)
    {
      return -EINVAL;
    }

  for (; ; )
    {
      irqstate_t flags;
      bool queued;
      int result;

      flags = up_irq_save();
      queued = g_channels[index].queued;
      result = g_channels[index].result;
      up_irq_restore(flags);
      if (!queued)
        {
          return result == -EINPROGRESS ? OK : result;
        }

      if ((int32_t)(clock_systime_ticks() - deadline) >= 0)
        {
          /* HW_CTRL carries the heartbeat.  Printing through the failing
           * transport can block indefinitely before the caller can retry.
           * Its diagnostics are available through a read-only snapshot.
           */

          if (logical_channel != BK7258_MB_CHAN_HW_CTRL_TX)
            {
              bk7258_mailbox_dump_stats();
            }

          return -ETIMEDOUT;
        }

      /* UART link bring-up drains the physical mailbox explicitly because
       * the IRQ may not be dispatched while board_late_initialize() owns the
       * boot transaction.  PWC waits run in the same context and need the
       * same progress guarantee; otherwise CP can process 0x5 and enqueue
       * its
       * ACK while AP remains asleep waiting for an IRQ-side drain.
       */

      bk7258_mbox_kick_rx();
      nxsig_usleep(1000);
    }
}

int bk7258_mailbox_wait_hw_control(unsigned int timeout_ms)
{
  return wait_channel(BK7258_MB_CHAN_HW_CTRL_TX, timeout_ms);
}

int bk7258_mailbox_wait_pwc(unsigned int timeout_ms)
{
  return wait_channel(BK7258_MB_CHAN_PWC_TX, timeout_ms);
}

/****************************************************************************
 * Name: bk7258_mailbox_fill_counters
 *
 * Description:
 *   Same numbers bk7258_mailbox_dump_stats() prints, handed back as data so
 *   that a caller measuring throughput can subtract two snapshots.  It must
 *   not print: the console is one of this transport's own channels, so a
 *   readout that logged would add the traffic it is trying to measure.
 *
 ****************************************************************************/

void bk7258_mailbox_fill_counters(struct bk7258_net_counters *counters)
{
  struct bk7258_mbox_stats physical;

  if (counters == NULL)
    {
      return;
    }

  bk7258_mbox_get_stats(&physical);

  counters->mb_tx              = g_stats.tx;
  counters->mb_rx              = g_stats.rx;
  counters->mb_timeout         = g_stats.timeout;
  counters->mb_fifo_full       = g_stats.fifo_full;
  counters->mb_bad_ack         = g_stats.bad_ack;
  counters->mb_bad_header      = g_stats.bad_header;
  counters->mb_ack_overflow    = g_stats.ack_overflow;
  counters->mb_deferred        = g_stats.deferred_command;
  counters->mb_recovery_cycle  = g_stats.recovery_cycle;
  counters->mb_recovery_replay = g_stats.recovery_replay;
  counters->mb_link_ready      = g_stats.link_ready;
  counters->mb_link_down       = g_stats.link_down;
  counters->mb_link_state      = (uint8_t)g_link_state;
  counters->mb_ack_slots_used  = (uint8_t)ack_slots_used();
  counters->mb_busy            = g_active.busy;

  counters->mb0_rx             = physical.rx_messages;
  counters->mb0_write_full     = physical.write_full;
  counters->mb0_write_error    = physical.write_error;
  counters->mb0_read_error     = physical.read_error;
  counters->mb0_desc_full      = physical.descriptor_full;
  counters->mb0_desc_deferred  = physical.descriptor_deferred;
  counters->mb0_bad_source     = physical.bad_source;
  counters->mb0_bad_length     = physical.bad_length;
  counters->mb0_bad_address    = physical.bad_address;
}

void bk7258_mailbox_dump_stats(void)
{
  struct bk7258_mbox_stats physical;

  bk7258_mbox_get_stats(&physical);
  printf("mailbox: link=%u active=%u/%u tx=%lu rx=%lu timeout=%lu "
         "bad_ack=%lu reason=%08lx bad_header=%lu ack_used=%u fifo_full=%lu "
         "deferred=%lu recovery=%lu/%lu ready=%lu down=%lu\n",
         g_link_state, g_active.busy, busy_probe() != NULL,
         (unsigned long)g_stats.tx, (unsigned long)g_stats.rx,
         (unsigned long)g_stats.timeout, (unsigned long)g_stats.bad_ack,
          (unsigned long)g_stats.bad_ack_reason,
          (unsigned long)g_stats.bad_header, ack_slots_used(),
         (unsigned long)g_stats.fifo_full,
         (unsigned long)g_stats.deferred_command,
         (unsigned long)g_stats.recovery_cycle,
         (unsigned long)g_stats.recovery_replay,
         (unsigned long)g_stats.link_ready,
         (unsigned long)g_stats.link_down);
  printf("mbox0: rx=%lu bad_sid=%lu bad_len=%lu bad_addr=%lu "
         "wrerr=%lu rderr=%lu wrfull=%lu desc_full=%lu deferred=%lu\n",
         (unsigned long)physical.rx_messages,
         (unsigned long)physical.bad_source,
         (unsigned long)physical.bad_length,
         (unsigned long)physical.bad_address,
         (unsigned long)physical.write_error,
         (unsigned long)physical.read_error,
         (unsigned long)physical.write_full,
          (unsigned long)physical.descriptor_full,
          (unsigned long)physical.descriptor_deferred);
  printf("mailbox-last: ack=%08lx active=%08lx cmd=%08lx txerr=%08lx\n",
         (unsigned long)g_stats.last_ack_header,
         (unsigned long)g_stats.last_active_header,
         (unsigned long)g_stats.last_command_header,
         (unsigned long)g_stats.last_tx_error);
  bk7258_mbox_uart_dump_stats();
}
