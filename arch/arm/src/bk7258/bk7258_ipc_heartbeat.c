/****************************************************************************
 * arch/arm/src/bk7258/bk7258_ipc_heartbeat.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * BK7258 Armino-compatible HW_CTRL power-up and heartbeat service.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>

#include <sched.h>

#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>

#include "hardware/bk7258_mbox.h"
#include "include/bk7258_heartbeat.h"
#include "include/bk7258_netstats.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPC_CPU1_POWER_UP_INDICATION   1u
#define IPC_CPU1_HEART_BEAT_INDICATION 2u
#define IPC_ACK_STATE_COMPLETE         2u
#define IPC_RESPONSE_TIMEOUT_MS        600u
#define IPC_HEARTBEAT_INTERVAL         MSEC2TICK(2000)
#define IPC_HEARTBEAT_RETRY            MSEC2TICK(100)

/* How long a submitted heartbeat may stay unanswered before it is written
 * off
 * and a fresh one is sent.  See bk7258_ipc_heartbeat_poll().
 *
 * Five times the transport's own MB_TIMEOUT, so it only fires when a
 * completion really was lost rather than merely late, and an eighth of the
 * CP's 8 s budget, so writing one off still leaves room for three more
 * attempts before that budget is spent.
 */

#define IPC_HEARTBEAT_PENDING_MAX      MSEC2TICK(1000)
#define IPC_HEARTBEAT_POLL_US           10000u
/* The CP resets the system after eight seconds without this indication.
 * XTS deliberately creates short-lived threads above normal application
 * priority, so this service must remain schedulable during stress tests.
 */
#define IPC_HEARTBEAT_PRIORITY          SCHED_PRIORITY_MAX

static mutex_t g_ipc_lock = NXMUTEX_INITIALIZER;
static volatile int g_ipc_ack_result;
static volatile int g_ipc_start_result;
static volatile int g_heartbeat_result;
static volatile bool g_heartbeat_enabled;
static volatile bool g_heartbeat_pending;
static volatile uint32_t g_heartbeat_completions;
static clock_t g_next_heartbeat;
static clock_t g_heartbeat_pending_since;
static uint32_t g_heartbeat_failures;
static uint32_t g_heartbeat_abandoned;
static uint32_t g_heartbeat_submissions;
static uint32_t g_heartbeat_reported;
static bool g_ipc_started;
static struct bk7258_heartbeat_status g_worker_status;
static clock_t g_last_attempt;
static clock_t g_last_ack;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void bk7258_heartbeat_get_status(struct bk7258_heartbeat_status *status)
{
  struct bk7258_net_counters counters;
  irqstate_t flags;
  clock_t now;

  if (status == NULL)
    {
      return;
    }

  flags = up_irq_save();
  now = clock_systime_ticks();
  *status = g_worker_status;
  status->started = g_ipc_started;
  status->since_attempt_ms = status->attempts == 0 ? 0 :
    TICK2MSEC(now - g_last_attempt);
  status->since_ack_ms = status->acknowledgements == 0 ? 0 :
    TICK2MSEC(now - g_last_ack);
  bk7258_mailbox_fill_counters(&counters);
  up_irq_restore(flags);

  status->mailbox_tx = counters.mb_tx;
  status->mailbox_rx = counters.mb_rx;
  status->mailbox_timeouts = counters.mb_timeout;
  status->mailbox_bad_ack = counters.mb_bad_ack;
  status->mailbox_recoveries = counters.mb_recovery_cycle;
  status->mailbox_down = counters.mb_link_down;
  status->mailbox_state = counters.mb_link_state;
}

static int ipc_ack_result(const struct bk7258_mb_wire_message *ack,
                          int result)
{
  if (result == OK &&
      (ack == NULL || ack->reserved != IPC_ACK_STATE_COMPLETE))
    {
      result = -EREMOTEIO;
    }

  return result;
}

static void ipc_tx_complete(const struct bk7258_mb_wire_message *ack,
                            int result, void *arg)
{
  g_ipc_ack_result = ipc_ack_result(ack, result);
  if ((uintptr_t)arg == IPC_CPU1_HEART_BEAT_INDICATION &&
      g_ipc_ack_result == OK)
    {
      /* Completion runs with interrupts disabled in the mailbox transport. */

      g_worker_status.acknowledgements++;
      g_last_ack = clock_systime_ticks();
    }

  if ((uintptr_t)arg == IPC_CPU1_POWER_UP_INDICATION &&
      g_ipc_ack_result == OK)
    {
      g_heartbeat_result = OK;
      g_next_heartbeat = clock_systime_ticks() + IPC_HEARTBEAT_INTERVAL;
      g_heartbeat_enabled = true;
    }
}

static void ipc_heartbeat_complete(
  const struct bk7258_mb_wire_message *ack, int result, void *arg)
{
  (void)arg;
  g_heartbeat_result = ipc_ack_result(ack, result);
  g_heartbeat_completions++;
  g_heartbeat_pending = false;
}

static int ipc_send(uint8_t command, uint32_t parameter)
{
  struct bk7258_mb_wire_message message;
  volatile uint32_t *payload =
    (volatile uint32_t *)(uintptr_t)BK7258_IPC_TX_ADDRESS;
  int ret;

  ret = nxmutex_lock(&g_ipc_lock);
  if (ret < 0)
    {
      return ret;
    }

  *payload = parameter;
  __asm__ volatile("dmb sy" ::: "memory");

  memset(&message, 0, sizeof(message));
  message.header = command;
  message.payload_address = BK7258_IPC_TX_ADDRESS;
  message.payload_length =
    command == IPC_CPU1_HEART_BEAT_INDICATION ? sizeof(*payload) : 0;
  g_ipc_ack_result = -EINPROGRESS;

  ret = bk7258_mailbox_send_wire(BK7258_MB_CHAN_HW_CTRL_TX, &message,
                                  ipc_tx_complete,
                                  (void *)(uintptr_t)command);
  if (ret == OK)
    {
      ret = bk7258_mailbox_wait_hw_control(IPC_RESPONSE_TIMEOUT_MS);
      if (ret == OK)
        {
          ret = g_ipc_ack_result;
        }
    }

  nxmutex_unlock(&g_ipc_lock);
  return ret > 0 ? -EIO : ret;
}

static int ipc_heartbeat_send(void)
{
  irqstate_t flags;
  clock_t now;
  uint64_t elapsed;
  int ret;

  flags = up_irq_save();
  now = clock_systime_ticks();
  elapsed = g_worker_status.attempts == 0 ? 0 :
            TICK2MSEC(now - g_last_attempt);
  if (elapsed > g_worker_status.max_attempt_gap_ms)
    {
      g_worker_status.max_attempt_gap_ms = elapsed;
    }

  g_last_attempt = now;
  g_worker_status.attempts++;
  g_worker_status.sending = true;
  up_irq_restore(flags);

  ret = ipc_send(IPC_CPU1_HEART_BEAT_INDICATION, 0);

  flags = up_irq_save();
  elapsed = TICK2MSEC(clock_systime_ticks() - now);
  if (elapsed > g_worker_status.max_send_ms)
    {
      g_worker_status.max_send_ms = elapsed;
    }

  g_worker_status.last_result = ret;
  g_worker_status.failures += ret < 0;
  g_worker_status.sending = false;
  up_irq_restore(flags);
  return ret;
}

static int ipc_heartbeat_worker(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  for (; ; )
    {
      /* Match Armino's SLAVE_HB_TASK: submit the heartbeat synchronously
       * and wait for the CP ACK before starting the next interval.
       */

      nxsig_usleep(2000000u);

      /* Console output must never gate the next heartbeat, even on
       * failure.
       */

      (void)ipc_heartbeat_send();
    }

  return OK;
}

void bk7258_ipc_heartbeat_poll(void)
{
  struct bk7258_mb_wire_message message;
  volatile uint32_t *payload =
    (volatile uint32_t *)(uintptr_t)BK7258_IPC_TX_ADDRESS;
  clock_t now;
  int ret;

  if (!g_heartbeat_enabled)
    {
      return;
    }

  /* A submission that is never completed used to stop the heartbeat for
   * good,
   * which means a CP reset: this flag is what suppresses the next one, and
   * only the completion callback cleared it.
   *
   * Every ordinary failure does reach that callback -- the transport times
   * an
   * in-flight transaction out after MB_TIMEOUT, and begin_abort() fails the
   * queued ones -- but a frame still sitting in its channel queue when the
   * link leaves READY does not: dispatch_locked() will not promote it while
   * the link is recovering, and the two paths that drop to DOWN after a
   * failed
   * probe complete the probe rather than the queue.  The frame then waits
   * for
   * a READY that the missing heartbeat is about to prevent.
   *
   * Writing it off and sending another turns that into a delay.  The stale
   * frame is harmless if it does eventually go out: it carries the same
   * indication, so the peer sees a heartbeat either way.
   */

  if (g_heartbeat_pending)
    {
      if ((sclock_t)(clock_systime_ticks() - g_heartbeat_pending_since) <
          (sclock_t)IPC_HEARTBEAT_PENDING_MAX)
        {
          return;
        }

      g_heartbeat_pending = false;
      g_heartbeat_abandoned++;

      /* Powers of two only, like the failure report below: this path is
       * reached once per lost completion and the console it would print to
       * is
       * the mailbox that is already in trouble.
       */

      if ((g_heartbeat_abandoned & (g_heartbeat_abandoned - 1u)) == 0)
        {
          syslog(LOG_ERR, "IPC heartbeat submission abandoned, count=%lu\n",
                 (unsigned long)g_heartbeat_abandoned);
        }
    }

  if (g_heartbeat_completions != g_heartbeat_reported)
    {
      g_heartbeat_reported = g_heartbeat_completions;

      /* Only the first ACK is worth a line: it is the evidence that the
       * HW_CTRL round trip works at all.  The next ones say nothing new and
       * they used to print straight into the freshly-drawn NSH prompt
       * ("nsh> IPC: heartbeat queued count=1 ...") for the first six seconds
       * of every boot, which reads like a broken console.  A heartbeat that
       * stops being ACKed is not silent: the failure path below reports it
       * through syslog(LOG_ERR), and the consequence is loud anyway -- the
       * CP resets the part after CONFIG_INT_WDT_PERIOD_MS.
       */

      if (g_heartbeat_reported == 1u)
        {
          printf("IPC: heartbeat ACK result=%d, interval=%u ms\n",
                 g_heartbeat_result,
                 (unsigned int)TICK2MSEC(IPC_HEARTBEAT_INTERVAL));
        }
    }

  if (g_heartbeat_result < 0)
    {
      g_heartbeat_failures++;
      if ((g_heartbeat_failures & (g_heartbeat_failures - 1u)) == 0)
        {
          syslog(LOG_ERR, "IPC heartbeat failed: %d count=%lu\n",
                 g_heartbeat_result,
                 (unsigned long)g_heartbeat_failures);
        }

      g_heartbeat_result = OK;
    }

  now = clock_systime_ticks();
  if ((sclock_t)(now - g_next_heartbeat) < 0)
    {
      return;
    }

  *payload = 0;
  __asm__ volatile("dmb sy" ::: "memory");
  memset(&message, 0, sizeof(message));
  message.header = IPC_CPU1_HEART_BEAT_INDICATION;
  message.payload_address = BK7258_IPC_TX_ADDRESS;
  message.payload_length = sizeof(*payload);
  g_heartbeat_result = -EINPROGRESS;
  g_heartbeat_pending = true;
  g_heartbeat_pending_since = now;
  ret = bk7258_mailbox_send_wire(BK7258_MB_CHAN_HW_CTRL_TX, &message,
                                 ipc_heartbeat_complete, NULL);
  if (ret == OK)
    {
      g_heartbeat_submissions++;
      g_next_heartbeat = now + IPC_HEARTBEAT_INTERVAL;
    }
  else
    {
      g_heartbeat_pending = false;
      g_heartbeat_result = ret;
      g_next_heartbeat = now + IPC_HEARTBEAT_RETRY;
    }
}

int bk7258_ipc_heartbeat_start(void)
{
  int pid;
  int ret;

  if (g_ipc_started)
    {
      return g_ipc_start_result;
    }

  g_ipc_start_result = -EINPROGRESS;
  memset(&g_worker_status, 0, sizeof(g_worker_status));
  g_heartbeat_enabled = false;
  g_heartbeat_pending = false;
  g_heartbeat_completions = 0;
  g_heartbeat_failures = 0;
  g_heartbeat_abandoned = 0;
  g_heartbeat_submissions = 0;
  g_heartbeat_reported = 0;
  ret = ipc_send(IPC_CPU1_POWER_UP_INDICATION, 0);
  g_ipc_start_result = ret < 0 ? ret : OK;

  if (g_ipc_start_result == OK)
    {
      /* The CP starts its watchdog as soon as POWER_UP is acknowledged. Send
       * one heartbeat synchronously before handing periodic work to the
       * worker, so a delayed scheduler or link transition cannot consume the
       * entire first watchdog window.
       */

      ret = ipc_heartbeat_send();

      pid = kthread_create("ipc-heartbeat", IPC_HEARTBEAT_PRIORITY, 1536,
                           ipc_heartbeat_worker, NULL);
      if (pid < 0)
        {
          g_ipc_start_result = pid;
        }
    }

  g_ipc_started = g_ipc_start_result == OK;

  if (g_ipc_start_result == OK)
    {
      printf("IPC: v31 heartbeat worker active, interval=2000 ms; "
             "use xiaopai ipc status\n");
    }

  return g_ipc_start_result;
}
