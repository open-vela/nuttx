/****************************************************************************
 * arch/arm/src/bk7258/bk7258_syslog.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/syslog/syslog.h>
#include <nuttx/irq.h>

#include "hardware/bk7258_mbox.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BK7258_SYSLOG_FORCE_TIMEOUT_MS 20u

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int bk7258_syslog_putc(syslog_channel_t *channel, int ch)
{
  uint8_t byte = ch;

  (void)channel;
  return bk7258_mbox_uart_write(&byte, 1) == 1 ? ch : EOF;
}

/****************************************************************************
 * Name: bk7258_syslog_settle
 *
 * Description:
 *   Get what was just buffered out to the CP before returning, which is the
 *   whole point of the force variants: their caller is reporting something
 *   it
 *   may not survive.
 *
 *   The interrupt-context case used to return without doing anything, and
 *   that
 *   is precisely the case that matters.  An assertion on this part arrives
 *   as
 *   a HardFault -- a stack-limit violation under
 *   CONFIG_ARMV8M_STACKCHECK_HARDWARE does -- so _assert() runs in exception
 *   context with interrupts off.  Its register dump and stack trace went
 *   into
 *   the console ring, mb_uart_worker() was never going to run again to move
 *   them, and the board's only remaining symptom was the assertion LED plus
 *   the CP resetting the part eight seconds later over a missing heartbeat.
 *   Every AP crash looked identical and none of them said anything.
 *
 *   bk7258_mbox_uart_flush() is not an alternative in that state: it waits
 *   with nxsig_usleep().  So the fallback goes at the hardware directly.
 *
 *   Order matters.  When a scheduler is available the ordinary transport is
 *   still preferable -- it acknowledges frames and honours the peer's flow
 *   control, which the polled drain does neither of -- so that is tried
 *   first
 *   and the drain covers both the contexts it cannot run in and the times it
 *   runs but does not finish.
 *
 *   The drain is a no-op until a crash has been declared, and that gate
 *   lives
 *   in the drain rather than here on purpose.  Interrupt context is not the
 *   same thing as a crash -- an ordinary syslog from an ISR reaches this
 *   function too, and letting the drain run for one of those is what moved
 *   the
 *   CP's link-down four times earlier in a social-mode session.  So this
 *   asks unconditionally and the drain decides whether the situation
 *   warrants
 *   it.
 *
 ****************************************************************************/

static void bk7258_syslog_settle(void)
{
  if (up_interrupt_context() ||
      bk7258_mbox_uart_flush(BK7258_SYSLOG_FORCE_TIMEOUT_MS) < 0)
    {
      bk7258_mbox_uart_drain_polled();
    }
}

static int bk7258_syslog_force(syslog_channel_t *channel, int ch)
{
  int ret = bk7258_syslog_putc(channel, ch);

  if (ret != EOF)
    {
      bk7258_syslog_settle();
    }

  return ret;
}

static ssize_t bk7258_syslog_write(syslog_channel_t *channel,
                                   const char *buffer, size_t length)
{
  size_t total = 0;

  (void)channel;
  while (total < length)
    {
      size_t chunk = length - total;
      ssize_t ret;

      if (chunk > UINT16_MAX)
        {
          chunk = UINT16_MAX;
        }

      ret = bk7258_mbox_uart_write((const uint8_t *)buffer + total, chunk);
      if (ret <= 0)
        {
          return total != 0 ? (ssize_t)total : ret;
        }

      total += ret;
      if ((size_t)ret < chunk)
        {
          break;
        }
    }

  return total;
}

static ssize_t bk7258_syslog_write_force(syslog_channel_t *channel,
                                         const char *buffer, size_t length)
{
  ssize_t ret = bk7258_syslog_write(channel, buffer, length);

  if (ret > 0)
    {
      bk7258_syslog_settle();
    }

  return ret;
}

static int bk7258_syslog_flush(syslog_channel_t *channel)
{
  (void)channel;
  if (up_interrupt_context())
    {
      return -EAGAIN;
    }

  return bk7258_mbox_uart_flush(BK7258_SYSLOG_FORCE_TIMEOUT_MS);
}

static const struct syslog_channel_ops_s g_bk7258_syslog_ops =
{
  .sc_putc = bk7258_syslog_putc,
  .sc_force = bk7258_syslog_force,
  .sc_flush = bk7258_syslog_flush,
  .sc_write = bk7258_syslog_write,
  .sc_write_force = bk7258_syslog_write_force,
  .sc_close = NULL,
};

static syslog_channel_t g_bk7258_syslog_channel =
{
  .sc_ops = &g_bk7258_syslog_ops,
};

int bk7258_syslog_initialize(void)
{
  bk7258_mbox_uart_early_init();
  return syslog_channel_register(&g_bk7258_syslog_channel);
}
