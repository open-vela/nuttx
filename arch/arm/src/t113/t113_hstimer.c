/****************************************************************************
 * arch/arm/src/t113/t113_hstimer.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/timer.h>

#include "arm_internal.h"
#include "hardware/t113_hstimer.h"
#include "hardware/t113_ccu.h"
#include "hardware/t113_clk.h"
#include "t113_ccu.h"
#include "t113_hstimer.h"

#include <arch/t113/irq.h>

#if defined(CONFIG_TIMER) && \
    (defined(CONFIG_T113_HSTIMER0) || defined(CONFIG_T113_HSTIMER1))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default AHB0 clock frequency.  On T113 the AHB0 bus is typically
 * clocked at 200 MHz.  Adjust if your board configuration differs.
 */

#ifndef CONFIG_T113_HSTIMER_CLOCK
#  define CONFIG_T113_HSTIMER_CLOCK T113_HSTIMER_FREQUENCY
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct t113_hstimer_priv_s
{
  const struct timer_ops_s *ops;      /* Lower-half operations */
  int                       id;       /* Channel index (0 or 1) */
  int                       irq;      /* IRQ number */
  uint32_t                  clkfreq;  /* Clock after prescaler */
  uint8_t                   prescaler;
  tccb_t                    callback; /* User callback */
  void                     *cbarg;    /* Callback argument */
  uint32_t                  timeout;  /* Current timeout in microseconds */
  bool                      started;  /* True if timer is running */
  spinlock_t                lock;     /* Device-level spinlock */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  t113_hstimer_handler(int irq, void *context, void *arg);

static int  t113_hstimer_start(struct timer_lowerhalf_s *lower);
static int  t113_hstimer_stop(struct timer_lowerhalf_s *lower);
static int  t113_hstimer_getstatus(struct timer_lowerhalf_s *lower,
                                   struct timer_status_s *status);
static int  t113_hstimer_settimeout(struct timer_lowerhalf_s *lower,
                                    uint32_t timeout);
static int  t113_hstimer_maxtimeout(struct timer_lowerhalf_s *lower,
                                    uint32_t *maxtimeout);
static void t113_hstimer_setcallback(struct timer_lowerhalf_s *lower,
                                     tccb_t callback, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct timer_ops_s g_hstimer_ops =
{
  .start       = t113_hstimer_start,
  .stop        = t113_hstimer_stop,
  .getstatus   = t113_hstimer_getstatus,
  .settimeout  = t113_hstimer_settimeout,
  .setcallback = t113_hstimer_setcallback,
  .maxtimeout  = t113_hstimer_maxtimeout,
  .ioctl       = NULL,
};

#ifdef CONFIG_T113_HSTIMER0
static struct t113_hstimer_priv_s g_hstimer0_priv =
{
  .ops       = &g_hstimer_ops,
  .id        = 0,
  .irq       = T113_IRQ_HSTIMER0,
  .prescaler = HSTIMER_PRESCALE_1,
  .started   = false,
};
#endif

#ifdef CONFIG_T113_HSTIMER1
static struct t113_hstimer_priv_s g_hstimer1_priv =
{
  .ops       = &g_hstimer_ops,
  .id        = 1,
  .irq       = T113_IRQ_HSTIMER1,
  .prescaler = HSTIMER_PRESCALE_1,
  .started   = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_hstimer_setinterval
 *
 * Description:
 *   Program the 56-bit interval value.  INTV_LO must be written before
 *   INTV_HI per the hardware specification.
 *
 ****************************************************************************/

static void t113_hstimer_setinterval(int ch, uint64_t ticks)
{
  /* Clamp to 56-bit maximum */

  if (ticks > HSTIMER_MAX_COUNT)
    {
      ticks = HSTIMER_MAX_COUNT;
    }

  putreg32((uint32_t)(ticks & 0xffffffff), T113_HSTIMER_INTV_LO(ch));
  putreg32((uint32_t)((ticks >> 32) & HSTIMER_INTV_HI_MASK),
           T113_HSTIMER_INTV_HI(ch));
}

/****************************************************************************
 * Name: t113_hstimer_getcurrent
 *
 * Description:
 *   Read the current 56-bit counter value.  CURNT_LO must be read before
 *   CURNT_HI (hardware latches high part on LO read).
 *
 ****************************************************************************/

static uint64_t t113_hstimer_getcurrent(int ch)
{
  uint32_t lo;
  uint32_t hi;

  lo = getreg32(T113_HSTIMER_CURNT_LO(ch));
  hi = getreg32(T113_HSTIMER_CURNT_HI(ch)) & HSTIMER_CURNT_HI_MASK;

  return ((uint64_t)hi << 32) | lo;
}

/****************************************************************************
 * Name: t113_hstimer_handler
 *
 * Description:
 *   HSTimer interrupt handler.  Acknowledges the pending interrupt and
 *   invokes the user callback if registered.
 *
 ****************************************************************************/

static int t113_hstimer_handler(int irq, void *context, void *arg)
{
  struct t113_hstimer_priv_s *priv = (struct t113_hstimer_priv_s *)arg;
  uint32_t next_interval_us = 0;
  uint64_t ticks;

  /* Clear pending interrupt (write-1-to-clear) */

  putreg32(HSTIMER_IRQ_PEND_TMR(priv->id), T113_HSTIMER_IRQ_STAS);

  if (priv->callback != NULL)
    {
      if (priv->callback(&next_interval_us, priv->cbarg))
        {
          if (next_interval_us > 0)
            {
              /* Reprogram interval with new timeout */

              ticks = (uint64_t)priv->clkfreq * next_interval_us
                      / 1000000;
              t113_hstimer_setinterval(priv->id, ticks);

              /* Trigger reload.  This RMW races with start/stop/
               * setcallback on the other CPU, which hold priv->lock but
               * cannot block this IRQ.  modifyreg32 serializes against
               * any other modifyreg32 on this register.
               */

              modifyreg32(T113_HSTIMER_CTRL(priv->id), 0,
                          HSTIMER_CTRL_RELOAD);
            }
        }
      else
        {
          /* Callback returned false - stop the timer */

          t113_hstimer_stop((struct timer_lowerhalf_s *)priv);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: t113_hstimer_start
 *
 * Description:
 *   Start the timer.
 *
 ****************************************************************************/

static int t113_hstimer_start(struct timer_lowerhalf_s *lower)
{
  struct t113_hstimer_priv_s *priv = (struct t113_hstimer_priv_s *)lower;
  int ch = priv->id;
  irqstate_t flags;
  uint32_t ctrl;

  flags = spin_lock_irqsave(&priv->lock);

  if (priv->started)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return -EBUSY;
    }

  /* Set periodic mode, prescaler, reload, then enable */

  ctrl = HSTIMER_CTRL_MODE_PERIODIC |
         HSTIMER_CTRL_CLK_PRESCALE(priv->prescaler) |
         HSTIMER_CTRL_RELOAD;
  putreg32(ctrl, T113_HSTIMER_CTRL(ch));

  /* Enable interrupt for this channel */

  if (priv->callback != NULL)
    {
      uint32_t irqen = getreg32(T113_HSTIMER_IRQ_EN);
      irqen |= HSTIMER_IRQ_EN_TMR(ch);
      putreg32(irqen, T113_HSTIMER_IRQ_EN);
      up_enable_irq(priv->irq);
    }

  /* Enable the timer */

  ctrl = getreg32(T113_HSTIMER_CTRL(ch));
  ctrl |= HSTIMER_CTRL_EN;
  putreg32(ctrl, T113_HSTIMER_CTRL(ch));

  priv->started = true;

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: t113_hstimer_stop
 *
 * Description:
 *   Stop the timer.
 *
 ****************************************************************************/

static int t113_hstimer_stop(struct timer_lowerhalf_s *lower)
{
  struct t113_hstimer_priv_s *priv = (struct t113_hstimer_priv_s *)lower;
  int ch = priv->id;
  irqstate_t flags;
  uint32_t reg;

  flags = spin_lock_irqsave(&priv->lock);

  if (!priv->started)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return OK;
    }

  /* Disable interrupt for this channel */

  up_disable_irq(priv->irq);

  reg = getreg32(T113_HSTIMER_IRQ_EN);
  reg &= ~HSTIMER_IRQ_EN_TMR(ch);
  putreg32(reg, T113_HSTIMER_IRQ_EN);

  /* Clear any pending interrupt */

  putreg32(HSTIMER_IRQ_PEND_TMR(ch), T113_HSTIMER_IRQ_STAS);

  /* Disable the timer (clear EN bit) */

  reg = getreg32(T113_HSTIMER_CTRL(ch));
  reg &= ~HSTIMER_CTRL_EN;
  putreg32(reg, T113_HSTIMER_CTRL(ch));

  /* After clearing EN, wait at least 2 AHB0 clock cycles before any
   * further timer access.  DSB + ISB provide sufficient delay.
   */

  UP_DSB();
  UP_ISB();

  priv->started = false;

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: t113_hstimer_getstatus
 *
 * Description:
 *   Get the current timer status.
 *
 ****************************************************************************/

static int t113_hstimer_getstatus(struct timer_lowerhalf_s *lower,
                                  struct timer_status_s *status)
{
  struct t113_hstimer_priv_s *priv = (struct t113_hstimer_priv_s *)lower;

  DEBUGASSERT(priv != NULL && status != NULL);

  status->flags = 0;

  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback != NULL)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  /* Timeout is the programmed interval in microseconds */

  status->timeout = priv->timeout;

  /* Compute remaining time from current counter value.
   * The counter counts DOWN from the interval value.
   */

  if (priv->clkfreq > 0)
    {
      uint64_t current = t113_hstimer_getcurrent(priv->id);
      uint64_t us = current * 1000000ull / priv->clkfreq;
      status->timeleft = (uint32_t)us;
    }
  else
    {
      status->timeleft = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: t113_hstimer_settimeout
 *
 * Description:
 *   Set a new timeout value in microseconds.
 *
 ****************************************************************************/

static int t113_hstimer_settimeout(struct timer_lowerhalf_s *lower,
                                   uint32_t timeout)
{
  struct t113_hstimer_priv_s *priv = (struct t113_hstimer_priv_s *)lower;
  uint64_t ticks;

  if (timeout == 0)
    {
      return -EINVAL;
    }

  /* Convert microseconds to timer ticks */

  ticks = (uint64_t)priv->clkfreq * timeout / 1000000;

  if (ticks > HSTIMER_MAX_COUNT)
    {
      return -ERANGE;
    }

  /* Program the interval registers */

  t113_hstimer_setinterval(priv->id, ticks);

  /* Trigger a reload if timer is already running */

  if (priv->started)
    {
      uint32_t ctrl = getreg32(T113_HSTIMER_CTRL(priv->id));
      ctrl |= HSTIMER_CTRL_RELOAD;
      putreg32(ctrl, T113_HSTIMER_CTRL(priv->id));
    }

  priv->timeout = timeout;
  return OK;
}

/****************************************************************************
 * Name: t113_hstimer_maxtimeout
 *
 * Description:
 *   Get the maximum supported timeout value in microseconds.
 *
 ****************************************************************************/

static int t113_hstimer_maxtimeout(struct timer_lowerhalf_s *lower,
                                   uint32_t *maxtimeout)
{
  struct t113_hstimer_priv_s *priv = (struct t113_hstimer_priv_s *)lower;

  if (maxtimeout == NULL)
    {
      return -EINVAL;
    }

  if (priv->clkfreq == 0)
    {
      *maxtimeout = 0;
      return -EINVAL;
    }

  /* 56-bit counter at clkfreq Hz.
   * max_us = HSTIMER_MAX_COUNT * 1000000 / clkfreq
   * Cap to UINT32_MAX since the API uses uint32_t.
   */

  uint64_t max_us = HSTIMER_MAX_COUNT / priv->clkfreq * 1000000;

  *maxtimeout = (max_us > UINT32_MAX) ? UINT32_MAX : (uint32_t)max_us;

  return OK;
}

/****************************************************************************
 * Name: t113_hstimer_setcallback
 *
 * Description:
 *   Set the timer expiration callback.
 *
 ****************************************************************************/

static void t113_hstimer_setcallback(struct timer_lowerhalf_s *lower,
                                     tccb_t callback, void *arg)
{
  struct t113_hstimer_priv_s *priv = (struct t113_hstimer_priv_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  priv->callback = callback;
  priv->cbarg    = arg;

  if (callback != NULL && priv->started)
    {
      /* Enable interrupt if timer is already running */

      uint32_t irqen = getreg32(T113_HSTIMER_IRQ_EN);
      irqen |= HSTIMER_IRQ_EN_TMR(priv->id);
      putreg32(irqen, T113_HSTIMER_IRQ_EN);
      up_enable_irq(priv->irq);
    }
  else if (callback == NULL)
    {
      /* Disable interrupt when clearing callback */

      up_disable_irq(priv->irq);

      uint32_t irqen = getreg32(T113_HSTIMER_IRQ_EN);
      irqen &= ~HSTIMER_IRQ_EN_TMR(priv->id);
      putreg32(irqen, T113_HSTIMER_IRQ_EN);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_hstimer_initialize
 *
 * Description:
 *   Initialize a T113 High-Speed Timer channel and return the timer
 *   lower-half driver instance.
 *
 * Input Parameters:
 *   timer - Timer channel number (0 or 1).
 *
 * Returned Value:
 *   A pointer to the timer lower-half driver instance on success;
 *   NULL on failure.
 *
 ****************************************************************************/

struct timer_lowerhalf_s *t113_hstimer_initialize(int timer)
{
  struct t113_hstimer_priv_s *priv;

  switch (timer)
    {
#ifdef CONFIG_T113_HSTIMER0
      case 0:
        priv = &g_hstimer0_priv;
        break;
#endif
#ifdef CONFIG_T113_HSTIMER1
      case 1:
        priv = &g_hstimer1_priv;
        break;
#endif
      default:
        tmrerr("ERROR: Invalid HSTimer channel %d\n", timer);
        return NULL;
    }

  /* HSTIMER module clock + reset deassert.  Idempotent: an OR-set on
   * a bit already at 1 is a no-op.  Mirrors stm32_tim_init() pattern.
   */

  t113_ccu_modify(T113_CCU_HSTIMER_BGR, 0, T113_CCU_HSTIMER_GATING);
  t113_ccu_modify(T113_CCU_HSTIMER_BGR, 0, T113_CCU_HSTIMER_RST);

  /* Compute effective clock frequency:
   * clkfreq = AHB0_CLK / (1 << prescaler)
   */

  priv->clkfreq = CONFIG_T113_HSTIMER_CLOCK >> priv->prescaler;

  /* Reset the channel: disable timer and clear pending interrupts */

  putreg32(0, T113_HSTIMER_CTRL(priv->id));

  UP_DSB();
  UP_ISB();

  putreg32(HSTIMER_IRQ_PEND_TMR(priv->id), T113_HSTIMER_IRQ_STAS);

  /* Zero the interval */

  t113_hstimer_setinterval(priv->id, 0);

  /* Attach interrupt handler */

  irq_attach(priv->irq, t113_hstimer_handler, priv);

  priv->started  = false;
  priv->callback = NULL;
  priv->cbarg    = NULL;
  priv->timeout  = 0;

  return (struct timer_lowerhalf_s *)priv;
}

#endif /* CONFIG_TIMER && (CONFIG_T113_HSTIMER0 || CONFIG_T113_HSTIMER1) */
