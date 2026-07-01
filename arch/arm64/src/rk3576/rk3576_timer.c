/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_timer.c
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
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "arm64_internal.h"
#include "hardware/rk3576_timer.h"
#include "rk3576_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer timeout (microseconds) */

#define TIMER_TIMEOUT_US           10000

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Timer controller base addresses */

const uint32_t g_timer_base[RK3576_TIMER_COUNT] =
{
  RK3576_TIMER0_ADDR,
  RK3576_TIMER1_ADDR,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_timer_validate
 *
 * Description:
 *   Validate controller and channel numbers.
 *
 ****************************************************************************/

static inline int rk3576_timer_validate(int controller, int channel)
{
  if (controller < 0 || controller >= RK3576_TIMER_COUNT)
    {
      tmrerr("TIMER: invalid controller %d\n", controller);
      return -EINVAL;
    }

  if (channel < 0 || channel >= RK3576_TIMER_CHANNELS)
    {
      tmrerr("TIMER: invalid channel %d\n", channel);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_timer_init
 *
 * Description:
 *   Initialize all channels in a timer controller. Disable all.
 *
 ****************************************************************************/

void rk3576_timer_init(int controller)
{
  uint32_t base;

  if (controller < 0 || controller >= RK3576_TIMER_COUNT)
    {
      return;
    }

  base = g_timer_base[controller];

  /* Disable all channels and interrupts */

  putreg32(0, base + TIMER_CTRL);

  /* Clear any pending interrupts */

  putreg32(TIMER_INT_CH0 | TIMER_INT_CH1, base + TIMER_INTCLR);

  tmrinfo("TIMER%d: initialized\n", controller);
}

/****************************************************************************
 * Name: rk3576_timer_set_mode
 *
 * Description:
 *   Set timer mode (free-running or user-defined count).
 *
 ****************************************************************************/

int rk3576_timer_set_mode(int controller, int channel, int mode)
{
  uint32_t base;
  uint32_t ctrl;
  int ret;

  ret = rk3576_timer_validate(controller, channel);
  if (ret < 0)
    {
      return ret;
    }

  base = g_timer_base[controller];

  ctrl = getreg32(base + TIMER_CTRL);

  /* Set mode for the channel */

  if (channel == 0)
    {
      ctrl &= ~TIMER_CTRL_MODE_MASK;
      ctrl |= (mode << TIMER_CTRL_MODE_SHIFT) & TIMER_CTRL_MODE_MASK;
    }
  else
    {
      /* Channel 1 uses the same mode bit */

      ctrl &= ~TIMER_CTRL_MODE_MASK;
      ctrl |= (mode << TIMER_CTRL_MODE_SHIFT) & TIMER_CTRL_MODE_MASK;
    }

  putreg32(ctrl, base + TIMER_CTRL);

  tmrinfo("TIMER%d_CH%d: mode %d\n", controller, channel, mode);
  return OK;
}

/****************************************************************************
 * Name: rk3576_timer_set_count
 *
 * Description:
 *   Set the load count value for a channel.
 *
 ****************************************************************************/

int rk3576_timer_set_count(int controller, int channel, uint32_t count)
{
  uint32_t base;
  int ret;

  ret = rk3576_timer_validate(controller, channel);
  if (ret < 0)
    {
      return ret;
    }

  base = g_timer_base[controller];

  /* Write load count */

  putreg32(count, base + TIMER_LOAD_COUNT(channel));

  tmrinfo("TIMER%d_CH%d: count %u\n", controller, channel, count);
  return OK;
}

/****************************************************************************
 * Name: rk3576_timer_set_period_us
 *
 * Description:
 *   Set timer period in microseconds. Converts to clock ticks.
 *
 ****************************************************************************/

int rk3576_timer_set_period_us(int controller, int channel, uint32_t us)
{
  uint32_t ticks;

  /* Convert microseconds to clock ticks:
   * ticks = us * CLK_FREQ / 1000000
   */

  ticks = (uint32_t)((uint64_t)TIMER_CLK_FREQ * us / 1000000);

  return rk3576_timer_set_count(controller, channel, ticks);
}

/****************************************************************************
 * Name: rk3576_timer_start
 *
 * Description:
 *   Start a timer channel.
 *
 ****************************************************************************/

void rk3576_timer_start(int controller, int channel)
{
  uint32_t base;
  uint32_t ctrl;

  if (rk3576_timer_validate(controller, channel) < 0)
    {
      return;
    }

  base = g_timer_base[controller];

  ctrl = getreg32(base + TIMER_CTRL);

  /* Enable the channel */

  ctrl |= (1 << channel);

  /* Enable interrupt for the channel */

  ctrl |= (1 << (channel + 4));

  putreg32(ctrl, base + TIMER_CTRL);

  tmrinfo("TIMER%d_CH%d: started\n", controller, channel);
}

/****************************************************************************
 * Name: rk3576_timer_stop
 *
 * Description:
 *   Stop a timer channel.
 *
 ****************************************************************************/

void rk3576_timer_stop(int controller, int channel)
{
  uint32_t base;
  uint32_t ctrl;

  if (rk3576_timer_validate(controller, channel) < 0)
    {
      return;
    }

  base = g_timer_base[controller];

  ctrl = getreg32(base + TIMER_CTRL);

  /* Disable the channel */

  ctrl &= ~(1 << channel);

  /* Disable interrupt for the channel */

  ctrl &= ~(1 << (channel + 4));

  putreg32(ctrl, base + TIMER_CTRL);

  tmrinfo("TIMER%d_CH%d: stopped\n", controller, channel);
}

/****************************************************************************
 * Name: rk3576_timer_get_count
 *
 * Description:
 *   Get the current count value of a channel.
 *
 ****************************************************************************/

uint32_t rk3576_timer_get_count(int controller, int channel)
{
  uint32_t base;

  if (rk3576_timer_validate(controller, channel) < 0)
    {
      return 0;
    }

  base = g_timer_base[controller];
  return getreg32(base + TIMER_CURRENT_COUNT(channel));
}

/****************************************************************************
 * Name: rk3576_timer_is_running
 *
 * Description:
 *   Check if a timer channel is running.
 *
 ****************************************************************************/

int rk3576_timer_is_running(int controller, int channel)
{
  uint32_t base;
  uint32_t ctrl;

  if (rk3576_timer_validate(controller, channel) < 0)
    {
      return -EINVAL;
    }

  base = g_timer_base[controller];
  ctrl = getreg32(base + TIMER_CTRL);

  return (ctrl & (1 << channel)) ? 1 : 0;
}

/****************************************************************************
 * Name: rk3576_timer_clear_irq
 *
 * Description:
 *   Clear the interrupt status for a channel.
 *
 ****************************************************************************/

void rk3576_timer_clear_irq(int controller, int channel)
{
  uint32_t base;

  if (rk3576_timer_validate(controller, channel) < 0)
    {
      return;
    }

  base = g_timer_base[controller];

  /* Write 1 to clear the interrupt status */

  putreg32(1 << channel, base + TIMER_INTCLR);
}
