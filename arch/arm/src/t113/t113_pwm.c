/****************************************************************************
 * arch/arm/src/t113/t113_pwm.c
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

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/pwm.h>

#include "arm_internal.h"
#include "hardware/t113_pwm.h"
#include "hardware/t113_ccu.h"
#include "hardware/t113_clk.h"
#include "t113_ccu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define T113_PWM_CLK_HZ   T113_PWM_FREQUENCY

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct t113_pwm_s
{
  const struct pwm_ops_s *ops;
  uint8_t channel;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int t113_pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int t113_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);
static int t113_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                          FAR const struct pwm_info_s *info);
static int t113_pwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int t113_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                          int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pwm_ops_s g_pwm_ops =
{
  .setup    = t113_pwm_setup,
  .shutdown = t113_pwm_shutdown,
  .start    = t113_pwm_start,
  .stop     = t113_pwm_stop,
  .ioctl    = t113_pwm_ioctl,
};

static struct t113_pwm_s g_pwm[T113_PWM_NCHANNELS];

static spinlock_t g_pwm_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int t113_pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  struct t113_pwm_s *priv = (struct t113_pwm_s *)dev;
  irqstate_t flags;
  uint32_t pccr;
  uint32_t pccr_reg;
  int pair;

  flags = spin_lock_irqsave(&g_pwm_lock);

  /* Deassert PWM reset and enable clock gate via CCU.  The PWM BGR is
   * shared across the CCU; route through t113_ccu_modify so the AMP
   * hwspinlock backend serializes against sibling images.
   */

  t113_ccu_modify(T113_CCU_PWM_BGR, 0, (1 << 16) | (1 << 0));

  /* Program per-pair PCCR: select HOSC, divider 1, enable clock gating.
   * PCCR is shared by the two channels in a pair (0/1, 2/3, 4/5, 6/7).
   */

  pair = priv->channel >> 1;
  pccr_reg = T113_PWM_PCCR01 + pair * 4;
  pccr = getreg32(pccr_reg);
  pccr &= ~(T113_PWM_PCCR_SRC_MASK | T113_PWM_PCCR_DIV_MASK);
  pccr |= (0 << T113_PWM_PCCR_SRC_SHIFT);   /* HOSC */
  pccr |= T113_PWM_PCCR_CLK_GATING;
  putreg32(pccr, pccr_reg);

  spin_unlock_irqrestore(&g_pwm_lock, flags);

  return OK;
}

static int t113_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  struct t113_pwm_s *priv = (struct t113_pwm_s *)dev;
  irqstate_t flags;
  uint8_t ch = priv->channel;

  /* Disable channel */

  flags = spin_lock_irqsave(&g_pwm_lock);
  putreg32(getreg32(T113_PWM_PER) & ~(1 << ch), T113_PWM_PER);
  putreg32(getreg32(T113_PWM_PCGR) & ~(1 << ch), T113_PWM_PCGR);
  spin_unlock_irqrestore(&g_pwm_lock, flags);

  return OK;
}

static int t113_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                          FAR const struct pwm_info_s *info)
{
  struct t113_pwm_s *priv = (struct t113_pwm_s *)dev;
  irqstate_t flags;
  uint8_t ch = priv->channel;
  uint32_t period;
  uint32_t active;
  uint32_t prescaler;
  uint32_t pcr;
  uint32_t ppr;
  uint32_t per;
  uint32_t pcgr;

  if (info->frequency == 0 || info->frequency > T113_PWM_CLK_HZ)
    {
      return -EINVAL;
    }

  /* Calculate prescaler and period.
   * PWM freq = CLK_HZ / (prescaler + 1) / (period + 1)
   * Start with prescaler=0, increase if period > 65535.
   */

  prescaler = 0;
  period = (T113_PWM_CLK_HZ / info->frequency) - 1;

  while (period > 65535 && prescaler < 255)
    {
      prescaler++;
      period = (T113_PWM_CLK_HZ / ((prescaler + 1) * info->frequency)) - 1;
    }

  if (period > 65535)
    {
      return -EINVAL;
    }

  /* duty is ub16 (0x0000-0xFFFF maps to 0-100%).
   * Treat 0xFFFF as always-high (active = period + 1).
   */

  if (info->duty >= 0xffff)
    {
      active = period + 1;
    }
  else
    {
      active = (uint32_t)(((uint64_t)(period + 1) * info->duty) >> 16);
    }

  /* Build PCR honoring polarity from info->cpol. */

  pcr = prescaler & T113_PWM_PCR_PRESCAL_MASK;
  if (info->cpol != PWM_CPOL_LOW)
    {
      pcr |= T113_PWM_PCR_ACT_STA;
    }

  ppr = ((period & 0xffff) << T113_PWM_PPR_PERIOD_SHIFT) |
        ((active & 0xffff) << T113_PWM_PPR_ACTIVE_SHIFT);

  /* Disable channel in PER first, update PCR/PPR/PCGR, then re-enable.
   * Protects PCCR/PER/PCGR RMW against the paired channel.
   */

  flags = spin_lock_irqsave(&g_pwm_lock);

  per = getreg32(T113_PWM_PER);
  per &= ~(1 << ch);
  putreg32(per, T113_PWM_PER);

  putreg32(pcr, T113_PWM_PCR(ch));
  putreg32(ppr, T113_PWM_PPR(ch));

  pcgr = getreg32(T113_PWM_PCGR) | (1 << ch);
  putreg32(pcgr, T113_PWM_PCGR);

  per |= (1 << ch);
  putreg32(per, T113_PWM_PER);

  spin_unlock_irqrestore(&g_pwm_lock, flags);

  return OK;
}

static int t113_pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  struct t113_pwm_s *priv = (struct t113_pwm_s *)dev;
  irqstate_t flags;
  uint8_t ch = priv->channel;

  flags = spin_lock_irqsave(&g_pwm_lock);
  putreg32(getreg32(T113_PWM_PER) & ~(1 << ch), T113_PWM_PER);
  spin_unlock_irqrestore(&g_pwm_lock, flags);
  return OK;
}

static int t113_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                          int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void t113_pwm_initialize(int channel)
{
  char devpath[16];

  DEBUGASSERT(channel >= 0 && channel < T113_PWM_NCHANNELS);

  g_pwm[channel].ops     = &g_pwm_ops;
  g_pwm[channel].channel = channel;

  snprintf(devpath, sizeof(devpath), "/dev/pwm%d", channel);
  pwm_register(devpath, (FAR struct pwm_lowerhalf_s *)&g_pwm[channel]);
}
