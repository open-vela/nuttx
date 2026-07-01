/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_pwm.c
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
#include "hardware/rk3576_pwm.h"
#include "rk3576_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM source clock (from CRU, configurable) */

#define PWM_SRC_CLK_HZ             24000000   /* 24MHz default */

/* Register access helpers */


/****************************************************************************
 * Private Data
 ****************************************************************************/

/* PWM controller base addresses */

const uint32_t g_pwm_base[RK3576_PWM_COUNT] =
{
  RK3576_PWM0_ADDR,              /* PWM0: 0xff250000 */
  RK3576_PWM1_ADDR,              /* PWM1: 0xff260000 */
  RK3576_PWM2_ADDR,              /* PWM2: 0xff270000 */
  RK3576_PWM3_ADDR,              /* PWM3: 0xff280000 */
};

/* PWM channels per controller */

const int g_pwm_channels[RK3576_PWM_COUNT] =
{
  PWM0_CHANNELS,
  PWM1_CHANNELS,
  PWM2_CHANNELS,
  PWM3_CHANNELS,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_pwm_validate
 *
 * Description:
 *   Validate controller and channel numbers.
 *
 ****************************************************************************/

static int rk3576_pwm_validate(int controller, int channel)
{
  if (controller < 0 || controller >= RK3576_PWM_COUNT)
    {
      pwmerr("PWM: invalid controller %d\n", controller);
      return -EINVAL;
    }

  if (channel < 0 || channel >= g_pwm_channels[controller])
    {
      pwmerr("PWM: invalid channel %d on controller %d\n",
             channel, controller);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: rk3576_pwm_get_channel_base
 *
 * Description:
 *   Get the base address for a specific PWM channel.
 *
 ****************************************************************************/

static uint32_t rk3576_pwm_get_channel_base(int controller, int channel)
{
  return g_pwm_base[controller] + (channel * PWM_CHANNEL_STRIDE);
}

/****************************************************************************
 * Name: rk3576_pwm_calc_prescale
 *
 * Description:
 *   Calculate the prescaler value for a given period.
 *   Returns prescaler index and the actual period count.
 *
 ****************************************************************************/

static int rk3576_pwm_calc_prescale(uint32_t period_ns, uint32_t *cnt)
{
  uint32_t src_hz = PWM_SRC_CLK_HZ;
  uint32_t period_cycles;
  int prescale;

  /* Calculate period in clock cycles */

  period_cycles = (uint32_t)((uint64_t)src_hz * period_ns / 1000000000);

  /* Find prescaler that keeps counter within 16-bit range */

  for (prescale = 0; prescale <= 10; prescale++)
    {
      uint32_t scaled = period_cycles >> prescale;
      if (scaled > 0 && scaled <= 0xffff)
        {
          *cnt = scaled;
          return prescale;
        }
    }

  /* Use maximum prescale */

  *cnt = period_cycles >> 10;
  return 10;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_pwm_init
 *
 * Description:
 *   Initialize all PWM controllers. Disable all channels.
 *
 ****************************************************************************/

void rk3576_pwm_init(void)
{
  int ctrl;
  int ch;

  for (ctrl = 0; ctrl < RK3576_PWM_COUNT; ctrl++)
    {
      uint32_t base = g_pwm_base[ctrl];

      /* Disable all channels */

      for (ch = 0; ch < g_pwm_channels[ctrl]; ch++)
        {
          uint32_t ch_base = base + (ch * PWM_CHANNEL_STRIDE);
          putreg32(0, ch_base + PWM_CTRL);
        }
    }

  pwminfo("PWM: initialized %d controllers\n", RK3576_PWM_COUNT);
}

/****************************************************************************
 * Name: rk3576_pwm_configure
 *
 * Description:
 *   Configure PWM period and duty cycle in nanoseconds.
 *
 ****************************************************************************/

int rk3576_pwm_configure(int controller, int channel,
                         uint32_t period_ns, uint32_t duty_ns)
{
  uint32_t ch_base;
  uint32_t period_cnt;
  uint32_t duty_cnt;
  int prescale;
  uint32_t ctrl;
  irqstate_t flags;
  int ret;

  ret = rk3576_pwm_validate(controller, channel);
  if (ret < 0)
    {
      return ret;
    }

  ch_base = rk3576_pwm_get_channel_base(controller, channel);

  /* Calculate prescaler and period count */

  prescale = rk3576_pwm_calc_prescale(period_ns, &period_cnt);
  duty_cnt = (uint32_t)((uint64_t)period_cnt * duty_ns / period_ns);

  pwminfo("PWM%d_CH%d: period=%u ns, duty=%u ns, prescale=div%d, "
          "period_cnt=%u, duty_cnt=%u\n",
          controller, channel, period_ns, duty_ns,
          1 << prescale, period_cnt, duty_cnt);

  flags = up_irq_save();

  /* Disable channel during configuration */

  putreg32(0, ch_base + PWM_CTRL);

  /* Set period and duty */

  putreg32(period_cnt, ch_base + PWM_PERIOD);
  putreg32(duty_cnt, ch_base + PWM_DUTY);

  /* Configure control register */

  ctrl = PWM_CTRL_ENABLE |
         PWM_CTRL_MODE_CONTINUOUS |
         PWM_CTRL_CLK_SELECT_DIV |
         (prescale << PWM_CTRL_PRESCALE_SHIFT);

  putreg32(ctrl, ch_base + PWM_CTRL);

  up_irq_restore(flags);

  return OK;
}

/****************************************************************************
 * Name: rk3576_pwm_set_frequency
 *
 * Description:
 *   Set PWM frequency and duty cycle percentage.
 *   Convenience function for common use cases.
 *
 ****************************************************************************/

int rk3576_pwm_set_frequency(int controller, int channel,
                             uint32_t freq_hz, uint8_t duty_percent)
{
  uint32_t period_ns;
  uint32_t duty_ns;

  if (freq_hz == 0)
    {
      return -EINVAL;
    }

  if (duty_percent > 100)
    {
      duty_percent = 100;
    }

  period_ns = 1000000000 / freq_hz;
  duty_ns = (uint32_t)period_ns * duty_percent / 100;

  return rk3576_pwm_configure(controller, channel, period_ns, duty_ns);
}

/****************************************************************************
 * Name: rk3576_pwm_enable
 *
 * Description:
 *   Enable PWM output on a channel.
 *
 ****************************************************************************/

int rk3576_pwm_enable(int controller, int channel)
{
  uint32_t ch_base;
  uint32_t ctrl;
  irqstate_t flags;
  int ret;

  ret = rk3576_pwm_validate(controller, channel);
  if (ret < 0)
    {
      return ret;
    }

  ch_base = rk3576_pwm_get_channel_base(controller, channel);

  flags = up_irq_save();

  ctrl = getreg32(ch_base + PWM_CTRL);
  ctrl |= PWM_CTRL_ENABLE;
  putreg32(ctrl, ch_base + PWM_CTRL);

  up_irq_restore(flags);

  pwminfo("PWM%d_CH%d: enabled\n", controller, channel);
  return OK;
}

/****************************************************************************
 * Name: rk3576_pwm_disable
 *
 * Description:
 *   Disable PWM output on a channel.
 *
 ****************************************************************************/

int rk3576_pwm_disable(int controller, int channel)
{
  uint32_t ch_base;
  uint32_t ctrl;
  irqstate_t flags;
  int ret;

  ret = rk3576_pwm_validate(controller, channel);
  if (ret < 0)
    {
      return ret;
    }

  ch_base = rk3576_pwm_get_channel_base(controller, channel);

  flags = up_irq_save();

  ctrl = getreg32(ch_base + PWM_CTRL);
  ctrl &= ~PWM_CTRL_ENABLE;
  putreg32(ctrl, ch_base + PWM_CTRL);

  up_irq_restore(flags);

  pwminfo("PWM%d_CH%d: disabled\n", controller, channel);
  return OK;
}

/****************************************************************************
 * Name: rk3576_pwm_is_enabled
 *
 * Description:
 *   Check if a PWM channel is enabled.
 *
 ****************************************************************************/

int rk3576_pwm_is_enabled(int controller, int channel)
{
  uint32_t ch_base;
  uint32_t ctrl;
  int ret;

  ret = rk3576_pwm_validate(controller, channel);
  if (ret < 0)
    {
      return ret;
    }

  ch_base = rk3576_pwm_get_channel_base(controller, channel);
  ctrl = getreg32(ch_base + PWM_CTRL);

  return (ctrl & PWM_CTRL_ENABLE) ? 1 : 0;
}
