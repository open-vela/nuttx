/****************************************************************************
 * arch/arm/src/mcx-nxxx/n947_ctimer_pwm.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/pwm.h>

#include "arm_internal.h"
#include "hardware/nxxx_clock.h"
#include "hardware/n947/n947_ctimer.h"
#include "hardware/nxxx_memorymap.h"
#include "nxxx_clockconfig.h"
#include "n947_ctimer_pwm.h"

#ifdef CONFIG_N947_CTIMER_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef weak_function
#  define weak_function __attribute__((weak))
#endif

#define N947_CTIMER_NMATCH              4
#define N947_CTIMER_DUTY_SCALE          65536u
#define N947_CTIMER_DUTY_MASK           0x0000ffffu

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct n947_ctimer_pwm_config_s
{
  uintptr_t base;
  struct clock_regs_s clk_regs;
  struct clock_gate_reg_s clk_gate;
  uint32_t clk_source;
  uint32_t clk_freq;
  uint8_t timer;
};

struct n947_ctimer_pwm_dev_s
{
  struct pwm_lowerhalf_s lower;
  const struct n947_ctimer_pwm_config_s *config;
  uint32_t frequency;
  ub16_t duty;
  uint8_t channel;
  uint8_t period_channel;
  uint8_t initialized;
  bool running;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t ctimer_getreg(struct n947_ctimer_pwm_dev_s *priv,
                              uint32_t offset);
static void ctimer_putreg(struct n947_ctimer_pwm_dev_s *priv,
                          uint32_t offset, uint32_t value);
static void ctimer_modifyreg(struct n947_ctimer_pwm_dev_s *priv,
                             uint32_t offset, uint32_t clearbits,
                             uint32_t setbits);
static int ctimer_pwm_enable_clock(struct n947_ctimer_pwm_dev_s *priv);
static void ctimer_pwm_reset_counter(struct n947_ctimer_pwm_dev_s *priv);
static int ctimer_pwm_configure(struct n947_ctimer_pwm_dev_s *priv,
                                FAR const struct pwm_info_s *info);

static int ctimer_pwm_setup(FAR struct pwm_lowerhalf_s *lower);
static int ctimer_pwm_shutdown(FAR struct pwm_lowerhalf_s *lower);
#ifdef CONFIG_PWM_PULSECOUNT
static int ctimer_pwm_start(FAR struct pwm_lowerhalf_s *lower,
                            FAR const struct pwm_info_s *info,
                            FAR void *handle);
#else
static int ctimer_pwm_start(FAR struct pwm_lowerhalf_s *lower,
                            FAR const struct pwm_info_s *info);
#endif
static int ctimer_pwm_stop(FAR struct pwm_lowerhalf_s *lower);
static int ctimer_pwm_ioctl(FAR struct pwm_lowerhalf_s *lower, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pwm_ops_s g_ctimer_pwmops =
{
  .setup    = ctimer_pwm_setup,
  .shutdown = ctimer_pwm_shutdown,
  .start    = ctimer_pwm_start,
  .stop     = ctimer_pwm_stop,
  .ioctl    = ctimer_pwm_ioctl,
};

#ifdef CONFIG_N947_CTIMER0_PWM
static const struct n947_ctimer_pwm_config_s g_ctimer0_pwm_config =
{
  .base       = N947_CTIMER0_BASE,
  .clk_regs   = SYSCON_CTIMERCLK0,
  .clk_gate   = CLOCK_GATE_TIMER0,
  .clk_source = FRO_HF_TO_CTIMER0,
  .clk_freq   = N947_CTIMER_FRO_HF_FREQ,
  .timer      = 0,
};

static struct n947_ctimer_pwm_dev_s g_ctimer0_pwm =
{
  .lower.ops = &g_ctimer_pwmops,
  .config    = &g_ctimer0_pwm_config,
};
#endif

#ifdef CONFIG_N947_CTIMER1_PWM
static const struct n947_ctimer_pwm_config_s g_ctimer1_pwm_config =
{
  .base       = N947_CTIMER1_BASE,
  .clk_regs   = SYSCON_CTIMERCLK1,
  .clk_gate   = CLOCK_GATE_TIMER1,
  .clk_source = FRO_HF_TO_CTIMER1,
  .clk_freq   = N947_CTIMER_FRO_HF_FREQ,
  .timer      = 1,
};

static struct n947_ctimer_pwm_dev_s g_ctimer1_pwm =
{
  .lower.ops = &g_ctimer_pwmops,
  .config    = &g_ctimer1_pwm_config,
};
#endif

#ifdef CONFIG_N947_CTIMER2_PWM
static const struct n947_ctimer_pwm_config_s g_ctimer2_pwm_config =
{
  .base       = N947_CTIMER2_BASE,
  .clk_regs   = SYSCON_CTIMERCLK2,
  .clk_gate   = CLOCK_GATE_TIMER2,
  .clk_source = FRO_HF_TO_CTIMER2,
  .clk_freq   = N947_CTIMER_FRO_HF_FREQ,
  .timer      = 2,
};

static struct n947_ctimer_pwm_dev_s g_ctimer2_pwm =
{
  .lower.ops = &g_ctimer_pwmops,
  .config    = &g_ctimer2_pwm_config,
};
#endif

#ifdef CONFIG_N947_CTIMER3_PWM
static const struct n947_ctimer_pwm_config_s g_ctimer3_pwm_config =
{
  .base       = N947_CTIMER3_BASE,
  .clk_regs   = SYSCON_CTIMERCLK3,
  .clk_gate   = CLOCK_GATE_TIMER3,
  .clk_source = FRO_HF_TO_CTIMER3,
  .clk_freq   = N947_CTIMER_FRO_HF_FREQ,
  .timer      = 3,
};

static struct n947_ctimer_pwm_dev_s g_ctimer3_pwm =
{
  .lower.ops = &g_ctimer_pwmops,
  .config    = &g_ctimer3_pwm_config,
};
#endif

#ifdef CONFIG_N947_CTIMER4_PWM
static const struct n947_ctimer_pwm_config_s g_ctimer4_pwm_config =
{
  .base       = N947_CTIMER4_BASE,
  .clk_regs   = SYSCON_CTIMERCLK4,
  .clk_gate   = CLOCK_GATE_TIMER4,
  .clk_source = FRO_HF_TO_CTIMER4,
  .clk_freq   = N947_CTIMER_FRO_HF_FREQ,
  .timer      = 4,
};

static struct n947_ctimer_pwm_dev_s g_ctimer4_pwm =
{
  .lower.ops = &g_ctimer_pwmops,
  .config    = &g_ctimer4_pwm_config,
};
#endif

/****************************************************************************
 * Public Weak Functions
 ****************************************************************************/

int weak_function n947_ctimer_pwm_boardinitialize(int timer, int channel)
{
  UNUSED(timer);
  UNUSED(channel);
  return OK;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t ctimer_getreg(struct n947_ctimer_pwm_dev_s *priv,
                              uint32_t offset)
{
  return getreg32(priv->config->base + offset);
}

static void ctimer_putreg(struct n947_ctimer_pwm_dev_s *priv,
                          uint32_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

static void ctimer_modifyreg(struct n947_ctimer_pwm_dev_s *priv,
                             uint32_t offset, uint32_t clearbits,
                             uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

static int ctimer_pwm_enable_clock(struct n947_ctimer_pwm_dev_s *priv)
{
  int ret;

  ret = nxxx_set_periphclock(priv->config->clk_regs,
                             priv->config->clk_source, 1);
  if (ret < 0)
    {
      pwmerr("ERROR: CTIMER%u clock select failed: %d\n",
             priv->config->timer, ret);
      return ret;
    }

  ret = nxxx_set_clock_gate(priv->config->clk_gate, true);
  if (ret < 0)
    {
      pwmerr("ERROR: CTIMER%u clock gate failed: %d\n",
             priv->config->timer, ret);
    }

  return ret;
}

static void ctimer_pwm_reset_counter(struct n947_ctimer_pwm_dev_s *priv)
{
  ctimer_putreg(priv, N947_CTIMER_TCR_OFFSET, CTIMER_TCR_CRST);
  ctimer_putreg(priv, N947_CTIMER_TCR_OFFSET, 0);
}

static int ctimer_pwm_configure(struct n947_ctimer_pwm_dev_s *priv,
                                FAR const struct pwm_info_s *info)
{
  irqstate_t flags;
  uint64_t cycles;
  uint64_t pulse;
  uint32_t duty;
  uint32_t period;
  uint32_t regval;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(info != NULL);

  if (info->frequency == 0)
    {
      return -EINVAL;
    }

  cycles = ((uint64_t)priv->config->clk_freq + (info->frequency / 2)) /
           info->frequency;
  if (cycles < 2 || cycles > UINT32_MAX)
    {
      pwmerr("ERROR: CTIMER%u unsupported frequency: %lu Hz\n",
             priv->config->timer, (unsigned long)info->frequency);
      return -ERANGE;
    }

  period = (uint32_t)cycles - 1;
  duty = (uint32_t)info->duty & N947_CTIMER_DUTY_MASK;

  /* MCX CTIMER PWM mode follows the SDK formula: the period channel resets
   * the counter, while the output channel match marks the start of the high
   * portion.  Therefore 0% duty is encoded as "match after the period" so
   * the output event never occurs.
   */

  if (duty == 0)
    {
      pulse = (uint64_t)period + 1;
    }
  else
    {
      pulse = ((uint64_t)period * (N947_CTIMER_DUTY_SCALE - duty)) >> 16;
    }

  if (pulse > UINT32_MAX)
    {
      return -ERANGE;
    }

  flags = enter_critical_section();

  ctimer_putreg(priv, N947_CTIMER_TCR_OFFSET, 0);
  ctimer_pwm_reset_counter(priv);

  ctimer_putreg(priv, N947_CTIMER_CTCR_OFFSET, CTIMER_CTCR_CTMODE_TIMER);
  ctimer_putreg(priv, N947_CTIMER_PR_OFFSET, 0);
  ctimer_putreg(priv, N947_CTIMER_PC_OFFSET, 0);

  regval = ctimer_getreg(priv, N947_CTIMER_MCR_OFFSET);
  regval &= ~(CTIMER_MCR_MR_MASK(priv->channel) |
              CTIMER_MCR_MR_MASK(priv->period_channel));
  regval |= CTIMER_MCR_MRR(priv->period_channel);
  ctimer_putreg(priv, N947_CTIMER_MCR_OFFSET, regval);

  ctimer_modifyreg(priv, N947_CTIMER_EMR_OFFSET,
                   CTIMER_EMR_EM(priv->channel) |
                   CTIMER_EMR_EMC_MASK(priv->channel),
                   0);

  ctimer_putreg(priv, N947_CTIMER_MR_OFFSET(priv->period_channel), period);
  ctimer_putreg(priv, N947_CTIMER_MR_OFFSET(priv->channel),
                (uint32_t)pulse);

  ctimer_modifyreg(priv, N947_CTIMER_PWMC_OFFSET,
                   CTIMER_PWMC_PWMEN(priv->period_channel),
                   CTIMER_PWMC_PWMEN(priv->channel));

  ctimer_putreg(priv, N947_CTIMER_IR_OFFSET, CTIMER_IR_ALL);
  ctimer_putreg(priv, N947_CTIMER_TCR_OFFSET, CTIMER_TCR_CEN);

  priv->frequency = info->frequency;
  priv->duty = info->duty;
  priv->running = true;

  leave_critical_section(flags);

  pwminfo("CTIMER%u MAT%u: freq=%lu duty=%08lx period=%lu pulse=%lu\n",
          priv->config->timer, priv->channel,
          (unsigned long)info->frequency, (unsigned long)info->duty,
          (unsigned long)period, (unsigned long)pulse);

  return OK;
}

/****************************************************************************
 * Name: ctimer_pwm_setup
 ****************************************************************************/

static int ctimer_pwm_setup(FAR struct pwm_lowerhalf_s *lower)
{
  struct n947_ctimer_pwm_dev_s *priv =
    (struct n947_ctimer_pwm_dev_s *)lower;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv != NULL);

  if (priv->initialized > 0)
    {
      priv->initialized++;
      return OK;
    }

  ret = ctimer_pwm_enable_clock(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = n947_ctimer_pwm_boardinitialize(priv->config->timer,
                                        priv->channel);
  if (ret < 0)
    {
      pwmerr("ERROR: CTIMER%u board init failed: %d\n",
             priv->config->timer, ret);
      return ret;
    }

  flags = enter_critical_section();

  ctimer_putreg(priv, N947_CTIMER_TCR_OFFSET, 0);
  ctimer_pwm_reset_counter(priv);
  ctimer_putreg(priv, N947_CTIMER_IR_OFFSET, CTIMER_IR_ALL);
  ctimer_putreg(priv, N947_CTIMER_PR_OFFSET, 0);
  ctimer_putreg(priv, N947_CTIMER_PC_OFFSET, 0);
  ctimer_putreg(priv, N947_CTIMER_CTCR_OFFSET, CTIMER_CTCR_CTMODE_TIMER);
  ctimer_putreg(priv, N947_CTIMER_MCR_OFFSET, 0);
  ctimer_putreg(priv, N947_CTIMER_EMR_OFFSET, 0);
  ctimer_putreg(priv, N947_CTIMER_PWMC_OFFSET, 0);

  priv->frequency = 0;
  priv->duty = 0;
  priv->running = false;
  priv->initialized = 1;

  leave_critical_section(flags);

  pwminfo("CTIMER%u MAT%u setup\n",
          priv->config->timer, priv->channel);
  return OK;
}

/****************************************************************************
 * Name: ctimer_pwm_shutdown
 ****************************************************************************/

static int ctimer_pwm_shutdown(FAR struct pwm_lowerhalf_s *lower)
{
  struct n947_ctimer_pwm_dev_s *priv =
    (struct n947_ctimer_pwm_dev_s *)lower;

  DEBUGASSERT(priv != NULL);

  if (priv->initialized == 0)
    {
      return OK;
    }

  priv->initialized--;
  if (priv->initialized > 0)
    {
      return OK;
    }

  ctimer_pwm_stop(lower);
  ctimer_putreg(priv, N947_CTIMER_MCR_OFFSET, 0);
  ctimer_putreg(priv, N947_CTIMER_EMR_OFFSET, 0);
  ctimer_putreg(priv, N947_CTIMER_PWMC_OFFSET, 0);

  nxxx_set_clock_gate(priv->config->clk_gate, false);
  return OK;
}

/****************************************************************************
 * Name: ctimer_pwm_start
 ****************************************************************************/

#ifdef CONFIG_PWM_PULSECOUNT
static int ctimer_pwm_start(FAR struct pwm_lowerhalf_s *lower,
                            FAR const struct pwm_info_s *info,
                            FAR void *handle)
#else
static int ctimer_pwm_start(FAR struct pwm_lowerhalf_s *lower,
                            FAR const struct pwm_info_s *info)
#endif
{
  struct n947_ctimer_pwm_dev_s *priv =
    (struct n947_ctimer_pwm_dev_s *)lower;

#ifdef CONFIG_PWM_PULSECOUNT
  UNUSED(handle);
#endif

  DEBUGASSERT(priv != NULL);

  if (info == NULL)
    {
      return -EINVAL;
    }

  if (priv->initialized == 0)
    {
      int ret;

      ret = ctimer_pwm_setup(lower);
      if (ret < 0)
        {
          return ret;
        }
    }

  return ctimer_pwm_configure(priv, info);
}

/****************************************************************************
 * Name: ctimer_pwm_stop
 ****************************************************************************/

static int ctimer_pwm_stop(FAR struct pwm_lowerhalf_s *lower)
{
  struct n947_ctimer_pwm_dev_s *priv =
    (struct n947_ctimer_pwm_dev_s *)lower;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  flags = enter_critical_section();

  ctimer_putreg(priv, N947_CTIMER_TCR_OFFSET, 0);
  ctimer_pwm_reset_counter(priv);
  ctimer_modifyreg(priv, N947_CTIMER_PWMC_OFFSET,
                   CTIMER_PWMC_PWMEN(priv->channel), 0);
  ctimer_modifyreg(priv, N947_CTIMER_EMR_OFFSET,
                   CTIMER_EMR_EM(priv->channel), 0);
  ctimer_putreg(priv, N947_CTIMER_IR_OFFSET, CTIMER_IR_ALL);
  priv->running = false;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ctimer_pwm_ioctl
 ****************************************************************************/

static int ctimer_pwm_ioctl(FAR struct pwm_lowerhalf_s *lower, int cmd,
                            unsigned long arg)
{
  UNUSED(lower);
  UNUSED(cmd);
  UNUSED(arg);
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n947_ctimer_pwminitialize
 ****************************************************************************/

struct pwm_lowerhalf_s *n947_ctimer_pwminitialize(int timer, int channel)
{
  struct n947_ctimer_pwm_dev_s *priv;

  if (channel < 0 || channel >= N947_CTIMER_NMATCH)
    {
      pwmerr("ERROR: invalid CTIMER PWM channel: %d\n", channel);
      return NULL;
    }

  if (CONFIG_N947_CTIMER_PWM_PERIOD_MATCH < 0 ||
      CONFIG_N947_CTIMER_PWM_PERIOD_MATCH >= N947_CTIMER_NMATCH)
    {
      pwmerr("ERROR: invalid CTIMER PWM period channel: %d\n",
             CONFIG_N947_CTIMER_PWM_PERIOD_MATCH);
      return NULL;
    }

  if (channel == CONFIG_N947_CTIMER_PWM_PERIOD_MATCH)
    {
      pwmerr("ERROR: CTIMER PWM channel %d is reserved for period\n",
             channel);
      return NULL;
    }

  switch (timer)
    {
#ifdef CONFIG_N947_CTIMER0_PWM
      case 0:
        priv = &g_ctimer0_pwm;
        break;
#endif

#ifdef CONFIG_N947_CTIMER1_PWM
      case 1:
        priv = &g_ctimer1_pwm;
        break;
#endif

#ifdef CONFIG_N947_CTIMER2_PWM
      case 2:
        priv = &g_ctimer2_pwm;
        break;
#endif

#ifdef CONFIG_N947_CTIMER3_PWM
      case 3:
        priv = &g_ctimer3_pwm;
        break;
#endif

#ifdef CONFIG_N947_CTIMER4_PWM
      case 4:
        priv = &g_ctimer4_pwm;
        break;
#endif

      default:
        pwmerr("ERROR: CTIMER%d PWM is not configured\n", timer);
        return NULL;
    }

  priv->channel = channel;
  priv->period_channel = CONFIG_N947_CTIMER_PWM_PERIOD_MATCH;

  pwminfo("CTIMER%d PWM: output MAT%d period MAT%d\n",
          timer, priv->channel, priv->period_channel);
  return &priv->lower;
}

#endif /* CONFIG_N947_CTIMER_PWM */
