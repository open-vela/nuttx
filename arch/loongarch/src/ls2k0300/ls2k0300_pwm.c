/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_pwm.c
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
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <fixedmath.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/pwm.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_pwm.h"

#if defined(CONFIG_LS2K0300_PWM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LS2K0300_PWM_CLOCK_FREQ 200000000UL

struct ls2k0300_pwm_priv_s
{
  struct pwm_lowerhalf_s lower;
  uintptr_t base;
  uint32_t clock_freq;
  uint32_t low_buffer;
  uint32_t full_buffer;
  bool initialized;
};

static uint32_t pwm_getreg(FAR struct ls2k0300_pwm_priv_s *priv,
                           uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static void pwm_putreg(FAR struct ls2k0300_pwm_priv_s *priv,
                       uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static int ls2k0300_pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct ls2k0300_pwm_priv_s *priv =
      (FAR struct ls2k0300_pwm_priv_s *)dev;
  uint32_t regval;

  regval = pwm_getreg(priv, LS2K0300_PWM_CTRL);
  regval &= ~PWM_CTRL_EN;
  pwm_putreg(priv, LS2K0300_PWM_CTRL, regval);

  regval = pwm_getreg(priv, LS2K0300_PWM_CTRL);
  regval |= PWM_CTRL_RST;
  pwm_putreg(priv, LS2K0300_PWM_CTRL, regval);
  regval &= ~PWM_CTRL_RST;
  pwm_putreg(priv, LS2K0300_PWM_CTRL, regval);

  priv->initialized = true;
  return OK;
}

static int ls2k0300_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct ls2k0300_pwm_priv_s *priv =
      (FAR struct ls2k0300_pwm_priv_s *)dev;
  uint32_t regval;

  pwm_putreg(priv, LS2K0300_PWM_LOW_BUFFER, 1);
  if (!priv->full_buffer)
    {
      pwm_putreg(priv, LS2K0300_PWM_FULL_BUFFER, 10000);
    }

  regval = pwm_getreg(priv, LS2K0300_PWM_CTRL);
  regval |= PWM_CTRL_RST;
  pwm_putreg(priv, LS2K0300_PWM_CTRL, regval);
  regval &= ~PWM_CTRL_RST;
  pwm_putreg(priv, LS2K0300_PWM_CTRL, regval);

  regval = pwm_getreg(priv, LS2K0300_PWM_CTRL);
  regval &= ~(PWM_CTRL_EN | PWM_CTRL_OE);
  pwm_putreg(priv, LS2K0300_PWM_CTRL, regval);

  priv->initialized = false;
  return OK;
}

static int ls2k0300_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                             FAR const struct pwm_info_s *info)
{
  FAR struct ls2k0300_pwm_priv_s *priv =
      (FAR struct ls2k0300_pwm_priv_s *)dev;
  uint32_t regval;
  uint64_t val;

  val = (uint64_t)priv->clock_freq * info->frequency;
  val = val / 1000000UL;
  if (val < 1)
    {
      val = 1;
    }

  priv->full_buffer = (uint32_t)val;

  val = (uint64_t)priv->full_buffer * (uint64_t)info->duty;
  val = val / 65536UL;
  if (val < 1)
    {
      val = 1;
    }

  priv->low_buffer = (uint32_t)val;

  pwm_putreg(priv, LS2K0300_PWM_LOW_BUFFER, priv->low_buffer);
  pwm_putreg(priv, LS2K0300_PWM_FULL_BUFFER, priv->full_buffer);

  /* OE is active-low: 0=output enabled, 1=output masked.
   * Only set EN to enable counter, leave OE=0 for output.
   */

  regval = pwm_getreg(priv, LS2K0300_PWM_CTRL);
  regval &= ~PWM_CTRL_OE;
  regval |= PWM_CTRL_EN;
  pwm_putreg(priv, LS2K0300_PWM_CTRL, regval);

  return OK;
}

static int ls2k0300_pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct ls2k0300_pwm_priv_s *priv =
      (FAR struct ls2k0300_pwm_priv_s *)dev;
  uint32_t regval;

  pwm_putreg(priv, LS2K0300_PWM_LOW_BUFFER, 1);
  if (!priv->full_buffer)
    {
      pwm_putreg(priv, LS2K0300_PWM_FULL_BUFFER, 10000);
    }

  regval = pwm_getreg(priv, LS2K0300_PWM_CTRL);
  regval |= PWM_CTRL_RST;
  pwm_putreg(priv, LS2K0300_PWM_CTRL, regval);
  regval &= ~PWM_CTRL_RST;
  pwm_putreg(priv, LS2K0300_PWM_CTRL, regval);

  regval = pwm_getreg(priv, LS2K0300_PWM_CTRL);
  regval &= ~(PWM_CTRL_EN | PWM_CTRL_OE);
  pwm_putreg(priv, LS2K0300_PWM_CTRL, regval);

  return OK;
}

static int ls2k0300_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                             int cmd, unsigned long arg)
{
  FAR struct ls2k0300_pwm_priv_s *priv =
      (FAR struct ls2k0300_pwm_priv_s *)dev;
  uint32_t regval;

  switch (cmd)
    {
      case PWMIOC_START:
        ls2k0300_pwm_setup(dev);
        return ls2k0300_pwm_start(dev, (FAR const struct pwm_info_s *)arg);

      case PWMIOC_STOP:
        return ls2k0300_pwm_stop(dev);

      default:
        break;
    }

  switch (cmd)
    {
      case 0x2001:
        regval = pwm_getreg(priv, LS2K0300_PWM_CTRL);
        regval |= PWM_CTRL_INVERT;
        pwm_putreg(priv, LS2K0300_PWM_CTRL, regval);
        break;

      case 0x2002:
        regval = pwm_getreg(priv, LS2K0300_PWM_CTRL);
        regval &= ~PWM_CTRL_INVERT;
        pwm_putreg(priv, LS2K0300_PWM_CTRL, regval);
        break;

      default:
        return -ENOTTY;
    }

  return OK;
}

static const struct pwm_ops_s g_pwm_ops =
{
  ls2k0300_pwm_setup,
  ls2k0300_pwm_shutdown,
  ls2k0300_pwm_start,
  ls2k0300_pwm_stop,
  ls2k0300_pwm_ioctl
};

static struct ls2k0300_pwm_priv_s g_pwm_priv[4] =
{
  {
    .lower =
    {
      .ops = &g_pwm_ops
    },

    .base = LS2K0300_PWM0_BASE,
    .clock_freq = LS2K0300_PWM_CLOCK_FREQ,
  },
  {
    .lower =
    {
      .ops = &g_pwm_ops
    },

    .base = LS2K0300_PWM1_BASE,
    .clock_freq = LS2K0300_PWM_CLOCK_FREQ,
  },
  {
    .lower =
    {
      .ops = &g_pwm_ops
    },

    .base = LS2K0300_PWM2_BASE,
    .clock_freq = LS2K0300_PWM_CLOCK_FREQ,
  },
  {
    .lower =
    {
      .ops = &g_pwm_ops
    },

    .base = LS2K0300_PWM3_BASE,
    .clock_freq = LS2K0300_PWM_CLOCK_FREQ,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int ls2k0300_pwm_initialize(int port)
{
  char path[16];
  int ret;

  if (port < 0 || port > 3)
    {
      return -EINVAL;
    }

  snprintf(path, sizeof(path), "/dev/pwm%d", port);

  ret = pwm_register(path, &g_pwm_priv[port].lower);
  if (ret < 0)
    {
      pwmerr("ERROR: pwm_register failed: %d\n", ret);
      return ret;
    }

  return OK;
}

#endif
