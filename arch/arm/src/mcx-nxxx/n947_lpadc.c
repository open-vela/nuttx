/****************************************************************************
 * arch/arm/src/mcx-nxxx/n947_lpadc.c
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
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/nxxx_clock.h"
#include "hardware/n947/n947_lpadc.h"
#include "hardware/nxxx_memorymap.h"
#include "nxxx_clockconfig.h"
#include "n947_lpadc.h"

#ifdef CONFIG_N947_LPADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef weak_function
#  define weak_function __attribute__((weak))
#endif

#define N947_LPADC_MAX_CHANNELS          32
#define N947_LPADC_DEFAULT_TRIGGER       0
#define N947_LPADC_DEFAULT_COMMAND       1
#define N947_LPADC_DEFAULT_COMMAND_INDEX (N947_LPADC_DEFAULT_COMMAND - 1)
#define N947_LPADC_FIFO                  0
#define N947_LPADC_CAL_AVGS_DEFAULT      10
#define N947_LPADC_POWERUP_DELAY         0x80

#define N947_LPADC_IE_FIFO0              (LPADC_IE_FWMIE0 | \
                                           LPADC_IE_FOFIE0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct n947_lpadc_config_s
{
  uintptr_t base;
  struct clock_regs_s clk_regs;
  struct clock_gate_reg_s clk_gate;
  uint32_t clk_source;
  uint8_t intf;
  int irq;
};

struct n947_lpadc_priv_s
{
  const struct adc_callback_s *cb;
  const struct n947_lpadc_config_s *config;
  uint8_t nchannels;
  uint8_t chanlist[N947_LPADC_MAX_CHANNELS];
  uint8_t current;
  uint8_t pending;
  uint8_t initialized;
  bool rxint;
  bool busy;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t lpadc_getreg(struct n947_lpadc_priv_s *priv,
                             uint32_t offset);
static void lpadc_putreg(struct n947_lpadc_priv_s *priv, uint32_t offset,
                         uint32_t value);
static void lpadc_modifyreg(struct n947_lpadc_priv_s *priv, uint32_t offset,
                            uint32_t clearbits, uint32_t setbits);
static void lpadc_prepare_command(struct n947_lpadc_priv_s *priv,
                                  uint8_t channel);
static int lpadc_enable_clock(struct n947_lpadc_priv_s *priv);
static void lpadc_configure(struct adc_dev_s *dev);
static int lpadc_start_conversion(struct adc_dev_s *dev);

static int lpadc_bind(struct adc_dev_s *dev,
                      const struct adc_callback_s *callback);
static void lpadc_reset(struct adc_dev_s *dev);
static int lpadc_setup(struct adc_dev_s *dev);
static void lpadc_shutdown(struct adc_dev_s *dev);
static void lpadc_rxint(struct adc_dev_s *dev, bool enable);
static int lpadc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);
static int lpadc_interrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_lpadcops =
{
  .ao_bind     = lpadc_bind,
  .ao_reset    = lpadc_reset,
  .ao_setup    = lpadc_setup,
  .ao_shutdown = lpadc_shutdown,
  .ao_rxint    = lpadc_rxint,
  .ao_ioctl    = lpadc_ioctl,
};

#ifdef CONFIG_N947_LPADC0
static const struct n947_lpadc_config_s g_lpadc0config =
{
  .base       = NXXX_ADC0_BASE,
  .clk_regs   = SYSCON_ADC0CLK,
  .clk_gate   = CLOCK_GATE_ADC0,
  .clk_source = FRO_HF_TO_ADC0,
  .intf       = 0,
  .irq        = NXXX_IRQ_ADC0,
};

static struct n947_lpadc_priv_s g_lpadc0priv =
{
  .config = &g_lpadc0config,
};

static struct adc_dev_s g_lpadc0dev =
{
  .ad_ops  = &g_lpadcops,
  .ad_priv = &g_lpadc0priv,
};
#endif

#ifdef CONFIG_N947_LPADC1
static const struct n947_lpadc_config_s g_lpadc1config =
{
  .base       = NXXX_ADC1_BASE,
  .clk_regs   = SYSCON_ADC1CLK,
  .clk_gate   = CLOCK_GATE_ADC1,
  .clk_source = FRO_HF_TO_ADC1,
  .intf       = 1,
  .irq        = NXXX_IRQ_ADC1,
};

static struct n947_lpadc_priv_s g_lpadc1priv =
{
  .config = &g_lpadc1config,
};

static struct adc_dev_s g_lpadc1dev =
{
  .ad_ops  = &g_lpadcops,
  .ad_priv = &g_lpadc1priv,
};
#endif

/****************************************************************************
 * Public Weak Functions
 ****************************************************************************/

int weak_function n947_lpadc_boardinitialize(int intf,
                                             const uint8_t *chanlist,
                                             int nchannels)
{
  UNUSED(intf);
  UNUSED(chanlist);
  UNUSED(nchannels);
  return OK;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t lpadc_getreg(struct n947_lpadc_priv_s *priv,
                             uint32_t offset)
{
  return getreg32(priv->config->base + offset);
}

static void lpadc_putreg(struct n947_lpadc_priv_s *priv, uint32_t offset,
                         uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

static void lpadc_modifyreg(struct n947_lpadc_priv_s *priv, uint32_t offset,
                            uint32_t clearbits, uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

static void lpadc_prepare_command(struct n947_lpadc_priv_s *priv,
                                  uint8_t channel)
{
  /* Command IDs are 1..15, while CMD register indexes are 0..14.  Keep the
   * first command as a simple single-ended A-side conversion, standard
   * resolution, no averaging/loop/compare, no next command.
   */

  lpadc_putreg(priv,
               N947_LPADC_CMDL_OFFSET(N947_LPADC_DEFAULT_COMMAND_INDEX),
               LPADC_CMDL_ADCH(channel & 0x1f));
  lpadc_putreg(priv,
               N947_LPADC_CMDH_OFFSET(N947_LPADC_DEFAULT_COMMAND_INDEX),
               0);

  /* Software trigger 0 starts command 1 and stores A-side results in
   * FIFO0.
   */

  lpadc_putreg(priv, N947_LPADC_TCTRL_OFFSET(N947_LPADC_DEFAULT_TRIGGER),
               LPADC_TCTRL_TCMD(N947_LPADC_DEFAULT_COMMAND));
}

static int lpadc_enable_clock(struct n947_lpadc_priv_s *priv)
{
  int ret;

  ret = nxxx_set_periphclock(priv->config->clk_regs,
                             priv->config->clk_source, 1);
  if (ret < 0)
    {
      aerr("ERROR: LPADC%d clock select failed: %d\n",
           priv->config->intf, ret);
      return ret;
    }

  ret = nxxx_set_clock_gate(priv->config->clk_gate, true);
  if (ret < 0)
    {
      aerr("ERROR: LPADC%d clock gate failed: %d\n",
           priv->config->intf, ret);
    }

  return ret;
}

static void lpadc_configure(struct adc_dev_s *dev)
{
  struct n947_lpadc_priv_s *priv = dev->ad_priv;
  irqstate_t flags;
  uint32_t regval;
  int ret;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->nchannels > 0);

  ret = lpadc_enable_clock(priv);
  if (ret < 0)
    {
      return;
    }

  ret = n947_lpadc_boardinitialize(priv->config->intf, priv->chanlist,
                                   priv->nchannels);
  if (ret < 0)
    {
      aerr("ERROR: LPADC%d board init failed: %d\n",
           priv->config->intf, ret);
      return;
    }

  flags = enter_critical_section();

  /* Reset the module and FIFO, then configure the SDK-default subset used
   * by a software-triggered one-shot conversion.
   */

  lpadc_modifyreg(priv, N947_LPADC_CTRL_OFFSET, LPADC_CTRL_ADCEN, 0);
  lpadc_modifyreg(priv, N947_LPADC_CTRL_OFFSET, 0, LPADC_CTRL_RST);
  lpadc_modifyreg(priv, N947_LPADC_CTRL_OFFSET, LPADC_CTRL_RST, 0);

  lpadc_putreg(priv, N947_LPADC_IE_OFFSET, 0);
  lpadc_putreg(priv, N947_LPADC_DE_OFFSET, 0);
  lpadc_putreg(priv, N947_LPADC_STAT_OFFSET, LPADC_STAT_W1C_MASK);
  lpadc_putreg(priv, N947_LPADC_TSTAT_OFFSET, LPADC_TSTAT_W1C_MASK);

  lpadc_modifyreg(priv, N947_LPADC_CTRL_OFFSET,
                  LPADC_CTRL_DOZEN | LPADC_CTRL_CAL_AVGS_MASK,
                  LPADC_CTRL_CAL_AVGS(N947_LPADC_CAL_AVGS_DEFAULT));

  regval = LPADC_CFG_PUDLY(N947_LPADC_POWERUP_DELAY) |
           LPADC_CFG_REFSEL(0) |
           LPADC_CFG_PWRSEL(0) |
           LPADC_CFG_TPRICTRL(0);
  lpadc_putreg(priv, N947_LPADC_CFG_OFFSET, regval);
  lpadc_putreg(priv, N947_LPADC_PAUSE_OFFSET, 0);
  lpadc_putreg(priv, N947_LPADC_FCTRL_OFFSET(0), LPADC_FCTRL_FWMARK(0));
  lpadc_putreg(priv, N947_LPADC_FCTRL_OFFSET(1), LPADC_FCTRL_FWMARK(0));

  lpadc_putreg(priv, N947_LPADC_TCTRL_OFFSET(1), 0);
  lpadc_putreg(priv, N947_LPADC_TCTRL_OFFSET(2), 0);
  lpadc_putreg(priv, N947_LPADC_TCTRL_OFFSET(3), 0);

  priv->current = 0;
  priv->pending = 0;
  priv->busy = false;
  lpadc_prepare_command(priv, priv->chanlist[0]);

  lpadc_modifyreg(priv, N947_LPADC_CTRL_OFFSET, 0,
                  LPADC_CTRL_RSTFIFO0 | LPADC_CTRL_RSTFIFO1);
  lpadc_modifyreg(priv, N947_LPADC_CTRL_OFFSET, 0, LPADC_CTRL_ADCEN);

  leave_critical_section(flags);
}

static int lpadc_start_conversion(struct adc_dev_s *dev)
{
  struct n947_lpadc_priv_s *priv = dev->ad_priv;
  irqstate_t flags;
  uint32_t status;
  int ret = OK;

  DEBUGASSERT(priv != NULL);

  flags = enter_critical_section();

  status = lpadc_getreg(priv, N947_LPADC_STAT_OFFSET);
  if (priv->busy || (status & LPADC_STAT_ADC_ACTIVE) != 0)
    {
      ret = -EBUSY;
      goto out;
    }

  priv->pending = priv->current;
  lpadc_prepare_command(priv, priv->chanlist[priv->pending]);

  lpadc_putreg(priv, N947_LPADC_STAT_OFFSET,
               status & LPADC_STAT_W1C_MASK);
  lpadc_putreg(priv, N947_LPADC_TSTAT_OFFSET, LPADC_TSTAT_W1C_MASK);

  priv->busy = true;

  /* Writes to SWTRIG are ignored while CTRL[ADCEN] is clear. */

  lpadc_putreg(priv, N947_LPADC_SWTRIG_OFFSET,
               LPADC_SWTRIG_SWT(N947_LPADC_DEFAULT_TRIGGER));

out:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: lpadc_bind
 ****************************************************************************/

static int lpadc_bind(struct adc_dev_s *dev,
                      const struct adc_callback_s *callback)
{
  struct n947_lpadc_priv_s *priv = dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: lpadc_reset
 ****************************************************************************/

static void lpadc_reset(struct adc_dev_s *dev)
{
  lpadc_configure(dev);
}

/****************************************************************************
 * Name: lpadc_setup
 ****************************************************************************/

static int lpadc_setup(struct adc_dev_s *dev)
{
  struct n947_lpadc_priv_s *priv = dev->ad_priv;
  int ret;

  DEBUGASSERT(priv != NULL);

  if (priv->initialized > 0)
    {
      priv->initialized++;
      return OK;
    }

  priv->current = 0;
  priv->pending = 0;
  priv->busy = false;

  lpadc_putreg(priv, N947_LPADC_STAT_OFFSET, LPADC_STAT_W1C_MASK);

  ret = irq_attach(priv->config->irq, lpadc_interrupt, dev);
  if (ret < 0)
    {
      aerr("ERROR: LPADC%d irq_attach failed: %d\n",
           priv->config->intf, ret);
      return ret;
    }

  priv->initialized = 1;
  up_enable_irq(priv->config->irq);
  return OK;
}

/****************************************************************************
 * Name: lpadc_shutdown
 ****************************************************************************/

static void lpadc_shutdown(struct adc_dev_s *dev)
{
  struct n947_lpadc_priv_s *priv = dev->ad_priv;

  DEBUGASSERT(priv != NULL);

  if (priv->initialized == 0)
    {
      return;
    }

  priv->initialized--;
  if (priv->initialized > 0)
    {
      return;
    }

  lpadc_rxint(dev, false);
  up_disable_irq(priv->config->irq);
  irq_detach(priv->config->irq);
  priv->busy = false;
}

/****************************************************************************
 * Name: lpadc_rxint
 ****************************************************************************/

static void lpadc_rxint(struct adc_dev_s *dev, bool enable)
{
  struct n947_lpadc_priv_s *priv = dev->ad_priv;

  DEBUGASSERT(priv != NULL);

  priv->rxint = enable;

  if (enable)
    {
      lpadc_putreg(priv, N947_LPADC_STAT_OFFSET, LPADC_STAT_W1C_MASK);
      lpadc_modifyreg(priv, N947_LPADC_IE_OFFSET, 0,
                      N947_LPADC_IE_FIFO0);
    }
  else
    {
      lpadc_modifyreg(priv, N947_LPADC_IE_OFFSET,
                      N947_LPADC_IE_FIFO0, 0);
    }
}

/****************************************************************************
 * Name: lpadc_ioctl
 ****************************************************************************/

static int lpadc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  struct n947_lpadc_priv_s *priv = dev->ad_priv;
  int ret;

  DEBUGASSERT(priv != NULL);

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        ret = lpadc_start_conversion(dev);
        break;

      case ANIOC_GET_NCHANNELS:
        ret = priv->nchannels;
        break;

      default:
        aerr("ERROR: LPADC%d unknown cmd: %d\n",
             priv->config->intf, cmd);
        ret = -ENOTTY;
        break;
    }

  UNUSED(arg);
  return ret;
}

/****************************************************************************
 * Name: lpadc_interrupt
 ****************************************************************************/

static int lpadc_interrupt(int irq, void *context, void *arg)
{
  struct adc_dev_s *dev = arg;
  struct n947_lpadc_priv_s *priv;
  uint32_t status;
  uint32_t count;
  uint32_t result;
  int ret;

  UNUSED(irq);
  UNUSED(context);

  DEBUGASSERT(dev != NULL);
  priv = dev->ad_priv;
  DEBUGASSERT(priv != NULL);

  status = lpadc_getreg(priv, N947_LPADC_STAT_OFFSET);

  if ((status & LPADC_STAT_FOF0) != 0)
    {
      lpadc_modifyreg(priv, N947_LPADC_CTRL_OFFSET, 0,
                      LPADC_CTRL_RSTFIFO0);
      priv->busy = false;

      if (priv->cb != NULL && priv->cb->au_reset != NULL)
        {
          priv->cb->au_reset(dev);
        }
    }

  count = LPADC_FCTRL_FCOUNT(lpadc_getreg(priv,
                                          N947_LPADC_FCTRL_OFFSET(
                                          N947_LPADC_FIFO)));

  while (count > 0)
    {
      result = lpadc_getreg(priv,
                            N947_LPADC_RESFIFO_OFFSET(N947_LPADC_FIFO));
      if ((result & LPADC_RESFIFO_VALID) == 0)
        {
          break;
        }

      if (priv->cb != NULL && priv->cb->au_receive != NULL)
        {
          ret = priv->cb->au_receive(dev, priv->chanlist[priv->pending],
                                     result & LPADC_RESFIFO_D_MASK);
          if (ret < 0 && priv->cb->au_reset != NULL)
            {
              priv->cb->au_reset(dev);
            }
        }

      priv->current++;
      if (priv->current >= priv->nchannels)
        {
          priv->current = 0;
        }

      priv->busy = false;
      count = LPADC_FCTRL_FCOUNT(lpadc_getreg(priv,
                                              N947_LPADC_FCTRL_OFFSET(
                                              N947_LPADC_FIFO)));
    }

  lpadc_putreg(priv, N947_LPADC_STAT_OFFSET,
               status & LPADC_STAT_W1C_MASK);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n947_lpadc_initialize
 ****************************************************************************/

struct adc_dev_s *n947_lpadc_initialize(int intf,
                                        const uint8_t *chanlist,
                                        int nchannels)
{
  struct adc_dev_s *dev;
  struct n947_lpadc_priv_s *priv;

  if (chanlist == NULL || nchannels <= 0 ||
      nchannels > N947_LPADC_MAX_CHANNELS)
    {
      aerr("ERROR: invalid LPADC channel list\n");
      return NULL;
    }

  switch (intf)
    {
#ifdef CONFIG_N947_LPADC0
      case 0:
        dev = &g_lpadc0dev;
        break;
#endif

#ifdef CONFIG_N947_LPADC1
      case 1:
        dev = &g_lpadc1dev;
        break;
#endif

      default:
        aerr("ERROR: invalid LPADC instance: %d\n", intf);
        return NULL;
    }

  priv = dev->ad_priv;
  DEBUGASSERT(priv != NULL);

  memcpy(priv->chanlist, chanlist, nchannels);
  priv->nchannels = nchannels;
  priv->current = 0;
  priv->pending = 0;
  priv->busy = false;
  priv->rxint = false;

  ainfo("LPADC%d: nchannels=%d\n", intf, nchannels);
  return dev;
}

#endif /* CONFIG_N947_LPADC */
