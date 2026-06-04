/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_ctp_board.c
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

#include <errno.h>
#include <debug.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "hardware/t113_gpio.h"
#include "t113_gpio.h"
#include "t113_i2c.h"

#ifdef CONFIG_T113_CTP_GT9XX
#  include <nuttx/input/gt9xx.h>
#endif

#ifdef CONFIG_T113_CTP_FT5X06
#  include <nuttx/input/ft5x06.h>
#endif

#ifdef CONFIG_T113_CTP_TLSC6X
#  include <nuttx/input/tlsc6x.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CTP_I2C_BUS      2
#define CTP_GT911_ADDR   0x5d
#define CTP_FT5X06_ADDR  0x38
#define CTP_TLSC6X_ADDR  0x2e
#define CTP_I2C_FREQ     400000

/****************************************************************************
 * GT9xx Board Callbacks
 ****************************************************************************/

#ifdef CONFIG_T113_CTP_GT9XX

static int t113_gt9xx_irq_attach(const struct gt9xx_board_s *state,
                                 xcpt_t isr, FAR void *arg);
static void t113_gt9xx_irq_enable(const struct gt9xx_board_s *state,
                                  bool enable);
static int t113_gt9xx_set_power(const struct gt9xx_board_s *state,
                                bool on);

static const struct gt9xx_board_s g_t113_gt9xx =
{
  .irq_attach = t113_gt9xx_irq_attach,
  .irq_enable = t113_gt9xx_irq_enable,
  .set_power  = t113_gt9xx_set_power,
};

static int t113_gt9xx_irq_attach(const struct gt9xx_board_s *state,
                                 xcpt_t isr, FAR void *arg)
{
  int ret;

  UNUSED(state);

  ret = t113_gpio_irq_attach(T113_CTP_INT_PIN,
                             T113_EINT_MODE_RISING, isr, arg);
  if (ret < 0)
    {
      ierr("gt9xx irq_attach failed: %d\n", ret);
    }

  return ret;
}

static void t113_gt9xx_irq_enable(const struct gt9xx_board_s *state,
                                  bool enable)
{
  UNUSED(state);
  t113_gpio_irq_enable(T113_CTP_INT_PIN, enable);
}

static int t113_gt9xx_set_power(const struct gt9xx_board_s *state,
                                bool on)
{
  UNUSED(state);
  UNUSED(on);

  /* VCC-LCD rail is always on in hardware - no software power gate. */

  return OK;
}

#endif /* CONFIG_T113_CTP_GT9XX */

/****************************************************************************
 * FT5x06 Board Callbacks
 ****************************************************************************/

#ifdef CONFIG_T113_CTP_FT5X06

static int t113_ft5x06_attach(FAR const struct ft5x06_config_s *config,
                              xcpt_t isr, FAR void *arg);
static void t113_ft5x06_enable(FAR const struct ft5x06_config_s *config,
                               bool enable);
static void t113_ft5x06_clear(FAR const struct ft5x06_config_s *config);
static void t113_ft5x06_wakeup(FAR const struct ft5x06_config_s *config);
static void t113_ft5x06_nreset(FAR const struct ft5x06_config_s *config,
                               bool state);

static const struct ft5x06_config_s g_t113_ft5x06_cfg =
{
  .address   = CTP_FT5X06_ADDR,
  .frequency = CTP_I2C_FREQ,
#ifndef CONFIG_FT5X06_POLLMODE
  .attach    = t113_ft5x06_attach,
  .enable    = t113_ft5x06_enable,
  .clear     = t113_ft5x06_clear,
#endif
  .wakeup    = t113_ft5x06_wakeup,
  .nreset    = t113_ft5x06_nreset,
};

#ifndef CONFIG_FT5X06_POLLMODE

static int t113_ft5x06_attach(FAR const struct ft5x06_config_s *config,
                              xcpt_t isr, FAR void *arg)
{
  UNUSED(config);
  return t113_gpio_irq_attach(T113_CTP_INT_PIN,
                              T113_EINT_MODE_FALLING, isr, arg);
}

static void t113_ft5x06_enable(FAR const struct ft5x06_config_s *config,
                               bool enable)
{
  UNUSED(config);
  t113_gpio_irq_enable(T113_CTP_INT_PIN, enable);
}

static void t113_ft5x06_clear(FAR const struct ft5x06_config_s *config)
{
  UNUSED(config);

  /* EINT status is cleared by the dispatcher in t113_gpio.c.  Nothing
   * additional required here.
   */
}

#endif /* !CONFIG_FT5X06_POLLMODE */

static void t113_ft5x06_wakeup(FAR const struct ft5x06_config_s *config)
{
  UNUSED(config);

  /* No dedicated wakeup pin on this board - rely on nreset sequencing. */
}

static void t113_ft5x06_nreset(FAR const struct ft5x06_config_s *config,
                               bool state)
{
  UNUSED(config);
  t113_gpio_write(T113_CTP_RST_PIN, state);
}

#endif /* CONFIG_T113_CTP_FT5X06 */

/****************************************************************************
 * TLSC6X Board Callbacks
 ****************************************************************************/

#ifdef CONFIG_T113_CTP_TLSC6X

static int t113_tlsc6x_attach(FAR const struct tlsc6x_config_s *cfg,
                              xcpt_t isr, FAR void *arg)
{
  UNUSED(cfg);
  return t113_gpio_irq_attach(T113_CTP_INT_PIN,
                              T113_EINT_MODE_FALLING, isr, arg);
}

static void t113_tlsc6x_enable(FAR const struct tlsc6x_config_s *cfg,
                               bool enable)
{
  UNUSED(cfg);
  t113_gpio_irq_enable(T113_CTP_INT_PIN, enable);
}

static void t113_tlsc6x_nreset(FAR const struct tlsc6x_config_s *cfg,
                               bool state)
{
  UNUSED(cfg);
  t113_gpio_write(T113_CTP_RST_PIN, state);
}

static const struct tlsc6x_config_s g_t113_tlsc6x_cfg =
{
  .address   = CTP_TLSC6X_ADDR,
  .frequency = CTP_I2C_FREQ,

  /* Transform left at 0/false -> driver picks up Kconfig defaults
   * (exchange_xy=y, revert_x=y, max 800x480 - set in defconfig).
   */

  .attach    = t113_tlsc6x_attach,
  .enable    = t113_tlsc6x_enable,
  .nreset    = t113_tlsc6x_nreset,
};

#endif /* CONFIG_T113_CTP_TLSC6X */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_ctp_initialize
 *
 * Description:
 *   Bring the capacitive touch panel on J27 up and register the NuttX
 *   input driver at `devpath` (e.g. "/dev/input0").
 *
 *   Sequence (GT911 path):
 *     1. Configure RST(PD18) OUTPUT=0, INT(PD19) OUTPUT=0  - hold reset
 *        and force address latch to 0x5D
 *     2. 10 ms delay
 *     3. RST high  - release reset
 *     4. 60 us delay (address latch window)
 *     5. INT as INPUT with EINT trigger  - trigger edge is set when the
 *        upper driver calls irq_attach via the board callback
 *     6. 100 ms delay  - GT911 firmware init window
 *     7. Open TWI2 and register gt9xx at 0x5D
 *
 *   FT5x06 skips the address trick in step 1; the pin is configured
 *   as INPUT from the start.
 *
 ****************************************************************************/

int t113_ctp_initialize(const char *devpath)
{
  FAR struct i2c_master_s *bus;
#ifdef CONFIG_T113_CTP_TLSC6X
  int ret;
#endif

  /* TLSC6X reset sequence:
   *
   *    RST = HIGH
   *    sleep 1 ms
   *    RST = LOW
   *    sleep 20 ms
   *    RST = HIGH
   *    sleep 30 ms
   *
   * INT pin is never driven by the MCU - it stays INPUT throughout
   * (the IC drives it).  The GT911 address-latch trick that pulls
   * INT low at reset release would fight the IC's push-pull output
   * and corrupt power-on, so it is not applied here.
   */

  t113_gpio_config(T113_CTP_RST_PIN);
  t113_gpio_write(T113_CTP_RST_PIN, true);
  up_mdelay(1);
  t113_gpio_write(T113_CTP_RST_PIN, false);
  up_mdelay(20);
  t113_gpio_write(T113_CTP_RST_PIN, true);
  up_mdelay(30);

  /* INT as INPUT (the upper driver attaches its ISR later). */

  t113_gpio_config(T113_CTP_INT_PIN);
  t113_gpio_irq_initialize();

  /* Open TWI2 master.  Note: t113_bringup() already calls
   * t113_i2cbus_initialize(2) and (under CONFIG_I2C_DRIVER) registers
   * /dev/i2c2; this second call returns the same singleton, so we get
   * the bus pointer here and skip the i2c_register() to avoid a stale
   * second char node.  The redundant clock/IRQ re-init it does is
   * harmless and worth the simpler API.
   */

  bus = t113_i2cbus_initialize(CTP_I2C_BUS);
  if (bus == NULL)
    {
      ierr("t113_i2cbus_initialize(%d) failed\n", CTP_I2C_BUS);
      return -ENODEV;
    }

#ifdef CONFIG_T113_CTP_TLSC6X
  ret = tlsc6x_register(devpath, bus, &g_t113_tlsc6x_cfg);
  if (ret < 0)
    {
      ierr("tlsc6x_register(%s) failed: %d\n", devpath, ret);
      return ret;
    }
#else
  UNUSED(devpath);
  UNUSED(bus);
#endif

  return OK;
}
