/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_touch.c
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

#include <debug.h>
#include <errno.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/irq.h>
#include <arch/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/gt9xx.h>

#include "espressif/esp_gpio.h"
#include "espressif/esp_i2c.h"

#ifdef CONFIG_INPUT_GT9XX

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GT911 touch controller of the 7" 1024x600 kit: I2C0 (SCL=8/SDA=7),
 * default address 0x5d, INT_TP wired to GPIO23 with a DuPont jumper.
 */

#define TP_I2C_PORT   0
#define TP_I2C_ADDR   0x5d
#define TP_INT_PIN    23

/* The vendor BSP for this board runs the GT911 in polling mode (its touch
 * INT line is not used), so in addition to the GPIO interrupt a periodic
 * watchdog invokes the driver's data path.  50 Hz is plenty for UI input.
 */

#define TP_POLL_DELAY MSEC2TICK(20)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  tp_irq_attach(const struct gt9xx_board_s *state, xcpt_t isr,
                          FAR void *arg);
static void tp_irq_enable(const struct gt9xx_board_s *state, bool enable);
static int  tp_set_power(const struct gt9xx_board_s *state, bool on);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gt9xx_board_s g_gt9xx_board =
{
  .irq_attach = tp_irq_attach,
  .irq_enable = tp_irq_enable,
  .set_power  = tp_set_power,
};

static struct wdog_s g_tp_wdog;
static xcpt_t g_tp_isr;
static FAR void *g_tp_arg;
static volatile bool g_tp_polling;

/****************************************************************************
 * Name: tp_poll_expiry
 *
 * Description:
 *   Periodic poll: invoke the touch driver's interrupt handler as if the
 *   INT line had fired, then re-arm.  Runs in timer interrupt context,
 *   which is the same context class as the real GPIO ISR.
 *
 ****************************************************************************/

static void tp_poll_expiry(wdparm_t arg)
{
  if (g_tp_isr != NULL)
    {
      g_tp_isr(0, NULL, g_tp_arg);
    }

  if (g_tp_polling)
    {
      wd_start(&g_tp_wdog, TP_POLL_DELAY, tp_poll_expiry, 0);
    }
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int tp_irq_attach(const struct gt9xx_board_s *state, xcpt_t isr,
                         FAR void *arg)
{
  g_tp_isr = isr;
  g_tp_arg = arg;

  esp_configgpio(TP_INT_PIN, INPUT | PULLUP | RISING);

  /* esp_gpio_irq() performs the pin-to-IRQ translation, allocates the
   * interrupt adapter and attaches + enables the pin interrupt in one go
   * (same path the board button driver uses).
   */

  return esp_gpio_irq(TP_INT_PIN, isr, arg);
}

static void tp_irq_enable(const struct gt9xx_board_s *state, bool enable)
{
  if (enable)
    {
      esp_gpioirqenable(TP_INT_PIN);
      if (!g_tp_polling)
        {
          g_tp_polling = true;
          wd_start(&g_tp_wdog, TP_POLL_DELAY, tp_poll_expiry, 0);
        }
    }
  else
    {
      g_tp_polling = false;
      wd_cancel(&g_tp_wdog);
      esp_gpioirqdisable(TP_INT_PIN);
    }
}

static int tp_set_power(const struct gt9xx_board_s *state, bool on)
{
  /* The touch panel is permanently powered by the display subboard */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_touch_initialize
 *
 * Description:
 *   Register the GT911 touch panel as /dev/input0.
 *
 ****************************************************************************/

int board_touch_initialize(void)
{
  struct i2c_master_s *i2c;

  syslog(LOG_INFO, "Touch: i2c init\n");
  i2c = esp_i2cbus_initialize(TP_I2C_PORT);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "Touch: failed to get I2C%d bus\n", TP_I2C_PORT);
      return -ENODEV;
    }

  syslog(LOG_INFO, "Touch: gt9xx_register\n");
  return gt9xx_register("/dev/input0", i2c, TP_I2C_ADDR, &g_gt9xx_board);
}

/****************************************************************************
 * Name: board_touch_diag
 *
 * Description:
 *   Boot-time GT911 register probe: dump the product ID, the config
 *   version and X/Y resolution from the configuration area, and sample
 *   the touch status register once for the boot log.
 *
 ****************************************************************************/

static int tp_reg_read(FAR struct i2c_master_s *i2c, uint16_t reg,
                       FAR uint8_t *buf, size_t len)
{
  uint8_t regbuf[2] =
  {
    reg >> 8, reg & 0xff
  };

  struct i2c_msg_s msgv[2] =
  {
    {
      .frequency = 400000,
      .addr      = TP_I2C_ADDR,
      .flags     = 0,
      .buffer    = regbuf,
      .length    = 2
    },
    {
      .frequency = 400000,
      .addr      = TP_I2C_ADDR,
      .flags     = I2C_M_READ,
      .buffer    = buf,
      .length    = len
    }
  };

  return I2C_TRANSFER(i2c, msgv, 2);
}

void board_touch_diag(void)
{
  FAR struct i2c_master_s *i2c = esp_i2cbus_initialize(TP_I2C_PORT);
  uint8_t pid[5];
  uint8_t cfg[5];
  uint8_t status;

  if (i2c == NULL)
    {
      return;
    }

  memset(pid, 0, sizeof(pid));
  if (tp_reg_read(i2c, 0x8140, pid, 4) == 0)
    {
      syslog(LOG_INFO, "TouchDiag: product id '%c%c%c%c'\n",
             pid[0], pid[1], pid[2], pid[3]);
    }

  if (tp_reg_read(i2c, 0x8047, cfg, 5) == 0)
    {
      syslog(LOG_INFO,
             "TouchDiag: cfg ver 0x%02x, x_max %d, y_max %d\n",
             cfg[0], cfg[1] | (cfg[2] << 8), cfg[3] | (cfg[4] << 8));
    }

  status = 0xff;
  if (tp_reg_read(i2c, 0x814e, &status, 1) == 0)
    {
      syslog(LOG_INFO, "TouchDiag: status 0x%02x\n", status);
    }
}

#endif /* CONFIG_INPUT_GT9XX */
