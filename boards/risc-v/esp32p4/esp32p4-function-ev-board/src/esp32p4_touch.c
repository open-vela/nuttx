/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_touch.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied.  See the License for the specific language governing
 * permissions and limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/gt9xx.h>

#include "espressif/esp_i2c.h"
#include "esp32p4-function-ev-board.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Goodix GT911 I2C address (7-bit). The address is latched by the
 * controller while leaving reset and depends on the INT level at that
 * moment; on this panel it comes up at 0x5d. */

#define TP_I2C_ADDR  0x5d

/* The panel's INT line is not wired to the ESP32-P4, so the driver
 * cannot use a real GPIO interrupt. The line is simulated with a
 * watchdog that re-fires the registered ISR at a fixed rate; the I2C
 * traffic itself always happens in the reader thread when it services
 * the pending flag. */

#define TP_POLL_INTERVAL_MS 20

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* State of the simulated interrupt source */

static xcpt_t g_tp_isr;        /* ISR registered by the GT9xx driver */
static FAR void *g_tp_arg;     /* Argument handed back to the ISR */
static struct wdog_s g_tp_wdog;
static volatile bool g_tp_polling;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tp_poll_expiry
 *
 * Description:
 *   Watchdog expiry: invoke the touch driver ISR exactly as a real
 *   INT-line interrupt would, then re-arm. Runs in timer context,
 *   which is the same context class as a GPIO ISR; the driver ISR
 *   only raises its pending flag and notifies poll() waiters.
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
      wd_start(&g_tp_wdog, MSEC2TICK(TP_POLL_INTERVAL_MS),
               tp_poll_expiry, 0);
    }
}

/****************************************************************************
 * Name: tp_irq_attach
 *
 * Description:
 *   Remember the driver ISR so the watchdog can invoke it.
 *
 ****************************************************************************/

static int tp_irq_attach(const struct gt9xx_board_s *state, xcpt_t isr,
                         FAR void *arg)
{
  g_tp_isr = isr;
  g_tp_arg = arg;
  return OK;
}

/****************************************************************************
 * Name: tp_irq_enable
 *
 * Description:
 *   Start or stop the simulated interrupt stream.
 *
 ****************************************************************************/

static void tp_irq_enable(const struct gt9xx_board_s *state, bool enable)
{
  if (enable)
    {
      if (!g_tp_polling)
        {
          g_tp_polling = true;
          wd_start(&g_tp_wdog, MSEC2TICK(TP_POLL_INTERVAL_MS),
                   tp_poll_expiry, 0);
        }
    }
  else
    {
      g_tp_polling = false;
      wd_cancel(&g_tp_wdog);
    }
}

/****************************************************************************
 * Name: tp_set_power
 *
 * Description:
 *   The touch controller shares the display power domain and is
 *   already supplied once the display path is up; report success
 *   without toggling anything.
 *
 ****************************************************************************/

static int tp_set_power(const struct gt9xx_board_s *state, bool on)
{
  return OK;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gt9xx_board_s g_tp_board =
{
  .irq_attach = tp_irq_attach,
  .irq_enable = tp_irq_enable,
  .set_power  = tp_set_power,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gt911_initialize
 *
 * Description:
 *   Initialize the Goodix GT911 touch controller on I2C0 (GPIO7 SDA,
 *   GPIO8 SCL). The controller is registered with the simulated
 *   interrupt source described above, since the panel INT line is
 *   not routed to the SoC.
 *
 * Input Parameters:
 *   busno - I2C bus number (0 for I2C0)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_gt911_initialize(int busno)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  iinfo("Initializing GT911 on I2C%d, addr=0x%02x\n", busno, TP_I2C_ADDR);

  /* Get I2C bus */

  i2c = esp_i2cbus_initialize(busno);
  if (i2c == NULL)
    {
      ierr("ERROR: Failed to get I2C%d bus\n", busno);
      return -ENODEV;
    }

  /* Register the GT911 driver with the simulated interrupt board
   * hooks. Touch coordinates are read by the driver whenever a client
   * services the pending flag raised by the watchdog. */

  ret = gt9xx_register("/dev/input0", i2c, TP_I2C_ADDR, &g_tp_board);
  if (ret < 0)
    {
      ierr("ERROR: gt9xx_register failed: %d\n", ret);
    }

  return ret;
}
