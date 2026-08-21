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

/* GT911 capacitive touch screen on ESP32-P4-Function-EV-Board.
 *
 * Per the Espressif user guide V1.8 and esp-bsp (BSP_LCD_TOUCH_INT/RST =
 * GPIO_NUM_NC), the GT911 INT and RESET lines are NOT wired on this board:
 *   - they only come out on the LCD adapter J6 header (pins INT_TP /
 *     RESET_TP) and need manual dupont wires to be used;
 *   - the touch controller runs in polling mode instead.
 *
 * The GT911 is connected via I2C0 (SCL=GPIO8, SDA=GPIO7, shared with the
 * ES8311 codec and the SC2336 camera).  With INT floating at power-up the
 * controller latches I2C address 0x5D.  RESET is held high by the adapter
 * R10 pull-up, so no explicit reset sequence is needed.
 *
 * The GT9xx driver (drivers/input/gt9xx.c, CONFIG_INPUT_GT9XX) handles
 * the I2C register protocol.  Its read() performs a direct I2C read and
 * does not require an interrupt line, so the board callbacks below are
 * no-ops.  This file only registers the driver on /dev/input0.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/gt9xx.h>

#include "espressif/esp_i2c.h"

#include "esp32p4-function-ev-board.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  funev_gt911_irq_attach(FAR const struct gt9xx_board_s *state,
                                   xcpt_t isr, FAR void *arg);
static void funev_gt911_irq_enable(FAR const struct gt9xx_board_s *state,
                                   bool enable);
static int  funev_gt911_set_power(FAR const struct gt9xx_board_s *state,
                                  bool on);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gt9xx_board_s g_funev_gt911_board =
{
  .irq_attach = funev_gt911_irq_attach,
  .irq_enable = funev_gt911_irq_enable,
  .set_power  = funev_gt911_set_power
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: funev_gt911_irq_attach
 *
 * Description:
 *   Attach the touch interrupt handler.  INT is not wired on this board
 *   (BSP_LCD_TOUCH_INT = NC), so this is a no-op; the driver reads touch
 *   data by polling over I2C.
 *
 ****************************************************************************/

static int funev_gt911_irq_attach(FAR const struct gt9xx_board_s *state,
                                  xcpt_t isr, FAR void *arg)
{
  return OK;
}

/****************************************************************************
 * Name: funev_gt911_irq_enable
 *
 * Description:
 *   Enable/disable touch interrupts.  No-op for the same reason as above.
 *
 ****************************************************************************/

static void funev_gt911_irq_enable(FAR const struct gt9xx_board_s *state,
                                   bool enable)
{
}

/****************************************************************************
 * Name: funev_gt911_set_power
 *
 * Description:
 *   Power on/off the touch panel.  The adapter board keeps RESET high via
 *   R10 pull-up and power is always on, so this is a no-op.
 *
 ****************************************************************************/

static int funev_gt911_set_power(FAR const struct gt9xx_board_s *state,
                                 bool on)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: funev_touchscreen_init
 *
 * Description:
 *   Initialize the GT911 touch screen controller (polling mode).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int funev_touchscreen_init(void)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  /* 1. Get I2C bus from arch */

  i2c = esp_i2cbus_initialize(ESPRESSIF_I2C0);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "touch init: I2C0 init failed\n");
      return -ENODEV;
    }

  /* 2. Register GT911 with the NuttX GT9xx input driver */

  ret = gt9xx_register("/dev/input0", i2c,
                       FUNEV_GT911_I2C_ADDR,
                       &g_funev_gt911_board);
  if (ret < 0)
    {
      syslog(LOG_ERR, "touch init: gt9xx_register failed: %d\n", ret);
      esp_i2cbus_uninitialize(i2c);
      return ret;
    }

  syslog(LOG_INFO,
         "GT911 touch ready @0x%02x on I2C0 (/dev/input0, polling)\n",
         FUNEV_GT911_I2C_ADDR);

  return OK;
}
