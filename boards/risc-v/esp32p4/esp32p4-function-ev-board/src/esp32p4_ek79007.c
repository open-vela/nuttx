/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_ek79007.c
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

/* EK79007AD MIPI-DSI panel driver for the ESP32-P4-Function-EV-Board.
 *
 * Panel: AML070JGI50-07403L, 7.0 inch, 1024(RGB) x 600, landscape.
 * Driver IC: EK79007AD (1536-ch source driver + TCON, MIPI interface).
 * The companion EK73217BCGA gate drivers are driven by the EK79007's
 * built-in TCON, so software only configures the EK79007AD over MIPI-DSI.
 *
 * Init sequence follows the Espressif ESP-IDF esp_lcd_ek79007 component:
 *   1. Hardware reset (GRB pin, active low)
 *   2. 0xB2 (PAD_CONTROL) = 0x10  -> 2-lane mode
 *   3. Vendor registers 0x80..0x86
 *   4. 0x11 (sleep out), delay 120 ms
 *   5. Video start + 0x29 (display on) handled by up_fbinitialize
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/video/mipi_dsi.h>

#include <arch/board/board.h>

#include "esp32p4-function-ev-board.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EK79007 vendor commands (from ESP-IDF esp_lcd_ek79007) */

#define EK79007_PAD_CONTROL     0xb2
#define EK79007_DSI_2_LANE      0x10
#define EK79007_DSI_4_LANE      0x00

#define EK79007_CMD_SWRESET     0x01
#define EK79007_CMD_SLPOUT      0x11
#define EK79007_CMD_DISPON      0x29

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Command table entry: DCS write, optional payload, optional delay */

struct ek79007_cmd_s
{
  uint8_t cmd;
  FAR const uint8_t *data;
  uint8_t len;
  uint16_t delay_ms;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Vendor-specific init (Espressif esp_lcd_ek79007 default sequence) */

static const uint8_t g_ek79007_r80[] = {0x8b};
static const uint8_t g_ek79007_r81[] = {0x78};
static const uint8_t g_ek79007_r82[] = {0x84};
static const uint8_t g_ek79007_r83[] = {0x88};
static const uint8_t g_ek79007_r84[] = {0xa8};
static const uint8_t g_ek79007_r85[] = {0xe3};
static const uint8_t g_ek79007_r86[] = {0x88};
static const uint8_t g_ek79007_lane2[] = {EK79007_DSI_2_LANE};

static const struct ek79007_cmd_s g_ek79007_init[] =
{
  /* 2-lane MIPI-DSI mode */

  {EK79007_PAD_CONTROL, g_ek79007_lane2, sizeof(g_ek79007_lane2), 0},

  /* Vendor registers */

  {0x80, g_ek79007_r80, sizeof(g_ek79007_r80), 0},
  {0x81, g_ek79007_r81, sizeof(g_ek79007_r81), 0},
  {0x82, g_ek79007_r82, sizeof(g_ek79007_r82), 0},
  {0x83, g_ek79007_r83, sizeof(g_ek79007_r83), 0},
  {0x84, g_ek79007_r84, sizeof(g_ek79007_r84), 0},
  {0x85, g_ek79007_r85, sizeof(g_ek79007_r85), 0},
  {0x86, g_ek79007_r86, sizeof(g_ek79007_r86), 0},

  /* Sleep out, then wait 120 ms */

  {EK79007_CMD_SLPOUT, NULL, 0, 120},
};

#define EK79007_INIT_COUNT \
  (sizeof(g_ek79007_init) / sizeof(g_ek79007_init[0]))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ek79007_send_init(FAR struct mipi_dsi_device *device)
{
  size_t i;
  int n;

  for (i = 0; i < EK79007_INIT_COUNT; i++)
    {
      FAR const struct ek79007_cmd_s *cmd = &g_ek79007_init[i];

      n = mipi_dsi_dcs_write(device, cmd->cmd, cmd->data, cmd->len);
      if (n < 0)
        {
          syslog(LOG_ERR, "ek79007: DCS write 0x%02x failed: %d\n",
                 cmd->cmd, n);
          return n;
        }

      if (cmd->delay_ms > 0)
        {
          up_mdelay(cmd->delay_ms);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: funev_ek79007_initialize
 *
 * Description:
 *   Register a MIPI-DSI device for the EK79007 panel, attach to the DSI
 *   host, perform a hardware reset and send the vendor init sequence.
 *   Returns the device handle used by up_fbinitialize for sleep-out /
 *   display-on.
 *
 * Input Parameters:
 *   host - The MIPI-DSI host (from esp_mipi_dsi_host_get).
 *
 * Returned Value:
 *   Pointer to the mipi_dsi_device, or NULL on failure.
 *
 ****************************************************************************/

FAR struct mipi_dsi_device *
funev_ek79007_initialize(FAR struct mipi_dsi_host *host)
{
  FAR struct mipi_dsi_device *device;
  int ret;

  if (host == NULL)
    {
      return NULL;
    }

  /* Register a DSI device on virtual channel 0 */

  device = mipi_dsi_device_register(host, "ek79007", 0);
  if (device == NULL)
    {
      syslog(LOG_ERR, "ek79007: device register failed\n");
      return NULL;
    }

  /* Attach to host (host must already be configured/running) */

  ret = mipi_dsi_attach(device);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ek79007: attach failed: %d\n", ret);
      goto errout;
    }

  /* Hardware reset (GRB pin, active low): assert 10 ms, release, 20 ms */

  funev_lcd_reset();

  /* Send vendor init register set (includes sleep-out + 120 ms) */

  ret = ek79007_send_init(device);
  if (ret < 0)
    {
      goto errout;
    }

  syslog(LOG_INFO, "ek79007: panel init OK (1024x600, 2-lane)\n");
  return device;

errout:

  /* No mipi_dsi_device_unregister() in this NuttX; the device stays
   * registered but unusable.  Return NULL so the caller bails out.
   */

  UNUSED(device);
  return NULL;
}
