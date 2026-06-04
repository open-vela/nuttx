/****************************************************************************
 * drivers/lcd/gc9503cv_dsi.c
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
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/lcd/gc9503cv_dsi.h>
#include <nuttx/video/mipi_dsi.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gc9503cv_dev_s
{
  FAR struct mipi_dsi_device         *device; /* Allocated by DSI core */
  FAR const struct gc9503cv_config_s *cfg;    /* Board-supplied glue */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Single panel instance.  Bring-up boards have exactly one GC9503CV. */

static struct gc9503cv_dev_s g_gc9503cv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gc9503cv_hw_reset
 *
 * Description:
 *   Apply the hardware reset pulse required by the GC9503CV.  The panel
 *   wants a level-clean edge into RESETB; some boards otherwise leave it
 *   floating low through power-up, which the IC interprets as "stay in
 *   deep power-down" and silently swallows the DCS init that follows.
 *   The three-step pulse below — pre-high, low, release — gives a
 *   guaranteed clean falling edge before the rising release:
 *
 *     RESETB high   10 ms   (establish a known-deasserted starting level)
 *     RESETB low    10 ms   (assert; >=10 us minimum, 10 ms swallows RC)
 *     RESETB high  120 ms   (release; wait for internal power-on per
 *                            datasheet timing diagram)
 *
 ****************************************************************************/

static int gc9503cv_hw_reset(FAR struct gc9503cv_dev_s *priv)
{
  int ret;

  ret = priv->cfg->reset(false);         /* Pre-deassert RESETB (high)      */
  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(10);                         /* 10 ms high settle               */

  ret = priv->cfg->reset(true);          /* Drive RESETB low (assert)       */
  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(10);                         /* 10 ms low pulse                 */

  ret = priv->cfg->reset(false);         /* Release RESETB high (deassert)  */
  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(120);                        /* Wait for panel power-on settle  */
  return OK;
}

/****************************************************************************
 * Name: gc9503cv_init_sequence
 *
 * Description:
 *   Send the GC9503CV DCS init command stream over DSI.  The byte sequence
 *   below is the GC9503CV IC programming list recommended by the panel
 *   datasheet for 480x800 RGB888 video mode at ~31 MHz pixel clock.  Each
 *   row is one DCS write whose first byte is the command opcode followed
 *   by zero or more parameter bytes.  Two rows (DCS 0x11 sleep-out and
 *   0x29 display-on) require a 120 ms settle delay afterwards per the
 *   datasheet timing diagram.
 *
 *   After the stream completes a DCS 0x09 (Get Display Status) read is
 *   issued as a panel-alive sanity check.  The read is best-effort: some
 *   DSI host return paths are not yet wired up at bring-up time, so a
 *   read failure is logged but does not abort init.
 *
 ****************************************************************************/

struct gc9503cv_dcs_op
{
  uint8_t  delay_ms;     /* up_mdelay() after the write; 0 for no delay     */
  uint8_t  len;          /* bytes in payload (incl. cmd byte at index 0)    */
  uint8_t  payload[64];  /* opcode + parameters; longest row is 1+52 bytes  */
};

static const struct gc9503cv_dcs_op g_init_sequence[] =
{
  { .delay_ms =   0, .len =  6,
    .payload = { 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00 } },
  { .delay_ms =   0, .len =  3,
    .payload = { 0xF6, 0x5A, 0x87 } },
  { .delay_ms =   0, .len =  2, .payload = { 0xC1, 0x3F } },
  { .delay_ms =   0, .len =  2, .payload = { 0xC2, 0x0E } },
  { .delay_ms =   0, .len =  2, .payload = { 0xC6, 0xF8 } },
  { .delay_ms =   0, .len =  2, .payload = { 0xC9, 0x10 } },
  { .delay_ms =   0, .len =  2, .payload = { 0xCD, 0x25 } },
  { .delay_ms =   0, .len =  5,
    .payload = { 0x86, 0x88, 0xA3, 0xA3, 0x31 } },
  { .delay_ms =   0, .len =  4,
    .payload = { 0x87, 0x04, 0x03, 0x66 } },
  { .delay_ms =   0, .len =  2, .payload = { 0xAC, 0x45 } },
  { .delay_ms =   0, .len =  2, .payload = { 0xF8, 0x8A } },
  { .delay_ms =   0, .len =  2, .payload = { 0xA7, 0x47 } },
  { .delay_ms =   0, .len =  2, .payload = { 0xA0, 0xDD } },
  { .delay_ms =   0, .len =  2, .payload = { 0xB1, 0x04 } },
  { .delay_ms =   0, .len =  5,
    .payload = { 0xFA, 0x08, 0x08, 0x08, 0x04 } },
  { .delay_ms =   0, .len =  2, .payload = { 0x71, 0x48 } },
  { .delay_ms =   0, .len =  2, .payload = { 0x72, 0x48 } },
  { .delay_ms =   0, .len =  3, .payload = { 0x73, 0x00, 0x44 } },
  { .delay_ms =   0, .len =  2, .payload = { 0xA3, 0xEE } },
  { .delay_ms =   0, .len =  4, .payload = { 0xFD, 0x3C, 0x3C, 0x00 } },
  { .delay_ms =   0, .len =  2, .payload = { 0x97, 0xEE } },
  { .delay_ms =   0, .len =  2, .payload = { 0x83, 0x93 } },
  { .delay_ms =   0, .len =  3, .payload = { 0x77, 0x00, 0x08 } },
  { .delay_ms =   0, .len =  2, .payload = { 0x9A, 0x98 } },
  { .delay_ms =   0, .len =  2, .payload = { 0x9B, 0x98 } },
  { .delay_ms =   0, .len =  3, .payload = { 0x82, 0x6F, 0x6F } },
  { .delay_ms =   0, .len =  2, .payload = { 0x80, 0x0D } },
  { .delay_ms =   0, .len =  3, .payload = { 0x7A, 0x13, 0x1A } },
  { .delay_ms =   0, .len =  3, .payload = { 0x7B, 0x13, 0x1A } },
  { .delay_ms =   0, .len = 33,
    .payload = { 0x6D,
                 0x1E, 0x1D, 0x1D, 0x1D, 0x1D, 0x1E, 0x02, 0x1E,
                 0x19, 0x1E, 0x1A, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
                 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1A, 0x1E, 0x19,
                 0x1E, 0x01, 0x1E, 0x1D, 0x1D, 0x1D, 0x1D, 0x1E } },
  { .delay_ms =   0, .len =  9,
    .payload = { 0x60,
                 0x18, 0x08, 0x10, 0x70, 0x18, 0x07, 0x10, 0x70 } },
  { .delay_ms =   0, .len = 17,
    .payload = { 0x64,
                 0x18, 0x06, 0x03, 0x26, 0x70, 0x03, 0x18, 0x05,
                 0x03, 0x26, 0x70, 0x03, 0x10, 0x70, 0x10, 0x70 } },
  { .delay_ms =   0, .len = 17,
    .payload = { 0x65,
                 0x18, 0x04, 0x03, 0x26, 0x70, 0x03, 0x18, 0x03,
                 0x03, 0x26, 0x70, 0x03, 0x10, 0x70, 0x10, 0x70 } },
  { .delay_ms =   0, .len = 17,
    .payload = { 0x66,
                 0x18, 0x02, 0x03, 0x26, 0x70, 0x03, 0x18, 0x01,
                 0x03, 0x26, 0x70, 0x03, 0x10, 0x70, 0x10, 0x70 } },
  { .delay_ms =   0, .len = 17,
    .payload = { 0x67,
                 0x18, 0x00, 0x03, 0x26, 0x70, 0x03, 0x10, 0x01,
                 0x03, 0x26, 0x70, 0x03, 0x10, 0x70, 0x10, 0x70 } },
  { .delay_ms =   0, .len = 14,
    .payload = { 0x68,
                 0x00, 0x08, 0x1A, 0x08, 0x19, 0x00, 0x00, 0x08,
                 0x1A, 0x08, 0x19, 0x00, 0x00 } },
  { .delay_ms =   0, .len =  8,
    .payload = { 0x69, 0x04, 0x22, 0x14, 0x22, 0x14, 0x22, 0x08 } },
  { .delay_ms =   0, .len =  9,
    .payload = { 0x6A,
                 0x08, 0x20, 0x48, 0x16, 0x45, 0x67, 0x01, 0x23 } },
  { .delay_ms =   0, .len =  2, .payload = { 0x6B, 0xA7 } },
  { .delay_ms =   0, .len = 53,
    .payload = { 0xD1,
                 0x00, 0x00, 0x00, 0x2F, 0x00, 0x4F, 0x00, 0x78,
                 0x00, 0x9C, 0x00, 0xD0, 0x00, 0xEC, 0x01, 0x1C,
                 0x01, 0x4E, 0x01, 0x9E, 0x01, 0xD7, 0x02, 0x22,
                 0x02, 0x64, 0x02, 0x66, 0x02, 0x9B, 0x02, 0xD0,
                 0x02, 0xED, 0x03, 0x10, 0x03, 0x26, 0x03, 0x43,
                 0x03, 0x56, 0x03, 0x70, 0x03, 0x82, 0x03, 0x9A,
                 0x03, 0xC0, 0x03, 0xFF } },
  { .delay_ms =   0, .len = 53,
    .payload = { 0xD2,
                 0x00, 0x00, 0x00, 0x2F, 0x00, 0x4F, 0x00, 0x78,
                 0x00, 0x9C, 0x00, 0xD0, 0x00, 0xEC, 0x01, 0x1C,
                 0x01, 0x4E, 0x01, 0x9E, 0x01, 0xD7, 0x02, 0x22,
                 0x02, 0x64, 0x02, 0x66, 0x02, 0x9B, 0x02, 0xD0,
                 0x02, 0xED, 0x03, 0x10, 0x03, 0x26, 0x03, 0x43,
                 0x03, 0x56, 0x03, 0x70, 0x03, 0x82, 0x03, 0x9A,
                 0x03, 0xC0, 0x03, 0xFF } },
  { .delay_ms =   0, .len = 53,
    .payload = { 0xD3,
                 0x00, 0x00, 0x00, 0x2F, 0x00, 0x4F, 0x00, 0x78,
                 0x00, 0x9C, 0x00, 0xD0, 0x00, 0xEC, 0x01, 0x1C,
                 0x01, 0x4E, 0x01, 0x9E, 0x01, 0xD7, 0x02, 0x22,
                 0x02, 0x64, 0x02, 0x66, 0x02, 0x9B, 0x02, 0xD0,
                 0x02, 0xED, 0x03, 0x10, 0x03, 0x26, 0x03, 0x43,
                 0x03, 0x56, 0x03, 0x70, 0x03, 0x82, 0x03, 0x9A,
                 0x03, 0xC0, 0x03, 0xFF } },
  { .delay_ms =   0, .len = 53,
    .payload = { 0xD4,
                 0x00, 0x00, 0x00, 0x2F, 0x00, 0x4F, 0x00, 0x78,
                 0x00, 0x9C, 0x00, 0xD0, 0x00, 0xEC, 0x01, 0x1C,
                 0x01, 0x4E, 0x01, 0x9E, 0x01, 0xD7, 0x02, 0x22,
                 0x02, 0x64, 0x02, 0x66, 0x02, 0x9B, 0x02, 0xD0,
                 0x02, 0xED, 0x03, 0x10, 0x03, 0x26, 0x03, 0x43,
                 0x03, 0x56, 0x03, 0x70, 0x03, 0x82, 0x03, 0x9A,
                 0x03, 0xC0, 0x03, 0xFF } },
  { .delay_ms =   0, .len = 53,
    .payload = { 0xD5,
                 0x00, 0x00, 0x00, 0x2F, 0x00, 0x4F, 0x00, 0x78,
                 0x00, 0x9C, 0x00, 0xD0, 0x00, 0xEC, 0x01, 0x1C,
                 0x01, 0x4E, 0x01, 0x9E, 0x01, 0xD7, 0x02, 0x22,
                 0x02, 0x64, 0x02, 0x66, 0x02, 0x9B, 0x02, 0xD0,
                 0x02, 0xED, 0x03, 0x10, 0x03, 0x26, 0x03, 0x43,
                 0x03, 0x56, 0x03, 0x70, 0x03, 0x82, 0x03, 0x9A,
                 0x03, 0xC0, 0x03, 0xFF } },
  { .delay_ms =   0, .len = 53,
    .payload = { 0xD6,
                 0x00, 0x00, 0x00, 0x2F, 0x00, 0x4F, 0x00, 0x78,
                 0x00, 0x9C, 0x00, 0xD0, 0x00, 0xEC, 0x01, 0x1C,
                 0x01, 0x4E, 0x01, 0x9E, 0x01, 0xD7, 0x02, 0x22,
                 0x02, 0x64, 0x02, 0x66, 0x02, 0x9B, 0x02, 0xD0,
                 0x02, 0xED, 0x03, 0x10, 0x03, 0x26, 0x03, 0x43,
                 0x03, 0x56, 0x03, 0x70, 0x03, 0x82, 0x03, 0x9A,
                 0x03, 0xC0, 0x03, 0xFF } },

  /* Sleep Out, then 120 ms settle before any further commands. */

  { .delay_ms = 120, .len =  1, .payload = { 0x11 } },

  { .delay_ms =   0, .len = 33,
    .payload = { 0x6D,
                 0x1E, 0x10, 0x0E, 0x0C, 0x0A, 0x1E, 0x02, 0x1E,
                 0x19, 0x1E, 0x1A, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
                 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1A, 0x1E, 0x19,
                 0x1E, 0x01, 0x1E, 0x09, 0x0B, 0x0D, 0x0F, 0x1E } },
  { .delay_ms =   0, .len =  3, .payload = { 0x77, 0x00, 0x00 } },
  { .delay_ms =   0, .len =  2, .payload = { 0xA6, 0x56 } },

  /* Pixel format: 24-bit RGB (DPI = 0b111, DBI = 0b111). */

  { .delay_ms =   0, .len =  2, .payload = { 0x3A, 0x77 } },

  /* Display On, then 120 ms before video stream is expected. */

  { .delay_ms = 120, .len =  1, .payload = { 0x29 } },
};

static int gc9503cv_init_sequence(FAR struct mipi_dsi_device *device)
{
  size_t total = sizeof(g_init_sequence) / sizeof(g_init_sequence[0]);
  ssize_t ret;
  size_t i;

  lcdinfo("init_sequence: %zu DCS commands\n", total);

  for (i = 0; i < total; i++)
    {
      ret = mipi_dsi_dcs_write_buffer(device,
                                      g_init_sequence[i].payload,
                                      g_init_sequence[i].len);
      if (ret < 0)
        {
          lcderr("ERROR: DCS write [%zu/%zu] %02x failed: %zd\n",
                 i, total, g_init_sequence[i].payload[0], ret);
          return (int)ret;
        }

      if (g_init_sequence[i].delay_ms != 0)
        {
          up_mdelay(g_init_sequence[i].delay_ms);
        }
    }

  lcdinfo("init_sequence: all %zu DCS writes ok\n", total);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gc9503cv_dsi_register
 *
 * Description:
 *   Register a GC9503CV MIPI DSI panel as a peripheral on the supplied DSI
 *   host.  The panel is configured for 2 data lanes, RGB888 pixel format
 *   and DSI video mode with low-power command transmission, then handed
 *   off to the host via mipi_dsi_attach().
 *
 * Input Parameters:
 *   host - DSI host returned by mipi_dsi_host_get().  Must not be NULL.
 *   cfg  - Board glue providing the reset GPIO callback.  Must not be NULL
 *          and cfg->reset must not be NULL.
 *
 * Returned Value:
 *   OK on success.  A negated errno value on failure:
 *     -EINVAL : invalid arguments
 *     -EBUSY  : a panel instance is already registered
 *     -ENOMEM : DSI device allocation failed
 *     other   : propagated from mipi_dsi_attach()
 *
 ****************************************************************************/

int gc9503cv_dsi_register(FAR struct mipi_dsi_host *host,
                          FAR const struct gc9503cv_config_s *cfg)
{
  FAR struct mipi_dsi_device *dev;
  int ret;

  if (host == NULL || cfg == NULL || cfg->reset == NULL)
    {
      return -EINVAL;
    }

  if (g_gc9503cv.device != NULL)
    {
      return -EBUSY;
    }

  dev = mipi_dsi_device_register(host, "gc9503cv", 0);
  if (dev == NULL)
    {
      lcderr("ERROR: mipi_dsi_device_register failed\n");
      return -ENOMEM;
    }

  dev->lanes      = 2;
  dev->format     = MIPI_DSI_FMT_RGB888;
  dev->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM;

  /* hs_rate = pixel_clk * bpp / lanes = 24 MHz * 24 / 2 = 288 Mbps/lane.
   * This is informational for the host PLL configuration; the actual rate
   * is derived from board layer pixel_clk_hz.
   */

  dev->hs_rate    = 288000000u;
  dev->lp_rate    = 10000000u;

  g_gc9503cv.device = dev;
  g_gc9503cv.cfg    = cfg;

  ret = mipi_dsi_attach(dev);
  if (ret < 0)
    {
      lcderr("ERROR: mipi_dsi_attach failed: %d\n", ret);
      return ret;
    }

  ret = gc9503cv_hw_reset(&g_gc9503cv);
  if (ret < 0)
    {
      lcderr("ERROR: gc9503cv_hw_reset failed: %d\n", ret);
      return ret;
    }

  /* Optional pre-init hook: lets the board start the DSI HS clock lane
   * (continuous mode) so the DPHY byte clock is stable before LP DCS.
   */

  if (cfg->pre_init != NULL)
    {
      ret = cfg->pre_init();
      if (ret < 0)
        {
          lcderr("ERROR: gc9503cv pre_init failed: %d\n", ret);
          return ret;
        }
    }

  ret = gc9503cv_init_sequence(dev);
  if (ret < 0)
    {
      lcderr("ERROR: gc9503cv_init_sequence failed: %d\n", ret);
      return ret;
    }

  return OK;
}
