/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_hdmi.c
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
#include "hardware/rk3576_hdmi.h"
#include "rk3576_hdmi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HDMI timeout (microseconds) */

#define HDMI_TIMEOUT_US            100000
#define HDMI_PLL_TIMEOUT_US        10000
#define HDMI_PHY_TIMEOUT_US        5000

/* Default resolution */

#define HDMI_DEFAULT_RESOLUTION    RK3576_HDMI_RES_1280x720

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* HDMI base address */

const uint32_t g_hdmi_base = RK3576_HDMI_ADDR;

/* Video timing tables (CEA-861 timings) */

static const struct rk3576_hdmi_timing_s g_hdmi_timings[] =
{
  /* 640x480@60Hz */
  {
    .hactive = 640, .hsync_start = 656, .hsync_end = 752, .htotal = 800,
    .vactive = 480, .vsync_start = 490, .vsync_end = 492, .vtotal = 525,
    .pixel_clock = 25200000,
  },

  /* 800x600@60Hz */
  {
    .hactive = 800, .hsync_start = 840, .hsync_end = 968, .htotal = 1056,
    .vactive = 600, .vsync_start = 601, .vsync_end = 605, .vtotal = 628,
    .pixel_clock = 40000000,
  },

  /* 1024x768@60Hz */
  {
    .hactive = 1024, .hsync_start = 1048, .hsync_end = 1184, .htotal = 1344,
    .vactive = 768, .vsync_start = 771, .vsync_end = 777, .vtotal = 806,
    .pixel_clock = 65000000,
  },

  /* 1280x720@60Hz */
  {
    .hactive = 1280, .hsync_start = 1328, .hsync_end = 1440, .htotal = 1650,
    .vactive = 720, .vsync_start = 725, .vsync_end = 730, .vtotal = 750,
    .pixel_clock = 74250000,
  },

  /* 1280x1024@60Hz */
  {
    .hactive = 1280, .hsync_start = 1328, .hsync_end = 1440, .htotal = 1688,
    .vactive = 1024, .vsync_start = 1025, .vsync_end = 1028, .vtotal = 1066,
    .pixel_clock = 108000000,
  },

  /* 1920x1080@60Hz */
  {
    .hactive = 1920, .hsync_start = 2008, .hsync_end = 2052, .htotal = 2200,
    .vactive = 1080, .vsync_start = 1084, .vsync_end = 1089, .vtotal = 1125,
    .pixel_clock = 148500000,
  },
};

#define HDMI_TIMING_COUNT  (sizeof(g_hdmi_timings) / sizeof(g_hdmi_timings[0]))

/* HDMI state */

struct rk3576_hdmi_dev_s
{
  int resolution;                /* Current resolution */
  int format;                    /* Current format */
  int mode;                      /* HDMI/DVI mode */
  bool connected;                /* Cable connected */
  bool enabled;                  /* Output enabled */
  bool initialized;              /* Initialized */
};

static struct rk3576_hdmi_dev_s g_hdmi_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_hdmi_wait_phy
 *
 * Description:
 *   Wait for PHY to be ready.
 *
 ****************************************************************************/

static int rk3576_hdmi_wait_phy(void)
{
  int timeout = HDMI_PHY_TIMEOUT_US;

  while (timeout--)
    {
      if (getreg32(g_hdmi_base + HDMI_PHY_STATUS) & HDMI_PHY_STATUS_LOCKED)
        {
          return OK;
        }

      up_udelay(10);
    }

  uerr("HDMI: PHY lock timeout\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_hdmi_config_phy
 *
 * Description:
 *   Configure HDMI PHY for the given pixel clock.
 *
 ****************************************************************************/

static int rk3576_hdmi_config_phy(uint32_t pixel_clock)
{
  uint32_t ctrl;
  int ret;

  /* Reset PHY */

  putreg32(HDMI_PHY_CTRL_RESET, g_hdmi_base + HDMI_PHY_CTRL);
  up_udelay(10);

  /* Configure PHY mode based on pixel clock */

  ctrl = HDMI_PHY_CTRL_ENABLE;
  ctrl &= ~HDMI_PHY_CTRL_MODE_MASK;

  if (pixel_clock <= 74250000)
    {
      /* Low-speed mode (TMDS) — up to 74.25 MHz */

      ctrl |= HDMI_PHY_CTRL_MODE_TMDS;
    }
  else
    {
      /* High-speed mode (HDMI) — above 74.25 MHz */

      ctrl |= HDMI_PHY_CTRL_MODE_HDMI;
    }

  putreg32(ctrl, g_hdmi_base + HDMI_PHY_CTRL);

  /* Wait for PHY to lock */

  ret = rk3576_hdmi_wait_phy();
  if (ret < 0)
    {
      return ret;
    }

  uinfo("HDMI: PHY configured, clock %u Hz\n", pixel_clock);
  return OK;
}

/****************************************************************************
 * Name: rk3576_hdmi_config_video
 *
 * Description:
 *   Configure HDMI video output.
 *
 ****************************************************************************/

static int rk3576_hdmi_config_video(int resolution)
{
  uint32_t ctrl;
  const struct rk3576_hdmi_timing_s *timing;

  if (resolution < 0 || (unsigned int)resolution >= HDMI_TIMING_COUNT)
    {
      return -EINVAL;
    }

  timing = &g_hdmi_timings[resolution];

  /* Disable video during configuration */

  putreg32(0, g_hdmi_base + HDMI_VIDEO_CTRL);

  /* Configure video format */

  ctrl = 0;
  switch (g_hdmi_dev.format)
    {
      case RK3576_HDMI_FMT_RGB888:
        ctrl |= HDMI_VIDEO_CTRL_FORMAT_RGB888;
        break;
      case RK3576_HDMI_FMT_RGB565:
        ctrl |= HDMI_VIDEO_CTRL_FORMAT_RGB565;
        break;
      case RK3576_HDMI_FMT_YCBCR444:
        ctrl |= HDMI_VIDEO_CTRL_FORMAT_YCbCr444;
        break;
      case RK3576_HDMI_FMT_YCBCR422:
        ctrl |= HDMI_VIDEO_CTRL_FORMAT_YCbCr422;
        break;
    }

  putreg32(ctrl, g_hdmi_base + HDMI_VI_FORMAT);

  /* Configure video input timing */

  putreg32((uint32_t)timing->hactive | ((uint32_t)timing->htotal << 16),
           g_hdmi_base + HDMI_VI_TIMING_H);

  putreg32((uint32_t)timing->vactive | ((uint32_t)timing->vtotal << 16),
           g_hdmi_base + HDMI_VI_TIMING_V);

  /* Configure PHY */

  int ret = rk3576_hdmi_config_phy(timing->pixel_clock);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable video output */

  ctrl = HDMI_VIDEO_CTRL_ENABLE;
  ctrl |= (g_hdmi_dev.format << HDMI_VIDEO_CTRL_FORMAT_SHIFT) &
          HDMI_VIDEO_CTRL_FORMAT_MASK;
  putreg32(ctrl, g_hdmi_base + HDMI_VIDEO_CTRL);

  uinfo("HDMI: video configured %dx%d@%u Hz\n",
           timing->hactive, timing->vactive,
           timing->pixel_clock / 1000);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_hdmi_init
 *
 * Description:
 *   Initialize HDMI controller.
 *
 ****************************************************************************/

void rk3576_hdmi_init(void)
{
  /* Initialize state */

  memset(&g_hdmi_dev, 0, sizeof(struct rk3576_hdmi_dev_s));
  g_hdmi_dev.resolution = HDMI_DEFAULT_RESOLUTION;
  g_hdmi_dev.format = RK3576_HDMI_FMT_RGB888;
  g_hdmi_dev.mode = RK3576_HDMI_MODE_HDMI;

  /* Reset HDMI controller */

  putreg32(HDMI_SYS_CTRL_SW_RESET, g_hdmi_base + HDMI_SYS_CTRL);
  up_udelay(100);

  /* Enable system clock */

  putreg32(HDMI_SYS_CTRL_ENABLE | HDMI_SYS_CTRL_CLK_EN,
           g_hdmi_base + HDMI_SYS_CTRL);

  /* Clear all interrupts */

  putreg32(0xffffffff, g_hdmi_base + HDMI_INT_CLEAR);

  /* Wait for system to be ready */

  {
    int timeout = HDMI_TIMEOUT_US;
    while (timeout--)
      {
        uint32_t status = getreg32(g_hdmi_base + HDMI_SYS_STATUS);
        if (status & HDMI_SYS_STATUS_TX_READY)
          {
            break;
          }

        up_udelay(100);
      }
  }

  /* Check for connected display */

  uint32_t status = getreg32(g_hdmi_base + HDMI_SYS_STATUS);
  g_hdmi_dev.connected = (status & HDMI_SYS_STATUS_HPD) ? true : false;

  g_hdmi_dev.initialized = true;

  uinfo("HDMI: initialized, display %s\n",
           g_hdmi_dev.connected ? "connected" : "not detected");
}

/****************************************************************************
 * Name: rk3576_hdmi_set_resolution
 *
 * Description:
 *   Set HDMI output resolution.
 *
 ****************************************************************************/

int rk3576_hdmi_set_resolution(int resolution)
{
  if (resolution < 0 || (unsigned int)resolution >= HDMI_TIMING_COUNT)
    {
      uerr("HDMI: invalid resolution %d\n", resolution);
      return -EINVAL;
    }

  g_hdmi_dev.resolution = resolution;

  /* Reconfigure video if enabled */

  if (g_hdmi_dev.enabled)
    {
      int ret = rk3576_hdmi_config_video(resolution);
      if (ret < 0)
        {
          return ret;
        }
    }

  const struct rk3576_hdmi_timing_s *timing = &g_hdmi_timings[resolution];
  uinfo("HDMI: resolution set to %dx%d\n",
           timing->hactive, timing->vactive);

  return OK;
}

/****************************************************************************
 * Name: rk3576_hdmi_set_format
 *
 * Description:
 *   Set HDMI video format (RGB888, RGB565, YCbCr).
 *
 ****************************************************************************/

int rk3576_hdmi_set_format(int format)
{
  if (format < RK3576_HDMI_FMT_RGB888 || format > RK3576_HDMI_FMT_YCBCR422)
    {
      return -EINVAL;
    }

  g_hdmi_dev.format = format;

  /* Reconfigure video if enabled */

  if (g_hdmi_dev.enabled)
    {
      return rk3576_hdmi_config_video(g_hdmi_dev.resolution);
    }

  return OK;
}

/****************************************************************************
 * Name: rk3576_hdmi_set_mode
 *
 * Description:
 *   Set HDMI output mode (HDMI or DVI).
 *
 ****************************************************************************/

int rk3576_hdmi_set_mode(int mode)
{
  if (mode != RK3576_HDMI_MODE_HDMI && mode != RK3576_HDMI_MODE_DVI)
    {
      return -EINVAL;
    }

  g_hdmi_dev.mode = mode;

  uinfo("HDMI: mode set to %s\n", mode == 0 ? "HDMI" : "DVI");
  return OK;
}

/****************************************************************************
 * Name: rk3576_hdmi_enable
 *
 * Description:
 *   Enable HDMI output.
 *
 ****************************************************************************/

void rk3576_hdmi_enable(void)
{
  if (!g_hdmi_dev.initialized)
    {
      return;
    }

  /* Configure video output */

  int ret = rk3576_hdmi_config_video(g_hdmi_dev.resolution);
  if (ret < 0)
    {
      uerr("HDMI: video config failed: %d\n", ret);
      return;
    }

  g_hdmi_dev.enabled = true;

  uinfo("HDMI: enabled\n");
}

/****************************************************************************
 * Name: rk3576_hdmi_disable
 *
 * Description:
 *   Disable HDMI output.
 *
 ****************************************************************************/

void rk3576_hdmi_disable(void)
{
  /* Disable video output */

  putreg32(0, g_hdmi_base + HDMI_VIDEO_CTRL);

  /* Disable PHY */

  putreg32(0, g_hdmi_base + HDMI_PHY_CTRL);

  g_hdmi_dev.enabled = false;

  uinfo("HDMI: disabled\n");
}

/****************************************************************************
 * Name: rk3576_hdmi_is_connected
 *
 * Description:
 *   Check if an HDMI display is connected.
 *
 ****************************************************************************/

int rk3576_hdmi_is_connected(void)
{
  /* Re-read HPD status */

  uint32_t status = getreg32(g_hdmi_base + HDMI_SYS_STATUS);
  g_hdmi_dev.connected = (status & HDMI_SYS_STATUS_HPD) ? true : false;

  return g_hdmi_dev.connected ? 1 : 0;
}

/****************************************************************************
 * Name: rk3576_hdmi_get_resolution
 *
 * Description:
 *   Get current HDMI resolution.
 *
 ****************************************************************************/

int rk3576_hdmi_get_resolution(void)
{
  return g_hdmi_dev.resolution;
}

/****************************************************************************
 * Name: rk3576_hdmi_read_edid
 *
 * Description:
 *   Read EDID from connected display via I2C.
 *
 ****************************************************************************/

int rk3576_hdmi_read_edid(uint8_t *buf, int len)
{
  if (buf == NULL || len <= 0)
    {
      return -EINVAL;
    }

  if (!g_hdmi_dev.connected)
    {
      return -ENODEV;
    }

  /* EDID is typically 128 bytes, can be read via DDC (I2C) */

  /* For now, return a minimal EDID structure */

  memset(buf, 0, len);

  /* EDID header */

  buf[0] = 0x00;
  buf[1] = 0xff;
  buf[2] = 0xff;
  buf[3] = 0xff;
  buf[4] = 0xff;
  buf[5] = 0xff;
  buf[6] = 0xff;
  buf[7] = 0x00;

  /* Manufacturer ID (VESA) */

  buf[8] = 0x00;
  buf[9] = 0x00;

  /* Product code */

  buf[10] = 0x00;
  buf[11] = 0x00;

  /* Serial number */

  buf[12] = 0x00;
  buf[13] = 0x00;
  buf[14] = 0x00;
  buf[15] = 0x00;

  /* Week/year of manufacture */

  buf[16] = 0x0a;  /* Week 10 */
  buf[17] = 0x1a;  /* Year 2026 */

  /* EDID version */

  buf[18] = 0x01;  /* Version 1 */
  buf[19] = 0x03;  /* Revision 3 */

  uinfo("HDMI: EDID read %d bytes (stub)\n", len);
  return len;
}
