/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_lcd_board.c
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
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/compiler.h>
#include <nuttx/arch.h>
#include <nuttx/lcd/gc9503cv_dsi.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/video/fb.h>
#include <nuttx/video/mipi_display.h>
#include <nuttx/video/mipi_dsi.h>

#include "hardware/t113_gpio.h"
#include "hardware/t113_pinmap.h"
#include "t113_de.h"
#include "t113_dsi.h"
#include "t113_gpio.h"
#include "t113_tcon.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* X4B GC9503CV reset pin (PD17, active low - see GC9503CV datasheet,
 * RESETB).
 */

#define X4B_PANEL_RESET_GPIO \
  T113_GPIO_ENCODE(T113_GPIO_PORTD, 17, T113_GPIO_OUTPUT, \
                   T113_GPIO_PULL_NONE, T113_GPIO_DRV_LEVEL1)

/* DSI lane pads (PD0-PD5, alt-func 4) at maximum drive level.  The pinmap
 * default (LEVEL0) is too weak for the differential pair termination
 * loading on the X4B connector - the LP-escape eye closes and DCS bytes
 * never reach the GC9503CV.  Vendor sources program DRV_LEVEL3 on every
 * DSI pad; mirror that here.
 */

#define X4B_DSI_D0P_GPIO \
  T113_GPIO_ENCODE(T113_GPIO_PORTD, 0, T113_GPIO_FUNC4, \
                   T113_GPIO_PULL_NONE, T113_GPIO_DRV_LEVEL3)
#define X4B_DSI_D0N_GPIO \
  T113_GPIO_ENCODE(T113_GPIO_PORTD, 1, T113_GPIO_FUNC4, \
                   T113_GPIO_PULL_NONE, T113_GPIO_DRV_LEVEL3)
#define X4B_DSI_D1P_GPIO \
  T113_GPIO_ENCODE(T113_GPIO_PORTD, 2, T113_GPIO_FUNC4, \
                   T113_GPIO_PULL_NONE, T113_GPIO_DRV_LEVEL3)
#define X4B_DSI_D1N_GPIO \
  T113_GPIO_ENCODE(T113_GPIO_PORTD, 3, T113_GPIO_FUNC4, \
                   T113_GPIO_PULL_NONE, T113_GPIO_DRV_LEVEL3)
#define X4B_DSI_CKP_GPIO \
  T113_GPIO_ENCODE(T113_GPIO_PORTD, 4, T113_GPIO_FUNC4, \
                   T113_GPIO_PULL_NONE, T113_GPIO_DRV_LEVEL3)
#define X4B_DSI_CKN_GPIO \
  T113_GPIO_ENCODE(T113_GPIO_PORTD, 5, T113_GPIO_FUNC4, \
                   T113_GPIO_PULL_NONE, T113_GPIO_DRV_LEVEL3)

/* Backlight is driven from PWM0 on PD16.  Channel index for /dev/pwmN. */

#define X4B_BACKLIGHT_PWM_CHANNEL  0
#define X4B_BACKLIGHT_PWM_DEVPATH  "/dev/pwm0"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* t113_pwm_initialize() lives in arch/arm/src/t113/t113_pwm.c but has no
 * public header (only the register defs in hardware/t113_pwm.h).  Match
 * the forward declaration used in t113_bringup.c.
 */

void t113_pwm_initialize(int channel);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* EVB4 timing.  Vendor wants 31 MHz pclk via PLL_VIDEO0/8, but our TCON
 * module clock is hardcoded to 24 MHz HOSC with dclk_div=1, giving 24 MHz
 * actual pclk.  Tell DSI host to match (pclk=24M, bitrate=288 Mbps) so
 * TCON output rate and DSI input rate agree - clock domain mismatch
 * starves the DSI host of pixel data.
 */

static const struct t113_tcon_timing_s g_x4b_timing =
{
  .pixel_clk_hz = 24000000,
  .hactive      = 480,
  .hbp          = 74,
  .hfp          = 54,
  .hsync        = 10,
  .vactive      = 800,
  .vbp          = 33,
  .vfp          = 4,
  .vsync        = 8,
  .if_type      = T113_TCON_IF_DSI,
  .lanes        = 2,
  .format       = T113_TCON_FMT_RGB888,
};

static const struct t113_de_config_s g_x4b_de =
{
  .width  = 480,
  .height = 800,
  .bpp    = 32,        /* ARGB8888 - primary plane */
};

/* Same X4B 480x800 timing expressed in DSI-host units: hbp/vbp INCLUDE
 * sync per panel-vendor convention (the host subtracts internally).
 */

static const struct t113_dsi_video_s g_x4b_dsi_video =
{
  .pixel_clk_hz = 24000000,
  .hactive      = 480,
  .htotal       = 618,
  .hbp          = 74,        /* incl hsync */
  .hsync        = 10,
  .vactive      = 800,
  .vtotal       = 845,
  .vbp          = 33,        /* incl vsync */
  .vsync        = 8,
  .lanes        = 2,
  .format       = 0,         /* RGB888 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x4b_panel_reset
 *
 * Description:
 *   GC9503CV reset callback.  Drives the RESETB pin: active=true -> low,
 *   active=false -> high.  t113_gpio_write returns void; this wrapper
 *   adapts it to the int-returning callback expected by the panel driver.
 *
 ****************************************************************************/

static int x4b_panel_reset(bool active)
{
  t113_gpio_write(X4B_PANEL_RESET_GPIO, !active);
  return OK;
}

/****************************************************************************
 * Name: x4b_dsi_clk_enable
 *
 * Description:
 *   Pre-init hook called by the GC9503CV panel driver between hw_reset and
 *   the DCS init stream.  Pulses the DSI HSC instruction so the DPHY clock
 *   lane enters HS continuous mode before the first LP DCS write.  Without
 *   this the byte clock is unstable while D0 escapes and the DDIC silently
 *   swallows every command.
 *
 ****************************************************************************/

static int x4b_dsi_clk_enable(void)
{
  static const uint16_t lanes[] =
  {
    X4B_DSI_D0P_GPIO, X4B_DSI_D0N_GPIO,
    X4B_DSI_D1P_GPIO, X4B_DSI_D1N_GPIO,
    X4B_DSI_CKP_GPIO, X4B_DSI_CKN_GPIO,
  };

  size_t i;
  int ret;

  /* The panel power-on sequence does the three-step reset first, then
   * waits 30 ms before switching the DSI lanes to their alternate
   * function.  The 120 ms hw_reset settle has already elapsed by now, so
   * here we switch all six DSI physical lines to alt-func 4 (DSI) and
   * then pulse HSC.
   */

  for (i = 0; i < sizeof(lanes) / sizeof(lanes[0]); i++)
    {
      ret = t113_gpio_config(lanes[i]);
      if (ret < 0)
        {
          lcderr("ERROR: DSI lane pinmux[%u] failed: %d\n",
                 (unsigned)i, ret);
          return ret;
        }
    }

  return t113_dsi_start_clk();
}

/****************************************************************************
 * Name: x4b_lcd_pinmux
 *
 * Description:
 *   Configure all DSI lane pads, the panel reset GPIO and the backlight
 *   PWM pad to their alternate functions.
 *
 ****************************************************************************/

static int x4b_lcd_pinmux(void)
{
  /* Pre-reset stage: only reset GPIO and backlight PWM.  DSI lanes are
   * configured AFTER hw_reset by x4b_dsi_clk_enable (vendor lcd_power_on
   * does panel_reset -> 30ms -> pin_cfg(1)).
   */

  static const uint16_t pins[] =
  {
    T113_PWM0_1,
    X4B_PANEL_RESET_GPIO,
  };

  size_t i;
  int ret;

  for (i = 0; i < sizeof(pins) / sizeof(pins[0]); i++)
    {
      ret = t113_gpio_config(pins[i]);
      if (ret < 0)
        {
          lcderr("ERROR: t113_gpio_config(%u) failed: %d\n",
                 (unsigned)i, ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: x4b_backlight_init
 *
 * Description:
 *   Bring the backlight up to 100% via /dev/pwm0.  t113_pwm_initialize()
 *   has already been called from t113_bringup() before us, so /dev/pwm0
 *   is registered.  We open it, push 1 kHz / full-duty, and start.
 *
 *   The fd is intentionally leaked: NuttX's PWM upper-half drives
 *   pwm_shutdown() from the last close(), which would gate the output
 *   off and kill the backlight the moment we leave this function.
 *   Holding the descriptor for the lifetime of the system keeps the
 *   PWM running.
 *
 ****************************************************************************/

static int x4b_backlight_init(void)
{
  struct pwm_info_s info =
  {
    .frequency = 1000,        /* 1 kHz */
    .duty      = 0xffff,      /* full scale = 100 % */
  };

  int fd;
  int ret;

  fd = open(X4B_BACKLIGHT_PWM_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      ret = -errno;
      lcderr("ERROR: open(%s) failed: %d\n",
             X4B_BACKLIGHT_PWM_DEVPATH, ret);
      return ret;
    }

  ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)&info);
  if (ret < 0)
    {
      ret = -errno;
      lcderr("ERROR: PWMIOC_SETCHARACTERISTICS failed: %d\n", ret);
      close(fd);
      return ret;
    }

  ret = ioctl(fd, PWMIOC_START, 0);
  if (ret < 0)
    {
      ret = -errno;
      lcderr("ERROR: PWMIOC_START failed: %d\n", ret);
      close(fd);
      return ret;
    }

  /* Leak fd on purpose - see header comment. */

  return OK;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* X4B panel config - passed to gc9503cv_dsi_register() in Task 6.2. */

const struct gc9503cv_config_s g_x4b_panel_cfg =
{
  .reset    = x4b_panel_reset,
  .pre_init = x4b_dsi_clk_enable,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_lcd_initialize
 *
 * Description:
 *   Bring up the X4B GC9503CV MIPI DSI panel end-to-end.
 *
 *   Sequence:
 *     1. Configure DSI lane / PWM / reset GPIO pinmux.
 *     2. Initialise the DSI host (clock + register block).
 *     3. Initialise TCON-LCD0 with X4B timing.
 *     4. Initialise the DE mixer + framebuffer.
 *     5. Register the GC9503CV panel against the DSI host (this triggers
 *        hw_reset -> DCS init via the panel driver's attach hook).
 *     6. Register /dev/fb0 so userspace can mmap the primary plane.
 *     7. Bring backlight up to 100%.
 *
 *   Called from t113_bringup() under CONFIG_LCD_GC9503CV_DSI.
 *
 * Returned Value:
 *   OK on success; the first non-zero errno encountered on failure.
 *
 ****************************************************************************/

int t113_lcd_initialize(void)
{
  int ret;

  ret = x4b_lcd_pinmux();
  if (ret < 0)
    {
      lcderr("ERROR: x4b_lcd_pinmux failed: %d\n", ret);
      return ret;
    }

  ret = t113_dsi_initialize();
  if (ret < 0)
    {
      lcderr("ERROR: t113_dsi_initialize failed: %d\n", ret);
      return ret;
    }

  ret = t113_de_initialize(&g_x4b_de);
  if (ret < 0)
    {
      lcderr("ERROR: t113_de_initialize failed: %d\n", ret);
      return ret;
    }

  ret = gc9503cv_dsi_register(t113_dsi_get_host(), &g_x4b_panel_cfg);
  if (ret < 0)
    {
      lcderr("ERROR: gc9503cv_dsi_register failed: %d\n", ret);
      return ret;
    }

  /* Bring TCON up AFTER panel DCS init completes - vendor lcd_open_flow
   * order (panel_init -> tcon_enable).  Running TCON before the panel is
   * initialized leaves it pushing pixels into a half-configured DSI host.
   */

  ret = t113_tcon_initialize(&g_x4b_timing);
  if (ret < 0)
    {
      lcderr("ERROR: t113_tcon_initialize failed: %d\n", ret);
      return ret;
    }

  /* Panel DCS init has finished - switch the DSI host to HS sync-pulse
   * video before the TCON starts pushing pixels.
   */

  ret = t113_dsi_start_video(&g_x4b_dsi_video);
  if (ret < 0)
    {
      lcderr("ERROR: t113_dsi_start_video failed: %d\n", ret);
      return ret;
    }

  ret = fb_register_device(0, 0, t113_de_get_fb_vtable());
  if (ret < 0)
    {
      lcderr("ERROR: fb_register_device(0,0) failed: %d\n", ret);
      return ret;
    }

  ret = x4b_backlight_init();
  if (ret < 0)
    {
      lcderr("ERROR: x4b_backlight_init failed: %d\n", ret);
      return ret;
    }

  lcdinfo("X4B GC9503CV panel up\n");
  return OK;
}
