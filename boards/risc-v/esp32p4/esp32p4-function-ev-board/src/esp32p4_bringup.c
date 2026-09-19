/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_bringup.c
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
#include <syslog.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_ESPRESSIF_MIPI_DSI
#  include <nuttx/video/mipi_dsi.h>
#  include <nuttx/kmalloc.h>
#  include <nuttx/timers/pwm.h>
#  ifdef CONFIG_VIDEO_FB
#    include <nuttx/video/fb.h>
#  endif
#endif

#include "esp_board_ledc.h"
#include "esp_board_spiflash.h"
#include "esp_board_i2c.h"
#include "esp_board_bmp180.h"

#include "espressif/esp_start.h"

#ifdef CONFIG_WATCHDOG
#  include "espressif/esp_wdt.h"
#endif

#ifdef CONFIG_TIMER
#  include "espressif/esp_gptimer.h"
#endif

#ifdef CONFIG_ONESHOT
#  include "espressif/esp_oneshot.h"
#endif

#ifdef CONFIG_RTC_DRIVER
#  include "espressif/esp_rtc.h"
#endif

#if defined(CONFIG_DEV_GPIO) || defined(CONFIG_ESPRESSIF_MIPI_DSI)
#  include "espressif/esp_gpio.h"
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_ESPRESSIF_EFUSE
#  include "espressif/esp_efuse.h"
#endif

#ifdef CONFIG_ESP_RMT
#  include "esp_board_rmt.h"
#endif

#ifdef CONFIG_ESPRESSIF_I2S
#  include "esp_board_i2s.h"
#endif

#ifdef CONFIG_ESPRESSIF_SPI
#  include "espressif/esp_spi.h"
#  include "esp_board_spidev.h"
#  ifdef CONFIG_ESPRESSIF_SPI_BITBANG
#    include "espressif/esp_spi_bitbang.h"
#  endif
#endif

#ifdef CONFIG_SPI_SLAVE_DRIVER
#  include "espressif/esp_spi.h"
#  include "esp_board_spislavedev.h"
#endif

#ifdef CONFIG_ESPRESSIF_TEMP
#  include "espressif/esp_temperature_sensor.h"
#endif

#ifdef CONFIG_ESP_MCPWM
#  include "esp_board_mcpwm.h"
#endif

#ifdef CONFIG_ESP_PCNT
#  include "espressif/esp_pcnt.h"
#  include "esp_board_pcnt.h"
#endif

#ifdef CONFIG_ESPRESSIF_ADC
#  include "esp_board_adc.h"
#endif

#ifdef CONFIG_PM
#  include "espressif/esp_pm.h"
#endif

#ifdef CONFIG_ESPRESSIF_MIPI_DSI
#  include "espressif/esp_ldo.h"
#  include "espressif/esp_mipi_dsi.h"
#endif

#ifdef CONFIG_SYSTEM_NXDIAG_ESPRESSIF_CHIP_WO_TOOL
#  include "espressif/esp_nxdiag.h"
#endif

#ifdef CONFIG_ESP_SDM
#  include "espressif/esp_sdm.h"
#endif

#ifdef CONFIG_COMP
#  include "espressif/esp_ana_cmpr.h"
#endif

#ifdef CONFIG_ESPRESSIF_USE_LP_CORE
#  include "espressif/esp_ulp.h"
#  ifdef CONFIG_ESPRESSIF_ULP_USE_TEST_BIN
#    include "ulp/ulp_code.h"
#  endif
#  ifdef CONFIG_ESPRESSIF_LP_MAILBOX
#    include "espressif/esp_lp_mailbox.h"
#  endif
#endif

#include "esp32p4-function-ev-board.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_MIPI_DSI
#  define BOARD_MIPI_DPHY_LDO_CHANNEL 3
#  define BOARD_MIPI_DPHY_LDO_VOLTAGE_MV 2500
#  define BOARD_LCD_HRES 1024
#  define BOARD_LCD_VRES 600
#  define BOARD_LCD_BPP 16
#  define BOARD_LCD_FB_SIZE \
    ((size_t)BOARD_LCD_HRES * BOARD_LCD_VRES * BOARD_LCD_BPP / 8)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_MIPI_DSI
static struct esp_ldo_config_t g_mipi_dphy_ldo =
{
  .chan_id = BOARD_MIPI_DPHY_LDO_CHANNEL,
  .voltage_mv = BOARD_MIPI_DPHY_LDO_VOLTAGE_MV,
  .handler = NULL,
};

static FAR struct mipi_dsi_device *g_lcd_dsi_device;
static FAR uint16_t *g_lcd_framebuffer;
#ifdef CONFIG_ESPRESSIF_LEDC
static int g_lcd_backlight_fd = -1;
#endif
#ifdef CONFIG_VIDEO_FB
static struct fb_videoinfo_s g_lcd_fb_videoinfo =
{
  .fmt = FB_FMT_RGB16_565,
  .xres = BOARD_LCD_HRES,
  .yres = BOARD_LCD_VRES,
  .nplanes = 1,
};

static struct fb_planeinfo_s g_lcd_fb_planeinfo;
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_MIPI_DSI
static int board_mipi_panel_write(uint8_t command, uint8_t value)
{
  ssize_t ret;

  ret = mipi_dsi_dcs_write(g_lcd_dsi_device, command, &value, 1);
  if (ret < 0)
    {
      return (int)ret;
    }

  return ret == 2 ? OK : -EIO;
}

static int board_mipi_panel_initialize(void)
{
  FAR struct mipi_dsi_host *host;
  int ret;

  host = esp_mipi_dsi_host_get();
  if (host == NULL)
    {
      return -ENODEV;
    }

  /* GPIO27 is the active-low LCD reset line on the Function-EV LCD
   * subboard.  Keep the panel in reset while the DSI host settles.
   */

  esp_configgpio(27, OUTPUT_FUNCTION_2);
  esp_gpiowrite(27, false);
  usleep(10000);
  esp_gpiowrite(27, true);
  usleep(20000);

  g_lcd_dsi_device = mipi_dsi_device_register(host, "ek79007ad", 0);
  if (g_lcd_dsi_device == NULL)
    {
      return -ENOMEM;
    }

  g_lcd_dsi_device->lanes = 2;
  g_lcd_dsi_device->format = MIPI_DSI_FMT_RGB565;
  g_lcd_dsi_device->mode_flags = MIPI_DSI_MODE_LPM;
  g_lcd_dsi_device->hs_rate = 500000000;
  g_lcd_dsi_device->lp_rate = 10000000;

  ret = mipi_dsi_attach(g_lcd_dsi_device);
  if (ret < 0)
    {
      return ret;
    }

  /* EK79007AD datasheet control registers:
   *   B2[4] = 1 selects the physical 2-lane interface;
   *   B1[3:2] = 00 selects 1024 x 600;
   *   B3 = 00 keeps the default gate/frame setting;
   *   B0[7] = 1 enables the panel charge pump/VCOM block.
   */

  ret = board_mipi_panel_write(0xb2, 0x10);
  if (ret < 0)
    {
      return ret;
    }

  ret = board_mipi_panel_write(0xb1, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  ret = board_mipi_panel_write(0xb3, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  ret = board_mipi_panel_write(0xb0, 0x80);
  if (ret < 0)
    {
      return ret;
    }

  ret = mipi_dsi_dcs_exit_sleep_mode(g_lcd_dsi_device);
  if (ret < 0)
    {
      return ret;
    }

  usleep(120000);

  ret = mipi_dsi_dcs_set_display_on(g_lcd_dsi_device);
  if (ret < 0)
    {
      return ret;
    }

  usleep(20000);
  return OK;
}

static void board_mipi_fill_framebuffer(void)
{
  int y;
  int x;

  for (y = 0; y < BOARD_LCD_VRES; y++)
    {
      for (x = 0; x < BOARD_LCD_HRES; x++)
        {
          uint16_t red = (uint16_t)((x * 31) / BOARD_LCD_HRES);
          uint16_t green = (uint16_t)((y * 63) / BOARD_LCD_VRES);
          uint16_t blue = (uint16_t)(31 - red);

          g_lcd_framebuffer[y * BOARD_LCD_HRES + x] =
            (uint16_t)((red << 11) | (green << 5) | blue);
        }
    }
}

#ifdef CONFIG_VIDEO_FB
static int board_lcd_getvideoinfo(FAR struct fb_vtable_s *vtable,
                                  FAR struct fb_videoinfo_s *vinfo)
{
  if (vtable == NULL || vinfo == NULL)
    {
      return -EINVAL;
    }

  memcpy(vinfo, &g_lcd_fb_videoinfo, sizeof(*vinfo));
  return OK;
}

static int board_lcd_getplaneinfo(FAR struct fb_vtable_s *vtable,
                                  int planeno,
                                  FAR struct fb_planeinfo_s *pinfo)
{
  if (vtable == NULL || planeno != 0 || pinfo == NULL ||
      g_lcd_framebuffer == NULL)
    {
      return -EINVAL;
    }

  memset(&g_lcd_fb_planeinfo, 0, sizeof(g_lcd_fb_planeinfo));
  g_lcd_fb_planeinfo.fbmem = g_lcd_framebuffer;
  g_lcd_fb_planeinfo.fblen = BOARD_LCD_FB_SIZE;
  g_lcd_fb_planeinfo.stride = BOARD_LCD_HRES * BOARD_LCD_BPP / 8;
  g_lcd_fb_planeinfo.display = 0;
  g_lcd_fb_planeinfo.bpp = BOARD_LCD_BPP;
  g_lcd_fb_planeinfo.xres_virtual = BOARD_LCD_HRES;
  g_lcd_fb_planeinfo.yres_virtual = BOARD_LCD_VRES;

  memcpy(pinfo, &g_lcd_fb_planeinfo, sizeof(*pinfo));
  return OK;
}

#ifdef CONFIG_FB_UPDATE
static int board_lcd_updatearea(FAR struct fb_vtable_s *vtable,
                                FAR const struct fb_area_s *area)
{
  uint8_t *row;
  size_t row_bytes;
  int ret;

  if (vtable == NULL || area == NULL || g_lcd_framebuffer == NULL ||
      area->w == 0 || area->h == 0 ||
      area->x + area->w > BOARD_LCD_HRES ||
      area->y + area->h > BOARD_LCD_VRES)
    {
      return -EINVAL;
    }

  row_bytes = (size_t)area->w * BOARD_LCD_BPP / 8;
  row = (uint8_t *)g_lcd_framebuffer +
        (size_t)area->y * BOARD_LCD_HRES * BOARD_LCD_BPP / 8 +
        (size_t)area->x * BOARD_LCD_BPP / 8;

  for (int y = 0; y < area->h; y++)
    {
      ret = esp_mipi_dsi_flush_framebuffer(row, row_bytes);
      if (ret < 0)
        {
          return ret;
        }

      row += BOARD_LCD_HRES * BOARD_LCD_BPP / 8;
    }

  return OK;
}
#endif

static struct fb_vtable_s g_lcd_fb_vtable =
{
  .getvideoinfo = board_lcd_getvideoinfo,
  .getplaneinfo = board_lcd_getplaneinfo,
#ifdef CONFIG_FB_UPDATE
  .updatearea = board_lcd_updatearea,
#endif
};
#endif

#ifdef CONFIG_ESPRESSIF_LEDC
static int board_mipi_backlight_enable(void)
{
  struct pwm_info_s info;
  int fd;
  int ret;

  if (g_lcd_backlight_fd >= 0)
    {
      return OK;
    }

  fd = open("/dev/pwm0", O_WRONLY);
  if (fd < 0)
    {
      return -errno;
  }

  memset(&info, 0, sizeof(info));
  info.frequency = 1000;
  info.duty = 0xffff;
  info.cpol = PWM_CPOL_HIGH;
  info.dcpol = PWM_CPOL_LOW;

  ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS,
              (unsigned long)(uintptr_t)&info);
  if (ret >= 0)
    {
      ret = ioctl(fd, PWMIOC_START, 0);
    }

  if (ret < 0)
    {
      close(fd);
      return -errno;
    }

  /* Keep the descriptor open.  The PWM upper-half shuts down the LEDC
   * timer when its final descriptor is closed, which would disable the
   * LCD backlight immediately after startup. */

  g_lcd_backlight_fd = fd;
  return OK;
}
#endif

static int board_mipi_video_initialize(void)
{
  struct esp_mipi_dsi_dpi_config_s dpi =
  {
    .h_res = BOARD_LCD_HRES,
    .v_res = BOARD_LCD_VRES,
    .hsync_pulse_width = 70,
    .hsync_back_porch = 160,
    .hsync_front_porch = 160,
    .vsync_pulse_width = 10,
    .vsync_back_porch = 23,
    .vsync_front_porch = 12,
    .dpi_clock_freq_mhz = 51,
    .virtual_channel = 0,
    .format = MIPI_DSI_FMT_RGB565,
  };
  int ret;

  g_lcd_framebuffer = kumm_memalign(64, BOARD_LCD_FB_SIZE);
  if (g_lcd_framebuffer == NULL)
    {
      return -ENOMEM;
    }

  ret = esp_mipi_dsi_configure_dpi(&dpi);
  if (ret < 0)
    {
      goto errout;
    }

  ret = esp_mipi_dsi_bind_framebuffer(g_lcd_framebuffer,
                                      BOARD_LCD_FB_SIZE,
                                      BOARD_LCD_HRES,
                                      BOARD_LCD_VRES,
                                      BOARD_LCD_BPP);
  if (ret < 0)
    {
      goto errout;
    }

  board_mipi_fill_framebuffer();

  ret = esp_mipi_dsi_flush_framebuffer(g_lcd_framebuffer,
                                       BOARD_LCD_FB_SIZE);
  if (ret < 0)
    {
      goto errout;
    }

  ret = esp_mipi_dsi_video_start();
  if (ret < 0)
    {
      goto errout;
    }

#ifdef CONFIG_ESPRESSIF_LEDC
  ret = board_mipi_backlight_enable();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to enable LCD backlight: %d\n", ret);
      esp_mipi_dsi_video_stop();
      goto errout;
    }
#endif

#ifdef CONFIG_VIDEO_FB
  /* Register last: fb_register_device() clears the exposed framebuffer. */
  ret = fb_register_device(0, 0, &g_lcd_fb_vtable);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register LCD framebuffer: %d\n", ret);
      esp_mipi_dsi_video_stop();
      goto errout;
    }

  board_mipi_fill_framebuffer();
  ret = esp_mipi_dsi_flush_framebuffer(g_lcd_framebuffer,
                                       BOARD_LCD_FB_SIZE);
  if (ret < 0)
    {
      esp_mipi_dsi_video_stop();
      goto errout;
    }
#endif

  syslog(LOG_INFO,
         "LCD video active: %ux%u RGB565 FB=%p size=%lu\n",
         BOARD_LCD_HRES, BOARD_LCD_VRES, g_lcd_framebuffer,
         (unsigned long)BOARD_LCD_FB_SIZE);
#ifdef CONFIG_VIDEO_FB
  syslog(LOG_INFO, "LCD framebuffer registered: /dev/fb0\n");
#endif
  return OK;

errout:
  kumm_free(g_lcd_framebuffer);
  g_lcd_framebuffer = NULL;
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_bringup
 *
 * Description:
 *   Perform architecture-specific initialization.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      _err("Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      _err("Failed to mount tmpfs at %s: %d\n", CONFIG_LIBC_TMPDIR, ret);
    }
#endif

#if defined(CONFIG_ESPRESSIF_EFUSE)
  ret = esp_efuse_initialize("/dev/efuse");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init EFUSE: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_MWDT0
  ret = esp_wdt_initialize("/dev/watchdog0", ESP_WDT_MWDT0);
  if (ret < 0)
    {
      _err("Failed to initialize WDT: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_MWDT1
  ret = esp_wdt_initialize("/dev/watchdog1", ESP_WDT_MWDT1);
  if (ret < 0)
    {
      _err("Failed to initialize WDT: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_RWDT
  ret = esp_wdt_initialize("/dev/watchdog2", ESP_WDT_RWDT);
  if (ret < 0)
    {
      _err("Failed to initialize WDT: %d\n", ret);
    }
#endif

#ifdef CONFIG_TIMER
  ret = esp_timer_initialize(0);
  if (ret < 0)
    {
      _err("Failed to initialize Timer 0: %d\n", ret);
    }

#ifndef CONFIG_ONESHOT
  ret = esp_timer_initialize(1);
  if (ret < 0)
    {
      _err("Failed to initialize Timer 1: %d\n", ret);
    }
#endif
#endif

#ifdef CONFIG_ONESHOT
  ret = esp_oneshot_initialize();
  if (ret < 0)
    {
      _err("Failed to initialize Oneshot Timer: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_RMT
  ret = board_rmt_txinitialize(RMT_OUTPUT_PIN);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_rmt_txinitialize() failed: %d\n", ret);
    }

  ret = board_rmt_rxinitialize(RMT_INPUT_PIN);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_rmt_txinitialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RTC_DRIVER
  /* Initialize the RTC driver */

  ret = esp_rtc_driverinit();
  if (ret < 0)
    {
      _err("Failed to initialize the RTC driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_SPI
#  if defined(CONFIG_ESPRESSIF_SPI2_SLAVE) && defined(CONFIG_ESPRESSIF_SPI2)
  ret = board_spislavedev_initialize(ESPRESSIF_SPI2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI%d Slave driver: %d\n",
             ESPRESSIF_SPI2, ret);
    }
#  elif defined(CONFIG_ESPRESSIF_SPI2)
  ret = board_spidev_initialize(ESPRESSIF_SPI2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init spidev 2: %d\n", ret);
    }
#  endif

#  if defined(CONFIG_ESPRESSIF_SPI3_SLAVE) && defined(CONFIG_ESPRESSIF_SPI3)
  ret = board_spislavedev_initialize(ESPRESSIF_SPI3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI%d Slave driver: %d\n",
             ESPRESSIF_SPI3, ret);
    }
#  elif defined(CONFIG_ESPRESSIF_SPI3)
  ret = board_spidev_initialize(ESPRESSIF_SPI3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init spidev 3: %d\n", ret);
    }
#  endif

#  ifdef CONFIG_ESPRESSIF_SPI_BITBANG
  ret = board_spidev_initialize(ESPRESSIF_SPI_BITBANG);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init spidev 3: %d\n", ret);
    }
#  endif /* CONFIG_ESPRESSIF_SPI_BITBANG */

#  ifdef CONFIG_ESPRESSIF_LPSPI0
  ret = board_spidev_initialize(ESPRESSIF_LPSPI0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init lpspi: %d\n", ret);
    }
#  endif
#endif /* CONFIG_ESPRESSIF_SPI */

#ifdef CONFIG_ESPRESSIF_SPIFLASH
  ret = board_spiflash_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI Flash\n");
    }
#endif

#if defined(CONFIG_ESPRESSIF_I2S)
  /* Configure I2S peripheral interfaces */

  ret = board_i2s_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2S driver: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C_DRIVER)
  /* Configure I2C peripheral interfaces */

  ret = board_i2c_init();

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_MIPI_DSI
  /* Power the P4 MIPI D-PHY before initializing the host.  Framebuffer
   * setup and video streaming are intentionally deferred to the next
   * bring-up stage.
   */

  {
    struct esp_mipi_dsi_bus_config_s dsi_config =
    {
      .num_data_lanes = CONFIG_ESPRESSIF_MIPI_DSI_LANES,
      .lane_bit_rate_mbps = CONFIG_ESPRESSIF_MIPI_DSI_LANE_BITRATE_MBPS,
    };

    ret = esp_ldo_channel_acquire(&g_mipi_dphy_ldo);
    if (ret < 0)
      {
        syslog(LOG_ERR, "Failed to enable MIPI D-PHY LDO: %d\n", ret);
      }
    else
      {
        ret = esp_mipi_dsi_initialize(&dsi_config);
        if (ret < 0)
          {
            syslog(LOG_ERR, "Failed to initialize MIPI DSI host: %d\n", ret);
          }
        else
          {
            syslog(LOG_INFO,
                   "MIPI DSI host initialized: lanes=%lu bitrate=%lu Mbps\n",
                   (unsigned long)dsi_config.num_data_lanes,
                   (unsigned long)dsi_config.lane_bit_rate_mbps);
          }
      }
  }
#endif

#ifdef CONFIG_ESPRESSIF_MIPI_DSI
  ret = board_mipi_panel_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize EK79007AD panel: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "EK79007AD panel command initialization complete\n");
    }
#endif

#ifdef CONFIG_SENSORS_BMP180
  /* Try to register BMP180 device in I2C0 */

  ret = board_bmp180_initialize(0);

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize BMP180 "
             "Driver for I2C0: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_SDM
  struct esp_sdm_chan_config_s config =
  {
    .gpio_num = 5,
    .sample_rate_hz = 1000 * 1000,
    .flags = 0,
  };

  struct dac_dev_s *dev = esp_sdminitialize(config);
  ret = dac_register("/dev/dac0", dev);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize DAC driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_TEMP
  struct esp_temp_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG(10, 50);
  ret = esp_temperature_sensor_initialize(cfg);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize temperature sensor driver: %d\n",
             ret);
    }
#endif
#ifdef CONFIG_ESPRESSIF_TWAI0

  /* Initialize TWAI and register the TWAI driver. */

  ret = board_twai_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: TWAI0 board_twai_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_TWAI1

  /* Initialize TWAI and register the TWAI driver. */

  ret = board_twai_setup(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: TWAI1 board_twai_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_TWAI2

  /* Initialize TWAI and register the TWAI driver. */

  ret = board_twai_setup(2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: TWAI2 board_twai_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = esp_gpio_init();
  if (ret < 0)
    {
      ierr("Failed to initialize GPIO Driver: %d\n", ret);
    }
#endif

#if defined(CONFIG_INPUT_BUTTONS) && defined(CONFIG_INPUT_BUTTONS_LOWER)
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      ierr("ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_LEDC
  ret = board_ledc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_ledc_setup() failed: %d\n", ret);
    }
#endif /* CONFIG_ESPRESSIF_LEDC */

#ifdef CONFIG_ESPRESSIF_MIPI_DSI
  ret = board_mipi_video_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize LCD video: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_MCPWM_CAPTURE
  ret = board_capture_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_capture_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_MCPWM_MOTOR
  ret = board_motor_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_motor_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_PCNT
  ret = board_pcnt_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_pcnt_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_PM
  /* Configure PM */

  ret = esp_pmconfigure();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_pmconfigure failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SYSTEM_NXDIAG_ESPRESSIF_CHIP_WO_TOOL
  ret = esp_nxdiag_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_nxdiag_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_ADC
  ret = board_adc_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_adc_init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_ANA_COMPR0
  ret = esp_cmprinitialize(ESPRESSIF_COMP0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_cmprinitialize(%d) failed: %d\n",
             ESPRESSIF_COMP0, ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_ANA_COMPR1
  ret = esp_cmprinitialize(ESPRESSIF_COMP1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_cmprinitialize(%d) failed: %d\n",
             ESPRESSIF_COMP1, ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_EMAC
  ret = board_emac_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_emac_init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_USE_LP_CORE
#  ifdef CONFIG_ESPRESSIF_LP_MAILBOX
  esp_lp_mailbox_init();
#  endif

  /* ULP initialization should be the handled later than
   * peripherals to use supported peripherals properly on ULP core
   */

  ret = esp_ulp_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_ulp_init failed: %d\n", ret);
    }
  else
    {
#  ifdef CONFIG_ESPRESSIF_ULP_USE_TEST_BIN
      esp_ulp_load_bin((char *)esp_ulp_bin, esp_ulp_bin_len);
#  endif
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  return ret;
}
