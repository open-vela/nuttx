/****************************************************************************
 * boards/arm/stm32h7/stm32h750b-dk/src/stm32_bringup.c
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
#include <arch/board/board.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usbmonitor.h>

#ifdef CONFIG_STM32H7_OTGFS
#include "stm32_usbhost.h"
#endif

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#endif

#ifdef CONFIG_RPTUN
#  include "stm32_rptun.h"
#endif

#ifdef CONFIG_RPMSG_UART
#  include <nuttx/serial/uart_rpmsg.h>
#endif

#include "stm32_gpio.h"

#include "stm32h750b-dk.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void convert_lcd_rgb565(void)
{
  /* Put LCD_{R0,R1,R2,G0,G1,B0,B1,B2} in low level */

  stm32_configgpio(GPIO_LCD_R0);
  stm32_gpiowrite(GPIO_LCD_R0, 0);
  stm32_configgpio(GPIO_LCD_R1);
  stm32_gpiowrite(GPIO_LCD_R1, 0);
  stm32_configgpio(GPIO_LCD_R2);
  stm32_gpiowrite(GPIO_LCD_R2, 0);
  stm32_configgpio(GPIO_LCD_G0);
  stm32_gpiowrite(GPIO_LCD_G0, 0);
  stm32_configgpio(GPIO_LCD_G1);
  stm32_gpiowrite(GPIO_LCD_G1, 0);
  stm32_configgpio(GPIO_LCD_B0);
  stm32_gpiowrite(GPIO_LCD_B0, 0);
  stm32_configgpio(GPIO_LCD_B1);
  stm32_gpiowrite(GPIO_LCD_B1, 0);
  stm32_configgpio(GPIO_LCD_B2);
  stm32_gpiowrite(GPIO_LCD_B2, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_RPMSG_UART
/****************************************************************************
 * Name: rpmsg_serialinit
 ****************************************************************************/

void rpmsg_serialinit(void)
{
#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  uart_rpmsg_init("cm4", "proxy", 4096, false);
#endif

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM4
#  ifdef CONFIG_RPMSG_UART_CONSOLE
  uart_rpmsg_init("cm7", "proxy", 4096, true);
#  else
  uart_rpmsg_init("cm7", "proxy", 4096, false);
#  endif
#endif
}
#endif

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y &&
 *   CONFIG_NSH_ARCHINIT:
 *     Called from the NSH library
 *
 ****************************************************************************/

#ifdef HAVE_SDIO
#  if defined(CONFIG_FS_FAT) && defined(CONFIG_FS_LINKS)

static void vg_setup_data_layout(void)
{
  struct stat st;
  int ret;

  ret = mkdir("/mnt/emmc/data", 0755);
  if (ret < 0 && errno != EEXIST)
    {
      syslog(LOG_WARNING,
             "WARNING: mkdir /mnt/emmc/data failed: %d\n", ret);
    }

  /* One-time stage0 → stage1: /mnt/emmc/velaguard → /mnt/emmc/data/velaguard */

  if (stat("/mnt/emmc/velaguard", &st) == 0 &&
      stat("/mnt/emmc/data/velaguard", &st) != 0)
    {
      ret = rename("/mnt/emmc/velaguard", "/mnt/emmc/data/velaguard");
      if (ret < 0)
        {
          syslog(LOG_WARNING,
                 "WARNING: migrate velaguard dir failed: %d\n", ret);
        }
      else
        {
          syslog(LOG_INFO,
                 "Migrated /mnt/emmc/velaguard -> /mnt/emmc/data/velaguard\n");
        }
    }

  ret = mkdir("/mnt/emmc/data/velaguard/config", 0755);
  if (ret < 0 && errno != EEXIST)
    {
      syslog(LOG_WARNING,
             "WARNING: mkdir velaguard/config failed: %d\n", ret);
    }

  ret = mkdir("/mnt/emmc/data/agent", 0755);
  if (ret < 0 && errno != EEXIST)
    {
      syslog(LOG_WARNING,
             "WARNING: mkdir data/agent failed: %d\n", ret);
    }

  ret = mkdir("/mnt/emmc/data/agent/config", 0755);
  if (ret < 0 && errno != EEXIST)
    {
      syslog(LOG_WARNING,
             "WARNING: mkdir agent/config failed: %d\n", ret);
    }

  ret = mkdir("/mnt/emmc/data/agent/memory", 0755);
  if (ret < 0 && errno != EEXIST)
    {
      syslog(LOG_WARNING,
             "WARNING: mkdir agent/memory failed: %d\n", ret);
    }

  ret = mkdir("/mnt/emmc/data/agent/sessions", 0755);
  if (ret < 0 && errno != EEXIST)
    {
      syslog(LOG_WARNING,
             "WARNING: mkdir agent/sessions failed: %d\n", ret);
    }

  ret = mkdir("/mnt/emmc/data/agent/skills", 0755);
  if (ret < 0 && errno != EEXIST)
    {
      syslog(LOG_WARNING,
             "WARNING: mkdir agent/skills failed: %d\n", ret);
    }

  if (lstat("/data", &st) != 0)
    {
      ret = symlink("/mnt/emmc/data", "/data");
      if (ret < 0)
        {
          syslog(LOG_WARNING,
                 "WARNING: symlink /data -> /mnt/emmc/data failed: %d\n",
                 ret);
        }
      else
        {
          syslog(LOG_INFO, "Symlink /data -> /mnt/emmc/data\n");
        }
    }
}

#  endif /* CONFIG_FS_FAT && CONFIG_FS_LINKS */
#endif /* HAVE_SDIO */

int stm32_bringup(void)
{
  int ret;
#ifdef CONFIG_RAMMTD
  uint8_t *ramstart;
#endif

  UNUSED(ret);

  convert_lcd_rgb565();

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n",  ret);
    }
#endif /* CONFIG_FS_PROCFS */

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RPTUN
#  ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  stm32_rptun_init("cm4");
#  else
  stm32_rptun_init("cm7");
#  endif
#endif

#ifdef CONFIG_INPUT_FT5X06
  /* Initialize the touchscreen.
   * WARNING: stm32_tsc_setup() cannot be called from the IDLE thread.
   */

  ret = stm32_tsc_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_tsc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RAMMTD
  /* Create a RAM MTD device if configured */

  ramstart = kmm_malloc(128 * 1024);
  if (ramstart == NULL)
    {
      syslog(LOG_ERR, "ERROR: Allocation for RAM MTD failed\n");
    }
  else
    {
      /* Initialized the RAM MTD */

      struct mtd_dev_s *mtd = rammtd_initialize(ramstart, 128 * 1024);
      if (mtd == NULL)
        {
          syslog(LOG_ERR, "ERROR: rammtd_initialize failed\n");
          kmm_free(ramstart);
        }
      else
        {
          /* Erase the RAM MTD */

          ret = mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: IOCTL MTDIOC_BULKERASE failed\n");
            }

#ifdef CONFIG_FS_LITTLEFS
          /* Register the MTD driver so that it can be accessed from the
           * VFS.
           */

          ret = register_mtddriver("/dev/rammtd", mtd, 0755, NULL);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: Failed to register MTD driver: %d\n",
                     ret);
            }

          /* Mount the LittleFS file system */

          ret = nx_mount("/dev/rammtd", "/mnt/lfs", "littlefs", 0,
                         "forceformat");
          if (ret < 0)
            {
              syslog(LOG_ERR,
                     "ERROR: Failed to mount LittleFS at /mnt/lfs: %d\n",
                     ret);
            }
#endif
        }
    }
#endif

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  stm32_usbhost_initialize()
   * starts a thread will monitor for USB connection and
   * disconnection events.
   */

  ret = stm32_usbhost_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize USB host: %d\n",
             ret);
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to start USB monitor: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_UART7_RS485
  ret = symlink("/dev/ttyS2", "/dev/rs485");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to create symlink for RS485: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32H7_USART2
  stm32_configgpio(GPIO_ESP_EN);
  stm32_configgpio(GPIO_ESP_RST);
#endif

#ifdef HAVE_SDIO
  /* Initialize on-board eMMC (SDMMC1) and try FAT at /mnt/emmc.
   * First boot may lack a filesystem — mount failure is logged only;
   * format manually with NSH: mkfatfs /dev/mmcsd0
   */

  ret = stm32_sdio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize eMMC/SDIO: %d\n", ret);
    }
#ifdef CONFIG_FS_FAT
  else
    {
      ret = nx_mount("/dev/mmcsd0", "/mnt/emmc", "vfat", 0, NULL);
      if (ret < 0)
        {
          syslog(LOG_WARNING,
                 "WARNING: eMMC FAT mount /mnt/emmc failed: %d "
                 "(try: mkfatfs /dev/mmcsd0)\n", ret);
        }
      else
        {
          syslog(LOG_INFO, "eMMC mounted at /mnt/emmc\n");
#  if defined(CONFIG_FS_FAT) && defined(CONFIG_FS_LINKS)
          vg_setup_data_layout();
#  endif
        }
    }
#endif
#endif

  return OK;
}
