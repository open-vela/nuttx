/****************************************************************************
 * boards/loongarch/ls2k0300/hummingbird-ls2k0300/src/ls2k0300_bringup.c
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

#include <sys/mount.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/analog/adc.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>

#include <arch/board/board.h>

#include "pinmux.h"
#include "ls2k0300.h"
#include "hardware/ls2k0300_memorymap.h"

#ifdef CONFIG_LS2K0300_ADC
extern struct adc_dev_s *ls2k0300_adc_initialize(void);
#endif

#ifdef CONFIG_LS2K0300_RTC
extern int ls2k0300_rtc_initialize(void);
#endif

#ifdef CONFIG_LS2K0300_I2C
extern struct i2c_master_s *ls2k0300_i2c_initialize(int port);
#endif

#ifdef CONFIG_LS2K0300_THERMAL
extern int ls2k0300_thermal_initialize(void);
#endif

#ifdef CONFIG_LS2K0300_PWM
extern int ls2k0300_pwm_initialize(int port);
#endif

#ifdef CONFIG_LS2K0300_SPI
extern struct spi_dev_s *ls2k0300_spiflash_initialize(int port);
#endif

#ifdef CONFIG_LS2K0300_SPIIO
extern struct spi_dev_s *ls2k0300_spiio_initialize(int port);
#endif

#ifdef CONFIG_LS2K0300_WDT
extern int ls2k0300_wdt_initialize(void);
#endif

#ifdef CONFIG_LS2K0300_GPIO
extern int ls2k0300_gpio_initialize(void);
#endif

#ifdef CONFIG_LS2K0300_PINCTRL
extern int ls2k0300_pinctrl_initialize(void);
#endif

#ifdef CONFIG_LS2K0300_ETHERNET
extern int ls2k0300_ethernet_initialize(void);
#endif

void ls2k0300_hardware_init(void)
{
  /* Configure GPIO50/51 for I2C1 function (MAIN_FUNC = 0x3) */

  ls_pinmux_pin_setup(50, LS_PINMUX_MODE_AS_MAIN_FUNC);
  ls_pinmux_pin_setup(51, LS_PINMUX_MODE_AS_MAIN_FUNC);

  /* UART2 pin mux configuration
   * GPIO44 (UART2_TX) -> MAIN_FUNC (0x3)
   * GPIO45 (UART2_RX) -> MAIN_FUNC (0x3)
   */

  ls_pinmux_pin_setup(44, LS_PINMUX_MODE_AS_MAIN_FUNC);
  ls_pinmux_pin_setup(45, LS_PINMUX_MODE_AS_MAIN_FUNC);

  /* SPI0 pin mux configuration
   * GPIO64 (SPI0_CLK)  -> MAIN_FUNC (0x3)
   * GPIO65 (SPI0_MISO) -> MAIN_FUNC (0x3)
   * GPIO66 (SPI0_MOSI) -> MAIN_FUNC (0x3)
   * GPIO67 (SPI0_CSN1) -> GPIO (0x0), used as MCP3204 software chip select
   */

  ls_pinmux_pin_setup(64, LS_PINMUX_MODE_AS_MAIN_FUNC);
  ls_pinmux_pin_setup(65, LS_PINMUX_MODE_AS_MAIN_FUNC);
  ls_pinmux_pin_setup(66, LS_PINMUX_MODE_AS_MAIN_FUNC);
  ls_pinmux_pin_setup(67, LS_PINMUX_MODE_AS_GPIO);

  /* SPI2 (SPIIO2) pin mux configuration
   * GPIO64 (SPI2_CLK)  -> MAIN_FUNC (0x3) -- shared with SPI0
   * GPIO65 (SPI2_MISO) -> MAIN_FUNC (0x3) -- shared with SPI0
   * GPIO66 (SPI2_MOSI) -> MAIN_FUNC (0x3) -- shared with SPI0
   * GPIO85 (Flash CS)  -> GPIO (0x0), software chip select
   */

  /* PWM2 pin mux: GPIO88 -> SECOND_FUNC (0x2), hardware PWM output */

  ls_pinmux_pin_setup(88, LS_PINMUX_MODE_AS_SECOND_FUNC);

  /* Blind-write GPIO85 pinmux (GPIO80-95 group, register 0x160004a4).
   * Must be AFTER all RMW pinmux calls — RMW reads 0xFFFFFFFF and corrupts
   * GPIO85 bits.  Value: GPIO85=b00(GPIO), GPIO88=b10(SECOND_FUNC),
   * all others=b11(MAIN_FUNC).
   */

  *(volatile uint32_t *)PHYS_TO_UNCACHED(0x160004a4) = 0xfffef3ff;

  /* Configure GPIO85 as output, CS defaults high (idle state).
   * DIR register polarity: 0=output, 1=input
   * (see ls2k0300_gpio.c gpio_setpintype).
   */

  writeb(0, (volatile void *)(PHYS_TO_UNCACHED(LS2K0300_GPIO_BASE) + 0x800 + 85));  /* DIR=0 → OUTPUT */
  writeb(1, (volatile void *)(PHYS_TO_UNCACHED(LS2K0300_GPIO_BASE) + 0x900 + 85));  /* OUT=high (CS idle) */

  /* Enable SPIIO2/SPIIO3 clock gate (GENERAL_CFG5 bits 17-18) */

  {
    volatile uint32_t *cfg5 =
        (volatile uint32_t *)PHYS_TO_UNCACHED(LS2K0300_GENERAL_CFG5);
    *cfg5 |= (3U << 17);
  }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ls2k0300_bringup(void)
{
  int ret = OK;

  _info("bringup: start\n");

  ls2k0300_hardware_init();

#ifdef CONFIG_FS_PROCFS
  _info("bringup: mounting procfs\n");
  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
    }
#endif

#ifdef CONFIG_LS2K0300_ADC
  _info("bringup: ADC\n");
    {
      FAR struct adc_dev_s *adc = ls2k0300_adc_initialize();
      if (adc != NULL)
        {
          ret = adc_register("/dev/adc0", adc);
          if (ret < 0)
            {
              serr("ERROR: adc_register failed: %d\n", ret);
            }
        }
    }
#endif

#ifdef CONFIG_LS2K0300_I2C0
  _info("bringup: I2C0\n");
    {
      FAR struct i2c_master_s *i2c = ls2k0300_i2c_initialize(0);
      if (i2c != NULL)
        {
          ret = i2c_register(i2c, 0);
          if (ret < 0)
            {
              serr("ERROR: i2c_register failed for I2C0: %d\n", ret);
            }
        }
    }
#endif

#ifdef CONFIG_LS2K0300_I2C1
  _info("bringup: I2C1\n");
    {
      FAR struct i2c_master_s *i2c = ls2k0300_i2c_initialize(1);
      if (i2c != NULL)
        {
          ret = i2c_register(i2c, 1);
          if (ret < 0)
            {
              serr("ERROR: i2c_register failed for I2C1: %d\n", ret);
            }
        }
    }
#endif

#ifdef CONFIG_LS2K0300_I2C2
  _info("bringup: I2C2\n");
    {
      FAR struct i2c_master_s *i2c = ls2k0300_i2c_initialize(2);
      if (i2c != NULL)
        {
          ret = i2c_register(i2c, 2);
          if (ret < 0)
            {
              serr("ERROR: i2c_register failed for I2C2: %d\n", ret);
            }
        }
    }
#endif

#ifdef CONFIG_LS2K0300_I2C3
  _info("bringup: I2C3\n");
    {
      FAR struct i2c_master_s *i2c = ls2k0300_i2c_initialize(3);
      if (i2c != NULL)
        {
          ret = i2c_register(i2c, 3);
          if (ret < 0)
            {
              serr("ERROR: i2c_register failed for I2C3: %d\n", ret);
            }
        }
    }
#endif

#ifdef CONFIG_LS2K0300_THERMAL
  _info("bringup: THERMAL\n");
  ret = ls2k0300_thermal_initialize();
  if (ret < 0)
    {
      serr("ERROR: ls2k0300_thermal_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_LS2K0300_PWM0
  _info("bringup: PWM0\n");
  ret = ls2k0300_pwm_initialize(0);
  if (ret < 0)
    {
      serr("ERROR: ls2k0300_pwm_initialize failed for PWM0: %d\n", ret);
    }
#endif

#ifdef CONFIG_LS2K0300_PWM1
  _info("bringup: PWM1\n");
  ret = ls2k0300_pwm_initialize(1);
  if (ret < 0)
    {
      serr("ERROR: ls2k0300_pwm_initialize failed for PWM1: %d\n", ret);
    }
#endif

#ifdef CONFIG_LS2K0300_PWM2
  _info("bringup: PWM2\n");
  ret = ls2k0300_pwm_initialize(2);
  if (ret < 0)
    {
      serr("ERROR: ls2k0300_pwm_initialize failed for PWM2: %d\n", ret);
    }
#endif

#ifdef CONFIG_LS2K0300_PWM3
  _info("bringup: PWM3\n");
  ret = ls2k0300_pwm_initialize(3);
  if (ret < 0)
    {
      serr("ERROR: ls2k0300_pwm_initialize failed for PWM3: %d\n", ret);
    }
#endif

#ifdef CONFIG_LS2K0300_SPI0
  _info("bringup: SPI0\n");
    {
      FAR struct spi_dev_s *spi = ls2k0300_spiflash_initialize(0);
      if (spi == NULL)
        {
          serr("ERROR: ls2k0300_spiflash_initialize failed for SPI0\n");
        }
    }
#endif

#ifdef CONFIG_LS2K0300_SPI1
  _info("bringup: SPI1\n");
    {
      FAR struct spi_dev_s *spi = ls2k0300_spiflash_initialize(1);
      if (spi == NULL)
        {
          serr("ERROR: ls2k0300_spiflash_initialize failed for SPI1\n");
        }
    }
#endif

#ifdef CONFIG_LS2K0300_SPIIO3
  _info("bringup: SPIIO3\n");
    {
      FAR struct spi_dev_s *spi = ls2k0300_spiio_initialize(1);
      if (spi == NULL)
        {
          serr("ERROR: ls2k0300_spiio_initialize failed for SPIIO3\n");
        }
    }
#endif

#ifdef CONFIG_LS2K0300_WDT
  _info("bringup: WDT\n");
  ret = ls2k0300_wdt_initialize();
  if (ret < 0)
    {
      serr("ERROR: ls2k0300_wdt_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_LS2K0300_PINCTRL
  _info("bringup: PINCTRL\n");
  ret = ls2k0300_pinctrl_initialize();
  if (ret < 0)
    {
      serr("ERROR: ls2k0300_pinctrl_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_LS2K0300_GPIO
  _info("bringup: GPIO\n");
  ret = ls2k0300_gpio_initialize();
  if (ret < 0)
    {
      serr("ERROR: ls2k0300_gpio_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_LS2K0300_SPIIO2
  /* SPIIO2 SPI Flash init: after pinctrl and GPIO are ready.
   * GPIO85 pinmux was blind-written in hardware_init(); re-assert here
   * because ls2k0300_gpio_initialize() resets all pins to GPIO_INPUT_PIN.
   */

    {
      uintptr_t gpio_base = PHYS_TO_UNCACHED(LS2K0300_GPIO_BASE);

      /* Blind-write pinmux again (GPIO init may have RMW'd it) */

      *(volatile uint32_t *)PHYS_TO_UNCACHED(0x160004a4) = 0xfffef3ff;

      /* DIR=0 (output), OUT=1 (CS idle high) */

      writeb(0, (volatile void *)(gpio_base + 0x800 + 85));
      writeb(1, (volatile void *)(gpio_base + 0x900 + 85));
    }

  _info("bringup: SPIIO2 SPI Flash\n");
    {
      FAR struct spi_dev_s *spi = ls2k0300_spiio_initialize(0);
      if (spi == NULL)
        {
          serr("ERROR: ls2k0300_spiio_initialize failed for SPIIO2\n");
        }
      else
        {
          /* Software reset flash chip (recover from stale state) */

          SPI_SELECT(spi, SPIDEV_FLASH(0), true);
          SPI_SEND(spi, 0x66);  /* Enable Reset */
          SPI_SELECT(spi, SPIDEV_FLASH(0), false);
          up_udelay(1);

          SPI_SELECT(spi, SPIDEV_FLASH(0), true);
          SPI_SEND(spi, 0x99);  /* Reset */
          SPI_SELECT(spi, SPIDEV_FLASH(0), false);
          up_mdelay(30);

          /* Probe GD25 flash */

          FAR struct mtd_dev_s *mtd = gd25_initialize(spi, 0);
          if (mtd == NULL)
            {
              serr("ERROR: gd25_initialize failed on SPIIO2\n");
            }
          else
            {
              ret = register_mtddriver("/dev/spiflash", mtd, 0755, NULL);
              if (ret < 0)
                {
                  serr("ERROR: register_mtddriver failed: %d\n", ret);
                }
              else
                {
                  mkdir("/spiflash", 0777);
                  ret = nx_mount("/dev/spiflash", "/spiflash",
                                 "littlefs", 0, "autoformat");
                  if (ret < 0)
                    {
                      serr("ERROR: Failed to mount LittleFS: %d\n", ret);
                    }
                  else
                    {
                      _info("bringup: LittleFS mounted at /spiflash\n");
                    }
                }
            }
        }
    }
#endif

#ifdef CONFIG_LS2K0300_ETHERNET
  _info("bringup: ETHERNET\n");
  ret = ls2k0300_ethernet_initialize();
  if (ret < 0)
    {
      serr("ERROR: ls2k0300_ethernet_initialize failed: %d\n", ret);
    }
#endif

  _info("bringup: done\n");

  return ret;
}
