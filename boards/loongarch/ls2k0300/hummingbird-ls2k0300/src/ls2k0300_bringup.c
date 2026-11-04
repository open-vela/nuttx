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
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/clock.h>
#include <nuttx/analog/adc.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/video/fb.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/es8388.h>
#include <nuttx/sdio.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/partition.h>

#include <arch/board/board.h>

#include "pinmux.h"
#include "ls2k0300.h"
#include "hardware/ls2k0300_memorymap.h"

#ifdef CONFIG_LS2K0300_ADC
extern struct adc_dev_s *ls2k0300_adc_initialize(void);
#endif

#ifdef CONFIG_LS2K0300_RTC
extern int ls2k0300_rtc_initialize(void);
extern void ls2k0300_rtc_hw_reinit(void);
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

#ifdef CONFIG_LS2K0300_DC
extern void ls2k0300_dc_show_test_pattern(void);
#endif

#ifdef CONFIG_LS2K0300_ETHERNET
extern int ls2k0300_ethernet_initialize(void);
#endif

#ifdef CONFIG_LS2K0300_I2S
extern struct i2s_dev_s *ls2k0300_i2s_initialize(void);
#endif

#ifdef CONFIG_LS2K0300_SDIO
extern FAR struct sdio_dev_s *ls2k0300_sdio_initialize(void);
#endif

/* ES8388 codec.  The codec is on I2C2 (GPIO52/53) at address 0x11.  Its
 * analog output drives the 3.5mm headphone jack and its analog input is
 * fed by the microphone connected to the same jack.
 */

#define LS2K0300_ES8388_I2C_PORT    2
#define LS2K0300_ES8388_I2C_ADDR    0x11
#define LS2K0300_ES8388_I2C_FREQ    100000

#ifdef CONFIG_LS2K0300_I2S
static struct i2c_master_s *g_i2c2;
#endif

#if defined(CONFIG_LS2K0300_I2S) && defined(CONFIG_AUDIO_ES8388)
static struct es8388_lower_s g_es8388_lower;
#endif

#if defined(CONFIG_LS2K0300_I2S) && defined(CONFIG_AUDIO_ES8388)
/****************************************************************************
 * Name: ls2k0300_audio_initialize
 *
 * Description:
 *   Initialize the I2S controller and the ES8388 codec and register the
 *   playback and capture devices as /dev/audio/pcm0p and
 *   /dev/audio/pcm0c.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ls2k0300_audio_initialize(void)
{
  FAR struct audio_lowerhalf_s *es8388;
  FAR struct i2s_dev_s *i2s;
  int ret;

  /* Get the I2C bus that carries the codec.  The instance is shared with
   * the board I2C2 driver.
   */

  if (g_i2c2 == NULL)
    {
      g_i2c2 = ls2k0300_i2c_initialize(LS2K0300_ES8388_I2C_PORT);
    }

  if (g_i2c2 == NULL)
    {
      serr("ERROR: Failed to initialize I2C%d\n",
           LS2K0300_ES8388_I2C_PORT);
      return -ENODEV;
    }

  /* Get the I2S interface that carries the audio data */

  i2s = ls2k0300_i2s_initialize();
  if (i2s == NULL)
    {
      serr("ERROR: Failed to initialize the I2S controller\n");
      return -ENODEV;
    }

  g_es8388_lower.address = LS2K0300_ES8388_I2C_ADDR;
  g_es8388_lower.frequency = LS2K0300_ES8388_I2C_FREQ;

  /* The ES8388 driver is a complete audio lower half: it configures the
   * codec over I2C and moves the audio data through the I2S interface.
   */

  es8388 = es8388_initialize(g_i2c2, i2s, &g_es8388_lower);
  if (es8388 == NULL)
    {
      serr("ERROR: Failed to initialize the ES8388\n");
      return -ENODEV;
    }

  ret = audio_register("pcm0p", es8388);
  if (ret < 0)
    {
      serr("ERROR: Failed to register /dev/audio/pcm0p: %d\n", ret);
      return ret;
    }

  es8388 = es8388_initialize(g_i2c2, i2s, &g_es8388_lower);
  if (es8388 == NULL)
    {
      serr("ERROR: Failed to initialize the ES8388\n");
      return -ENODEV;
    }

  ret = audio_register("pcm0c", es8388);
  if (ret < 0)
    {
      serr("ERROR: Failed to register /dev/audio/pcm0c: %d\n", ret);
      return ret;
    }

  _info("bringup: ES8388 registered as pcm0p/pcm0c\n");
  return OK;
}
#endif

#ifdef CONFIG_LS2K0300_SDIO
/****************************************************************************
 * Name: ls2k0300_sdio_report
 *
 * Description:
 *   Report the geometry of the card once the MMC/SD core has probed it.
 *   Purely diagnostic.
 *
 ****************************************************************************/

static void ls2k0300_sdio_report(FAR const char *devname)
{
  struct geometry geo;
  int fd;

  fd = open(devname, O_RDONLY);
  if (fd < 0)
    {
      return;
    }

  if (ioctl(fd, BIOC_GEOMETRY, (unsigned long)(uintptr_t)&geo) == OK)
    {
      unsigned long long sectors =
        (unsigned long long)geo.geo_nsectors;
      unsigned long long bytes = sectors * geo.geo_sectorsize;

      _info("bringup: %s: %llu sectors x %lu bytes (%llu MiB)%s\n",
            devname, sectors, (unsigned long)geo.geo_sectorsize,
            bytes / (1024 * 1024),
            geo.geo_writeenabled ? "" : ", read-only");
    }

  close(fd);
}
#endif /* CONFIG_LS2K0300_SDIO */

#ifdef CONFIG_LS2K0300_SDIO_AUTOMOUNT
/****************************************************************************
 * Name: ls2k0300_partition_handler
 *
 * Description:
 *   parse_block_partition() callback.  Registers the first partition of
 *   the card as its own block device: the FAT driver mounts a filesystem,
 *   it does not look inside a partition table.
 *
 ****************************************************************************/

static void ls2k0300_partition_handler(FAR struct partition_s *part,
                                       FAR void *arg)
{
  FAR bool *registered = (FAR bool *)arg;

  if (*registered || part->index != 0)
    {
      return;
    }

  if (register_blockpartition("/dev/mmcsd0p1", 0666, "/dev/mmcsd0",
                              part->firstblock, part->nblocks) == 0)
    {
      *registered = true;
    }
}

/****************************************************************************
 * Name: ls2k0300_sdio_mount
 *
 * Description:
 *   Mount the card at /mnt.  A card written by a PC carries an MBR and a
 *   single FAT partition, which has to be mounted through its own block
 *   device; a card formatted without a partition table is mounted as-is.
 *
 ****************************************************************************/

static void ls2k0300_sdio_mount(void)
{
  FAR const char *devname = "/dev/mmcsd0";
  bool registered = false;
  int ret;

  if (parse_block_partition("/dev/mmcsd0", ls2k0300_partition_handler,
                            &registered) == 0 && registered)
    {
      devname = "/dev/mmcsd0p1";
    }

  mkdir("/mnt", 0777);
  ret = nx_mount(devname, "/mnt", "vfat", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount %s: %d\n", devname, ret);
    }
  else
    {
      _info("bringup: %s mounted at /mnt\n", devname);
    }
}
#endif /* CONFIG_LS2K0300_SDIO_AUTOMOUNT */

void ls2k0300_hardware_init(void)
{
  /* GPIO50/51 configured for the I2C1 function (MAIN_FUNC = 0x3) */

  ls_pinmux_pin_setup(50, LS_PINMUX_MODE_AS_MAIN_FUNC);
  ls_pinmux_pin_setup(51, LS_PINMUX_MODE_AS_MAIN_FUNC);

  /* GPIO48/49 configured for the I2C0 function (touch screen GT9147:
   * SCL=48, SDA=49)
   */

  ls_pinmux_pin_setup(48, LS_PINMUX_MODE_AS_MAIN_FUNC);
  ls_pinmux_pin_setup(49, LS_PINMUX_MODE_AS_MAIN_FUNC);

#ifdef CONFIG_LS2K0300_I2C2
  /* GPIO52/53 configured for the I2C2 function (ES8388 audio codec:
   * SCL=52, SDA=53)
   */

  ls_pinmux_pin_setup(52, LS_PINMUX_MODE_AS_MAIN_FUNC);
  ls_pinmux_pin_setup(53, LS_PINMUX_MODE_AS_MAIN_FUNC);
#endif

  /* UART2 pinmux configuration
   * GPIO44 (UART2_TX) -> MAIN_FUNC (0x3)
   * GPIO45 (UART2_RX) -> MAIN_FUNC (0x3)
   */

  ls_pinmux_pin_setup(44, LS_PINMUX_MODE_AS_MAIN_FUNC);
  ls_pinmux_pin_setup(45, LS_PINMUX_MODE_AS_MAIN_FUNC);

  /* SPI0 pinmux configuration
   * GPIO64 (SPI0_CLK)  -> MAIN_FUNC (0x3)
   * GPIO65 (SPI0_MISO) -> MAIN_FUNC (0x3)
   * GPIO66 (SPI0_MOSI) -> MAIN_FUNC (0x3)
   * GPIO67 (SPI0_CSN1) -> GPIO (0x0), used as the software chip select
   *                       of the MCP3204
   */

  ls_pinmux_pin_setup(64, LS_PINMUX_MODE_AS_MAIN_FUNC);
  ls_pinmux_pin_setup(65, LS_PINMUX_MODE_AS_MAIN_FUNC);
  ls_pinmux_pin_setup(66, LS_PINMUX_MODE_AS_MAIN_FUNC);
  ls_pinmux_pin_setup(67, LS_PINMUX_MODE_AS_GPIO);

  /* SPI2 (SPIIO2) pinmux configuration
   * GPIO64 (SPI2_CLK)  -> MAIN_FUNC (0x3) - shared with SPI0
   * GPIO65 (SPI2_MISO) -> MAIN_FUNC (0x3) - shared with SPI0
   * GPIO66 (SPI2_MOSI) -> MAIN_FUNC (0x3) - shared with SPI0
   * GPIO85 (Flash CS)  -> GPIO (0x0), software chip select
   */

#ifdef CONFIG_LS2K0300_SDIO1
  /* SDIO1 (SD card socket): GPIO100-105 -> MAIN_FUNC.
   * Blind write instead of read-modify-write: GPIO96-111 share one word
   * and reading it back is unreliable.
   * Value: GPIO96-99=GPIO, GPIO100-105=MAIN, the reserved bits=0.
   */

  *(volatile uint32_t *)PHYS_TO_UNCACHED(0x160004a8) = 0x000fff00;
#endif

  /* PWM2 pinmux: GPIO88 -> SECOND_FUNC (0x2), the hardware PWM output */

  ls_pinmux_pin_setup(88, LS_PINMUX_MODE_AS_SECOND_FUNC);

  /* Blind-write GPIO85 pinmux (GPIO80-95 group, register 0x160004a4).
   * Must be AFTER all RMW pinmux calls — RMW reads 0xFFFFFFFF and corrupts
   * GPIO85 bits.  Value: GPIO85=b00(GPIO), GPIO88=b10(SECOND_FUNC),
   * all others=b11(MAIN_FUNC).
   */

  *(volatile uint32_t *)PHYS_TO_UNCACHED(0x160004a4) = 0xfffef3ff;

#ifdef CONFIG_LS2K0300_I2S
  /* I2S pinmux: GPIO76(MCLK)/77(BCLK)/78(LR)/79(DI) -> MAIN_FUNC.
   * GPIO80(DO) has already been set to MAIN_FUNC by the GPIO_CFG5 blind
   * write above.
   *
   * GPIO_CFG4 covers GPIO64-79: GPIO64-66 are SPI0 (MAIN_FUNC), GPIO67 is
   * the software chip select (GPIO), GPIO68-75 are unused (they keep the
   * GPIO mode of the reset default).
   * Like GPIO_CFG5 this is a blind write that does not rely on reading
   * the register back.
   */

  *(volatile uint32_t *)PHYS_TO_UNCACHED(0x160004a0) = 0xff00003f;
#endif

  /* GPIO85 configured as an output, the CS is high by default (idle).
   * Note the polarity of the DIR register: 0=output, 1=input
   * (see gpio_setpintype in ls2k0300_gpio.c).
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

#ifdef CONFIG_FS_TMPFS
  /* RAM-backed scratch space.  The PTT capture is written here instead
   * of the SD card: the recording only lives for the few seconds until
   * ASR has read it, and keeping the SDIO DMA off the bus while the I2S
   * DMA is running removes one source of the short capture dropouts.
   */

  _info("bringup: mounting tmpfs at /tmp\n");
  mkdir("/tmp", 0777);
  ret = mount(NULL, "/tmp", "tmpfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount tmpfs at /tmp: %d\n", ret);
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
#ifdef CONFIG_LS2K0300_I2S
          /* The audio codec shares this bus */

          g_i2c2 = i2c;
#endif
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

#ifdef CONFIG_LS2K0300_DC
  /* Display controller: must come after the SPIIO2 section above, which
   * blind-writes the GPIO80-95 pinmux register and would clobber the
   * GPIO88 (backlight) setting done inside the DC driver.
   */

  _info("bringup: DC LCD\n");
  ret = fb_register(0, 0);
  if (ret < 0)
    {
      serr("ERROR: fb_register(0,0) failed: %d\n", ret);
    }
  else
    {
      /* fb_register() zeroes the framebuffer as a side effect, so the
       * acceptance test card is drawn afterwards.
       */

      ls2k0300_dc_show_test_pattern();
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

#if defined(CONFIG_LS2K0300_I2S) && defined(CONFIG_AUDIO_ES8388)
  _info("bringup: I2S/ES8388\n");
  ret = ls2k0300_audio_initialize();
  if (ret < 0)
    {
      serr("ERROR: ls2k0300_audio_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_LS2K0300_SDIO
  /* SD/SDIO host: the MMC/SD core probes the card and registers the block
   * device before returning, so the report and the mount below can follow
   * immediately.
   */

  _info("bringup: SDIO\n");
    {
      FAR struct sdio_dev_s *sdio = ls2k0300_sdio_initialize();
      if (sdio == NULL)
        {
          serr("ERROR: ls2k0300_sdio_initialize failed\n");
        }
      else
        {
          ls2k0300_sdio_report("/dev/mmcsd0");
#ifdef CONFIG_LS2K0300_SDIO_AUTOMOUNT
          ls2k0300_sdio_mount();
#endif
        }
    }
#endif

#ifdef CONFIG_LS2K0300_RTC
  /* The RTC driver is brought up from clock_initialize(), long before the
   * board clocks settle.  Its TOY load is dropped if the 32.768 kHz
   * oscillator is not running yet, so repeat the enable/load here while
   * the time is still easy to fix.
   */

  _info("bringup: RTC reinit\n");
  ls2k0300_rtc_hw_reinit();

  /* clock_initialize() already seeded the system time from the RTC; redo
   * it now that the counter actually runs, otherwise time() stays at the
   * bogus value read during early boot.
   */

  clock_synchronize(NULL);
#endif

  _info("bringup: done\n");

  return ret;
}
