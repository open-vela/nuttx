/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_bringup.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <syslog.h>
#include <sys/param.h>
#include <termios.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_RPTUN
#  include <nuttx/rptun/rptun.h>
#endif

#include "t113_gpio.h"

#ifdef CONFIG_T113_SPI0
#  include <nuttx/spi/spi.h>
#  include <nuttx/spi/qspi.h>
#  include "t113_spi.h"
#endif

#if defined(CONFIG_MTD_MX35) || defined(CONFIG_MTD_GD5F) || \
    defined(CONFIG_MTD_F50L)
#  include <nuttx/mtd/mtd.h>
#  include <sys/mount.h>
#endif

#if defined(CONFIG_T113_TWI0) || defined(CONFIG_T113_TWI1) || \
    defined(CONFIG_T113_TWI2) || defined(CONFIG_T113_TWI3)
#  include <nuttx/i2c/i2c_master.h>
#  include "t113_i2c.h"
#endif

#ifdef CONFIG_T113_RTC
void t113_rtc_initialize(void);
#endif

#ifdef CONFIG_T113_PWM
void t113_pwm_initialize(int channel);
#endif

#ifdef CONFIG_T113_GPADC
void t113_adc_initialize(void);
#endif

#ifdef CONFIG_T113_LRADC
void t113_lradc_initialize(void);
#endif

#ifdef CONFIG_T113_LRADC_KEYPAD
void t113_keypad_initialize(void);
#endif

#ifdef CONFIG_T113_WDT
void t113_wdt_initialize(void);
#endif

#ifdef CONFIG_T113_CE
#  include "t113_ce.h"
#endif

#if defined(CONFIG_T113_HSTIMER0) || defined(CONFIG_T113_HSTIMER1)
#  include <nuttx/timers/timer.h>
#  include "t113_hstimer.h"
#endif

#if defined(CONFIG_T113_CAN0) || defined(CONFIG_T113_CAN1)
#  include "t113_can.h"
#endif

#ifdef CONFIG_T113_HWSPINLOCK
#  include "t113_hwspinlock.h"
#endif

#include "t113_boot.h"
#include "t113_ccu.h"

#ifdef CONFIG_USBMSC
#  include <nuttx/usb/usbmsc.h>
#endif

#ifdef CONFIG_T113_UART_DMA
#  include <nuttx/wqueue.h>
#  include "t113_serial.h"
#endif

#ifdef CONFIG_T113_SMHC
extern int t113_board_smhc_initialize(void);
#endif

#ifdef CONFIG_T113_AUDIO
#  include <nuttx/audio/audio.h>
#  include "t113_audio.h"
#endif

#ifdef CONFIG_T113_DMIC
#  include <nuttx/audio/audio.h>
#  include "t113_dmic.h"
#endif

#include "t113-evb.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* mtd0..mtd2 size in bytes; 0 = remainder of NAND.  Adjusting a
 * partition size means editing this array - start blocks are
 * derived by accumulation in the register loop.
 *
 * The arrays are file-scope (no BOARDCTL_BOOT_IMAGE gate) because the
 * 3-partition layout is a NAND physical contract shared by every
 * NuttX config that mounts SPI NAND, not just configs that ship the
 * boot DSL.  t113_get_mtd_part() is the public lookup gated to the
 * configs that actually consume it.
 */

static const size_t g_t113_part_size[3] =
{
  1 * 1024 * 1024,    /* mtd0  1 MB    boot0 SPL */
  32 * 1024 * 1024,   /* mtd1 32 MB    fwromfs.img (ROMFS: cfg + images) */
  0,                  /* mtd2 remain   user data */
};

static FAR struct mtd_dev_s *g_t113_parts[3];

#ifdef CONFIG_BOARDCTL_BOOT_IMAGE
FAR struct mtd_dev_s *t113_get_mtd_part(int mtdn)
{
  if ((unsigned)mtdn >= nitems(g_t113_parts))
    {
      return NULL;
    }

  return g_t113_parts[mtdn];
}
#endif

#ifdef CONFIG_T113_UART_DMA
static struct work_s g_dma_rx_poll_work;

#define DMA_RX_POLL_INTERVAL_MS 50
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_T113_UART_DMA
static void t113_dma_rx_poll_worker(FAR void *arg)
{
  UNUSED(arg);
  t113_serial_dma_poll();
  work_queue(HPWORK, &g_dma_rx_poll_work,
             t113_dma_rx_poll_worker, NULL,
             MSEC2TICK(DMA_RX_POLL_INTERVAL_MS));
}
#endif

#if defined(CONFIG_T113_SPI0) && \
    (defined(CONFIG_MTD_MX35) || defined(CONFIG_MTD_GD5F) || \
     defined(CONFIG_MTD_F50L))

/****************************************************************************
 * Name: t113_spinand_register
 *
 * Description:
 *   Register a SPI NAND MTD device and create the unified 3-partition
 *   layout used by this board.  Partition sizes come from
 *   g_t113_part_size[], start blocks are accumulated as the loop
 *   walks the array; mtdN -> g_t113_parts[N] for boot lookup.
 *
 *     mtd0: 1 MB     boot0 SPL
 *     mtd1: 32 MB    fwromfs.img (ROMFS: cfg + images)
 *     mtd2: remain   user data
 *
 ****************************************************************************/

static int t113_spinand_register(FAR struct mtd_dev_s *mtd)
{
#ifdef CONFIG_MTD_PARTITION
  struct mtd_geometry_s geo;
  char   path[16];
  int    ppe;
  int    start = 0;
  size_t i;
  int    ret;

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)&geo);
  if (ret < 0)
    {
      return ret;
    }

  ppe = geo.erasesize / geo.blocksize;

  for (i = 0; i < nitems(g_t113_part_size); i++)
    {
      int nblk;

      /* Sizes must be a multiple of erasesize; truncating the tail
       * silently shifts the next partition's start.  0 means "rest
       * of NAND" and is exempt.
       */

      DEBUGASSERT(g_t113_part_size[i] == 0 ||
                  g_t113_part_size[i] % geo.erasesize == 0);

      nblk = g_t113_part_size[i] > 0
           ? (int)(g_t113_part_size[i] / geo.erasesize)
           : (int)geo.neraseblocks - start;
      DEBUGASSERT(nblk > 0);

      g_t113_parts[i] = mtd_partition(mtd, start * ppe, nblk * ppe);
      if (g_t113_parts[i] == NULL)
        {
          ferr("mtd_partition(%zu) failed\n", i);
          return -ENOMEM;
        }

      snprintf(path, sizeof(path), "/dev/mtd%d", (int)i);
      register_mtddriver(path, g_t113_parts[i], 0755, NULL);
      start += nblk;
    }

#ifdef CONFIG_FS_ROMFS
  /* Wrap mtd1 as a block driver and mount it read-only as ROMFS at /fw.
   * Mounted unconditionally - boot0 needs /fw/boot*.cfg, AP-side configs
   * get the same view for diagnostics.  finfo (not ferr) on failure: AP
   * configs without a packed fwromfs.img on mtd1 fail mount cleanly,
   * which is expected, not a bug worth shouting about.
   */

  ret = ftl_initialize(1, g_t113_parts[1]);
  if (ret < 0)
    {
      finfo("ftl_initialize(1) failed: %d\n", ret);
    }
  else
    {
      ret = mount("/dev/mtdblock1", "/fw", "romfs", MS_RDONLY, NULL);
      if (ret < 0)
        {
          ret = -errno;
          finfo("mount /dev/mtdblock1 /fw: %d\n", ret);
        }
    }
#endif

#ifdef CONFIG_MTD_DHARA
    {
      int user_mtd = nitems(g_t113_parts) - 1;

      ret = dhara_initialize(user_mtd, g_t113_parts[user_mtd]);
      if (ret < 0)
        {
          ferr("dhara_initialize(%d) failed: %d\n", user_mtd, ret);
        }
    }
#endif

  return OK;
#else
  return register_mtddriver("/dev/mtd0", mtd, 0755, NULL);
#endif
}

#endif /* CONFIG_T113_SPI0 && (MX35 || GD5F || F50L) */

#ifdef CONFIG_T113_RPTUN_SLAVE
/****************************************************************************
 * Name: t113_rptun_slave_rpmsg_nsh
 *
 * Description:
 *   Secondary NSH instance for the AMP slave.  Opens /dev/ttyCORE1 (the
 *   slave-side rpmsg-uart endpoint), redirects stdin/stdout/stderr to it,
 *   and runs nsh_consolemain().  Lets master access the slave shell via
 *   `cu -l /dev/ttyCORE1` while the native UART2 console keeps running
 *   on the INIT task in parallel.  The two shells are independent.
 *
 ****************************************************************************/

extern int nsh_consolemain(int argc, FAR char *argv[]);

static int t113_rptun_slave_rpmsg_nsh(int argc, FAR char *argv[])
{
  struct termios tio;
  int fd;

  fd = open("/dev/ttyCORE1", O_RDWR);
  if (fd < 0)
    {
      syslog(LOG_ERR, "SLAVE: open /dev/ttyCORE1 failed: %d\n", errno);
      return -errno;
    }

  /* /dev/ttyCORE1 was registered with isconsole=false, so the serial
   * framework left tc_oflag/tc_iflag at 0 - neither \n->\r\n on output
   * nor \r->\n on input.  That makes the rpmsg-uart channel unusable
   * from `cu`: prompts render with ladder indentation and Enter (CR
   * from the host terminal) is never accepted by readline (which only
   * matches LF).  Force the canonical-console flags here, matching the
   * uart_register() defaults applied for isconsole=true devices.
   */

  if (tcgetattr(fd, &tio) == 0)
    {
      tio.c_iflag |= ICRNL;
      tio.c_oflag |= OPOST | ONLCR;
      tio.c_lflag |= ECHO | ICANON | ISIG;
      tcsetattr(fd, TCSANOW, &tio);
    }

  dup2(fd, 0);
  dup2(fd, 1);
  dup2(fd, 2);
  if (fd > 2)
    {
      close(fd);
    }

  return nsh_consolemain(0, NULL);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int t113_bringup(void)
{
#ifdef CONFIG_T113_RPTUN_SLAVE
  int amp_ret;
#endif
#if defined(CONFIG_T113_SPI0) && \
    (defined(CONFIG_MTD_MX35) || defined(CONFIG_MTD_GD5F) || \
     defined(CONFIG_MTD_F50L))
  FAR struct mtd_dev_s *mtd;
#endif
#if defined(CONFIG_T113_SPI0) && \
    (defined(CONFIG_MX35_QSPI) || defined(CONFIG_MTD_GD5F_QSPI) || \
     defined(CONFIG_MTD_F50L_QSPI))
  FAR struct qspi_dev_s *qspi;
#endif
#if defined(CONFIG_T113_SPI0) && \
    ((defined(CONFIG_MTD_MX35) && !defined(CONFIG_MX35_QSPI)) || \
     (defined(CONFIG_MTD_GD5F) && !defined(CONFIG_MTD_GD5F_QSPI)) || \
     (defined(CONFIG_MTD_F50L) && !defined(CONFIG_MTD_F50L_QSPI)))
  FAR struct spi_dev_s *spi;
#endif
#if defined(CONFIG_T113_HSTIMER0) || defined(CONFIG_T113_HSTIMER1)
  FAR struct timer_lowerhalf_s *lower;
#endif
#if defined(CONFIG_T113_TWI0) || defined(CONFIG_T113_TWI1) || \
    defined(CONFIG_T113_TWI2) || defined(CONFIG_T113_TWI3)
  FAR struct i2c_master_s *i2c;
#endif
#if defined(CONFIG_USBMSC) && !defined(CONFIG_EXAMPLES_NANDTEST)
  FAR void *msc_handle = NULL;
#endif
#ifdef CONFIG_T113_USBHOST
  int usbret;
#endif
#ifdef CONFIG_T113_AUDIO
  FAR struct audio_lowerhalf_s *audio_adc_lower;
  FAR struct audio_lowerhalf_s *audio_dac_lower;
#endif
#ifdef CONFIG_T113_DMIC
  FAR struct audio_lowerhalf_s *audio_dmic_lower;
#endif
  const uint16_t usb_host_pwr =
    T113_GPIO_ENCODE(T113_GPIO_PORTD, 22, T113_GPIO_OUTPUT,
                     T113_GPIO_PULL_NONE, T113_GPIO_DRV_LEVEL0);
  int ret = 0;

#ifdef CONFIG_T113_RPTUN_SLAVE
  /* core1 has no physical UART of its own: its console is the rpmsg-tty
   * channel to the master and its early boot log lands in RAMLOG.  No GIC
   * SPI needs re-targeting to CPU1 here; future slave-owned peripherals
   * should re-target their own IRQs with up_affinity_irq().
   */

  syslog(LOG_INFO, "SLAVE: t113_bringup entry on cpu=%d\n",
         (int)up_cpu_index());

  /* Master (Linux remoteproc or core0-NuttX) ELF-loaded this slave.  Attach
   * the rptun slave with a ver=1 static resource table (no spin) on the
   * GIC-SGI doorbell; the master parsed the same table out of this ELF and
   * placed the vrings at the addresses it declares.
   */

  amp_ret = t113_rptun_init();
  if (amp_ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: t113_rptun_init failed: %d\n", amp_ret);
      return amp_ret;
    }

  /* Spawn an NSH bound to /dev/ttyCORE1 so a NuttX master's
   * `cu -l /dev/ttyCORE1` lands at a real shell.  When the master is the
   * core0-NuttX rptun loader this gives an interactive slave shell over
   * rpmsg; when the master is Linux the channel simply goes unused.  This
   * is core1's only interactive console -- it has no physical UART.
   */

  amp_ret = task_create("nsh-rpmsg", SCHED_PRIORITY_DEFAULT,
                        CONFIG_DEFAULT_TASK_STACKSIZE,
                        t113_rptun_slave_rpmsg_nsh, NULL);
  if (amp_ret < 0)
    {
      syslog(LOG_ERR, "SLAVE: nsh-rpmsg task_create failed: %d\n",
             errno);
    }

  /* Slave doesn't probe board peripherals -- its job is the rptun
   * channel to the master.  Skip the rest of bringup (master-owned
   * hardware).
   */

  return OK;
#endif

#ifdef CONFIG_BMP
  /* BMP: CPU1 is a pure task core.  Skip all peripheral probes and
   * only attach rptun as slave; the console comes up later via
   * rpmsg_uart on /dev/console (isconsole=true inside rpmsg_serialinit).
   */

  if (this_cpu() != 0)
    {
      return t113_rptun_init();
    }
#endif

#ifdef CONFIG_T113_HWSPINLOCK
  /* Bring up the hardware spinlock module.  Must run after the CCU has
   * been initialized (done in arm_boot()) and before any consumer that
   * might want to acquire a lock.
   */

  t113_hwspinlock_initialize();
#endif

  /* Resolve the CCU shared-resource lock.  In AMP builds this binds the
   * CCU helpers to the hardware spinlock instance dedicated to the CCU;
   * in non-AMP builds this is a no-op (the helpers use a private sw
   * spinlock).  Must run after t113_hwspinlock_initialize() so the
   * hwspinlock framework has 32 dev_s instances ready to be looked up.
   */

  t113_ccu_init();

#ifdef CONFIG_FS_TMPFS
  /* Mount tmpfs at /tmp.  Used by ADB-pulled audio files (nxplayer/
   * nxrecorder) and any transient scratch space.  Non-fatal on error so
   * the rest of bringup proceeds.
   */

  ret = nx_mount(NULL, "/tmp", "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nx_mount /tmp tmpfs failed: %d\n", ret);
    }
#endif

  t113_gpio_init();

  /* Pin mux is now handled by each driver's init function via
   * t113_gpio_config() + board.h pin selections.  No raw register
   * writes needed here.
   */

#ifdef CONFIG_T113_RPTUN_DSP
  /* Prepare UART2 for the HiFi4 DSP: configure PE2/PE3 pin mux and
   * enable the UART2 BUS clock + de-assert reset.  No NuttX serial
   * driver is bound -- the DSP firmware programs LCR/baud itself.
   */

  t113_dsp_uart_prepare();
#endif

  /* PD22: USB Host VBUS power enable - drive high.  The data register
   * is written before switching the pad to output so the pin comes up
   * high without a low-going glitch.
   */

  t113_gpio_write(usb_host_pwr, true);
  t113_gpio_config(usb_host_pwr);

#ifdef CONFIG_T113_USBHOST
  usbret = board_usbhost_initialize();
  if (usbret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_usbhost_initialize failed: %d\n",
             usbret);
    }
#endif

#ifdef CONFIG_T113_CE
  ret = t113_ce_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: t113_ce_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_T113_RTC
  t113_rtc_initialize();
#endif

#ifdef CONFIG_T113_PWM
  t113_pwm_initialize(0);
#endif

#ifdef CONFIG_T113_GPADC
  t113_adc_initialize();
#endif

#ifdef CONFIG_T113_LRADC
  t113_lradc_initialize();
#endif

#ifdef CONFIG_T113_LRADC_KEYPAD
  t113_keypad_initialize();
#endif

#ifdef CONFIG_T113_WDT
  t113_wdt_initialize();
#endif

#ifdef CONFIG_T113_SPI0

#ifdef CONFIG_MTD_MX35
  mtd = NULL;

#ifdef CONFIG_MX35_QSPI
  qspi = t113_qspi_initialize(0);
  if (qspi != NULL)
    {
      mtd = mx35_initialize(qspi);
    }
#else
  spi = t113_spibus_initialize(0);
  if (spi != NULL)
    {
      mtd = mx35_initialize(spi, 0);
    }
#endif

  if (mtd != NULL)
    {
      ret = t113_spinand_register(mtd);
    }
#endif /* CONFIG_MTD_MX35 */

#ifdef CONFIG_MTD_GD5F
  mtd = NULL;

#ifdef CONFIG_MTD_GD5F_QSPI
  qspi = t113_qspi_initialize(0);
  if (qspi != NULL)
    {
      mtd = gd5f_qspi_initialize(qspi);
    }
#else
  spi = t113_spibus_initialize(0);
  if (spi != NULL)
    {
      mtd = gd5f_initialize(spi, 0);
    }
#endif

  if (mtd != NULL)
    {
      ret = t113_spinand_register(mtd);
    }
#endif /* CONFIG_MTD_GD5F */

#ifdef CONFIG_MTD_F50L
  mtd = NULL;

#ifdef CONFIG_MTD_F50L_QSPI
  qspi = t113_qspi_initialize(0);
  if (qspi != NULL)
    {
      mtd = f50l_qspi_initialize(qspi);
    }
#else
  spi = t113_spibus_initialize(0);
  if (spi != NULL)
    {
      mtd = f50l_initialize(spi, 0);
    }
#endif

  if (mtd != NULL)
    {
      ret = t113_spinand_register(mtd);
    }
#endif /* CONFIG_MTD_F50L */

#endif /* CONFIG_T113_SPI0 */

#ifdef CONFIG_T113_HSTIMER0
  lower = t113_hstimer_initialize(0);
  if (lower != NULL)
    {
      timer_register("/dev/timer0", lower);
    }
#endif

#ifdef CONFIG_T113_HSTIMER1
  lower = t113_hstimer_initialize(1);
  if (lower != NULL)
    {
      timer_register("/dev/timer1", lower);
    }
#endif

#ifdef CONFIG_T113_TWI0
  i2c = t113_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "t113_bringup: TWI0 init failed\n");
    }
#ifdef CONFIG_I2C_DRIVER
  else
    {
      i2c_register(i2c, 0);
    }
#endif
#endif

#ifdef CONFIG_T113_TWI1
  i2c = t113_i2cbus_initialize(1);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "t113_bringup: TWI1 init failed\n");
    }
#ifdef CONFIG_I2C_DRIVER
  else
    {
      i2c_register(i2c, 1);
    }
#endif
#endif

#ifdef CONFIG_T113_TWI2
  i2c = t113_i2cbus_initialize(2);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "t113_bringup: TWI2 init failed\n");
    }
#ifdef CONFIG_I2C_DRIVER
  else
    {
      i2c_register(i2c, 2);
    }
#endif
#endif

#ifdef CONFIG_T113_CTP
  ret = t113_ctp_initialize("/dev/input0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "t113_bringup: CTP init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_T113_CAN0
  ret = t113_can_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: CAN0 init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_T113_CAN1
  ret = t113_can_initialize(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: CAN1 init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_T113_SMHC
  ret = t113_board_smhc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: t113_board_smhc_initialize failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_USBMSC) && !defined(CONFIG_EXAMPLES_NANDTEST)
  /* Auto-start USB Mass Storage on mtdblock2 (SPI NAND user partition).
   * This allows headless operation without requiring NSH serial console.
   * Disabled when nandtest is present - we need manual control.
   */

  ret = usbmsc_configure(1, &msc_handle);
  if (ret >= 0)
    {
      ret = usbmsc_bindlun(msc_handle, "/dev/mtdblock2", 0, 0, 0, false);
    }

  if (ret >= 0)
    {
      ret = usbmsc_exportluns(msc_handle);
    }

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: usbmsc auto-start failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_T113_UART_DMA
  work_queue(HPWORK, &g_dma_rx_poll_work,
             t113_dma_rx_poll_worker, NULL,
             MSEC2TICK(DMA_RX_POLL_INTERVAL_MS));
#endif

#ifdef CONFIG_LCD_GC9503CV_DSI
  ret = t113_lcd_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: t113_lcd_initialize failed: %d\n", ret);

      /* Non-fatal: continue bringup so a serial-console session is
       * still usable for debugging the LCD failure.
       */

      ret = OK;
    }
#endif

#ifdef CONFIG_BMP
  /* CPU0 rptun master up, then wake CPU1.  T113 lacks PSCI, so CPU1
   * never self-releases - nx_smp_start()'s up_cpu_start loop is a
   * no-op under CONFIG_SMP_NCPUS=1 and we must call it ourselves.
   */

  ret = t113_rptun_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: t113_rptun_init (master) failed: %d\n", ret);
      return ret;
    }

  ret = up_cpu_start(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: up_cpu_start(1) failed: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_T113_RPTUN_CORE1) || defined(CONFIG_T113_RPTUN_DSP)
  /* core0-NuttX as the rptun ELF master.  Register an rptun device per
   * remote core it loads (/dev/rptun/core1 and/or /dev/rptun/dsp); loading +
   * releasing each remote happen later via "rptun start /dev/rptun/<core>".
   */

  ret = t113_rptun_master_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: t113_rptun_master_init failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_T113_AUDIO
  /* mq-r adaptation: the PAM8301 PA's SD pin is hard-wired to VCC
   * (always on) and PC7 on this board is the SPI flash /HOLD signal,
   * so there is no GPIO-controlled PA enable to set up here.  Future
   * board variants with a real PA EN GPIO can register a board hook
   * before audio_register at this point.
   */

  audio_adc_lower = t113_codec_adc_initialize();
  if (audio_adc_lower == NULL)
    {
      syslog(LOG_ERR, "ERROR: t113_codec_adc_initialize failed\n");
    }
  else
    {
      ret = audio_register("audio1", audio_adc_lower);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: audio_register /dev/audio/audio1 failed: %d\n",
                 ret);
        }
    }

  audio_dac_lower = t113_codec_dac_initialize();
  if (audio_dac_lower == NULL)
    {
      syslog(LOG_ERR, "ERROR: t113_codec_dac_initialize failed\n");
    }
  else
    {
      ret = audio_register("audio2", audio_dac_lower);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: audio_register /dev/audio/audio2 failed: %d\n",
                 ret);
        }
    }
#endif

#ifdef CONFIG_T113_DMIC
  /* PDM DMIC capture node.  /dev/audio/audio0 is the PDM mic input
   * (MQ-R wires an MP34DT06J PDM mic between PD19 DATA0 and PD20 CLK
   * via flying leads).  Independent of the codec audio path -- the
   * controller has its own register block at 0x02031000 and shares
   * only the audio PLL.
   */

  audio_dmic_lower = t113_dmic_initialize();
  if (audio_dmic_lower == NULL)
    {
      syslog(LOG_ERR, "ERROR: t113_dmic_initialize failed\n");
    }
  else
    {
      ret = audio_register("audio0", audio_dmic_lower);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: audio_register /dev/audio/audio0 failed: %d\n",
                 ret);
        }
    }
#endif

  return ret;
}
