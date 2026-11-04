/****************************************************************************
 * drivers/lcd/st7789.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7789.h>

#include "st7789.h"

/* [LCDDIAG] Panel-layer tracing for the black-screen investigation.
 * Every line is tagged "[LCDDIAG] st7789:" so the probes are easy to find and
 * remove.  The verbose probes (one per SPI transfer) are capped so the console
 * is not flooded once the startup sweep has proven whether data reaches the
 * panel; the important init/power probes are always printed.
 */

#include <syslog.h>

#define LCDDIAG_VERBOSE_LIMIT 0   /* [QUIET] 屏幕每秒重绘，会淹没 NSH */

static unsigned long g_st7789_diag_n;

#define LCDDIAG(fmt, ...) \
  syslog(LOG_ERR, "[LCDDIAG] st7789: " fmt "\n", ##__VA_ARGS__)

#define LCDDIAG_V(fmt, ...) \
  do \
    { \
      if (g_st7789_diag_n < LCDDIAG_VERBOSE_LIMIT) \
        { \
          syslog(LOG_ERR, "[LCDDIAG] st7789: " fmt "\n", ##__VA_ARGS__); \
        } \
      g_st7789_diag_n++; \
    } \
  while (0)

#ifdef CONFIG_LCD_ST7789

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_SPI_CMDDATA has to be set for 4 wire interface */

#if !defined(CONFIG_LCD_ST7789_3WIRE) && !defined(CONFIG_SPI_CMDDATA)
#  error "CONFIG_SPI_CMDDATA option has to be set for SPI communication"
#endif

/* Verify that all configuration requirements have been met */

#ifndef CONFIG_LCD_ST7789_SPIMODE
#  define CONFIG_LCD_ST7789_SPIMODE SPIDEV_MODE0
#endif

/* SPI frequency */

#ifndef CONFIG_LCD_ST7789_FREQUENCY
#  define CONFIG_LCD_ST7789_FREQUENCY 1000000
#endif

/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 1
#endif

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER) || CONFIG_LCD_MAXPOWER < 1
#  define CONFIG_LCD_MAXPOWER 1
#endif

#if CONFIG_LCD_MAXPOWER > 255
#  error "CONFIG_LCD_MAXPOWER must be less than 256 to fit in uint8_t"
#endif

/* Check orientation */

#if defined(CONFIG_LCD_PORTRAIT)
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE) ||\
      defined(CONFIG_LCD_RPORTRAIT) || defined(CONFIG_LCD_DYN_ORIENTATION)
#    error "Cannot define both portrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_RPORTRAIT)
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE) ||\
      defined(CONFIG_LCD_DYN_ORIENTATION)
#    error "Cannot define both rportrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_LANDSCAPE)
#  if defined(CONFIG_LCD_RLANDSCAPE) || defined(CONFIG_LCD_DYN_ORIENTATION)
#    error "Cannot define both landscape and any other orientations"
#  endif
#elif defined(CONFIG_LCD_DYN_ORIENTATION)
#  ifdef CONFIG_LCD_RPORTRAIT
#    error "Cannot define both landscape and dynamic orientation"
#  endif
#elif !defined(CONFIG_LCD_RPORTRAIT)
#  define CONFIG_LCD_RPORTRAIT 1
#endif

/* Define prefixes for 3 wire communication if used */

#ifdef CONFIG_LCD_ST7789_3WIRE
#  define LCD_ST7789_SPI_BITS 9
#  define LCD_ST7789_DATA_PREFIX (1 << 8)
#  define LCD_ST7789_CMD_PREFIX  (0 << 8)
#else
#  define LCD_ST7789_SPI_BITS 8
#  define LCD_ST7789_DATA_PREFIX (0)
#  define LCD_ST7789_CMD_PREFIX  (0)
#endif

/* Display Resolution */

#if !defined(CONFIG_LCD_ST7789_XRES)
#  define CONFIG_LCD_ST7789_XRES 240
#endif

#if !defined(CONFIG_LCD_ST7789_YRES)
#  define CONFIG_LCD_ST7789_YRES 320
#endif

#define ST7789_LUT_SIZE    CONFIG_LCD_ST7789_YRES

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#  define ST7789_XRES       CONFIG_LCD_ST7789_YRES
#  define ST7789_YRES       CONFIG_LCD_ST7789_XRES
#  define ST7789_XOFFSET    CONFIG_LCD_ST7789_YOFFSET
#  define ST7789_YOFFSET    CONFIG_LCD_ST7789_XOFFSET
#else
#  define ST7789_XRES       CONFIG_LCD_ST7789_XRES
#  define ST7789_YRES       CONFIG_LCD_ST7789_YRES
#  define ST7789_XOFFSET    CONFIG_LCD_ST7789_XOFFSET
#  define ST7789_YOFFSET    CONFIG_LCD_ST7789_YOFFSET
#endif

/* Color depth and format */

#ifdef CONFIG_LCD_ST7789_BPP
#  if (CONFIG_LCD_ST7789_BPP == 12)
#    define ST7789_BPP           12
#    define ST7789_COLORFMT      FB_FMT_RGB12_444
#    define ST7789_BYTESPP       2
#  elif (CONFIG_LCD_ST7789_BPP == 16)
#    define ST7789_BPP           16
#    define ST7789_COLORFMT      FB_FMT_RGB16_565
#    define ST7789_BYTESPP       2
#  else
#    define ST7789_BPP           16
#    define ST7789_COLORFMT      FB_FMT_RGB16_565
#    define ST7789_BYTESPP       2
#    warning "Invalid color depth.  Falling back to 16bpp"
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of this driver */

struct st7789_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  FAR struct spi_dev_s *spi;  /* SPI device */
  uint8_t bpp;                /* Selected color depth */
  uint8_t power;              /* Current power setting */

#ifdef CONFIG_LCD_DYN_ORIENTATION
  uint16_t xoff;
  uint16_t yoff;
#endif

  /* This is working memory allocated by the LCD driver for each LCD device
   * and for each color plane. This memory will hold one raster line of data.
   * The size of the allocated run buffer must therefore be at least
   * (bpp * xres / 8).  Actual alignment of the buffer must conform to the
   * bitwidth of the underlying pixel type.
   *
   * If there are multiple planes, they may share the same working buffer
   * because different planes will not be operate on concurrently.  However,
   * if there are multiple LCD devices, they must each have unique run
   * buffers.
   */

  uint16_t runbuffer[ST7789_LUT_SIZE];
};

  /* 3 wire interface for ST7789 requires the driver to send information
   * about command/data transfer as 9th bit of SPI transfer. This would
   * require non standard SPI interface that is not supported so a little
   * workaround is used here (inspire by SSD1351 driver). We split our
   * buffer into rows and send those rows separately with added 9th bit.
   * The price for this is a small overhead in SPI communication.
   */

#ifdef CONFIG_LCD_ST7789_3WIRE
uint16_t rowbuff[ST7789_XRES * ST7789_BYTESPP];
#endif

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* Misc. Helpers */

static void st7789_select(FAR struct spi_dev_s *spi, int bits);
static void st7789_deselect(FAR struct spi_dev_s *spi);

static inline void st7789_sendcmd(FAR struct st7789_dev_s *dev, uint8_t cmd);
static void st7789_sleep(FAR struct st7789_dev_s *dev, bool sleep);
#ifdef  CONFIG_LCD_DYN_ORIENTATION
static void st7789_setorientation(FAR struct st7789_dev_s *dev,
                                  uint8_t orientation);
#else
static void st7789_setorientation(FAR struct st7789_dev_s *dev);
#endif
static void st7789_display(FAR struct st7789_dev_s *dev, bool on);
static void st7789_setarea(FAR struct st7789_dev_s *dev,
                           uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1);
static void st7789_bpp(FAR struct st7789_dev_s *dev, int bpp);
static void st7789_wrram(FAR struct st7789_dev_s *dev,
                         FAR const uint8_t *buff, size_t size, size_t skip,
                         size_t count);
#ifndef CONFIG_LCD_NOGETRUN
static void st7789_rdram(FAR struct st7789_dev_s *dev,
                         FAR uint16_t *buff, size_t size);
#endif
static void st7789_fill(FAR struct st7789_dev_s *dev, uint16_t color);

/* LCD Data Transfer Methods */

static int st7789_putrun(FAR struct lcd_dev_s *dev,
                         fb_coord_t row, fb_coord_t col,
                         FAR const uint8_t *buffer, size_t npixels);
static int st7789_putarea(FAR struct lcd_dev_s *dev,
                          fb_coord_t row_start, fb_coord_t row_end,
                          fb_coord_t col_start, fb_coord_t col_end,
                          FAR const uint8_t *buffer, fb_coord_t stride);
#ifndef CONFIG_LCD_NOGETRUN
static int st7789_getrun(FAR struct lcd_dev_s *dev,
                         fb_coord_t row, fb_coord_t col,
                         FAR uint8_t *buffer, size_t npixels);
#endif

/* LCD Configuration */

static int st7789_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo);
static int st7789_getplaneinfo(FAR struct lcd_dev_s *dev,
                               unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo);

/* LCD Specific Controls */

static int st7789_getpower(FAR struct lcd_dev_s *dev);
static int st7789_setpower(FAR struct lcd_dev_s *dev, int power);
static int st7789_getcontrast(FAR struct lcd_dev_s *dev);
static int st7789_setcontrast(FAR struct lcd_dev_s *dev,
                              unsigned int contrast);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct st7789_dev_s g_lcddev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: st7789_select
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
 *
 * Input Parameters:
 *   spi   - Reference to the SPI driver structure
 *   bits  - Number of SPI bits
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void st7789_select(FAR struct spi_dev_s *spi, int bits)
{
  /* Select ST7789 chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  int lock;
  uint32_t freq;

  lock = SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_DISPLAY(0), true);

  /* Now make sure that the SPI bus is configured for the ST7789 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_LCD_ST7789_SPIMODE);
  SPI_SETBITS(spi, bits);
  freq = SPI_SETFREQUENCY(spi, CONFIG_LCD_ST7789_FREQUENCY);

  LCDDIAG_V("select: SPI_LOCK=%d bits=%d freq_got=%u mode=%d",
            lock, bits, (unsigned)freq, CONFIG_LCD_ST7789_SPIMODE);
}

/****************************************************************************
 * Name: st7789_deselect
 *
 * Description:
 *   De-select the SPI
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void st7789_deselect(FAR struct spi_dev_s *spi)
{
  /* De-select ST7789 chip and relinquish the SPI bus. */

  int lock;

  SPI_SELECT(spi, SPIDEV_DISPLAY(0), false);
  lock = SPI_LOCK(spi, false);

  LCDDIAG_V("deselect: SPI_LOCK=%d", lock);
}

/****************************************************************************
 * Name: st7789_sendcmd
 *
 * Description:
 *   Send a command to the driver.
 *
 ****************************************************************************/

static inline void st7789_sendcmd(FAR struct st7789_dev_s *dev, uint8_t cmd)
{
#ifdef CONFIG_LCD_ST7789_3WIRE
  uint16_t txbuf;

  /* Add command prefix (9th bit shoudl be 0 ) */

  txbuf = LCD_ST7789_CMD_PREFIX | cmd;

  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);
  SPI_SEND(dev->spi, txbuf);
  st7789_deselect(dev->spi);
#else
  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, cmd);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
  st7789_deselect(dev->spi);
#endif
}

/****************************************************************************
 * Name: st7789_sleep
 *
 * Description:
 *   Sleep or wake up the driver.
 *
 ****************************************************************************/

static void st7789_sleep(FAR struct st7789_dev_s *dev, bool sleep)
{
  LCDDIAG("sleep(%d) -> send %s", (int)sleep,
          sleep ? "SLPIN(0x10)" : "SLPOUT(0x11)");
  st7789_sendcmd(dev, sleep ? ST7789_SLPIN : ST7789_SLPOUT);
  up_mdelay(120);
}

/****************************************************************************
 * Name: st7789_display
 *
 * Description:
 *   Turn on or off the display.
 *
 ****************************************************************************/

static void st7789_display(FAR struct st7789_dev_s *dev, bool on)
{
#ifdef CONFIG_LCD_ST7789_INVCOLOR
  LCDDIAG("display(on=%d) -> %s + INVON(0x21)  [INVCOLOR=y]",
          (int)on, on ? "DISPON(0x29)" : "DISPOFF(0x28)");
#else
  LCDDIAG("display(on=%d) -> %s + INVOFF(0x20) [INVCOLOR=n]",
          (int)on, on ? "DISPON(0x29)" : "DISPOFF(0x28)");
#endif
  st7789_sendcmd(dev, on ? ST7789_DISPON : ST7789_DISPOFF);
#ifdef CONFIG_LCD_ST7789_INVCOLOR
  st7789_sendcmd(dev, ST7789_INVON);
#else
  st7789_sendcmd(dev, ST7789_INVOFF);
#endif
}

/****************************************************************************
 * Name: st7789_setorientation
 *
 * Description:
 *   Set screen orientation.
 *
 ****************************************************************************/
#ifdef CONFIG_LCD_DYN_ORIENTATION
static void st7789_setorientation(FAR struct st7789_dev_s *dev,
                                  uint8_t orientation)
{
  /* No need to change the orientation in PORTRAIT mode */

  if (orientation != LCD_PORTRAIT)
    {
      st7789_sendcmd(dev, ST7789_MADCTL);
      st7789_select(dev->spi, LCD_ST7789_SPI_BITS);
    }

  if (orientation == LCD_RLANDSCAPE)
    {
      /* RLANDSCAPE : MY=1 MV=1 */

      SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | 0xa0);
    }
  else if (orientation == LCD_LANDSCAPE)
    {
      /* LANDSCAPE : MX=1 MV=1 */

      SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | 0x70);
    }
  else if (orientation == LCD_RPORTRAIT)
    {
      /* RPORTRAIT : MX=1 MY=1 */

      SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | 0xc0);
    }

  st7789_deselect(dev->spi);
}
#else
static void st7789_setorientation(FAR struct st7789_dev_s *dev)
{
  /* Default value on reset */

  uint8_t madctl = 0x00;

  st7789_sendcmd(dev, ST7789_MADCTL);
  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);

#if !defined(CONFIG_LCD_PORTRAIT)

#  if defined(CONFIG_LCD_RLANDSCAPE)
  /* RLANDSCAPE : MY=1 MV=1 */

  madctl = 0xa0;

#  elif defined(CONFIG_LCD_LANDSCAPE)
  /* LANDSCAPE : MX=1 MV=1 */

  madctl = 0x70;

#  elif defined(CONFIG_LCD_RPORTRAIT)
  /* RPORTRAIT : MX=1 MY=1 */

  madctl = 0xc0;
#  endif

#endif

  /* Mirror X/Y for current setting */

#ifdef CONFIG_LCD_ST7789_MIRRORX
  madctl ^= 0x40;
#endif

#ifdef CONFIG_LCD_ST7789_MIRRORY
  madctl ^= 0x80;
#endif

#ifdef CONFIG_LCD_ST7789_BGR
  madctl |= 0x08;
#endif

  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | madctl);

  st7789_deselect(dev->spi);
}
#endif

static void st7789_ramctl(FAR struct st7789_dev_s *dev)
{
  /* ============ ==== ==== ====== ====== ======== ===== ====== ======
   *  Parameters   D7   D6   D5     D4     D3       D2    D1     D0
   * ============ ==== ==== ====== ====== ======== ===== ====== ======
   *  1st (LSB)    0    0    0      RM     0        0     DM1    DM0
   *  2nd (MSB)    1    1    EPF1   EPF0   ENDIAN   RIM   MDT1   MDT0
   * ============ ==== ==== ====== ====== ======== ===== ====== ======
   */

  uint16_t ramctl = 0x0;

  st7789_sendcmd(dev, ST7789_RAMCTRL);
  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);   /* [V841] two 8-bit params */

  /* Fill the reserved bits */

  ramctl |= 0xc000;

  /* Set EPF */

  ramctl |= 0x3000;

  /* Set RGB data endian */

#ifdef CONFIG_LCD_ST7789_DATA_ENDIAN_LITTLE
  ramctl |= 0x800;
#endif

  /* [V841] Send the two parameters as two bytes, in the order the register
   * table above states: 1st (RM / DM) then 2nd (reserved + EPF + ENDIAN).
   *
   * This used to be one 16-bit SPI transfer of 0xF000.  SPI clocks the MSB
   * out first, so the panel received 0xF0 followed by 0x00 -- the two bytes
   * the wrong way round.  ESP-IDF sends 0x00, 0xF0.
   *
   * The rest of this driver already does it the explicit way (see
   * st7789_setarea, which sends the high byte then the low byte as separate
   * 8-bit transfers); RAMCTRL was the single exception. */
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | (uint8_t)(ramctl & 0xff));
  SPI_SEND(dev->spi,
           LCD_ST7789_DATA_PREFIX | (uint8_t)((ramctl >> 8) & 0xff));
  st7789_deselect(dev->spi);
}

/****************************************************************************
 * Name: st7789_setarea
 *
 * Description:
 *   Set the rectangular area for an upcoming read or write from RAM.
 *
 ****************************************************************************/

static void st7789_setarea(FAR struct st7789_dev_s *dev,
                           uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1)
{
  /* Set row address */

  st7789_sendcmd(dev, ST7789_RASET);
  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);
#ifdef CONFIG_LCD_DYN_ORIENTATION
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | ((y0 + g_lcddev.yoff) >> 8));
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | ((y0 + g_lcddev.yoff) & 0xff));
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | ((y1 + g_lcddev.yoff) >> 8));
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | ((y1 + g_lcddev.yoff) & 0xff));
#else
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | ((y0 + ST7789_YOFFSET) >> 8));
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX |
                     ((y0 + ST7789_YOFFSET) & 0xff));
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | ((y1 + ST7789_YOFFSET) >> 8));
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX |
                     ((y1 + ST7789_YOFFSET) & 0xff));
#endif
  st7789_deselect(dev->spi);

  /* Set column address */

  st7789_sendcmd(dev, ST7789_CASET);
  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);
#ifdef CONFIG_LCD_DYN_ORIENTATION
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | ((x0 + g_lcddev.xoff) >> 8));
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | ((x0 + g_lcddev.xoff) & 0xff));
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | ((x1 + g_lcddev.xoff) >> 8));
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | ((x1 + g_lcddev.xoff) & 0xff));
#else
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | ((x0 + ST7789_XOFFSET) >> 8));
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX |
                     ((x0 + ST7789_XOFFSET) & 0xff));
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | ((x1 + ST7789_XOFFSET) >> 8));
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX |
                     ((x1 + ST7789_XOFFSET) & 0xff));
#endif
  st7789_deselect(dev->spi);
}

/****************************************************************************
 * Name: st7789_bpp
 *
 * Description:
 *   Set the color depth of the device.
 *
 ****************************************************************************/

static void st7789_bpp(FAR struct st7789_dev_s *dev, int bpp)
{
  uint8_t depth;

  /* REVISIT: Works only for 12 and 16 bpp! */

  /* [V841] 16 bpp must be 0x55, not 0x05.
   *
   * `bpp >> 2 | 1` yields 0x05, and it is only ever called with 16 here, so
   * the panel was being told "16 bits/pixel, RGB interface format 0".
   * ESP-IDF's esp_lcd_panel_st7789 programs colmod_val = 0x55, and the
   * bit-bang sequence in this same tree -- the one path that has actually
   * produced a picture on this board -- sends the same value:
   *
   *   esp32s3_board_lcd.c:1171   v771_cmd(0x3a); v771_data8(0x55);
   *
   * 0x55 and 0x05 both select 16 bits/pixel on the MCU side; the difference
   * is the RGB-interface field, which is what the reference uses.  Align
   * with the two things that are known to work rather than with arithmetic
   * that was never checked. */
  if (bpp == 16)
    {
      depth = 0x55;
    }
  else
    {
      depth = bpp >> 2 | 1;
    }
  st7789_sendcmd(dev, ST7789_COLMOD);
  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);
  SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | depth);
  st7789_deselect(dev->spi);

  /* Cache the new BPP */

  dev->bpp = bpp;
}

/****************************************************************************
 * Name: st7789_wrram
 *
 * Description:
 *   Write to the driver's RAM. It is possible to write multiples of size
 *   while skipping some values.
 *
 ****************************************************************************/

/* [V797] Bytes actually handed to SPI3.
 *
 * st7789_putarea() ends in an unconditional `return OK;` -- the value returned
 * by st7789_wrram() is discarded, and st7789_wrram() is itself void.  So
 * putarea=0 in the [V765] white-shot log means only "the upper layers accepted
 * the request"; it says nothing about whether any byte left the chip.  That is
 * why every software-side reading so far (display ok, page built, fb=3,
 * putarea>=0) has been consistent with a black screen.
 *
 * These counters are the first measurement of the transport itself: they are
 * incremented with each SPI_SNDBLOCK() size, so a shortfall against the
 * expected byte count is visible rather than inferred. */
static volatile uint32_t g_v797_spi_calls;
static volatile uint32_t g_v797_spi_bytes;

void st7789_v797_spi_stats(uint32_t *calls, uint32_t *bytes)
{
  if (calls != NULL)
    {
      *calls = g_v797_spi_calls;
    }

  if (bytes != NULL)
    {
      *bytes = g_v797_spi_bytes;
    }
}

static void st7789_wrram(FAR struct st7789_dev_s *dev,
                         FAR const uint8_t *buff, size_t size, size_t skip,
                         size_t count)
{
  size_t i;
#ifdef CONFIG_LCD_ST7789_3WIRE
  size_t j;
#endif

  st7789_sendcmd(dev, ST7789_RAMWR);

#ifdef CONFIG_LCD_ST7789_3WIRE
  if (count == 1)
    {
      /* We cannot send the entire buffer at once, split it to
       * separate rows.
       */

      count = ST7789_YRES;
      size = ST7789_XRES * ST7789_BYTESPP;
    }

  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);

  /* For each row */

  for (i = 0; i < count; i++)
    {
      /* Copy data to rowbuff and add 9th bit */

      for (j = 0; j < ST7789_XRES * ST7789_BYTESPP; j += 2)
        {
          /* Take care of correct byte order. */

          rowbuff[j] = LCD_ST7789_DATA_PREFIX |
                       (uint16_t)buff[j + 1 + (i * (size + skip))];
          rowbuff[j + 1] = LCD_ST7789_DATA_PREFIX |
                           (uint16_t)buff[j + (i * (size + skip))];
        }

      g_v797_spi_calls++;
      g_v797_spi_bytes += (uint32_t)size;
      SPI_SNDBLOCK(dev->spi, rowbuff, size);
    }
#else
  LCDDIAG_V("wrram: count=%u size=%u skip=%u nwords/tr=%u -> %u bytes via SPI;"
            " first_px=0x%04x",
            (unsigned)count, (unsigned)size, (unsigned)skip,
            (unsigned)(size / ST7789_BYTESPP), (unsigned)(count * size),
            (unsigned)((FAR const uint16_t *)buff)[0]);

  st7789_select(dev->spi, ST7789_BYTESPP * LCD_ST7789_SPI_BITS);

  for (i = 0; i < count; i++)
    {
      g_v797_spi_calls++;
      g_v797_spi_bytes += (uint32_t)size;
      SPI_SNDBLOCK(dev->spi, buff + (i * (size + skip)),
                   size / ST7789_BYTESPP);
    }
#endif

  st7789_deselect(dev->spi);
}

/****************************************************************************
 * Name: st7789_rdram
 *
 * Description:
 *   Read from the driver's RAM.
 *
 ****************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static void st7789_rdram(FAR struct st7789_dev_s *dev,
                         FAR uint16_t *buff, size_t size)
{
  st7789_sendcmd(dev, ST7789_RAMRD);

  st7789_select(dev->spi, ST7789_BYTESPP * 8);
  SPI_RECVBLOCK(dev->spi, buff, size);
  st7789_deselect(dev->spi);
}
#endif

/****************************************************************************
 * Name: st7789_fill
 *
 * Description:
 *   Fill the display with the specified color.
 *
 ****************************************************************************/

static void st7789_fill(FAR struct st7789_dev_s *dev, uint16_t color)
{
  int i;

  LCDDIAG_V("fill(color=0x%04x) full %dx%d", (unsigned)color,
            ST7789_XRES, ST7789_YRES);

  st7789_setarea(dev, 0, 0, ST7789_XRES - 1, ST7789_YRES - 1);

  st7789_sendcmd(dev, ST7789_RAMWR);
#ifdef CONFIG_LCD_ST7789_3WIRE
  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);

  for (i = 0; i < ST7789_XRES * ST7789_YRES; i++)
    {
      SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | (color & 0xff));
      SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | (color & 0xff00) >> 8);
    }
#else
  st7789_select(dev->spi, ST7789_BYTESPP * LCD_ST7789_SPI_BITS);

  for (i = 0; i < ST7789_XRES * ST7789_YRES; i++)
    {
      SPI_SEND(dev->spi, color);
    }
#endif

  st7789_deselect(dev->spi);
}

/****************************************************************************
 * Name:  st7789_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD:
 *
 *   dev     - The lcd device
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int st7789_putrun(FAR struct lcd_dev_s *dev,
                         fb_coord_t row, fb_coord_t col,
                         FAR const uint8_t *buffer, size_t npixels)
{
  FAR struct st7789_dev_s *priv = (FAR struct st7789_dev_s *)dev;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  st7789_setarea(priv, col, row, col + npixels - 1, row);
  st7789_wrram(priv, buffer, npixels * ST7789_BYTESPP, 0, 1);

  return OK;
}

/****************************************************************************
 * Name:  st7789_putarea
 *
 * Description:
 *   This method can be used to write a partial area to the LCD:
 *
 *   dev       - The lcd device
 *   row_start - Starting row to write to (range: 0 <= row < yres)
 *   row_end   - Ending row to write to (range: row_start <= row < yres)
 *   col_start - Starting column to write to (range: 0 <= col <= xres)
 *   col_end   - Ending column to write to
 *               (range: col_start <= col_end < xres)
 *   buffer    - The buffer containing the area to be written to the LCD
 *   stride    - Length of a line in bytes. This parameter may be necessary
 *               to allow the LCD driver to calculate the offset for partial
 *               writes when the buffer needs to be splited for row-by-row
 *               writing.
 *
 ****************************************************************************/

static int st7789_putarea(FAR struct lcd_dev_s *dev,
                          fb_coord_t row_start, fb_coord_t row_end,
                          fb_coord_t col_start, fb_coord_t col_end,
                          FAR const uint8_t *buffer, fb_coord_t stride)
{
  FAR struct st7789_dev_s *priv = (FAR struct st7789_dev_s *)dev;
  size_t cols = col_end - col_start + 1;
  size_t rows = row_end - row_start + 1;
  size_t row_size = cols * ST7789_BYTESPP;

  ginfo("row_start: %d row_end: %d col_start: %d col_end: %d\n",
         row_start, row_end, col_start, col_end);

  LCDDIAG_V("putarea: r=%d..%d c=%d..%d rows=%u cols=%u row_size=%u stride=%d"
            " mode=%s total_bytes=%u buf=%p",
            (int)row_start, (int)row_end, (int)col_start, (int)col_end,
            (unsigned)rows, (unsigned)cols, (unsigned)row_size, (int)stride,
            (stride == row_size) ? "full" : "rowbyrow",
            (unsigned)(rows * row_size), (FAR const void *)buffer);

  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  st7789_setarea(priv, col_start, row_start, col_end, row_end);

  /* If the stride is the same of the row, a single SPI transfer is enough.
   * That is always true for lcddev. For framebuffer, that indicates a full
   * screen or full row update.
   */

  if (stride == row_size)
    {
      /* simpler case, we can just send the whole buffer */

      ginfo("Using full screen/full row mode\n");
      st7789_wrram(priv, buffer, rows * row_size, 0, 1);
    }
  else
    {
      /* We have to go row by row */

      ginfo("Falling-back to row by row mode\n");
      st7789_wrram(priv, buffer, row_size, stride - row_size, rows);
    }

  return OK;
}

/****************************************************************************
 * Name:  st7789_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD:
 *
 *  dev     - The lcd device
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static int st7789_getrun(FAR struct lcd_dev_s *dev,
                         fb_coord_t row, fb_coord_t col,
                         FAR uint8_t *buffer, size_t npixels)
{
  FAR struct st7789_dev_s *priv = (FAR struct st7789_dev_s *)dev;
  FAR uint16_t *dest = (FAR uint16_t *)buffer;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  st7789_setarea(priv, col, row, col + npixels - 1, row);
  st7789_rdram(priv, dest, npixels);

  return OK;
}
#endif

/****************************************************************************
 * Name:  st7789_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int st7789_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdinfo("fmt: %d xres: %d yres: %d nplanes: 1\n",
          ST7789_COLORFMT, ST7789_XRES, ST7789_YRES);

  vinfo->fmt     = ST7789_COLORFMT;    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  vinfo->xres    = ST7789_XRES;        /* Horizontal resolution in pixel columns */
  vinfo->yres    = ST7789_YRES;        /* Vertical resolution in pixel rows */
  vinfo->nplanes = 1;                  /* Number of color planes supported */
  return OK;
}

/****************************************************************************
 * Name:  st7789_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int st7789_getplaneinfo(FAR struct lcd_dev_s *dev,
                               unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo)
{
  FAR struct st7789_dev_s *priv = (FAR struct st7789_dev_s *)dev;

  DEBUGASSERT(dev && pinfo && planeno == 0);
  lcdinfo("planeno: %d bpp: %d\n", planeno, ST7789_BPP);

  pinfo->putrun = st7789_putrun;                  /* Put a run into LCD memory */
  pinfo->putarea = st7789_putarea;                /* Put an area into LCD */
#ifndef CONFIG_LCD_NOGETRUN
  pinfo->getrun = st7789_getrun;                  /* Get a run from LCD memory */
#endif
  pinfo->buffer = (FAR uint8_t *)priv->runbuffer; /* Run scratch buffer */
  pinfo->bpp    = priv->bpp;                      /* Bits-per-pixel */
  pinfo->dev    = dev;                            /* The lcd device */
  return OK;
}

/****************************************************************************
 * Name:  st7789_getpower
 ****************************************************************************/

static int st7789_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct st7789_dev_s *priv = (FAR struct st7789_dev_s *)dev;

  lcdinfo("power: %d\n", priv->power);
  return priv->power;
}

/****************************************************************************
 * Name:  st7789_setpower
 ****************************************************************************/

static int st7789_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct st7789_dev_s *priv = (FAR struct st7789_dev_s *)dev;

  lcdinfo("power: %d\n", power);
  LCDDIAG("setpower(power=%d) MAXPOWER=%d", power, CONFIG_LCD_MAXPOWER);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
      /* [V760] Exit Sleep-In BEFORE turning the display on.
       *
       * The ST7789 powers up in Sleep-In mode.  While it is asleep the
       * DISPON(0x29) command is silently ignored -- the charge pump is off
       * and the panel keeps showing nothing at all.  st7789_display() only
       * sends DISPON plus INVON, so every "power on" that reached a
       * sleeping panel was a no-op.  That is the cold-boot black screen:
       * the one-shot SWRESET + SLPOUT + DISPON sequence in
       * st7789_lcdinitialize() is clocked out before the panel rail is up,
       * those bytes are lost, and nothing ever re-issued SLPOUT.  A warm
       * (EN) reset works because the rail never dropped, so the panel
       * stayed awake and the boot sequence landed.
       *
       * st7789_sleep(dev, false) sends SLPOUT(0x11) and then waits the
       * 120 ms the datasheet requires before the next command.  On a panel
       * that is already awake it is harmless, so emitting it on every
       * setpower(1) is safe.  Callers audited: only lcd_framebuffer.c
       * (once, at fb init) and the V760 panel waker (every 2~6 s).
       */

      st7789_sleep(priv, false);

      /* [V765] The same gap v760 left open, one step further along.
       * Waking the panel out of Sleep-In is only half of it.  Without
       * NORON(0x13) the controller may sit in inversion or partial mode,
       * where DISPON is accepted and GRAM is writable but nothing is
       * displayed.  The V742 watchdog re-asserts setpower(1) every 2 s,
       * so leaving NORON out here means the watchdog actively pushes the
       * panel back into that half-awake state on every round.
       * Placed after SLPOUT (which already carries its own 120 ms wait)
       * and before DISPON, matching the sequence verified on real
       * hardware: SWRESET / SLPOUT / COLMOD / MADCTL / INVON / NORON / DISPON. */
      st7789_sendcmd(priv, 0x13);          /* NORON */
      up_mdelay(10);

      /* Turn on the display */

      st7789_display(priv, true);


      /* Save the power */

      priv->power = power;
    }
  else
    {
      /* Turn off the display */

      st7789_display(priv, false);

      /* Save the power */

      priv->power = 0;
    }

  return OK;
}

/****************************************************************************
 * Name:  st7789_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int st7789_getcontrast(FAR struct lcd_dev_s *dev)
{
  lcdinfo("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name:  st7789_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int st7789_setcontrast(FAR struct lcd_dev_s *dev,
                              unsigned int contrast)
{
  lcdinfo("contrast: %d\n", contrast);
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  st7789_initialize
 *
 * Description:
 *   Initialize the ST7789 video hardware.  The initial state of the
 *   LCD is fully initialized, display memory cleared, and the LCD ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified LCD.  NULL is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LCD_DYN_ORIENTATION
FAR struct lcd_dev_s *st7789_lcdinitialize(FAR struct spi_dev_s *spi,
                                           uint8_t orientation,
                                           uint16_t xoff, uint16_t yoff)
#else
FAR struct lcd_dev_s *st7789_lcdinitialize(FAR struct spi_dev_s *spi)
#endif
{
  FAR struct st7789_dev_s *priv = &g_lcddev;

  /* Initialize the driver data structure */

  priv->dev.getvideoinfo = st7789_getvideoinfo;
  priv->dev.getplaneinfo = st7789_getplaneinfo;
  priv->dev.getpower     = st7789_getpower;
  priv->dev.setpower     = st7789_setpower;
  priv->dev.getcontrast  = st7789_getcontrast;
  priv->dev.setcontrast  = st7789_setcontrast;
  priv->spi              = spi;

#ifdef CONFIG_LCD_DYN_ORIENTATION
  g_lcddev.xoff = xoff;
  g_lcddev.yoff = yoff;
#endif

  /* Init the hardware and clear the display */

  LCDDIAG("lcdinitialize: ENTER (%dx%d bpp=%d spi=%p)",
          ST7789_XRES, ST7789_YRES, ST7789_BPP, (FAR void *)spi);

  /* [V325] 补软复位（移植小智 esp_lcd_panel_reset 的语义）。
   *
   * 本板 LCD **没有复位引脚**（RST=NC），而原实现只发 SLPOUT(0x11)、
   * 从不发 SWRESET(0x01)。ST7789 在多次热重启后会停在未定义状态：
   * 命令全部被 ACK、就是不显示 —— 这正是本项目"刷多少次都黑"的合理成因。
   * 小智侧靠 esp_lcd_panel_reset() 完成同一件事，NuttX 没有等价动作。 */

  /* [V774 -> RETRACTED in v781] "SWRESET IS THE CULPRIT" was
   * WRONG: the A/B behind it ran under a warm reset and the
   * v774 follow-up falsified it.  The reasoning is kept below
   * as a record; see the [V781] block for the restoration. */
  /* --- v774 rationale, retracted, kept for the record ---
   *
   * v771/v773 ran a single-variable A/B with the raw GPIO bit-bang sequence:
   *
   *   v771  SLPOUT/COLMOD/MADCTL/INVON/NORON/RAMWR/fill/DISPON   -> LIT
   *   v773  SWRESET + 200ms + the exact same sequence            -> BLACK
   *
   * Same pins, same routing, same fill, same backlight; the only difference
   * is one byte on the wire.  So SWRESET is what puts this panel into a
   * state it never leaves.
   *
   * Why it bites here specifically: the LCD has RST=NC, so after power-up
   * the controller sits in an undefined state in which it silently swallows
   * SPI commands -- every command is ACKed, GRAM is writable, the driver
   * reports "display ok", and the screen stays black.  v325 added SWRESET as
   * the only way to force a known state; on this part that cure is worse
   * than the disease.  The panel lights correctly without it, which is why
   * the raw sequence in esp32s3_board_lcd.c (no SWRESET) is the reference
   * bring-up path.
   */
  /* [V781] *** SWRESET(0x01) RESTORED -- the v774 removal was based on
   * a warm-reset A/B and has been DISPROVEN.  Both v773 (SWRESET, black)
   * and v774 (no SWRESET, still black on cold boot) were measured, and the
   * second one falsifies the hypothesis instead of confirming it.  The
   * removal was never reverted, so v771..v780 all initialised the panel
   * without the exact command v325 added for this board: with RST=NC the
   * controller otherwise stays in a state that silently swallows SPI
   * traffic (all commands ACKed, GRAM writable, display ok, screen black)
   * -- which is precisely the symptom v760 fixed by adding it. */
  st7789_sendcmd(priv, 0x01);            /* SWRESET */
  up_mdelay(150);
  LCDDIAG("lcdinitialize: SWRESET(0x01) RE-ISSUED + 150ms settle [V781]\n");

  st7789_sleep(priv, false);
  st7789_bpp(priv, ST7789_BPP);
#ifdef  CONFIG_LCD_DYN_ORIENTATION
  st7789_setorientation(priv, orientation);
#else
  st7789_setorientation(priv);
#endif
  st7789_ramctl(priv);

  /* [V765] ★★★ NORON(0x13) —— 本板初始化序列一直缺的那一条。
   *
   * 本板 LCD 的 RST 是 NC（不接），所以唯一能让面板回到确定状态的通路就是
   * SPI 的 SWRESET。而真实硬件验证通过的序列（本仓库 esp32s3_board_lcd.c
   * 里逐字抄下的实测通过程序）是：
   *
   *     SWRESET / SLPOUT / COLMOD(0x55) / MADCTL(0x00) / INVON / NORON / DISPON
   *
   * 原实现发的是：SWRESET → SLPOUT → bpp → MADCTL → RAMCTL → DISPON+INVON
   * —— 少了 NORON。全树 grep NORON / 0x13 在本文件是 0 命中，从来没发过。
   *
   * ST7789 数据手册对 0x13 的定义是 "Normal Display Mode ON"。控制器从
   * Sleep-In 醒来后若停在 inversion mode 或 partial mode，DISPON 会被正常
   * 接受、GRAM 也可写可读，但面板不输出图像。这个症状与冷上电现场完全
   * 对得上：SLPOUT 发了 171 次、DISPON+INVON 发了 172 次、fill(0xffff)
   * 也执行了、背光 GPIO16 有电、UI 报 display ok，而屏是黑的。
   *
   * 放在 fill(0xffff) 之前：NORON 复位的是显示模式而不是 GRAM 内容，
   * 顺序上先退出异常模式、再灌白屏，两者都做到才有确定结果。 */

  st7789_sendcmd(priv, 0x13);          /* NORON */
  up_mdelay(10);
  LCDDIAG("lcdinitialize: NORON(0x13) issued");

  /* [V766] ★★★ fill BEFORE DISPON —— 与小智固件的实际顺序对齐。
   *
   * 小智 Display::Start()（VOX3/ai_vox/ai_vox-main/examples/ai-vox3/
   * ai-vox3/display.cpp:50-54）的第一件事就是逐行把整屏填白，
   * 然后才 DISPON：
   *
   *     std::vector<uint16_t> buffer(width_, 0xFFFF);
   *     for (int y = 0; y < height_; y++) {
   *         esp_lcd_panel_draw_bitmap(panel, 0, y, width_, y + 1, buffer.data());
   *     }
   *     ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
   *
   * 本板原来反过来：DISPON + INVON 先发，白屏后灌。ST7789 的 charge
   * pump 在 DISPON 之后才开始工作，面板被点亮的那一刻 GRAM 里还是
   * 上电垃圾 —— 这正好对��「命令全部被 ACK、软件侧 display ok、
   * 屏全黑」这个现象。
   *
   * 引脚已经核对过是同一套（小智 BL=16/MOSI=21/SCLK=17/CS=15/DC=14，
   * 本板 DC=14/BCKL=16/cs=15/clk=17/mosi=21），所以这不是接线差异，
   * 是纯粹的时序顺序差异。 */

  /* [V885] black instead of white: this is the other white flash. */
  st7789_fill(priv, 0x0000);
  LCDDIAG("[V766] GRAM filled 0xffff BEFORE DISPON");

  st7789_display(priv, true);

  LCDDIAG("lcdinitialize: DONE [V781=SWRESET restored] (SWRESET+SLPOUT+NORON+fill(0xffff)"
          "+DISPON+%s) [V765][V766]",
#ifdef CONFIG_LCD_ST7789_INVCOLOR
          "INVON"
#else
          "INVOFF"
#endif
          );

  return &priv->dev;
}


/****************************************************************************
 * Name: st7789_display_on_only
 *
 * Description:
 *   Re-assert DISPON(0x29) without touching Sleep-In.
 *
 *   st7789_setpower(dev, 1) is the right way to wake a panel whose rail has
 *   just come up, but it starts with SLPOUT(0x11) and then waits the 120 ms
 *   the datasheet requires, and that wait is spent with the panel in
 *   Sleep-In, i.e. blank.  Re-issuing setpower(1) on a timer is therefore
 *   what makes the panel visibly blink: every beat blanks the screen for
 *   120 ms before turning it back on.
 *
 *   Once the panel is awake it stays awake, so a keep-alive only has to keep
 *   the display on.  This sends DISPON (+INVON) alone - no SLPOUT, no
 *   NORON, no 120 ms gap - so the screen is never blanked in between.
 *
 *   The V892 panel waker uses this for every beat after its bounded wake
 *   window.  The first V819_WAKE_ATTEMPTS calls still go through
 *   setpower(1), which is the only thing that can recover a panel that
 *   booted with no rail at all.
 ****************************************************************************/

int st7789_display_on_only(FAR struct lcd_dev_s *dev)
{
  FAR struct st7789_dev_s *priv = (FAR struct st7789_dev_s *)dev;

  if (priv == NULL)
    {
      return -EINVAL;
    }

  st7789_display(priv, true);   /* DISPON(0x29) (+INVON) only */
  priv->power = 1;

  LCDDIAG("[V892] display_on_only: DISPON re-asserted\n");

  return OK;
}

/****************************************************************************
 * Name: st7789_reconfigure
 *
 * Description:
 *   Re-send everything the panel forgets when its power rail drops.
 *
 *   st7789_setpower(1) only sends SLPOUT + NORON + DISPON.  That is enough
 *   for a panel that has been awake since boot, because it still holds the
 *   pixel format, the scan direction and the RAM control word that
 *   st7789_lcdinitialize() programmed into it.
 *
 *   It is NOT enough for a panel that lost its rail.  On this board the
 *   panel is powered by a hardware latch, so on a cold start
 *   st7789_lcdinitialize() runs while the panel has no supply at all: every
 *   one of those configuration writes is swallowed.  When the rail finally
 *   arrives the controller comes back on its power-on defaults, and the
 *   RGB565 stream the driver then pushes is decoded with the wrong pixel
 *   format -- every byte arrives correctly and the screen is still black,
 *   which is exactly the reported symptom.
 *
 *   This replays the configuration half of the init sequence, without the
 *   SWRESET and without the full-screen fill (both would blank or flash an
 *   already-running panel).  On a panel that is already configured it is a
 *   no-op.
 ****************************************************************************/

int st7789_reconfigure(FAR struct lcd_dev_s *dev)
{
  FAR struct st7789_dev_s *priv = (FAR struct st7789_dev_s *)dev;

  if (priv == NULL)
    {
      return -EINVAL;
    }

  /* Wake: the panel ignores GRAM writes until SLPOUT has settled. */

  st7789_sleep(priv, false);                 /* SLPOUT + 120 ms */

  /* The configuration half that a power-cycled panel has lost. */

  st7789_bpp(priv, ST7789_BPP);              /* COLMOD -- pixel format */
#ifndef CONFIG_LCD_DYN_ORIENTATION
  st7789_setorientation(priv);               /* MADCTL -- scan direction */
#else
  st7789_setorientation(priv, 0);
#endif
  st7789_ramctl(priv);                       /* RAMCTRL -- GRAM window */

  st7789_sendcmd(priv, 0x13);                /* NORON */
  up_mdelay(10);
  st7789_display(priv, true);                /* DISPON (+INVON) */

  LCDDIAG("[V833] reconfigure: SLPOUT+COLMOD+MADCTL+RAMCTL+NORON+DISPON\n");

  return OK;
}

#endif /* CONFIG_LCD_ST7789 */
