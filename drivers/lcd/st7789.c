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
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7789.h>

#include "st7789.h"

/* ESP32-S3 SPI CS keep-active: NuttX SPI_SEND drops CS between bytes, which
 * makes ST7789 exit RAMWR. The register below is forced to keep CS.
 */

/* ESP32-S3 SPI2 base: 0x60024000 (NOT 0x60005000!)
 * MISC offset: 0x20, bit 30: CS_KEEP_ACTIVE
 */

#define ESP32S3_SPI2_BASE   0x60024000
#define SPI2_MISC_REG       (ESP32S3_SPI2_BASE + 0x20)
#define SPI_CS_KEEP_ACTIVE  (1 << 30)

static inline void esp32s3_set_cs_keep_active(void)
{
  volatile uint32_t *reg = (volatile uint32_t *)SPI2_MISC_REG;
  *reg |= SPI_CS_KEEP_ACTIVE;
}

static inline void esp32s3_clr_cs_keep_active(void)
{
  volatile uint32_t *reg = (volatile uint32_t *)SPI2_MISC_REG;
  *reg &= ~SPI_CS_KEEP_ACTIVE;
}

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
#else
/* Byte swap buffer for non-3WIRE path: panel big-endian, swap high and low
 * bytes per pixel before sending
 */

uint8_t rowbuff_be[4096];   /* 4KB swap buffer: fewer/larger DMA blocks */
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

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_DISPLAY(0), true);

  /* Now make sure that the SPI bus is configured for the ST7789 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_LCD_ST7789_SPIMODE);
  SPI_SETBITS(spi, bits);
  SPI_SETFREQUENCY(spi, CONFIG_LCD_ST7789_FREQUENCY);

  /* Keep CS asserted across individual SPI_SEND calls inside wrram/setarea.
   * Without this, SPI_SETMODE above clears SPI_CS_KEEP_ACTIVE in MISC_REG,
   * CS gets deasserted after each byte, ST7796 exits RAMWR → garbled
   * display.
   */

  esp32s3_set_cs_keep_active();
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

  SPI_SELECT(spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(spi, false);
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
  if (orientation != LCD_PORTRAIT)
    {
      uint8_t madctl = 0x00;

      if (orientation == LCD_RLANDSCAPE)
        {
          madctl = 0xa0;  /* MY=1 MV=1 */
        }
      else if (orientation == LCD_LANDSCAPE)
        {
          madctl = 0x20;  /* MV=1 only, matching IDF swap_xy */
        }
      else if (orientation == LCD_RPORTRAIT)
        {
          madctl = 0xc0;  /* MX=1 MY=1 */
        }

      /* Command + data in ONE CS assertion (matching IDF pattern) */

      st7789_select(dev->spi, LCD_ST7789_SPI_BITS);
      SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
      SPI_SEND(dev->spi, ST7789_MADCTL);
      SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
      SPI_SEND(dev->spi, LCD_ST7789_DATA_PREFIX | madctl);
      st7789_deselect(dev->spi);
    }
}
#else
static void st7789_setorientation(FAR struct st7789_dev_s *dev)
{
  /* Default value on reset */

  uint8_t madctl = 0x00;

#if !defined(CONFIG_LCD_PORTRAIT)

#  if defined(CONFIG_LCD_RLANDSCAPE)
  madctl = 0xa0;   /* MY=1 MV=1 */
#  elif defined(CONFIG_LCD_LANDSCAPE)
  madctl = 0x20;   /* MV=1 (swap XY). +BGR→0x28 matches IDF */
#  elif defined(CONFIG_LCD_RPORTRAIT)
  madctl = 0xc0;   /* MX=1 MY=1 */
#  endif

#endif

#ifdef CONFIG_LCD_ST7789_MIRRORX
  madctl ^= 0x40;
#endif
#ifdef CONFIG_LCD_ST7789_MIRRORY
  madctl ^= 0x80;
#endif
#ifdef CONFIG_LCD_ST7789_BGR
  madctl |= 0x08;
#endif

  /* Command + data in ONE CS assertion (matching IDF pattern). Old code used
   * st7789_sendcmd() which broke CS between cmd and data, causing the data
   * byte to be interpreted as a new command.
   */

  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, ST7789_MADCTL);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
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
   *
   * IMPORTANT: Send in 8-bit mode to match IDF behavior.
   * IDF sends: ramctl_val_1=0x00, ramctl_val_2=0xF0 (big endian)
   * This means: byte1=0x00 (MCU), byte2=0xF0 (RGB: RM=1,DM=1,RIM=1,ENDIAN=1)
   */

  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);         /* 8-bit mode */
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, ST7789_RAMCTRL);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);

  /* Send two bytes: 0x00 (MCU param), 0xF0 (RGB param: big endian) */

  SPI_SEND(dev->spi, 0x00);  /* 1st byte: MCU param */
  SPI_SEND(dev->spi, 0xf0);  /* 2nd byte: RGB param (RM=1,DM=1,RIM=1,ENDIAN=1) */

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
  /* Row address — RASET command + 4 bytes data in one CS assertion.
   * CRITICAL: CS must stay LOW between command and data, matching IDF's
   * esp_lcd_panel_io_spi pattern. Old code broke CS between cmd and data,
   * causing the ST7796 to misinterpret data bytes as new commands.
   */

  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, ST7789_RASET);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
#ifdef CONFIG_LCD_DYN_ORIENTATION
  SPI_SEND(dev->spi, (y0 + g_lcddev.yoff) >> 8);
  SPI_SEND(dev->spi, (y0 + g_lcddev.yoff) & 0xff);
  SPI_SEND(dev->spi, (y1 + g_lcddev.yoff) >> 8);
  SPI_SEND(dev->spi, (y1 + g_lcddev.yoff) & 0xff);
#else
  SPI_SEND(dev->spi, (y0 + ST7789_YOFFSET) >> 8);
  SPI_SEND(dev->spi, (y0 + ST7789_YOFFSET) & 0xff);
  SPI_SEND(dev->spi, (y1 + ST7789_YOFFSET) >> 8);
  SPI_SEND(dev->spi, (y1 + ST7789_YOFFSET) & 0xff);
#endif
  st7789_deselect(dev->spi);

  /* Column address — CASET command + 4 bytes data in one CS assertion */

  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, ST7789_CASET);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
#ifdef CONFIG_LCD_DYN_ORIENTATION
  SPI_SEND(dev->spi, (x0 + g_lcddev.xoff) >> 8);
  SPI_SEND(dev->spi, (x0 + g_lcddev.xoff) & 0xff);
  SPI_SEND(dev->spi, (x1 + g_lcddev.xoff) >> 8);
  SPI_SEND(dev->spi, (x1 + g_lcddev.xoff) & 0xff);
#else
  SPI_SEND(dev->spi, (x0 + ST7789_XOFFSET) >> 8);
  SPI_SEND(dev->spi, (x0 + ST7789_XOFFSET) & 0xff);
  SPI_SEND(dev->spi, (x1 + ST7789_XOFFSET) >> 8);
  SPI_SEND(dev->spi, (x1 + ST7789_XOFFSET) & 0xff);
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

  depth = bpp >> 2 | 1;

  /* Command + data in ONE CS assertion (matching IDF pattern) */

  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, ST7789_COLMOD);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
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

static void st7789_wrram(FAR struct st7789_dev_s *dev,
                         FAR const uint8_t *buff, size_t size, size_t skip,
                         size_t count)
{
  size_t i;
  size_t j;

  /* Matching IDF: 8-bit mode throughout (lcd_cmd_bits=lcd_param_bits=8) */

  SPI_LOCK(dev->spi, true);
  SPI_SETMODE(dev->spi, CONFIG_LCD_ST7789_SPIMODE);
  SPI_SETBITS(dev->spi, LCD_ST7789_SPI_BITS);
  SPI_SETFREQUENCY(dev->spi, CONFIG_LCD_ST7789_FREQUENCY);
  SPI_SELECT(dev->spi, SPIDEV_DISPLAY(0), true);

  /* Keep CS asserted: SPI_SETMODE may clear SPI_CS_KEEP_ACTIVE in MISC_REG.
   * Without this, CS deasserts after each SPI_SEND → ST7796 exits RAMWR.
   */

  esp32s3_set_cs_keep_active();

  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, ST7789_RAMWR);

  /* Stay at 8-bit throughout (no SPI_SETBITS(16)) */

  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);

#ifdef CONFIG_LCD_ST7789_3WIRE
  if (count == 1)
    {
      count = ST7789_YRES;
      size = ST7789_XRES * ST7789_BYTESPP;
    }

  for (i = 0; i < count; i++)
    {
      for (j = 0; j < ST7789_XRES * ST7789_BYTESPP; j += 2)
        {
          rowbuff[j] = LCD_ST7789_DATA_PREFIX |
                       (uint16_t)buff[j + 1 + (i * (size + skip))];
          rowbuff[j + 1] = LCD_ST7789_DATA_PREFIX |
                           (uint16_t)buff[j + (i * (size + skip))];
        }

      SPI_SNDBLOCK(dev->spi, rowbuff, size);
    }
#else

  /* ST7789/ST7796 panel is BIG-ENDIAN (RAMCTL=0xF0, IDF: "Use big endian by
   * default"): the HIGH byte of each 16-bit pixel must be sent first. LVGL's
   * draw buffer is little-endian (low byte first), so every 2-byte pixel
   * must be swapped before SPI_SNDBLOCK -- exactly what the IDF does. Since
   * 2026-09-08 the swap+send is chunked through rowbuff_be (instead of one
   * pass over the whole buffer), so PUTAREA may hand over arbitrarily large
   * contiguous areas (e.g. a whole preview frame in one call) without ever
   * overflowing the working buffer. CS stays asserted across chunk
   * boundaries, so the panel remains in RAMWR.
   */

  for (i = 0; i < count; i++)
    {
      FAR const uint8_t *src = buff + (i * (size + skip));
      size_t left = size;

      while (left > 0)
        {
          size_t now = left;

          if (now > sizeof(rowbuff_be))
            {
              now = sizeof(rowbuff_be);
            }

          for (j = 0; j + 1 < now; j += 2)
            {
              rowbuff_be[j]     = src[j + 1];
              rowbuff_be[j + 1] = src[j];
            }

          SPI_SNDBLOCK(dev->spi, rowbuff_be, now);
          src += now;
          left -= now;
        }
    }
#endif

  SPI_SELECT(dev->spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(dev->spi, false);
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
  int row;

  /* Prepare row buffer for landscape mode (480 pixels wide) */

  uint8_t landscape_row_buf[ST7789_YRES * 2];

  for (int i = 0; i < ST7789_YRES; i++)
    {
      landscape_row_buf[i * 2]     = (uint8_t)(color >> 8);
      landscape_row_buf[i * 2 + 1] = (uint8_t)(color & 0xff);
    }

  /* CRITICAL: CASET + RASET + RAMWR + data must be in ONE CS assertion!
   * ST7796 exits write mode if CS goes HIGH between commands. Keep CS LOW
   * from CASET start to end of data.
   */

  st7789_select(dev->spi, LCD_ST7789_SPI_BITS);
  esp32s3_set_cs_keep_active();

  /* CASET command — sets physical Y range (0 to 479) for landscape mode
   * MV=1: CASET maps to physical Y direction (width 480)
   */

  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, ST7789_CASET);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
  SPI_SEND(dev->spi, 0x00);
  SPI_SEND(dev->spi, 0x00);
  SPI_SEND(dev->spi, (ST7789_YRES - 1) >> 8);
  SPI_SEND(dev->spi, (ST7789_YRES - 1) & 0xff);

  /* RASET command — sets physical X range (0 to 319) for landscape mode
   * MV=1: RASET maps to physical X direction (height 320)
   */

  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, ST7789_RASET);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
  SPI_SEND(dev->spi, 0x00);
  SPI_SEND(dev->spi, 0x00);
  SPI_SEND(dev->spi, (ST7789_XRES - 1) >> 8);
  SPI_SEND(dev->spi, (ST7789_XRES - 1) & 0xff);

  /* RAMWR command — start frame data write */

  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, ST7789_RAMWR);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);

  /* Send all rows: 320 rows, each 480 pixels wide */

  for (row = 0; row < ST7789_XRES; row++)
    {
      for (int i = 0; i < ST7789_YRES * 2; i++)
        {
          SPI_SEND(dev->spi, landscape_row_buf[i]);
        }
    }

  /* Clear CS_KEEP_ACTIVE and deselect CS */

  esp32s3_clr_cs_keep_active();
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
#ifdef CONFIG_LCD_LANDSCAPE
  /* In LANDSCAPE+MV=1, CASET maps to physical width (Y direction) =
   * ST7789_YRES, RASET maps physical height (X direction)=ST7789_XRES (see
   * st7789_fill comments). Therefore, landscape resolution for upper layer
   * (LVGL) must be width=ST7789_YRES, height=ST7789_XRES, Otherwise LVGL
   * only draws columns 0..ST7789_XRES → right 1/3 black screen.
   */

  vinfo->xres    = ST7789_YRES;        /* physical width 480 */
  vinfo->yres    = ST7789_XRES;        /* physical height 320 */
#else
  vinfo->xres    = ST7789_XRES;        /* Horizontal resolution in pixel columns */
  vinfo->yres    = ST7789_YRES;        /* Vertical resolution in pixel rows */
#endif
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
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
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

  st7789_sleep(priv, false);
  st7789_bpp(priv, ST7789_BPP);
#ifdef  CONFIG_LCD_DYN_ORIENTATION
  st7789_setorientation(priv, orientation);
#else
  st7789_setorientation(priv);
#endif
  st7789_ramctl(priv);
  st7789_display(priv, true);

  /* Clear display to black — without this, GRAM shows random power-up junk
   */

  st7789_fill(priv, 0x0000);

  return &priv->dev;
}

#endif /* CONFIG_LCD_ST7789 */
