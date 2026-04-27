/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_cam.c
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

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/cache.h>
#include <nuttx/wqueue.h>
#include <nuttx/video/imgdata.h>

#include <arch/board/board.h>

#include "esp32s3_clockconfig.h"
#include "esp32s3_gpio.h"
#include "esp32s3_dma.h"
#include "esp32s3_irq.h"

#include "xtensa.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/esp32s3_lcd_cam.h"
#include "hardware/esp32s3_dma.h"

#include "periph_ctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Use PLL=160MHz as clock resource for CAM */

#define ESP32S3_CAM_CLK_SEL       3
#define ESP32S3_CAM_CLK_MHZ       160

/* XCLK = 160 / (6 + 2/3) = 24 MHz */

#define ESP32S3_CAM_CLKM_DIV_NUM  6
#define ESP32S3_CAM_CLKM_DIV_A    3
#define ESP32S3_CAM_CLKM_DIV_B    2

/* DMA buffer configuration */

#define ESP32S3_CAM_DMA_BUFLEN    3840
#define ESP32S3_CAM_DMADESC_NUM   56
#define ESP32S3_CAM_HALF_BUF_SIZE 15360

/* VSYNC filter threshold */

#define ESP32S3_CAM_VSYNC_FILTER  7

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s3_cam_s
{
  struct imgdata_s data;

  bool capturing;

  int cpuint;
  uint8_t cpu;
  int32_t dma_channel;

  struct esp32s3_dmadesc_s dmadesc[ESP32S3_CAM_DMADESC_NUM];

  uint8_t *fb;                    /* DMA frame buffer (with overflow) */
  uint8_t *v4l2_buf;              /* V4L2 userptr buffer (no overflow) */
  uint32_t fb_size;               /* Frame size in bytes */
  uint32_t fb_alloc;              /* DMA buffer allocated size */
  uint32_t fb_pos;                /* Current write position */
  uint8_t vsync_cnt;              /* VSYNC counter for frame sync */
  uint32_t dma_received;          /* Bytes received by DMA so far */
  int dma_desc_offset;            /* Next DMA descriptor index to load */

  imgdata_capture_t cb;
  void *cb_arg;
  struct work_s frame_work;
  struct timeval frame_ts;

  int dma_cpuint;
  uint16_t width;
  uint16_t height;
  uint32_t pixfmt;

  spinlock_t lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32s3_cam_init(struct imgdata_s *data);
static int esp32s3_cam_uninit(struct imgdata_s *data);
static int esp32s3_cam_set_buf(struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               imgdata_format_t *datafmts,
                               uint8_t *addr, uint32_t size);
static int esp32s3_cam_validate_frame_setting(struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               imgdata_format_t *datafmts,
                               imgdata_interval_t *interval);
static int esp32s3_cam_start_capture(struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               imgdata_format_t *datafmts,
                               imgdata_interval_t *interval,
                               imgdata_capture_t callback,
                               void *arg);
static int esp32s3_cam_stop_capture(struct imgdata_s *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imgdata_ops_s g_cam_ops =
{
  .init                   = esp32s3_cam_init,
  .uninit                 = esp32s3_cam_uninit,
  .set_buf                = esp32s3_cam_set_buf,
  .validate_frame_setting = esp32s3_cam_validate_frame_setting,
  .start_capture          = esp32s3_cam_start_capture,
  .stop_capture           = esp32s3_cam_stop_capture,
};

static struct esp32s3_cam_s g_cam_priv =
{
  .data =
    {
      .ops = &g_cam_ops,
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void esp32s3_cam_setup_dma(struct esp32s3_cam_s *priv)
{
  uint8_t *pbuf = priv->fb;
  uint32_t remaining = priv->fb_alloc;
  uint32_t total_desc;
  uint32_t i;

  total_desc = (remaining + ESP32S3_CAM_DMA_BUFLEN - 1) /
               ESP32S3_CAM_DMA_BUFLEN;

  for (i = 0; i < total_desc && i < ESP32S3_CAM_DMADESC_NUM; i++)
    {
      uint32_t data_len;
      uint32_t buf_len;

      data_len = remaining > ESP32S3_CAM_DMA_BUFLEN ?
                 ESP32S3_CAM_DMA_BUFLEN : remaining;
      buf_len = (data_len + 63) & ~63;

      priv->dmadesc[i].ctrl = ESP32S3_DMA_CTRL_OWN |
          (buf_len << ESP32S3_DMA_CTRL_BUFLEN_S);
      priv->dmadesc[i].pbuf = pbuf;

      if (i < total_desc - 1)
        {
          priv->dmadesc[i].next = &priv->dmadesc[i + 1];
        }
      else
        {
          priv->dmadesc[i].next = NULL;
        }

      pbuf += data_len;
      remaining -= data_len;
    }
}

static void frame_complete_worker(void *arg)
{
  struct esp32s3_cam_s *priv = (struct esp32s3_cam_s *)arg;

  if (priv->cb == NULL || priv->fb == NULL)
    {
      return;
    }

  up_invalidate_dcache((uintptr_t)priv->fb,
                       (uintptr_t)priv->fb + priv->fb_size);

  if (priv->v4l2_buf != NULL)
    {
      memcpy(priv->v4l2_buf, priv->fb, priv->fb_size);
    }

  priv->cb(0, priv->fb_size, &priv->frame_ts, priv->cb_arg);
}

/****************************************************************************
 * Name: cam_interrupt
 *
 * Description:
 *   CAM VSYNC interrupt handler. Called when a frame is complete.
 *
 ****************************************************************************/

static int IRAM_ATTR cam_interrupt(int irq, void *context, void *arg)
{
  struct esp32s3_cam_s *priv = (struct esp32s3_cam_s *)arg;
  uint32_t status = getreg32(LCD_CAM_LC_DMA_INT_ST_REG);

  if (status & LCD_CAM_CAM_VSYNC_INT_ST_M)
    {
      /* Clear interrupt */

      putreg32(LCD_CAM_CAM_VSYNC_INT_CLR_M,
               LCD_CAM_LC_DMA_INT_CLR_REG);

      if (priv->capturing)
        {
          priv->vsync_cnt++;

          /* First VSYNC = start of frame (DMA begins filling buffer).
           * Second VSYNC = end of frame (buffer is complete).
           */

          if (priv->vsync_cnt == 1)
            {
              /* Frame capture just started, DMA is now receiving data.
               * Nothing to do here — wait for next VSYNC.
               */
            }
          else if (priv->vsync_cnt >= 2 && priv->cb)
            {
              uint32_t regval;

              regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
              regval &= ~LCD_CAM_CAM_START_M;
              putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

              regval = getreg32(LCD_CAM_CAM_CTRL_REG);
              regval |= LCD_CAM_CAM_UPDATE_REG_M;
              putreg32(regval, LCD_CAM_CAM_CTRL_REG);

              SET_GDMA_CH_BITS(DMA_IN_LINK_CH0_REG,
                               priv->dma_channel,
                               DMA_INLINK_STOP_CH0_M);

              /* Bounded wait for DMA to park */

              for (int tries = 1000; tries > 0; tries--)
                {
                  uint32_t regaddr = DMA_IN_LINK_CH0_REG +
                      priv->dma_channel * GDMA_REG_OFFSET;
                  if (getreg32(regaddr) & DMA_INLINK_PARK_CH0)
                    {
                      break;
                    }
                }

              priv->capturing = false;
              gettimeofday(&priv->frame_ts, NULL);
              work_queue(LPWORK, &priv->frame_work,
                         frame_complete_worker, priv, 0);
            }
        }
    }

  /* Clear any CAM HS interrupt too */

  if (status & LCD_CAM_CAM_HS_INT_ST_M)
    {
      putreg32(LCD_CAM_CAM_HS_INT_CLR_M,
               LCD_CAM_LC_DMA_INT_CLR_REG);
    }

  return 0;
}

/****************************************************************************
 * Name: esp32s3_cam_gpio_config
 *
 * Description:
 *   Configure GPIO pins for DVP camera interface.
 *
 ****************************************************************************/

static void esp32s3_cam_gpio_config(void)
{
  /* Data pins D0-D7 as input via GPIO matrix */

  esp32s3_configgpio(CONFIG_ESP32S3_CAM_D0_PIN, INPUT);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAM_D0_PIN,
                         CAM_DATA_IN0_IDX, false);

  esp32s3_configgpio(CONFIG_ESP32S3_CAM_D1_PIN, INPUT);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAM_D1_PIN,
                         CAM_DATA_IN1_IDX, false);

  esp32s3_configgpio(CONFIG_ESP32S3_CAM_D2_PIN, INPUT);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAM_D2_PIN,
                         CAM_DATA_IN2_IDX, false);

  esp32s3_configgpio(CONFIG_ESP32S3_CAM_D3_PIN, INPUT);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAM_D3_PIN,
                         CAM_DATA_IN3_IDX, false);

  esp32s3_configgpio(CONFIG_ESP32S3_CAM_D4_PIN, INPUT);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAM_D4_PIN,
                         CAM_DATA_IN4_IDX, false);

  esp32s3_configgpio(CONFIG_ESP32S3_CAM_D5_PIN, INPUT);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAM_D5_PIN,
                         CAM_DATA_IN5_IDX, false);

  esp32s3_configgpio(CONFIG_ESP32S3_CAM_D6_PIN, INPUT);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAM_D6_PIN,
                         CAM_DATA_IN6_IDX, false);

  esp32s3_configgpio(CONFIG_ESP32S3_CAM_D7_PIN, INPUT);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAM_D7_PIN,
                         CAM_DATA_IN7_IDX, false);

  /* PCLK input */

  esp32s3_configgpio(CONFIG_ESP32S3_CAM_PCLK_PIN, INPUT);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAM_PCLK_PIN,
                         CAM_PCLK_IDX, false);

  /* VSYNC input */

  esp32s3_configgpio(CONFIG_ESP32S3_CAM_VSYNC_PIN, INPUT);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAM_VSYNC_PIN,
                         CAM_V_SYNC_IDX, true);

  /* HREF (H_ENABLE) input */

  esp32s3_configgpio(CONFIG_ESP32S3_CAM_HREF_PIN, INPUT);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAM_HREF_PIN,
                         CAM_H_ENABLE_IDX, false);

#ifndef CONFIG_ESP32S3_CAM_XCLK_LEDC
  esp32s3_configgpio(CONFIG_ESP32S3_CAM_XCLK_PIN, OUTPUT);
  esp32s3_gpio_matrix_out(CONFIG_ESP32S3_CAM_XCLK_PIN,
                          CAM_CLK_IDX, false, false);
#endif
}

/****************************************************************************
 * Name: esp32s3_cam_enableclk
 *
 * Description:
 *   Enable CAM clock and configure XCLK output.
 *
 ****************************************************************************/

static void esp32s3_cam_enableclk(void)
{
  uint32_t regval;

  /* Enable LCD_CAM peripheral clock (shared with LCD) */

  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  modifyreg32(LCD_CAM_LCD_CLOCK_REG, 0, LCD_CAM_CLK_EN_M);

  regval = (ESP32S3_CAM_CLK_SEL << LCD_CAM_CAM_CLK_SEL_S) |
           (ESP32S3_CAM_CLKM_DIV_A << LCD_CAM_CAM_CLKM_DIV_A_S) |
           (ESP32S3_CAM_CLKM_DIV_B << LCD_CAM_CAM_CLKM_DIV_B_S) |
           (ESP32S3_CAM_CLKM_DIV_NUM << LCD_CAM_CAM_CLKM_DIV_NUM_S) |
           LCD_CAM_CAM_VS_EOF_EN_M |
           (ESP32S3_CAM_VSYNC_FILTER << LCD_CAM_CAM_VSYNC_FILTER_THRES_S);
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);
}

/****************************************************************************
 * Name: esp32s3_cam_dmasetup
 *
 * Description:
 *   Configure DMA for camera RX.
 *
 ****************************************************************************/

static int esp32s3_cam_dmasetup(struct esp32s3_cam_s *priv)
{
  priv->dma_channel = esp32s3_dma_request(ESP32S3_DMA_PERIPH_LCDCAM,
                                          1, 10, true);
  if (priv->dma_channel < 0)
    {
      snerr("ERROR: Failed to allocate GDMA channel\n");
      return -ENOMEM;
    }

  esp32s3_dma_set_ext_memblk(priv->dma_channel,
                             false,
                             ESP32S3_DMA_EXT_MEMBLK_64B);

  return OK;
}

/****************************************************************************
 * Name: esp32s3_cam_config
 *
 * Description:
 *   Configure CAM controller registers and interrupts.
 *
 ****************************************************************************/

static int esp32s3_cam_config(struct esp32s3_cam_s *priv)
{
  int ret;
  uint32_t regval;
  irqstate_t flags;

  /* Reset CAM module and AFIFO first */

  regval = LCD_CAM_CAM_RESET_M | LCD_CAM_CAM_AFIFO_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Configure CAM_CTRL1 after reset:
   *   - VSYNC filter enable
   *   - REC_DATA_BYTELEN = DMA node size - 1 (controls in_suc_eof)
   *   Note: 8-bit DVP sensor outputs BE RGB565 (high byte first).
   *         CAM_BYTE_ORDER only reverses bits within a byte in
   *         1-byte mode, so HW byte swap is not available here.
   *         Application must swap bytes if LE RGB565 is needed.
   */

  regval = LCD_CAM_CAM_VSYNC_FILTER_EN_M |
           ((ESP32S3_CAM_HALF_BUF_SIZE - 1) <<
            LCD_CAM_CAM_REC_DATA_BYTELEN_S);
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Update registers */

  regval = getreg32(LCD_CAM_CAM_CTRL_REG);
  regval |= LCD_CAM_CAM_UPDATE_REG_M;
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);

  /* Bypass RGB/YUV conversion (bit=0 means bypass) */

  putreg32(0, LCD_CAM_CAM_RGB_YUV_REG);

  CLR_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, priv->dma_channel,
                   DMA_INDSCR_BURST_EN_CH0_M |
                   DMA_IN_DATA_BURST_EN_CH0_M);

  if (esp32s3_cam_dmasetup(priv) != OK)
    {
      return -ENOMEM;
    }

  /* Disable all LCD_CAM interrupts and clear pending */

  putreg32(0, LCD_CAM_LC_DMA_INT_ENA_REG);
  putreg32(0xffffffff, LCD_CAM_LC_DMA_INT_CLR_REG);

  /* Setup interrupt with CPU interrupts disabled.
   * Order matters:
   *   1) setup_irq creates cpuint → peripheral IRQ mapping
   *   2) irq_attach registers the handler for that IRQ
   * Since spin_lock_irqsave masks CPU interrupts, even if the
   * peripheral line is asserted after setup_irq, the interrupt
   * won't be serviced until after irq_attach completes.
   */

  flags = spin_lock_irqsave(&priv->lock);

  priv->cpu = this_cpu();

  /* 1) Map peripheral → CPU interrupt (creates IRQ mapping) */

  priv->cpuint = esp32s3_setup_irq(priv->cpu,
                                   ESP32S3_PERIPH_LCD_CAM,
                                   ESP32S3_INT_PRIO_DEF,
                                   ESP32S3_CPUINT_LEVEL);
  DEBUGASSERT(priv->cpuint >= 0);

  /* 2) Attach handler (mapping now exists, so IRQ is valid) */

  ret = irq_attach(ESP32S3_IRQ_LCD_CAM, cam_interrupt, priv);
  DEBUGASSERT(ret == 0);
  UNUSED(ret);

  /* 3) Clear any spurious interrupt that fired during setup */

  putreg32(0xffffffff, LCD_CAM_LC_DMA_INT_CLR_REG);

  spin_unlock_irqrestore(&priv->lock, flags);

  up_enable_irq(ESP32S3_IRQ_LCD_CAM);

  regval = getreg32(LCD_CAM_LC_DMA_INT_ENA_REG);
  regval |= LCD_CAM_CAM_VSYNC_INT_ENA_M;
  putreg32(regval, LCD_CAM_LC_DMA_INT_ENA_REG);

  return OK;
}

/****************************************************************************
 * Name: esp32s3_cam_init
 ****************************************************************************/

static int esp32s3_cam_init(struct imgdata_s *data)
{
  struct esp32s3_cam_s *priv = (struct esp32s3_cam_s *)data;

  priv->capturing = false;
  priv->cb = NULL;
  priv->cb_arg = NULL;
  priv->fb = NULL;
  priv->fb_size = 0;
  priv->fb_pos = 0;
  priv->width = 0;
  priv->height = 0;
  priv->pixfmt = 0;

  spin_lock_init(&priv->lock);

  /* Configure GPIO pins */

  esp32s3_cam_gpio_config();

  /* Enable clock and configure XCLK */

  esp32s3_cam_enableclk();

  /* Configure CAM controller */

  return esp32s3_cam_config(priv);
}

/****************************************************************************
 * Name: esp32s3_cam_uninit
 ****************************************************************************/

static int esp32s3_cam_uninit(struct imgdata_s *data)
{
  struct esp32s3_cam_s *priv = (struct esp32s3_cam_s *)data;
  uint32_t regval;

  work_cancel_sync(LPWORK, &priv->frame_work);

  /* Stop capture */

  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_START_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Disable interrupt */

  up_disable_irq(ESP32S3_IRQ_LCD_CAM);
  irq_detach(ESP32S3_IRQ_LCD_CAM);
  esp32s3_teardown_irq(priv->cpu, ESP32S3_PERIPH_LCD_CAM, priv->cpuint);

  /* Release DMA */

  if (priv->dma_channel >= 0)
    {
      esp32s3_dma_release(priv->dma_channel);
      priv->dma_channel = -1;
    }

  /* Free frame buffer */

  if (priv->fb)
    {
      kmm_free(priv->fb);
      priv->fb = NULL;
    }

  priv->capturing = false;

  return OK;
}

/****************************************************************************
 * Name: esp32s3_cam_set_buf
 ****************************************************************************/

static int esp32s3_cam_set_buf(struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               imgdata_format_t *datafmts,
                               uint8_t *addr, uint32_t size)
{
  struct esp32s3_cam_s *priv = (struct esp32s3_cam_s *)data;
  uint32_t needed_size;
  uint32_t alloc_size;

  priv->width  = datafmts[IMGDATA_FMT_MAIN].width;
  priv->height = datafmts[IMGDATA_FMT_MAIN].height;
  priv->pixfmt = datafmts[IMGDATA_FMT_MAIN].pixelformat;

  if (addr != NULL && size > 0)
    {
      needed_size = size;
      priv->v4l2_buf = addr;
    }
  else
    {
      needed_size = priv->width * priv->height * 2;
      priv->v4l2_buf = NULL;
    }

  alloc_size = needed_size + ESP32S3_CAM_HALF_BUF_SIZE * 3;

  if (priv->fb == NULL || priv->fb_size != needed_size)
    {
      if (priv->fb != NULL)
        {
          kmm_free(priv->fb);
          priv->fb = NULL;
        }

      priv->fb = kmm_memalign(64, alloc_size);
      if (!priv->fb)
        {
          snerr("ERROR: Failed to allocate frame buffer\n");
          return -ENOMEM;
        }

      priv->fb_size = needed_size;
      priv->fb_alloc = alloc_size;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32s3_cam_validate_frame_setting
 ****************************************************************************/

static int esp32s3_cam_validate_frame_setting(struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               imgdata_format_t *datafmts,
                               imgdata_interval_t *interval)
{
  if (nr_datafmts < 1 || !datafmts)
    {
      return -EINVAL;
    }

  /* Pixel format is validated by the sensor driver (imgsensor);
   * imgdata only checks that width and height are non-zero.
   */

  if (datafmts[IMGDATA_FMT_MAIN].width == 0 ||
      datafmts[IMGDATA_FMT_MAIN].height == 0)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32s3_cam_start_capture
 ****************************************************************************/

static int esp32s3_cam_start_capture(struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               imgdata_format_t *datafmts,
                               imgdata_interval_t *interval,
                               imgdata_capture_t callback,
                               void *arg)
{
  struct esp32s3_cam_s *priv = (struct esp32s3_cam_s *)data;
  uint32_t regval;

  if (priv->capturing)
    {
      return -EBUSY;
    }

  if (!priv->fb)
    {
      return -EINVAL;
    }

  priv->cb = callback;
  priv->cb_arg = arg;
  priv->fb_pos = 0;
  priv->vsync_cnt = 0;
  priv->dma_received = 0;
  priv->dma_desc_offset = 0;

  memset(priv->fb, 0, priv->fb_alloc);

  /* Rebuild DMA descriptors (reset OWN bits consumed by prior DMA) */

  esp32s3_cam_setup_dma(priv);

  up_clean_dcache((uintptr_t)priv->fb,
                  (uintptr_t)priv->fb + priv->fb_alloc);

  /* Stop CAM first */

  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_START_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Reset CAM module + AFIFO */

  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval |= LCD_CAM_CAM_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  regval |= LCD_CAM_CAM_AFIFO_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_AFIFO_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Reset DMA channel */

  esp32s3_dma_reset_channel(priv->dma_channel, false);

  /* Load DMA descriptors */

  esp32s3_dma_load(priv->dmadesc, priv->dma_channel, false);
  esp32s3_dma_enable(priv->dma_channel, false);

  /* cam_update BEFORE cam_start */

  regval = getreg32(LCD_CAM_CAM_CTRL_REG);
  regval |= LCD_CAM_CAM_UPDATE_REG_M;
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);

  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval |= LCD_CAM_CAM_START_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  priv->capturing = true;

  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAM_VSYNC_PIN,
                         CAM_V_SYNC_IDX, false);
  up_udelay(10);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAM_VSYNC_PIN,
                         CAM_V_SYNC_IDX, true);

  return OK;
}

/****************************************************************************
 * Name: esp32s3_cam_stop_capture
 ****************************************************************************/

static int esp32s3_cam_stop_capture(struct imgdata_s *data)
{
  struct esp32s3_cam_s *priv = (struct esp32s3_cam_s *)data;
  uint32_t regval;

  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_START_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  regval = getreg32(LCD_CAM_CAM_CTRL_REG);
  regval |= LCD_CAM_CAM_UPDATE_REG_M;
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);

  priv->capturing = false;

  work_cancel_sync(LPWORK, &priv->frame_work);

  priv->cb = NULL;
  priv->cb_arg = NULL;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_cam_initialize
 *
 * Description:
 *   Initialize the ESP32-S3 CAM imgdata driver.
 *   Starts XCLK output for sensor communication.
 *
 * Returned Value:
 *   Pointer to imgdata_s on success; NULL on failure.
 *
 ****************************************************************************/

struct imgdata_s *esp32s3_cam_initialize(void)
{
  esp32s3_cam_enableclk();
  esp32s3_cam_gpio_config();

  return &g_cam_priv.data;
}
