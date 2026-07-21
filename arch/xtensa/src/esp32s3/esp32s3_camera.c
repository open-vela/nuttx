/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_camera.c
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

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/param.h>

#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/wqueue.h>

#include "esp32s3_camera.h"
#include "esp32s3_dma.h"
#include "esp32s3_gpio.h"
#include "esp32s3_irq.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/esp32s3_lcd_cam.h"
#include "hardware/esp32s3_soc.h"
#include "periph_ctrl.h"
#include "xtensa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32S3_CAMERA_SOURCE_CLOCK       160000000

/* Keep DMA chunks aligned to RGB565 scanlines for the standard 160, 320,
 * and 640 pixel widths.  A partial scanline can remain in the LCD_CAM FIFO
 * when the frame ends.
 */

#define ESP32S3_CAMERA_DMA_CHUNK_SIZE     3840
#define ESP32S3_CAMERA_DMA_DESC_NUM       \
  ((CONFIG_ESP32S3_CAMERA_MAX_FRAME_SIZE + \
    ESP32S3_CAMERA_DMA_CHUNK_SIZE - 1) / \
   ESP32S3_CAMERA_DMA_CHUNK_SIZE)

#ifdef CONFIG_ESP32S3_CAMERA_PCLK_INVERT
#  define ESP32S3_CAMERA_PCLK_INVERT       true
#else
#  define ESP32S3_CAMERA_PCLK_INVERT       false
#endif

#ifdef CONFIG_ESP32S3_CAMERA_VSYNC_INVERT
#  define ESP32S3_CAMERA_VSYNC_INVERT      true
#else
#  define ESP32S3_CAMERA_VSYNC_INVERT      false
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s3_camera_s
{
  struct imgdata_s data;
  struct esp32s3_dmadesc_s desc[ESP32S3_CAMERA_DMA_DESC_NUM];
  struct work_s work;
  spinlock_t lock;
  imgdata_capture_t callback;
  FAR void *callback_arg;
  FAR uint8_t *buffer;
  uint32_t frame_size;
  int32_t dma_channel;
  int cpuint;
  uint8_t cpu;
  bool initialized;
  bool streaming;
  bool capturing;
  bool synced;
};

/****************************************************************************
 * External Function Prototypes
 ****************************************************************************/

extern int cache_invalidate_addr(uint32_t addr, uint32_t size);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32s3_camera_init(FAR struct imgdata_s *data);
static int esp32s3_camera_uninit(FAR struct imgdata_s *data);
static int esp32s3_camera_set_buf(FAR struct imgdata_s *data,
                                 uint8_t nr_datafmts,
                                 FAR imgdata_format_t *datafmts,
                                 FAR uint8_t *addr, uint32_t size);
static int esp32s3_camera_validate(FAR struct imgdata_s *data,
                                  uint8_t nr_datafmts,
                                  FAR imgdata_format_t *datafmts,
                                  FAR imgdata_interval_t *interval);
static int esp32s3_camera_start(FAR struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               FAR imgdata_format_t *datafmts,
                               FAR imgdata_interval_t *interval,
                               imgdata_capture_t callback,
                               FAR void *arg);
static int esp32s3_camera_stop(FAR struct imgdata_s *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imgdata_ops_s g_esp32s3_camera_ops =
{
  .init                   = esp32s3_camera_init,
  .uninit                 = esp32s3_camera_uninit,
  .set_buf                = esp32s3_camera_set_buf,
  .validate_frame_setting = esp32s3_camera_validate,
  .start_capture          = esp32s3_camera_start,
  .stop_capture           = esp32s3_camera_stop,
};

static struct esp32s3_camera_s g_esp32s3_camera =
{
  .data =
    {
      .ops = &g_esp32s3_camera_ops,
    },
  .lock        = SP_UNLOCKED,
  .dma_channel = -1,
  .cpuint      = -ENOMEM,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_camera_received_size
 *
 * Description:
 *   Sum the actual received data length from the GDMA descriptor chain.
 *
 * Input Parameters:
 *   priv - Pointer to the camera driver state
 *
 * Returned Value:
 *   Total received bytes, capped at priv->frame_size.
 *
 ****************************************************************************/

static uint32_t esp32s3_camera_received_size(
  FAR struct esp32s3_camera_s *priv)
{
  uint32_t size = 0;
  int i;

  for (i = 0; i < ESP32S3_CAMERA_DMA_DESC_NUM; i++)
    {
      size += (priv->desc[i].ctrl >> ESP32S3_DMA_CTRL_DATALEN_S) &
              ESP32S3_DMA_CTRL_DATALEN_V;

      if (priv->desc[i].next == NULL)
        {
          break;
        }
    }

  return MIN(size, priv->frame_size);
}

/****************************************************************************
 * Name: esp32s3_camera_arm
 *
 * Description:
 *   Prepare the GDMA descriptor chain for the next frame and start the
 *   LCD_CAM receiver and DMA.
 *
 * Input Parameters:
 *   priv - Pointer to the camera driver state
 *
 ****************************************************************************/

static void esp32s3_camera_arm(FAR struct esp32s3_camera_s *priv)
{
  uint32_t remaining;
  uint32_t regval;
  uint32_t size;
  int i;

  remaining = priv->frame_size;
  for (i = 0; i < ESP32S3_CAMERA_DMA_DESC_NUM; i++)
    {
      size = MIN(remaining, ESP32S3_CAMERA_DMA_CHUNK_SIZE);
      priv->desc[i].ctrl = ESP32S3_DMA_CTRL_OWN |
                           (size << ESP32S3_DMA_CTRL_BUFLEN_S);
      priv->desc[i].pbuf = priv->buffer + i *
                           ESP32S3_CAMERA_DMA_CHUNK_SIZE;
      priv->desc[i].next = remaining > size ? &priv->desc[i + 1] : NULL;
      remaining -= size;
      if (remaining == 0)
        {
          break;
        }
    }

  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval |= LCD_CAM_CAM_RESET_M | LCD_CAM_CAM_AFIFO_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);
  regval &= ~(LCD_CAM_CAM_RESET_M | LCD_CAM_CAM_AFIFO_RESET_M);
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  esp32s3_dma_load(priv->desc, priv->dma_channel, false);
  esp32s3_dma_enable(priv->dma_channel, false);

  modifyreg32(LCD_CAM_CAM_CTRL_REG, 0, LCD_CAM_CAM_UPDATE_REG_M);
  modifyreg32(LCD_CAM_CAM_CTRL1_REG, 0, LCD_CAM_CAM_START_M);
}

/****************************************************************************
 * Name: esp32s3_camera_worker
 *
 * Description:
 *   High-priority work queue callback invoked after a frame has been
 *   captured.  Invalidates PSRAM cache (when the buffer is in external
 *   RAM), calls the upper-half capture callback, and re-arms the hardware
 *   if streaming continues.
 *
 * Input Parameters:
 *   arg - Pointer to the camera driver state (esp32s3_camera_s)
 *
 ****************************************************************************/

static void esp32s3_camera_worker(FAR void *arg)
{
  FAR struct esp32s3_camera_s *priv = arg;
  imgdata_capture_t callback;
  FAR void *callback_arg;
  struct timespec ts;
  struct timeval tv;
  irqstate_t flags;
  uint32_t size;

  flags = spin_lock_irqsave(&priv->lock);
  callback = priv->callback;
  callback_arg = priv->callback_arg;
  size = esp32s3_camera_received_size(priv);
  spin_unlock_irqrestore(&priv->lock, flags);

  if (esp32s3_ptr_extram(priv->buffer))
    {
      cache_invalidate_addr((uint32_t)priv->buffer, priv->frame_size);
    }

  clock_systime_timespec(&ts);
  TIMESPEC_TO_TIMEVAL(&tv, &ts);

  if (callback != NULL)
    {
      callback(size == priv->frame_size ? 0 : EIO, size, &tv,
               callback_arg);
    }

  flags = spin_lock_irqsave(&priv->lock);
  if (priv->streaming && priv->buffer != NULL)
    {
      priv->capturing = true;
      priv->synced = false;
      esp32s3_camera_arm(priv);
      putreg32(LCD_CAM_CAM_VSYNC_INT_CLR_M,
               LCD_CAM_LC_DMA_INT_CLR_REG);
      modifyreg32(LCD_CAM_LC_DMA_INT_ENA_REG, 0,
                  LCD_CAM_CAM_VSYNC_INT_ENA_M);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: esp32s3_camera_interrupt
 *
 * Description:
 *   LCD_CAM interrupt handler.  The first two VSYNC edges are used for
 *   frame-boundary alignment; subsequent VSYNC edges trigger a work queue
 *   callback that delivers the completed frame and re-arms the hardware.
 *
 * Input Parameters:
 *   irq     - Interrupt number
 *   context - Saved register context
 *   arg     - Pointer to the camera driver state
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32s3_camera_interrupt(int irq, FAR void *context,
                                              FAR void *arg)
{
  FAR struct esp32s3_camera_s *priv = arg;
  uint32_t status;

  status = getreg32(LCD_CAM_LC_DMA_INT_ST_REG);
  putreg32(status, LCD_CAM_LC_DMA_INT_CLR_REG);

  if ((status & LCD_CAM_CAM_VSYNC_INT_ST_M) == 0 || !priv->streaming)
    {
      return OK;
    }

  if (!priv->capturing)
    {
      priv->capturing = true;
      priv->synced = false;
      esp32s3_camera_arm(priv);
    }
  else if (!priv->synced)
    {
      esp32s3_camera_arm(priv);
      priv->synced = true;
    }
  else
    {
      modifyreg32(LCD_CAM_LC_DMA_INT_ENA_REG,
                  LCD_CAM_CAM_VSYNC_INT_ENA_M, 0);
      modifyreg32(LCD_CAM_CAM_CTRL1_REG, LCD_CAM_CAM_START_M, 0);
      esp32s3_dma_disable(priv->dma_channel, false);
      priv->capturing = false;
      priv->synced = false;
      work_queue(HPWORK, &priv->work, esp32s3_camera_worker, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32s3_camera_gpio_config
 *
 * Description:
 *   Configure all DVP camera interface GPIOs (PCLK, VSYNC, HREF, D0-D7
 *   and XCLK) through the GPIO matrix.
 *
 ****************************************************************************/

static void esp32s3_camera_gpio_config(void)
{
  static const uint8_t data_pins[8] =
  {
    CONFIG_ESP32S3_CAMERA_DATA0_PIN,
    CONFIG_ESP32S3_CAMERA_DATA1_PIN,
    CONFIG_ESP32S3_CAMERA_DATA2_PIN,
    CONFIG_ESP32S3_CAMERA_DATA3_PIN,
    CONFIG_ESP32S3_CAMERA_DATA4_PIN,
    CONFIG_ESP32S3_CAMERA_DATA5_PIN,
    CONFIG_ESP32S3_CAMERA_DATA6_PIN,
    CONFIG_ESP32S3_CAMERA_DATA7_PIN,
  };

  int i;

  esp32s3_configgpio(CONFIG_ESP32S3_CAMERA_PCLK_PIN, INPUT);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAMERA_PCLK_PIN, CAM_PCLK_IDX,
                        ESP32S3_CAMERA_PCLK_INVERT);

  esp32s3_configgpio(CONFIG_ESP32S3_CAMERA_VSYNC_PIN, INPUT);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAMERA_VSYNC_PIN, CAM_V_SYNC_IDX,
                        ESP32S3_CAMERA_VSYNC_INVERT);

  esp32s3_configgpio(CONFIG_ESP32S3_CAMERA_HREF_PIN, INPUT);
  esp32s3_gpio_matrix_in(CONFIG_ESP32S3_CAMERA_HREF_PIN, CAM_H_ENABLE_IDX,
                        false);

  for (i = 0; i < 8; i++)
    {
      esp32s3_configgpio(data_pins[i], INPUT);
      esp32s3_gpio_matrix_in(data_pins[i], CAM_DATA_IN0_IDX + i, false);
    }

  esp32s3_configgpio(CONFIG_ESP32S3_CAMERA_XCLK_PIN, OUTPUT);
  esp32s3_gpio_matrix_out(CONFIG_ESP32S3_CAMERA_XCLK_PIN, CAM_CLK_IDX,
                         false, false);
}

/****************************************************************************
 * Name: esp32s3_camera_hw_initialize
 *
 * Description:
 *   Enable LCD_CAM peripheral, configure the GPIO matrix, set up the
 *   pixel clock divider, request a GDMA channel, and attach the VSYNC
 *   interrupt.
 *
 * Input Parameters:
 *   priv - Pointer to the camera driver state
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp32s3_camera_hw_initialize(FAR struct esp32s3_camera_s *priv)
{
  irqstate_t flags;
  uint32_t divisor;
  uint32_t regval;
  int ret;

  divisor = ESP32S3_CAMERA_SOURCE_CLOCK /
            CONFIG_ESP32S3_CAMERA_XCLK_FREQUENCY;
  if (divisor == 0 ||
      ESP32S3_CAMERA_SOURCE_CLOCK %
      CONFIG_ESP32S3_CAMERA_XCLK_FREQUENCY != 0)
    {
      return -EINVAL;
    }

  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  esp32s3_camera_gpio_config();

  regval = (3 << LCD_CAM_CAM_CLK_SEL_S) |
           (divisor << LCD_CAM_CAM_CLKM_DIV_NUM_S) |
           (4 << LCD_CAM_CAM_VSYNC_FILTER_THRES_S);
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);

  regval = ((ESP32S3_CAMERA_DMA_CHUNK_SIZE - 1) <<
            LCD_CAM_CAM_REC_DATA_BYTELEN_S) |
           LCD_CAM_CAM_VSYNC_FILTER_EN_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);
  putreg32(0, LCD_CAM_CAM_RGB_YUV_REG);
  modifyreg32(LCD_CAM_CAM_CTRL_REG, 0, LCD_CAM_CAM_UPDATE_REG_M);
  modifyreg32(LCD_CAM_CAM_CTRL1_REG, 0, LCD_CAM_CAM_START_M);

  priv->dma_channel = esp32s3_dma_request(ESP32S3_DMA_PERIPH_LCDCAM,
                                          1, 1, false);
  if (priv->dma_channel < 0)
    {
      verr("ERROR: Failed to allocate camera GDMA channel\n");
      return -EBUSY;
    }

  esp32s3_dma_set_ext_memblk(priv->dma_channel, false,
                             ESP32S3_DMA_EXT_MEMBLK_16B);

  flags = spin_lock_irqsave(&priv->lock);
  priv->cpu = this_cpu();
  priv->cpuint = esp32s3_setup_irq(priv->cpu, ESP32S3_PERIPH_LCD_CAM,
                                   ESP32S3_INT_PRIO_DEF,
                                   ESP32S3_CPUINT_LEVEL);
  spin_unlock_irqrestore(&priv->lock, flags);
  if (priv->cpuint < 0)
    {
      ret = priv->cpuint;
      goto errout_dma;
    }

  ret = irq_attach(ESP32S3_IRQ_LCD_CAM, esp32s3_camera_interrupt, priv);
  if (ret < 0)
    {
      goto errout_irq;
    }

  putreg32(LCD_CAM_CAM_VSYNC_INT_CLR_M, LCD_CAM_LC_DMA_INT_CLR_REG);
  up_enable_irq(ESP32S3_IRQ_LCD_CAM);
  return OK;

errout_irq:
  esp32s3_teardown_irq(priv->cpu, ESP32S3_PERIPH_LCD_CAM, priv->cpuint);
  priv->cpuint = -ENOMEM;
errout_dma:
  esp32s3_dma_release(priv->dma_channel);
  priv->dma_channel = -1;
  return ret;
}

/****************************************************************************
 * Name: esp32s3_camera_init
 *
 * Description:
 *   Initialize the image-data lower half.  Hardware setup is deferred to
 *   esp32s3_camera_hw_initialize called from esp32s3_camera_initialize().
 *
 * Input Parameters:
 *   data - Pointer to the imgdata instance
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

static int esp32s3_camera_init(FAR struct imgdata_s *data)
{
  return OK;
}

/****************************************************************************
 * Name: esp32s3_camera_uninit
 *
 * Description:
 *   De-initialize the image-data lower half.  Currently only stops any
 *   ongoing capture.
 *
 * Input Parameters:
 *   data - Pointer to the imgdata instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp32s3_camera_uninit(FAR struct imgdata_s *data)
{
  return esp32s3_camera_stop(data);
}

/****************************************************************************
 * Name: esp32s3_camera_set_buf
 *
 * Description:
 *   Provide the frame buffer address and validate that it meets the
 *   16-byte alignment requirement of the GDMA engine.
 *
 * Input Parameters:
 *   data       - Pointer to the imgdata instance
 *   nr_datafmts - Number of provided format descriptors
 *   datafmts   - Format descriptors describing the frame geometry
 *   addr       - 16-byte aligned frame buffer address
 *   size       - Buffer size in bytes
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp32s3_camera_set_buf(FAR struct imgdata_s *data,
                                  uint8_t nr_datafmts,
                                  FAR imgdata_format_t *datafmts,
                                  FAR uint8_t *addr, uint32_t size)
{
  FAR struct esp32s3_camera_s *priv =
    (FAR struct esp32s3_camera_s *)data;
  irqstate_t flags;
  uint32_t frame_size;
  int ret;

  ret = esp32s3_camera_validate(data, nr_datafmts, datafmts, NULL);
  if (ret < 0)
    {
      return ret;
    }

  frame_size = datafmts[IMGDATA_FMT_MAIN].width *
               datafmts[IMGDATA_FMT_MAIN].height * 2;
  if (addr == NULL || ((uintptr_t)addr & 0xf) != 0 || size < frame_size)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&priv->lock);
  priv->buffer = addr;
  priv->frame_size = frame_size;
  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: esp32s3_camera_validate
 *
 * Description:
 *   Validate the requested capture format.  Only RGB565 with a non-zero
 *   resolution is supported.
 *
 * Input Parameters:
 *   data       - Pointer to the imgdata instance
 *   nr_datafmts - Number of provided format descriptors
 *   datafmts   - Format descriptors to validate
 *   interval   - Frame interval (unused, may be NULL)
 *
 * Returned Value:
 *   Zero (OK) if the format is supported; -ENOTSUP or -E2BIG otherwise.
 *
 ****************************************************************************/

static int esp32s3_camera_validate(FAR struct imgdata_s *data,
                                   uint8_t nr_datafmts,
                                   FAR imgdata_format_t *datafmts,
                                   FAR imgdata_interval_t *interval)
{
  uint32_t frame_size;

  if (nr_datafmts != 1 || datafmts == NULL ||
      datafmts[IMGDATA_FMT_MAIN].pixelformat != IMGDATA_PIX_FMT_RGB565 ||
      datafmts[IMGDATA_FMT_MAIN].width == 0 ||
      datafmts[IMGDATA_FMT_MAIN].height == 0)
    {
      return -ENOTSUP;
    }

  frame_size = datafmts[IMGDATA_FMT_MAIN].width *
               datafmts[IMGDATA_FMT_MAIN].height * 2;
  return frame_size <= CONFIG_ESP32S3_CAMERA_MAX_FRAME_SIZE ? OK :
                                                                  -E2BIG;
}

/****************************************************************************
 * Name: esp32s3_camera_start
 *
 * Description:
 *   Begin capture.  Validates the requested format, arms the GDMA
 *   descriptor chain, enables the VSYNC interrupt, and starts the
 *   LCD_CAM receiver.
 *
 * Input Parameters:
 *   data       - Pointer to the imgdata instance
 *   nr_datafmts - Number of provided format descriptors
 *   datafmts   - Format descriptors for the capture
 *   interval   - Frame interval (unused, may be NULL)
 *   callback   - Frame-complete callback
 *   arg        - Opaque argument passed to callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp32s3_camera_start(FAR struct imgdata_s *data,
                                uint8_t nr_datafmts,
                                FAR imgdata_format_t *datafmts,
                                FAR imgdata_interval_t *interval,
                                imgdata_capture_t callback,
                                FAR void *arg)
{
  FAR struct esp32s3_camera_s *priv =
    (FAR struct esp32s3_camera_s *)data;
  irqstate_t flags;
  int ret;

  ret = esp32s3_camera_validate(data, nr_datafmts, datafmts, interval);
  if (ret < 0 || callback == NULL)
    {
      return ret < 0 ? ret : -EINVAL;
    }

  flags = spin_lock_irqsave(&priv->lock);
  if (priv->buffer == NULL)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return -EINVAL;
    }

  priv->callback = callback;
  priv->callback_arg = arg;
  priv->capturing = true;
  priv->synced = false;
  priv->streaming = true;
  esp32s3_camera_arm(priv);
  putreg32(LCD_CAM_CAM_VSYNC_INT_CLR_M, LCD_CAM_LC_DMA_INT_CLR_REG);
  modifyreg32(LCD_CAM_LC_DMA_INT_ENA_REG, 0,
              LCD_CAM_CAM_VSYNC_INT_ENA_M);
  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: esp32s3_camera_stop
 *
 * Description:
 *   Stop capture.  Disables the VSYNC interrupt, halts the LCD_CAM
 *   receiver and GDMA channel, and clears streaming state.
 *
 * Input Parameters:
 *   data - Pointer to the imgdata instance
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

static int esp32s3_camera_stop(FAR struct imgdata_s *data)
{
  FAR struct esp32s3_camera_s *priv =
    (FAR struct esp32s3_camera_s *)data;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  modifyreg32(LCD_CAM_LC_DMA_INT_ENA_REG,
              LCD_CAM_CAM_VSYNC_INT_ENA_M, 0);
  modifyreg32(LCD_CAM_CAM_CTRL1_REG, LCD_CAM_CAM_START_M, 0);
  if (priv->capturing)
    {
      esp32s3_dma_disable(priv->dma_channel, false);
    }

  priv->callback = NULL;
  priv->callback_arg = NULL;
  priv->capturing = false;
  priv->synced = false;
  priv->streaming = false;
  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_camera_initialize
 *
 * Description:
 *   Initialize the LCD_CAM receiver and register its image-data lower half.
 *   Board logic must register an image sensor before creating a V4L2
 *   capture device.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp32s3_camera_initialize(void)
{
  FAR struct esp32s3_camera_s *priv = &g_esp32s3_camera;
  int ret;

  if (priv->initialized)
    {
      return OK;
    }

  ret = esp32s3_camera_hw_initialize(priv);
  if (ret < 0)
    {
      return ret;
    }

  imgdata_register(&priv->data);
  priv->initialized = true;
  return OK;
}
