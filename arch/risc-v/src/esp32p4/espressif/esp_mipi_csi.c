/****************************************************************************
 * arch/risc-v/src/esp32p4/espressif/esp_mipi_csi.c
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
 * MIPI CSI-2 camera capture lower half (NuttX imgdata_s).
 *
 * Pipeline: CSI D-PHY -> DWC CSI-2 host -> ISP (inline RAW8 Bayer to
 * RGB565 conversion) -> CSI bridge -> DW-GDMA (AXI) -> frame buffer.
 *
 * The hardware bring-up sequence is ported from the Espressif IDF driver
 * components/upper_hal_cam/csi/src/esp_cam_ctlr_csi.c (which cannot be
 * compiled here because it depends on FreeRTOS queues), and the ISP
 * programming from components/upper_hal_isp/src/isp_core.c.  The NuttX
 * glue idiom (LDO, clock tree, DW-GDMA driver usage) follows
 * esp_mipi_dsi.c in this directory.
 *
 * Buffer protocol: the v4l2 capture framework hands the driver exactly
 * one buffer at a time through set_buf(); each completed frame is
 * returned through the capture callback, from which the framework calls
 * set_buf() again with the next buffer.  Because the CSI source cannot
 * be paused, the DW-GDMA channel is always kept armed: when no v4l2
 * buffer is available at frame-done time the DMA is re-armed onto an
 * internal backup buffer and that frame is dropped.  set_buf() tries to
 * retarget an armed-but-idle backup transfer onto the new v4l2 buffer
 * ("hot swap") so that the full frame rate is reached whenever the
 * frame worker beats the sensor vertical blanking.
 *
 * Context discipline: the DW-GDMA trans-done callback runs in interrupt
 * context and only re-arms the DMA; the imgdata capture callback is
 * invoked from the low-priority work queue (same discipline as
 * esp32s3_cam.c) because the v4l2 framework may call the image sensor
 * stop entry (I2C traffic) from inside the callback.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/param.h>
#include <sys/time.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>
#include <nuttx/video/imgdata.h>

#include "esp_mipi_csi.h"

#include "esp_ldo_regulator.h"
#include "esp_clk_tree.h"
#include "esp_cache.h"
#include "esp_private/esp_clk_tree_common.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/dw_gdma.h"

#include "hal/mipi_csi_hal.h"
#include "hal/mipi_csi_ll.h"
#include "hal/isp_ll.h"
#include "hal/dw_gdma_ll.h"
#include "hal/color_types.h"

#include "soc/reg_base.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only one CSI host / bridge / ISP instance exists on the ESP32-P4 */

#define CSI_CTLR_ID                 0

/* ISP functional clock, same value as the IDF camera examples */

#define CSI_ISP_CLK_HZ              (80 * 1000 * 1000)

/* CSI bridge tuning values ported from esp_cam_ctlr_csi.c /
 * mipi_csi_hal.c: AXI burst length of the bridge master and the
 * almost-full flow control threshold of its 1024-entry FIFO.
 */

#define CSI_BRG_BURST_LEN           512

/* CSI-2 short/long packet data type window accepted by the bridge
 * (0x12..0x2f covers the YUV/RGB/RAW primary format range).
 */

#define CSI_BRG_DATA_TYPE_MIN       0x12
#define CSI_BRG_DATA_TYPE_MAX       0x2f

/* Bits per pixel entering the CSI bridge from the sensor (RAW8) and
 * leaving the ISP (RGB565).  In RAW passthrough mode input == output.
 */

#define CSI_IN_BPP                  8
#ifdef CONFIG_ESPRESSIF_MIPI_CSI_RAW_PASSTHROUGH
#  define CSI_OUT_BPP               8
#else
#  define CSI_OUT_BPP               16
#endif

/* DMA buffers must be aligned to the largest data cache line so that
 * esp_cache_msync() in M2C (invalidate) direction accepts them.
 */

#ifdef CONFIG_ESPRESSIF_CACHE_L2_CACHE_LINE_SIZE
#  define CSI_CACHE_LINE            CONFIG_ESPRESSIF_CACHE_L2_CACHE_LINE_SIZE
#else
#  define CSI_CACHE_LINE            64
#endif

#define CSI_IS_CACHE_ALIGNED(v)     ((((uintptr_t)(v)) & \
                                      (CSI_CACHE_LINE - 1)) == 0)

/* Color correction parameters (1/1000 units).
 *
 * White balance: calibrated 2026-08-25 against an sRGB color card
 * sampled straight out of the frame buffer over JTAG; the raw sensor
 * white point measured r/g=0.658, b/g=0.619, giving gains 1/ratio.
 *
 * Crosstalk matrix: the official Espressif SC2336 factory tuning
 * (esp-video-components esp_cam_sensor/sensors/sc2336/cfg/
 * sc2336_default_p4_eco5.json), 4862 K entry, rows summing to 1.0.
 * Their pipeline applies WB gains before the CCM, so the single
 * matrix programmed here is CCM x diag(wb) (column scaling).
 */

#define CSI_WB_GAIN_R_X1000         1521
#define CSI_WB_GAIN_G_X1000         1000
#define CSI_WB_GAIN_B_X1000         1614

/* 3A (auto exposure + auto white balance) parameters, from the same
 * official tuning file.  AE: luma target and dead band (eco5 agc),
 * exposure in sensor lines with VTS = 2000 (15 fps), anti-flicker
 * quantum = lines per 10 ms half-period of 50 Hz mains.  AWB: damping
 * 70/30, gain clamps, minimum white patch population.
 */

#define CSI_AE_TARGET               57
#define CSI_AE_LOW                  54
#define CSI_AE_HIGH                 64
#define CSI_AE_EXP_MIN              16
#define CSI_AE_EXP_MAX              1994
#define CSI_AE_FLICKER_LINES        300
#define CSI_AWB_MIN_PATCHES         100
#define CSI_WB_GAIN_MIN             1000
#define CSI_WB_GAIN_MAX             3500

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Supported sensor modes (frame sizes at 30 fps) */

struct esp_csi_mode_s
{
  uint16_t width;
  uint16_t height;
};

/* CSI capture device state.  struct imgdata_s must be the first member
 * so that the imgdata_s pointer handed to the framework can be cast
 * back to the device structure.
 */

struct esp_csi_dev_s
{
  struct imgdata_s data;              /* Must be first */

  struct wdog_s poll_wdog;            /* Poll-mode block-done detector */
  struct work_s poll_work;            /* Thread-context poll body */

  struct esp_mipi_csi_config_s config;

  mipi_csi_hal_context_t hal;         /* CSI host + bridge handles */
  esp_ldo_channel_handle_t ldo;       /* MIPI DPHY power rail */
  dw_gdma_channel_handle_t dma_chan;  /* CSI RX DMA channel */
  dw_gdma_dev_t *dma_hw;              /* DW-GDMA register file */
  int dma_chan_id;                    /* Channel index for LL access */

  bool initialized;                   /* Link brought up */
  bool capturing;                     /* Between start and stop */
  bool warned_unaligned;              /* Rate-limit alignment warnings */
  bool logged_first;                  /* First frame-done announced */

  uint16_t width;                     /* Active frame width, pixels */
  uint16_t height;                    /* Active frame height, lines */
  uint32_t out_size;                  /* Delivered frame size, bytes */
  uint32_t dma_beats;                 /* DMA block size, 64-bit beats */

  FAR uint8_t *backup;                /* Internal frame-drop buffer */
  uint32_t backup_size;               /* Backup buffer size, bytes */

  FAR uint8_t *armed;                 /* Buffer the DMA writes into */
  FAR uint8_t *pending;               /* Next v4l2 buffer, if any */
  FAR uint8_t *done;                  /* Completed frame to deliver */

  imgdata_capture_t cb;               /* Frame completion callback */
  FAR void *cb_arg;                   /* Callback argument */
  struct work_s work;                 /* Frame delivery worker */

  uint32_t frames;                    /* Frames delivered */
  uint32_t drops;                     /* Frames dropped into backup */
  uint32_t swaps;                     /* Successful set_buf hot swaps */
  uint32_t swap_races;                /* Hot swaps lost to frame start */
  uint32_t overruns;                  /* Completions with stale done */
  uint32_t sync_age;                  /* Frames since last bridge resync */

  spinlock_t lock;                    /* Protects the fields above */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp_csi_arm_dma(FAR struct esp_csi_dev_s *dev,
                            FAR uint8_t *buf);
static void esp_csi_dma_wait_idle(FAR struct esp_csi_dev_s *dev);
static void esp_csi_wait_frame_boundary(FAR struct esp_csi_dev_s *dev);
static void esp_csi_bridge_resync(FAR struct esp_csi_dev_s *dev,
                                  bool reopen);
#ifndef CONFIG_ESPRESSIF_MIPI_CSI_RAW_PASSTHROUGH
static void esp_csi_3a_tick(void);
#endif
static bool esp_csi_dma_done(dw_gdma_channel_handle_t chan,
                             const dw_gdma_trans_done_event_data_t *edata,
                             void *user_data);
static void esp_csi_frame_worker(FAR void *arg);
#ifndef CONFIG_ESPRESSIF_MIPI_CSI_RAW_PASSTHROUGH
static void esp_csi_isp_configure(FAR struct esp_csi_dev_s *dev,
                                  uint16_t width, uint16_t height);
#endif

/* imgdata operations */

static int esp_csi_init(FAR struct imgdata_s *data);
static int esp_csi_uninit(FAR struct imgdata_s *data);
static int esp_csi_set_buf(FAR struct imgdata_s *data,
                           uint8_t nr_datafmts,
                           FAR imgdata_format_t *datafmts,
                           FAR uint8_t *addr, uint32_t size);
static int esp_csi_validate_frame_setting(FAR struct imgdata_s *data,
                           uint8_t nr_datafmts,
                           FAR imgdata_format_t *datafmts,
                           FAR imgdata_interval_t *interval);
static int esp_csi_start_capture(FAR struct imgdata_s *data,
                           uint8_t nr_datafmts,
                           FAR imgdata_format_t *datafmts,
                           FAR imgdata_interval_t *interval,
                           imgdata_capture_t callback,
                           FAR void *arg);
static int esp_csi_stop_capture(FAR struct imgdata_s *data);
static FAR void *esp_csi_alloc(FAR struct imgdata_s *data,
                               uint32_t align_size, uint32_t size);
static void esp_csi_free(FAR struct imgdata_s *data, FAR void *addr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct esp_csi_mode_s g_csi_modes[] =
{
  {
    1024, 600      /* SC2336 1024x600 RAW8 @30fps, 2 lanes, 288Mbps */
  },
  {
    1280, 720      /* SC2336 1280x720 RAW8 @30fps, 2 lanes, 336Mbps */
  },
};

static const struct imgdata_ops_s g_csi_ops =
{
  .init                   = esp_csi_init,
  .uninit                 = esp_csi_uninit,
  .set_buf                = esp_csi_set_buf,
  .validate_frame_setting = esp_csi_validate_frame_setting,
  .start_capture          = esp_csi_start_capture,
  .stop_capture           = esp_csi_stop_capture,
  .alloc                  = esp_csi_alloc,
  .free                   = esp_csi_free,
};

static struct esp_csi_dev_s g_csi_dev =
{
  .data =
    {
      .ops = &g_csi_ops,
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_csi_arm_dma
 *
 * Description:
 *   Program one whole-frame block transfer (CSI bridge FIFO -> buf) and
 *   enable the DW-GDMA channel.  The channel must be disabled when this
 *   is called (a completed CONTIGUOUS block leaves it disabled).
 *
 *   The DMA engine is the flow controller, so the programmed size is
 *   exact: the channel moves out_size bytes (one full output frame)
 *   and stops, whatever the bridge does.  See the channel allocation
 *   in esp_csi_init for why this differs from the IDF driver.
 *
 *   May be called from interrupt context.
 *
 ****************************************************************************/

static void esp_csi_arm_dma(FAR struct esp_csi_dev_s *dev,
                            FAR uint8_t *buf)
{
  dw_gdma_block_transfer_config_t cfg;

  memset(&cfg, 0, sizeof(cfg));
  cfg.src.addr        = MIPI_CSI_BRG_MEM_BASE;
  cfg.src.burst_mode  = DW_GDMA_BURST_MODE_FIXED;
  cfg.src.burst_items = DW_GDMA_BURST_ITEMS_512;
  cfg.src.burst_len   = 16;
  cfg.src.width       = DW_GDMA_TRANS_WIDTH_64;
  cfg.dst.addr        = (uint32_t)(uintptr_t)buf;
  cfg.dst.burst_mode  = DW_GDMA_BURST_MODE_INCREMENT;
  cfg.dst.burst_items = DW_GDMA_BURST_ITEMS_512;
  cfg.dst.burst_len   = 16;
  cfg.dst.width       = DW_GDMA_TRANS_WIDTH_64;
  cfg.size            = dev->dma_beats;

  dw_gdma_channel_config_transfer(dev->dma_chan, &cfg);
  dw_gdma_channel_enable_ctrl(dev->dma_chan, true);
}

/****************************************************************************
 * Name: esp_csi_dma_wait_idle
 *
 * Description:
 *   Bounded busy-wait for the DMA channel enable bit to clear after a
 *   disable request (the hardware clears it once outstanding AXI beats
 *   have drained).  Called with the driver spinlock held; the wait is
 *   sub-microsecond in practice.
 *
 ****************************************************************************/

static void esp_csi_dma_wait_idle(FAR struct esp_csi_dev_s *dev)
{
  uint32_t mask = 1ul << dev->dma_chan_id;
  int spins = 1000;

  while ((dev->dma_hw->chen0.val & mask) != 0 && --spins > 0)
    {
    }

  if (spins <= 0)
    {
      verr("csi: DMA channel %d did not drain\n", dev->dma_chan_id);
    }
}

/****************************************************************************
 * Name: esp_csi_dma_done
 *
 * Description:
 *   DW-GDMA full-trans-done callback, invoked in interrupt context on
 *   every completed frame.  Re-arms the DMA immediately (onto the
 *   pending v4l2 buffer if one was handed over via set_buf, else onto
 *   the internal backup buffer) so that the CSI bridge always has a
 *   destination, then defers frame delivery to the worker.
 *
 ****************************************************************************/

static void esp_csi_poll(wdparm_t arg);

static bool esp_csi_dma_done(dw_gdma_channel_handle_t chan,
                             const dw_gdma_trans_done_event_data_t *edata,
                             void *user_data)
{
  FAR struct esp_csi_dev_s *dev = (FAR struct esp_csi_dev_s *)user_data;
  FAR uint8_t *completed;
  FAR uint8_t *next;
  bool log_first = false;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);

  if (!dev->capturing)
    {
      spin_unlock_irqrestore(&dev->lock, flags);
      return false;
    }

  completed = dev->armed;

  /* Keep the hardware armed: next v4l2 buffer if available, else the
   * backup buffer (that frame will be dropped).
   */

  if (dev->pending != NULL)
    {
      next = dev->pending;
      dev->pending = NULL;
    }
  else
    {
      next = dev->backup;
    }

  esp_csi_arm_dma(dev, next);
  dev->armed = next;

  if (completed != dev->backup && completed != NULL)
    {
      if (dev->done != NULL)
        {
          /* Should not happen: a v4l2 buffer completed while the
           * previous one has not been delivered yet.
           */

          dev->overruns++;
        }

      dev->done = completed;
      work_queue(LPWORK, &dev->work, esp_csi_frame_worker, dev, 0);
    }
  else
    {
      dev->drops++;
    }

  if (!dev->logged_first)
    {
      dev->logged_first = true;
      log_first = true;
    }

  spin_unlock_irqrestore(&dev->lock, flags);

  if (log_first)
    {
      /* One-shot bring-up breadcrumb: the CSI link and DMA are alive */

      vinfo("csi: first frame-done (buf=%p)\n", completed);
    }

  return false;
}

/****************************************************************************
 * Name: esp_csi_poll
 *
 * Description:
 *   Poll-mode replacement for the DW-GDMA completion interrupt.  The
 *   vendored GDMA ISR path (esp_intr_alloc) storms this port (level
 *   interrupt never cleared -> whole system freezes seconds after the
 *   first frame lands), so the completion is detected by watching the
 *   channel-enable bit instead: the hardware clears it when a block
 *   transfer finishes.  Runs in timer-interrupt context; the done
 *   handler only takes the spinlock, re-arms and queues work, all of
 *   which is IRQ-safe.
 *
 ****************************************************************************/

static void esp_csi_poll_worker(FAR void *arg)
{
  FAR struct esp_csi_dev_s *dev = (FAR struct esp_csi_dev_s *)arg;
  uint32_t mask = 1ul << dev->dma_chan_id;

  if (!dev->capturing)
    {
      return;
    }

  if ((dev->dma_hw->chen0.val & mask) == 0)
    {
      uint32_t brg_bad;

      esp_csi_dma_done(NULL, NULL, dev);

      /* Phase-slip auto-heal: if the bridge flagged discarded data, a
       * FIFO overrun or a row-count mismatch since the last check, the
       * byte stream lost data and the (exact-size) DMA blocks are no
       * longer locked to frame boundaries -- the image shows as split
       * or rolled.  Re-open the tap at the next frame boundary.
       */

      /* A stable-but-shifted phase produces NO bridge error events
       * (the DMA keeps up, the FIFO never overflows), so error-driven
       * healing alone cannot recover from it -- the image just stays
       * split.  Resync preventively every ~4 s as well: any phase
       * break, silent or not, is bounded to that window at the cost
       * of one skipped frame.
       */

      brg_bad = dev->hal.bridge_dev->int_raw.val & 0x0c;
      if (dev->capturing && ++dev->sync_age >= 60)
        {
          brg_bad |= 0x100;
        }

      if (brg_bad != 0 && dev->capturing)
        {
          irqstate_t f2;

          /* Restart the armed block from byte 0 as well: a few stale
           * FIFO bytes may already have leaked into it, and a block
           * with partial progress would complete mid-frame again and
           * trigger one more heal cycle before converging.
           */

          f2 = spin_lock_irqsave(&dev->lock);
          if (dev->capturing && dev->armed != NULL)
            {
              dw_gdma_channel_enable_ctrl(dev->dma_chan, false);
              esp_csi_dma_wait_idle(dev);
              esp_csi_arm_dma(dev, dev->armed);
            }

          spin_unlock_irqrestore(&dev->lock, f2);

          esp_csi_bridge_resync(dev, true);
          dev->swap_races++;
          dev->sync_age = 0;
        }

      return;
    }

  /* Runaway sentinel.  Self flow control bounds every block in
   * hardware, so the write pointer can never leave the armed buffer;
   * if it does anyway the transfer must die on the spot, before it
   * chews through the heap (post-mortem of the pre-self-FC crashes
   * found DAR far beyond the end of PSRAM).
   */

  irqstate_t flags = spin_lock_irqsave(&dev->lock);

  if (dev->capturing && dev->armed != NULL)
    {
      uint32_t dar = dev->dma_hw->ch[dev->dma_chan_id].dar0.val;
      uint32_t lo  = (uint32_t)(uintptr_t)dev->armed;

      if (dar < lo || dar > lo + dev->out_size)
        {
          verr("csi: RUNAWAY DMA dar=%08x armed=%08x+%u, stopping\n",
               (unsigned)dar, (unsigned)lo, (unsigned)dev->out_size);
          dw_gdma_channel_enable_ctrl(dev->dma_chan, false);
          esp_csi_dma_wait_idle(dev);
          dev->overruns++;
          esp_csi_arm_dma(dev, dev->armed);
        }
    }

  spin_unlock_irqrestore(&dev->lock, flags);
}

static void esp_csi_poll(wdparm_t arg)
{
  FAR struct esp_csi_dev_s *dev = (FAR struct esp_csi_dev_s *)arg;

  /* Timer-interrupt context: only queue the real work (the vendored
   * DW-GDMA re-arm APIs take their own locks and are not safe from
   * here) and re-arm the poll.
   */

  if (!dev->capturing)
    {
      return;
    }

  work_queue(HPWORK, &dev->poll_work, esp_csi_poll_worker, dev, 0);
  wd_start(&dev->poll_wdog, MSEC2TICK(10), esp_csi_poll, (wdparm_t)dev);
}

/****************************************************************************
 * Name: esp_csi_frame_worker
 *
 * Description:
 *   LPWORK worker: makes the completed frame CPU-visible (M2C cache
 *   sync) and invokes the imgdata capture callback.  The v4l2
 *   framework calls set_buf() (next buffer) or stop_capture() from
 *   inside that callback.
 *
 ****************************************************************************/


/****************************************************************************
 * Name: esp_csi_cache_inval
 *
 * Description:
 *   Chunked M2C cache invalidation.  A single whole-frame (1.2 MB)
 *   cache operation while the DSI scanout and CSI capture DMAs are both
 *   loading the PSRAM hard-freezes the system (A/B verified, dose
 *   dependent).  Short per-chunk operations keep each internal critical
 *   section brief and leave the fabric breathing room.
 *
 ****************************************************************************/

static void esp_csi_cache_inval(FAR uint8_t *buf, uint32_t size)
{
  uint32_t off;
  const uint32_t chunk = 32 * 1024;

  for (off = 0; off < size; off += chunk)
    {
      uint32_t n = (size - off) < chunk ? (size - off) : chunk;

      esp_cache_msync(buf + off, n, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    }
}

/****************************************************************************
 * Name: esp_csi_wait_frame_boundary
 *
 * Description:
 *   Bounded wait for the ISP frame-done event, i.e. for the sensor to
 *   enter vertical blanking.  Enabling the CSI bridge right after this
 *   point makes the next forwarded byte the first byte of a frame, so
 *   the (exact-size, self flow controlled) DMA blocks stay locked to
 *   frame boundaries.  If the sensor is not streaming yet the wait
 *   times out harmlessly: the bridge then opens before the first frame
 *   ever starts, which is aligned by construction.
 *
 *   Thread/worker context only (busy-waits up to ~50 ms); must NOT be
 *   called with the driver spinlock held.
 *
 ****************************************************************************/

static void esp_csi_wait_frame_boundary(FAR struct esp_csi_dev_s *dev)
{
#ifndef CONFIG_ESPRESSIF_MIPI_CSI_RAW_PASSTHROUGH
  isp_dev_t *isp = ISP_LL_GET_HW(CSI_CTLR_ID);
  int spins = 500;

  isp_ll_clear_intr(isp, ISP_LL_EVENT_FRAME);
  while ((isp_ll_get_intr_raw(isp) & ISP_LL_EVENT_FRAME) == 0 &&
         --spins > 0)
    {
      up_udelay(100);
    }
#else
  UNUSED(dev);
#endif
}

/****************************************************************************
 * Name: esp_csi_bridge_resync
 *
 * Description:
 *   Hard frame-phase resynchronization.  Merely toggling the bridge
 *   enable is not enough after a stall: the bridge FIFO keeps its
 *   stale mid-frame bytes and delivers them first, so the image stays
 *   rotated.  The reset pulse flushes the FIFO and pixel counters --
 *   and also clears configuration, so everything the driver ever
 *   programs into the bridge is written again -- then the tap is
 *   reopened only at a frame boundary.
 *
 *   Thread/worker context only (busy-waits); no spinlock held.
 *
 ****************************************************************************/

static void esp_csi_bridge_resync(FAR struct esp_csi_dev_s *dev,
                                  bool reopen)
{
  csi_brg_dev_t *brg = dev->hal.bridge_dev;

  mipi_csi_brg_ll_enable(brg, false);

  brg->csi_en.csi_brg_rst = 1;
  brg->csi_en.csi_brg_rst = 0;

  mipi_csi_brg_ll_set_burst_len(brg, CSI_BRG_BURST_LEN);
  mipi_csi_brg_ll_set_data_type_min(brg, CSI_BRG_DATA_TYPE_MIN);
  mipi_csi_brg_ll_set_data_type_max(brg, CSI_BRG_DATA_TYPE_MAX);
  mipi_csi_brg_ll_enable_color_conversion(brg, true);
  mipi_csi_brg_ll_set_color_mode_bypass(brg, true);
  mipi_csi_brg_ll_set_intput_data_h_pixel_num(brg, dev->width);
  mipi_csi_brg_ll_set_intput_data_v_row_num(brg, dev->height);
  brg->int_clr.val = 0x1f;

  esp_csi_wait_frame_boundary(dev);

  if (reopen)
    {
      mipi_csi_brg_ll_enable(brg, true);
    }
}

static void esp_csi_frame_worker(FAR void *arg)
{
  FAR struct esp_csi_dev_s *dev = (FAR struct esp_csi_dev_s *)arg;
  imgdata_capture_t cb;
  FAR void *cb_arg;
  FAR uint8_t *buf;
  uint32_t size;
  struct timeval tv;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);
  buf         = dev->done;
  dev->done   = NULL;
  cb          = dev->cb;
  cb_arg      = dev->cb_arg;
  size        = dev->out_size;
  if (buf != NULL && cb != NULL)
    {
      dev->frames++;
    }

  spin_unlock_irqrestore(&dev->lock, flags);

  if (buf == NULL || cb == NULL)
    {
      return;
    }

  /* Discard any (speculatively fetched) cache lines so the CPU reads
   * what the DMA wrote.  The buffer was fully invalidated before it
   * was armed, so this pass only guards against speculation, exactly
   * like the trailing M2C sync in the IDF driver.
   */

  /* No M2C pass here: every buffer is fully invalidated before it is
   * armed (set_buf / backup setup), so a completed frame is already
   * CPU-consistent apart from a negligible speculation window.  The
   * per-frame whole-buffer sync this replaced was a system-freeze
   * hazard under dual-DMA load.
   */

  gettimeofday(&tv, NULL);
  cb(0, size, &tv, cb_arg);

#ifndef CONFIG_ESPRESSIF_MIPI_CSI_RAW_PASSTHROUGH
  /* Thread context, no locks held: run the staggered 3A loop */

  esp_csi_3a_tick();
#endif
}

#ifndef CONFIG_ESPRESSIF_MIPI_CSI_RAW_PASSTHROUGH

/****************************************************************************
 * 3A: auto exposure and auto white balance
 *
 * A small port of the official esp_ipa behavior: the ISP AE unit
 * measures a 5x5 grid of block lumas and the AWB unit counts "white"
 * patches (pixels inside the calibrated luminance/ratio ranges) and
 * accumulates their R/G/B sums.  Every few frames the loop nudges the
 * sensor exposure/gain toward the target luma and re-derives the
 * white balance gains and color temperature, reprogramming the CCM
 * (vsync-latched, so runtime updates are safe).
 *
 * Sensor control goes through weak references to the SC2336 driver's
 * 3A entry points so this file links even without that driver.
 ****************************************************************************/

extern int sc2336_3a_set_exposure(uint32_t lines) __attribute__((weak));
extern int sc2336_3a_set_gain_index(int idx) __attribute__((weak));
extern uint32_t sc2336_3a_gain_x1000(int idx) __attribute__((weak));
extern int sc2336_3a_gain_count(void) __attribute__((weak));

/* Official SC2336 CCM table by color temperature (x1000, row-major),
 * and the AWB reference points mapping raw white-point ratios to the
 * same color temperatures (esp-video-components, p4_eco5 tuning).
 */

static const int16_t g_csi_ccm_x1000[][9] =
{
  { 1288,    2, -289,  -257, 1444, -187,  -209, -375, 1585 }, /* 2567K */
  { 1295, -186, -110,  -241, 1343, -102,   -55, -394, 1448 }, /* 3673K */
  { 1381, -258, -123,  -228, 1338, -110,   -33, -436, 1470 }, /* 3946K */
  { 1347, -239, -108,  -233, 1345, -112,   -43, -392, 1435 }, /* 4377K */
  { 1403, -296, -106,  -231, 1349, -118,   -34, -411, 1446 }, /* 4862K */
  { 1405, -312,  -92,  -205, 1324, -119,   -26, -376, 1401 }, /* 5625K */
  { 1368, -272,  -96,  -192, 1304, -112,   -30, -323, 1353 }, /* 6113K */
  { 1339, -246,  -92,  -182, 1294, -111,   -35, -284, 1319 }, /* 6575K */
  { 1393, -316,  -78,  -167, 1278, -111,   -22, -302, 1324 }, /* 6970K */
  { 1319, -278,  -41,  -106, 1187,  -82,   -15, -199, 1214 }, /* 10000K */
};

static const struct
{
  uint16_t rg_x1000;
  uint16_t bg_x1000;
} g_csi_awb_ref[] =
{
  { 978, 283 },   /* 2567K */
  { 760, 450 },   /* 3673K */
  { 723, 473 },   /* 3946K */
  { 675, 508 },   /* 4377K */
  { 631, 542 },   /* 4862K */
  { 574, 581 },   /* 5625K */
  { 543, 606 },   /* 6113K */
  { 509, 626 },   /* 6575K */
  { 496, 638 },   /* 6970K */
  { 376, 722 },   /* 10000K */
};

#define CSI_CCM_DEFAULT_IDX  4      /* 4862 K */

static struct
{
  uint32_t exp_lines;               /* Current sensor exposure */
  int      gain_idx;                /* Current gain ladder index */
  uint32_t wb_r_x1000;              /* Current white balance gains */
  uint32_t wb_b_x1000;
  int      ccm_idx;                 /* Current CCM table entry */
  uint32_t frame;                   /* Tick counter */
  bool     sensor_dirty;            /* Push exposure/gain on next tick */
} g_csi_3a;

/****************************************************************************
 * Name: esp_csi_ccm_program
 *
 * Description:
 *   Program the CCM as  table[ccm_idx] x diag(wb)  (the official
 *   pipeline applies WB gains before the CCM; column scaling folds
 *   both into the one matrix the hardware has).  Fixed point is
 *   sign-magnitude with ISP_LL_CCM_MATRIX_FRAC_BITS fraction bits.
 *
 ****************************************************************************/

static void esp_csi_ccm_program(isp_dev_t *isp, int ccm_idx,
                                uint32_t wb_r_x1000, uint32_t wb_b_x1000)
{
  const int16_t *m = g_csi_ccm_x1000[ccm_idx];
  const uint32_t wb_x1000[3] =
    {
      wb_r_x1000, 1000, wb_b_x1000
    };

  isp_ll_ccm_gain_t ccm[ISP_CCM_DIMENSION][ISP_CCM_DIMENSION];
  int i;
  int j;

  for (i = 0; i < 3; i++)
    {
      for (j = 0; j < 3; j++)
        {
          int64_t v = (int64_t)m[i * 3 + j] * wb_x1000[j] / 1000;

          v = (v << ISP_LL_CCM_MATRIX_FRAC_BITS) / 1000;

          if (v < 0)
            {
              ccm[i][j].val = (uint32_t)(-v) |
                (1u << (ISP_LL_CCM_MATRIX_INT_BITS +
                        ISP_LL_CCM_MATRIX_FRAC_BITS));
            }
          else
            {
              ccm[i][j].val = (uint32_t)v;
            }
        }
    }

  isp_ll_ccm_set_clk_ctrl_mode(isp, ISP_LL_PIPELINE_CLK_CTRL_AUTO);
  isp_ll_ccm_set_matrix(isp, ccm);
  isp_ll_shadow_update_ccm(isp, true);
  isp_ll_ccm_enable(isp, true);
}

/****************************************************************************
 * Name: esp_csi_3a_stats_init
 *
 * Description:
 *   Configure and enable the ISP AE and AWB statistic units.  Windows
 *   cover the full frame; the AWB luminance/ratio ranges come from the
 *   official tuning (raw-domain white patch classification).
 *
 ****************************************************************************/

static void esp_csi_3a_stats_init(isp_dev_t *isp,
                                  uint16_t width, uint16_t height)
{
  isp_ll_awb_rgb_ratio_t rmin;
  isp_ll_awb_rgb_ratio_t rmax;

  isp_ll_ae_set_clk_ctrl_mode(isp, ISP_LL_PIPELINE_CLK_CTRL_AUTO);
  isp_ll_ae_set_sample_point(isp, ISP_AE_SAMPLE_POINT_AFTER_DEMOSAIC);
  isp_ll_ae_set_window_range(isp, 0, width / 5, 0, height / 5);
  isp_ll_ae_set_subwin_pixnum_recip(isp, (width / 5) * (height / 5));
  isp_ll_ae_env_detector_set_thresh(isp, 0, 0);
  isp_ll_ae_enable(isp, true);
  isp_ll_ae_manual_update(isp);

  isp_ll_awb_set_clk_ctrl_mode(isp, ISP_LL_PIPELINE_CLK_CTRL_AUTO);
  isp_ll_awb_set_sample_point(isp, ISP_AWB_SAMPLE_POINT_BEFORE_CCM);
  isp_ll_awb_set_window_range(isp, 0, 0, width - 1, height - 1);
  isp_ll_awb_set_luminance_range(isp, 18, 208);

  rmin.val = 320 * 256 / 1000;      /* rg 0.32 .. 0.97 */
  rmax.val = 970 * 256 / 1000;
  isp_ll_awb_set_rg_ratio_range(isp, rmin, rmax);

  rmin.val = 220 * 256 / 1000;      /* bg 0.22 .. 0.80 */
  rmax.val = 800 * 256 / 1000;
  isp_ll_awb_set_bg_ratio_range(isp, rmin, rmax);

  isp_ll_awb_enable_algorithm_mode(isp, true);
  isp_ll_awb_enable(isp, true);
}

/****************************************************************************
 * Name: esp_csi_3a_ae
 *
 * Description:
 *   One damped auto-exposure step: mean of the 5x5 block lumas
 *   (uniform weights, per the official eco5 tuning) against the
 *   target; the exposure dial is turned first, sensor gain only when
 *   exposure saturates.  Exposure is quantized to 10 ms flicker
 *   periods (50 Hz mains) once long enough.
 *
 ****************************************************************************/

static void esp_csi_3a_ae(isp_dev_t *isp)
{
  uint32_t luma = 0;
  uint64_t total;
  uint32_t factor;
  uint32_t exp;
  uint32_t want_gain;
  int idx;
  int count;
  int i;

  if (sc2336_3a_set_exposure == NULL || sc2336_3a_gain_x1000 == NULL ||
      sc2336_3a_set_gain_index == NULL || sc2336_3a_gain_count == NULL)
    {
      return;
    }

  for (i = 0; i < 25; i++)
    {
      luma += isp_ll_ae_get_block_mean_lum(isp, i);
    }

  luma /= 25;
  isp_ll_ae_manual_update(isp);     /* Arm the next measurement */

  if (!g_csi_3a.sensor_dirty &&
      luma >= CSI_AE_LOW && luma <= CSI_AE_HIGH)
    {
      return;
    }

  if (luma == 0)
    {
      luma = 1;
    }

  factor = CSI_AE_TARGET * 1000 / luma;
  if (factor < 600)
    {
      factor = 600;
    }
  else if (factor > 1600)
    {
      factor = 1600;
    }

  factor = (factor + 1000) / 2;     /* 50% damping */

  total = (uint64_t)g_csi_3a.exp_lines *
          sc2336_3a_gain_x1000(g_csi_3a.gain_idx);
  total = total * factor / 1000;

  exp = (uint32_t)(total / 1000);   /* Exposure at unity gain */
  if (exp > CSI_AE_EXP_MAX)
    {
      exp = CSI_AE_EXP_MAX;
    }

  if (exp >= CSI_AE_FLICKER_LINES)
    {
      exp = exp - (exp % CSI_AE_FLICKER_LINES);
    }
  else if (exp < CSI_AE_EXP_MIN)
    {
      exp = CSI_AE_EXP_MIN;
    }

  want_gain = (uint32_t)(total / exp);
  count = sc2336_3a_gain_count();

  for (idx = 0; idx < count - 1; idx++)
    {
      if (sc2336_3a_gain_x1000(idx + 1) > want_gain)
        {
          break;
        }
    }

  if (exp != g_csi_3a.exp_lines || g_csi_3a.sensor_dirty)
    {
      if (sc2336_3a_set_exposure(exp) >= 0)
        {
          g_csi_3a.exp_lines = exp;
        }
    }

  if (idx != g_csi_3a.gain_idx || g_csi_3a.sensor_dirty)
    {
      if (sc2336_3a_set_gain_index(idx) >= 0)
        {
          g_csi_3a.gain_idx = idx;
        }
    }

  g_csi_3a.sensor_dirty = false;
}

/****************************************************************************
 * Name: esp_csi_3a_awb
 *
 * Description:
 *   One damped auto-white-balance step: the hardware white-patch
 *   R/G/B sums give the raw white point; gains are its reciprocal
 *   (70/30 damped), and the nearest calibrated reference point picks
 *   the CCM for that color temperature.
 *
 ****************************************************************************/

static void esp_csi_3a_awb(isp_dev_t *isp)
{
  uint32_t cnt = isp_ll_awb_get_white_patch_cnt(isp);
  uint32_t accr;
  uint32_t accg;
  uint32_t accb;
  uint32_t rg;
  uint32_t bg;
  uint32_t tr;
  uint32_t tb;
  uint32_t best_dist = UINT32_MAX;
  int best = g_csi_3a.ccm_idx;
  int i;

  if (cnt < CSI_AWB_MIN_PATCHES)
    {
      return;
    }

  accr = isp_ll_awb_get_accumulated_r_value(isp);
  accg = isp_ll_awb_get_accumulated_g_value(isp);
  accb = isp_ll_awb_get_accumulated_b_value(isp);

  if (accr == 0 || accg == 0 || accb == 0)
    {
      return;
    }

  rg = (uint32_t)((uint64_t)accr * 1000 / accg);
  bg = (uint32_t)((uint64_t)accb * 1000 / accg);

  if (rg < 320 || rg > 970 || bg < 220 || bg > 800)
    {
      return;
    }

  tr = 1000000 / rg;
  tb = 1000000 / bg;

  g_csi_3a.wb_r_x1000 = (7 * g_csi_3a.wb_r_x1000 + 3 * tr) / 10;
  g_csi_3a.wb_b_x1000 = (7 * g_csi_3a.wb_b_x1000 + 3 * tb) / 10;

  if (g_csi_3a.wb_r_x1000 < CSI_WB_GAIN_MIN)
    {
      g_csi_3a.wb_r_x1000 = CSI_WB_GAIN_MIN;
    }

  if (g_csi_3a.wb_r_x1000 > CSI_WB_GAIN_MAX)
    {
      g_csi_3a.wb_r_x1000 = CSI_WB_GAIN_MAX;
    }

  if (g_csi_3a.wb_b_x1000 < CSI_WB_GAIN_MIN)
    {
      g_csi_3a.wb_b_x1000 = CSI_WB_GAIN_MIN;
    }

  if (g_csi_3a.wb_b_x1000 > CSI_WB_GAIN_MAX)
    {
      g_csi_3a.wb_b_x1000 = CSI_WB_GAIN_MAX;
    }

  for (i = 0; i < (int)(sizeof(g_csi_awb_ref) /
                        sizeof(g_csi_awb_ref[0])); i++)
    {
      int32_t dr = (int32_t)rg - g_csi_awb_ref[i].rg_x1000;
      int32_t db = (int32_t)bg - g_csi_awb_ref[i].bg_x1000;
      uint32_t d = (uint32_t)(dr * dr + db * db);

      if (d < best_dist)
        {
          best_dist = d;
          best = i;
        }
    }

  g_csi_3a.ccm_idx = best;
  esp_csi_ccm_program(isp, best,
                      g_csi_3a.wb_r_x1000, g_csi_3a.wb_b_x1000);
}

/****************************************************************************
 * Name: esp_csi_3a_tick
 *
 * Description:
 *   Called once per delivered frame from the frame worker (thread
 *   context, no locks held).  AE and AWB run staggered every few
 *   frames to bound the I2C and register traffic.
 *
 ****************************************************************************/

static void esp_csi_3a_tick(void)
{
  isp_dev_t *isp = ISP_LL_GET_HW(CSI_CTLR_ID);

  g_csi_3a.frame++;

  if ((g_csi_3a.frame & 3) == 0)
    {
      esp_csi_3a_ae(isp);
    }

  if ((g_csi_3a.frame & 7) == 2)
    {
      esp_csi_3a_awb(isp);
    }
}

/****************************************************************************
 * Name: esp_csi_isp_configure
 *
 * Description:
 *   Program the ISP for inline RAW8 (Bayer BGGR) to RGB565 conversion
 *   of the CSI stream, with the demosaic stage enabled and fixed
 *   defaults for everything else (no 3A: expect a flat, possibly
 *   green-tinted image until white balance is tuned).  Sequence ported
 *   from upper_hal_isp/src/isp_core.c + isp_demosaic.c.
 *
 *   The module clock and divider are set up once in esp_csi_init();
 *   this function reprograms the geometry-dependent part and may be
 *   called again after a frame size change (ISP disabled first).
 *
 ****************************************************************************/

static void esp_csi_isp_configure(FAR struct esp_csi_dev_s *dev,
                                  uint16_t width, uint16_t height)
{
  isp_dev_t *isp = ISP_LL_GET_HW(CSI_CTLR_ID);
  isp_demosaic_grad_ratio_t grad_ratio;

  isp_ll_enable(isp, false);

  isp_ll_set_input_data_color_format(isp, ISP_COLOR_RAW8);
  isp_ll_set_output_data_color_format(isp, ISP_COLOR_RGB565);

  isp_ll_clk_enable(isp, true);
  isp_ll_set_input_data_source(isp, ISP_INPUT_DATA_SOURCE_CSI);
  isp_ll_enable_line_start_packet_exist(isp, false);
  isp_ll_enable_line_end_packet_exist(isp, false);
  isp_ll_set_intput_data_h_pixel_num(isp, width);
  isp_ll_set_intput_data_v_row_num(isp, height);
  isp_ll_set_bayer_mode(isp, COLOR_RAW_ELEMENT_ORDER_BGGR);
  isp_ll_shadow_set_mode(isp, ISP_SHADOW_MODE_UPDATE_ONLY_NEXT_VSYNC);

  /* Demosaic (Bayer interpolation) is required for any RGB output.
   * Gradient ratio 2.5 matches the IDF isp/multi_pipelines example.
   */

  /* Gradient ratio 1.5: the official SC2336 factory tuning value at
   * low gain (the 2.5 used by the IDF multi_pipelines example is a
   * generic default, not sensor-calibrated).  The decimal field is a
   * binary fraction (/256).
   */

  grad_ratio.val     = 0;
  grad_ratio.integer = 1;
  grad_ratio.decimal = 128;
  isp_ll_demosaic_set_grad_ratio(isp, grad_ratio);
  isp_ll_demosaic_set_padding_mode(isp,
      ISP_DEMOSAIC_EDGE_PADDING_MODE_SRND_DATA);
  isp_ll_demosaic_set_padding_data(isp, 0);
  isp_ll_demosaic_set_padding_line_tail_valid_start_pixel(isp, 0);
  isp_ll_demosaic_set_padding_line_tail_valid_end_pixel(isp, 0);
  isp_ll_demosaic_enable(isp, true);

  /* White balance and crosstalk correction through the CCM, starting
   * from the calibrated defaults; the AWB loop refines both at
   * runtime.  Reset the 3A state to the calibrated starting point and
   * schedule a sensor exposure/gain push on the first tick.
   */

  g_csi_3a.exp_lines    = 900;
  g_csi_3a.gain_idx     = 0;
  g_csi_3a.wb_r_x1000   = CSI_WB_GAIN_R_X1000;
  g_csi_3a.wb_b_x1000   = CSI_WB_GAIN_B_X1000;
  g_csi_3a.ccm_idx      = CSI_CCM_DEFAULT_IDX;
  g_csi_3a.frame        = 0;
  g_csi_3a.sensor_dirty = true;

  esp_csi_ccm_program(isp, g_csi_3a.ccm_idx,
                      g_csi_3a.wb_r_x1000, g_csi_3a.wb_b_x1000);

  /* AE/AWB statistic hardware */

  esp_csi_3a_stats_init(isp, width, height);

  /* Gamma correction (2.2) with black-level subtraction baked into
   * the curve: the sensor output is linear with a ~16/255 pedestal
   * (black patch measured at 10% linear), which washes the whole
   * image gray without this.  16-point curve, x steps must be powers
   * of two summing to 256 (uniform 16 here);
   * y = 255 * ((x-16)/239)^(1/2.2), clamped at 0.
   */

    {
      static const uint8_t gamma_y[16] =
        {
          0, 75, 102, 123, 140, 155, 168, 181,
          192, 203, 212, 222, 231, 239, 248, 255
        };

      isp_gamma_curve_points_t pts;
      int i;

      for (i = 0; i < 16; i++)
        {
          pts.pt[i].x = (i == 15) ? 255 : (uint8_t)((i + 1) * 16);
          pts.pt[i].y = gamma_y[i];
        }

      isp_ll_gamma_set_correction_curve(isp, COLOR_COMPONENT_R, &pts);
      isp_ll_gamma_set_correction_curve(isp, COLOR_COMPONENT_G, &pts);
      isp_ll_gamma_set_correction_curve(isp, COLOR_COMPONENT_B, &pts);
      isp_ll_gamma_enable(isp, true);
    }

  isp_ll_enable(isp, true);

  vinfo("csi: ISP RAW8(BGGR)->RGB565 %ux%u demosaic on\n",
        width, height);
}

#endif /* !CONFIG_ESPRESSIF_MIPI_CSI_RAW_PASSTHROUGH */

/****************************************************************************
 * Name: esp_csi_init
 *
 * Description:
 *   imgdata init: full CSI link bring-up.  Called by the v4l2
 *   framework from the first open() of the video device, in thread
 *   context, before the image sensor is initialized.
 *
 *   Sequence (ported from esp_cam_new_csi_ctlr + mipi_csi_brg_claim):
 *   DPHY LDO -> host bus clock -> PHY cfg clock -> bridge module
 *   clock -> HAL init (PHY PLL range, lanes, host) -> bridge tuning ->
 *   ISP clocking -> DW-GDMA channel.
 *
 ****************************************************************************/

static int esp_csi_init(FAR struct imgdata_s *data)
{
  FAR struct esp_csi_dev_s *dev = (FAR struct esp_csi_dev_s *)data;
  dw_gdma_channel_alloc_config_t dma_alloc;
  dw_gdma_event_callbacks_t dma_cbs;
  mipi_csi_hal_config_t hal_cfg;
  soc_module_clk_t phy_clk;
  int ret;

  if (dev->initialized)
    {
      return OK;
    }

  vinfo("csi: init: %u lanes @%u Mbps, LDO chan %d @%u mV\n",
        dev->config.data_lanes,
        (unsigned)dev->config.lane_rate_mbps,
        dev->config.phy_ldo_chan,
        (unsigned)dev->config.phy_ldo_mv);

  /* Power the MIPI DPHY rail from the on-chip LDO.  The channel is
   * refcount-shared with the DSI display (both non-adjustable), so the
   * config must stay memset-zeroed apart from channel and voltage.
   */

  if (dev->config.phy_ldo_chan >= 0)
    {
      esp_ldo_channel_config_t ldo_cfg;

      memset(&ldo_cfg, 0, sizeof(ldo_cfg));
      ldo_cfg.chan_id    = dev->config.phy_ldo_chan;
      ldo_cfg.voltage_mv = (int)dev->config.phy_ldo_mv;

      if (esp_ldo_acquire_channel(&ldo_cfg, &dev->ldo) != 0)
        {
          verr("csi: failed to acquire DPHY LDO channel %d\n",
               dev->config.phy_ldo_chan);
          return -EIO;
        }
    }

  /* CSI host bus clock and reset */

  PERIPH_RCC_ATOMIC()
    {
      mipi_csi_ll_enable_host_bus_clock(CSI_CTLR_ID, 0);
      mipi_csi_ll_enable_host_bus_clock(CSI_CTLR_ID, 1);
      mipi_csi_ll_reset_host_clock(CSI_CTLR_ID);
    }

  /* D-PHY configuration clock */

  phy_clk = (soc_module_clk_t)MIPI_CSI_PHY_CLK_SRC_DEFAULT;
  esp_clk_tree_enable_src(phy_clk, true);

  PERIPH_RCC_ATOMIC()
    {
      mipi_csi_ll_set_phy_clock_source(CSI_CTLR_ID,
                                       MIPI_CSI_PHY_CLK_SRC_DEFAULT);
      mipi_csi_ll_enable_phy_config_clock(CSI_CTLR_ID, 0);
      mipi_csi_ll_enable_phy_config_clock(CSI_CTLR_ID, 1);
    }

  /* CSI bridge module clock (inline replica of mipi_csi_brg_claim();
   * this driver is the only bridge user in this build, the ISP shares
   * the already-clocked bridge).
   */

  PERIPH_RCC_ATOMIC()
    {
      mipi_csi_ll_enable_brg_module_clock(CSI_CTLR_ID, true);
      mipi_csi_ll_reset_brg_module_clock(CSI_CTLR_ID);
      mipi_csi_brg_ll_enable_clock(MIPI_CSI_BRG_LL_GET_HW(CSI_CTLR_ID),
                                   true);
    }

  vinfo("csi: host/PHY/bridge clocks enabled\n");

  /* HAL init: D-PHY reset + PLL HS frequency range for the lane rate,
   * host lane count, and bridge frame geometry.  NOTE: the HAL applies
   * frame_height to the bridge h_pixel_num and frame_width to
   * v_row_num (the field names are swapped in the IDF HAL); the values
   * are re-programmed with the negotiated frame size at start_capture.
   */

  memset(&hal_cfg, 0, sizeof(hal_cfg));
  hal_cfg.lanes_num         = dev->config.data_lanes;
  hal_cfg.lane_bit_rate_mbps = (int)dev->config.lane_rate_mbps;
  hal_cfg.frame_height      = g_csi_modes[0].width;
  hal_cfg.frame_width       = g_csi_modes[0].height;
  hal_cfg.in_bpp            = CSI_IN_BPP;
  hal_cfg.out_bpp           = CSI_OUT_BPP;
  hal_cfg.byte_swap_en      = false;

  mipi_csi_hal_init(&dev->hal, &hal_cfg);

  /* Bridge tuning beyond the HAL defaults, as in esp_cam_new_csi_ctlr:
   * AXI burst length, accepted data type window, and (on chips that
   * have the bridge color converter) bypass it: the ISP performs the
   * RAW8 -> RGB565 conversion so the bridge must pass data through
   * unchanged.  On older chip revisions these two calls are no-ops.
   */

  mipi_csi_brg_ll_set_burst_len(dev->hal.bridge_dev, CSI_BRG_BURST_LEN);
  mipi_csi_brg_ll_set_data_type_min(dev->hal.bridge_dev,
                                    CSI_BRG_DATA_TYPE_MIN);
  mipi_csi_brg_ll_set_data_type_max(dev->hal.bridge_dev,
                                    CSI_BRG_DATA_TYPE_MAX);
  mipi_csi_brg_ll_enable_color_conversion(dev->hal.bridge_dev, true);
  mipi_csi_brg_ll_set_color_mode_bypass(dev->hal.bridge_dev, true);

  vinfo("csi: host+bridge configured (burst %d, dt 0x%02x..0x%02x)\n",
        CSI_BRG_BURST_LEN, CSI_BRG_DATA_TYPE_MIN, CSI_BRG_DATA_TYPE_MAX);

#ifndef CONFIG_ESPRESSIF_MIPI_CSI_RAW_PASSTHROUGH
  /* ISP module clock, functional clock source and divider (the
   * geometry-dependent programming happens in start_capture).
   */

    {
      isp_dev_t *isp = ISP_LL_GET_HW(CSI_CTLR_ID);
      hal_utils_clk_div_t clk_div;
      uint32_t src_hz = 0;
      uint32_t div;

      PERIPH_RCC_ATOMIC()
        {
          isp_ll_enable_module_clock(isp, true);
          isp_ll_reset_module_clock(isp);
        }

      esp_clk_tree_src_get_freq_hz((soc_module_clk_t)ISP_CLK_SRC_DEFAULT,
                                   ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                                   &src_hz);
      esp_clk_tree_enable_src((soc_module_clk_t)ISP_CLK_SRC_DEFAULT, true);

      div = src_hz / CSI_ISP_CLK_HZ;
      if (div < 1)
        {
          div = 1;
        }

      memset(&clk_div, 0, sizeof(clk_div));
      clk_div.integer = div;

      PERIPH_RCC_ATOMIC()
        {
          isp_ll_select_clk_source(isp, ISP_CLK_SRC_DEFAULT);
          isp_ll_set_clock_div(isp, &clk_div);
        }

      /* Reset control state and mask/clear all ISP interrupts (this
       * driver polls nothing from the ISP; errors surface as missing
       * frames and are diagnosed via the counters).
       */

      isp_ll_init(isp);

      vinfo("csi: ISP clock %u Hz (src %u Hz / %u)\n",
            (unsigned)(src_hz / div), (unsigned)src_hz, (unsigned)div);
    }
#endif /* !CONFIG_ESPRESSIF_MIPI_CSI_RAW_PASSTHROUGH */

  /* CSI RX DMA channel: the CSI bridge is the source peripheral but
   * the DMA engine itself is the flow controller (P2M_DMAC), so a block
   * transfer moves EXACTLY the programmed number of beats and then
   * stops, regardless of what the bridge signals.
   *
   * The IDF driver uses DW_GDMA_FLOW_CTRL_SRC (bridge terminates the
   * block at frame end, block size only an upper bound).  That works
   * with their microsecond interrupt re-arm, but with our poll-mode
   * re-arm a missed/uncredited frame-end leaves the channel writing
   * without ANY bound: JTAG post-mortem of a heap-corruption crash
   * found DAR=0x4c000080 -- the write pointer had marched linearly
   * through and beyond the entire PSRAM, and LVGL heap objects next
   * to the frame buffers were buried under pixel data.  Self flow
   * control turns any such phase slip into a shifted image instead of
   * memory corruption.
   */

  memset(&dma_alloc, 0, sizeof(dma_alloc));
  dma_alloc.src.block_transfer_type = DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS;
  dma_alloc.src.role                = DW_GDMA_ROLE_PERIPH_CSI;
  dma_alloc.src.handshake_type      = DW_GDMA_HANDSHAKE_HW;
  dma_alloc.src.num_outstanding_requests = 5;
  dma_alloc.src.status_fetch_addr   = MIPI_CSI_BRG_MEM_BASE;
  dma_alloc.dst.block_transfer_type = DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS;
  dma_alloc.dst.role                = DW_GDMA_ROLE_MEM;
  dma_alloc.dst.handshake_type      = DW_GDMA_HANDSHAKE_HW;
  dma_alloc.dst.num_outstanding_requests = 5;
  dma_alloc.flow_controller         = DW_GDMA_FLOW_CTRL_SELF;
  dma_alloc.chan_priority           = 1;

  if (dw_gdma_new_channel(&dma_alloc, &dev->dma_chan) != 0)
    {
      verr("csi: failed to create DW-GDMA channel\n");
      ret = -ENOMEM;
      goto errout_with_ldo;
    }

  /* Poll mode: do NOT register GDMA event callbacks - that would lazily
   * install the vendored ISR whose unserviced level interrupt freezes
   * the whole system (same class as the historical I2S-GDMA storm).
   * Completion is detected by esp_csi_poll() instead.
   */

  memset(&dma_cbs, 0, sizeof(dma_cbs));
  dma_cbs.on_full_trans_done = NULL;

  UNUSED(dma_cbs);

  dw_gdma_channel_get_id(dev->dma_chan, &dev->dma_chan_id);
  dev->dma_hw = DW_GDMA_LL_GET_HW(0);

  vinfo("csi: DW-GDMA channel %d ready (CSI src flow control)\n",
        dev->dma_chan_id);

  dev->capturing        = false;
  dev->warned_unaligned = false;
  dev->armed            = NULL;
  dev->pending          = NULL;
  dev->done             = NULL;
  dev->cb               = NULL;
  dev->cb_arg           = NULL;
  dev->initialized      = true;
  return OK;

errout_with_dma:
  dw_gdma_del_channel(dev->dma_chan);
  dev->dma_chan = NULL;

errout_with_ldo:
  if (dev->ldo != NULL)
    {
      esp_ldo_release_channel(dev->ldo);
      dev->ldo = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_csi_uninit
 *
 * Description:
 *   imgdata uninit: tear the pipeline down.  Called from the last
 *   close() of the video device, in thread context.
 *
 ****************************************************************************/

static int esp_csi_uninit(FAR struct imgdata_s *data)
{
  FAR struct esp_csi_dev_s *dev = (FAR struct esp_csi_dev_s *)data;

  if (!dev->initialized)
    {
      return OK;
    }

  esp_csi_stop_capture(data);
  work_cancel_sync(LPWORK, &dev->work);

#ifndef CONFIG_ESPRESSIF_MIPI_CSI_RAW_PASSTHROUGH
    {
      isp_dev_t *isp = ISP_LL_GET_HW(CSI_CTLR_ID);

      isp_ll_enable(isp, false);
      isp_ll_clk_enable(isp, false);

      PERIPH_RCC_ATOMIC()
        {
          isp_ll_enable_module_clock(isp, false);
        }
    }
#endif

  if (dev->dma_chan != NULL)
    {
      dw_gdma_del_channel(dev->dma_chan);
      dev->dma_chan = NULL;
    }

  PERIPH_RCC_ATOMIC()
    {
      mipi_csi_brg_ll_enable_clock(MIPI_CSI_BRG_LL_GET_HW(CSI_CTLR_ID),
                                   false);
      mipi_csi_ll_enable_brg_module_clock(CSI_CTLR_ID, false);
      mipi_csi_ll_enable_phy_config_clock(CSI_CTLR_ID, 0);
      mipi_csi_ll_enable_host_bus_clock(CSI_CTLR_ID, 0);
    }

  if (dev->backup != NULL)
    {
      kmm_free(dev->backup);
      dev->backup = NULL;
      dev->backup_size = 0;
    }

  if (dev->ldo != NULL)
    {
      esp_ldo_release_channel(dev->ldo);
      dev->ldo = NULL;
    }

  dev->initialized = false;
  vinfo("csi: uninitialized\n");
  return OK;
}

/****************************************************************************
 * Name: esp_csi_set_buf
 *
 * Description:
 *   Hand the driver the next capture buffer.  Called by the v4l2
 *   framework right before start_capture(), and from inside the
 *   capture callback (LPWORK context, framework spinlock held) for
 *   every subsequent frame.
 *
 *   The buffer is invalidated (M2C) before it is exposed to the DMA so
 *   no dirty CPU cache line can be evicted over DMA-written data.
 *
 *   If the DMA is currently armed on the backup buffer and that
 *   transfer has not received data yet, the transfer is retargeted
 *   onto the new buffer ("hot swap") so the frame about to start is
 *   captured instead of dropped.
 *
 ****************************************************************************/

static int esp_csi_set_buf(FAR struct imgdata_s *data,
                           uint8_t nr_datafmts,
                           FAR imgdata_format_t *datafmts,
                           FAR uint8_t *addr, uint32_t size)
{
  FAR struct esp_csi_dev_s *dev = (FAR struct esp_csi_dev_s *)data;
  uint32_t frame_size;
  irqstate_t flags;
  bool swapped = false;

  if (nr_datafmts < 1 || datafmts == NULL || addr == NULL)
    {
      return -EINVAL;
    }

  frame_size = (uint32_t)datafmts[IMGDATA_FMT_MAIN].width *
               datafmts[IMGDATA_FMT_MAIN].height * CSI_OUT_BPP / 8;

  if (dev->capturing && frame_size != dev->out_size)
    {
      verr("csi: set_buf frame size %u != active %u\n",
           (unsigned)frame_size, (unsigned)dev->out_size);
      return -EINVAL;
    }

  if (!CSI_IS_CACHE_ALIGNED(addr) || !CSI_IS_CACHE_ALIGNED(frame_size) ||
      size < frame_size)
    {
      if (!dev->warned_unaligned)
        {
          dev->warned_unaligned = true;
          vwarn("csi: rejecting buffer %p size %u: need %u-byte "
                "cache-line alignment and >= %u bytes\n",
                addr, (unsigned)size, CSI_CACHE_LINE,
                (unsigned)frame_size);
        }

      return -EINVAL;
    }

  /* Drop all cache lines of the buffer before the DMA may write it */

  esp_csi_cache_inval(addr, frame_size);

  flags = spin_lock_irqsave(&dev->lock);

  if (dev->capturing && dev->armed == dev->backup && dev->done == NULL &&
      dev->dma_hw != NULL)
    {
      /* The DMA idles on the backup buffer.  If the next frame has not
       * started (destination address register still at the backup
       * base), retarget the transfer onto the new v4l2 buffer.
       */

      uint32_t backup_base = (uint32_t)(uintptr_t)dev->backup;

      if (dev->dma_hw->ch[dev->dma_chan_id].dar0.dar0 == backup_base)
        {
          dw_gdma_channel_enable_ctrl(dev->dma_chan, false);
          esp_csi_dma_wait_idle(dev);

          if (dev->dma_hw->ch[dev->dma_chan_id].dar0.dar0 == backup_base)
            {
              esp_csi_arm_dma(dev, addr);
              dev->armed = addr;
              dev->swaps++;
              swapped = true;
            }
          else
            {
              /* Lost the race: the frame started while disabling.
               * Re-arm the backup so the hardware stays serviced; the
               * mangled frame lands in the backup and is dropped.
               */

              esp_csi_arm_dma(dev, dev->backup);
              dev->swap_races++;
            }
        }
    }

  if (!swapped)
    {
      dev->pending = addr;
    }

  spin_unlock_irqrestore(&dev->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: esp_csi_validate_frame_setting
 ****************************************************************************/

static int esp_csi_validate_frame_setting(FAR struct imgdata_s *data,
                           uint8_t nr_datafmts,
                           FAR imgdata_format_t *datafmts,
                           FAR imgdata_interval_t *interval)
{
  int i;

  if (nr_datafmts < 1 || datafmts == NULL)
    {
      return -EINVAL;
    }

  if (nr_datafmts > 1)
    {
      /* No sub-image (interleaved) stream support */

      return -ENOTSUP;
    }

  if (datafmts[IMGDATA_FMT_MAIN].pixelformat != IMGDATA_PIX_FMT_RGB565)
    {
      verr("csi: unsupported pixel format %u\n",
           (unsigned)datafmts[IMGDATA_FMT_MAIN].pixelformat);
      return -ENOTSUP;
    }

  for (i = 0; i < (int)nitems(g_csi_modes); i++)
    {
      if (datafmts[IMGDATA_FMT_MAIN].width == g_csi_modes[i].width &&
          datafmts[IMGDATA_FMT_MAIN].height == g_csi_modes[i].height)
        {
          break;
        }
    }

  if (i == (int)nitems(g_csi_modes))
    {
      verr("csi: unsupported frame size %ux%u\n",
           datafmts[IMGDATA_FMT_MAIN].width,
           datafmts[IMGDATA_FMT_MAIN].height);
      return -ENOTSUP;
    }

  if (interval != NULL && interval->numerator != 0 &&
      interval->denominator != 30 * interval->numerator)
    {
      /* The sensor modes are 30 fps; other rates are the sensor
       * driver's business, only note it.
       */

      vwarn("csi: frame interval %u/%u differs from 30 fps\n",
            (unsigned)interval->numerator,
            (unsigned)interval->denominator);
    }

  return OK;
}

/****************************************************************************
 * Name: esp_csi_start_capture
 *
 * Description:
 *   Arm the pipeline and start streaming into the buffer previously
 *   handed over via set_buf().  Called in thread context; the sensor
 *   is started by the framework right after this returns.
 *
 ****************************************************************************/

static int esp_csi_start_capture(FAR struct imgdata_s *data,
                           uint8_t nr_datafmts,
                           FAR imgdata_format_t *datafmts,
                           FAR imgdata_interval_t *interval,
                           imgdata_capture_t callback,
                           FAR void *arg)
{
  FAR struct esp_csi_dev_s *dev = (FAR struct esp_csi_dev_s *)data;
  FAR uint8_t *first;
  uint16_t width;
  uint16_t height;
  irqstate_t flags;
  int ret;

  if (!dev->initialized)
    {
      return -EPERM;
    }

  if (dev->capturing)
    {
      return -EBUSY;
    }

  ret = esp_csi_validate_frame_setting(data, nr_datafmts, datafmts,
                                       interval);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->pending == NULL)
    {
      verr("csi: start_capture without a buffer (set_buf missing?)\n");
      return -EIO;
    }

  width  = datafmts[IMGDATA_FMT_MAIN].width;
  height = datafmts[IMGDATA_FMT_MAIN].height;

  dev->width     = width;
  dev->height    = height;
  dev->out_size  = (uint32_t)width * height * CSI_OUT_BPP / 8;

  /* With self flow control the block size is EXACT, in 64-bit beats of
   * what actually crosses the DMA: the ISP output (RGB565 unless in
   * RAW passthrough).  The old CSI_IN_BPP-based value mirrored the IDF
   * peripheral-flow-control "upper bound" and was half the real frame.
   */

  dev->dma_beats = dev->out_size / 8;

  /* The internal backup buffer keeps the DMA armed when v4l2 has no
   * buffer ready at a frame boundary.  Allocated once per frame size
   * (not per frame) and kept across capture cycles.
   */

  if (dev->backup == NULL || dev->backup_size != dev->out_size)
    {
      if (dev->backup != NULL)
        {
          kmm_free(dev->backup);
        }

      dev->backup = kmm_memalign(CSI_CACHE_LINE, dev->out_size);
      if (dev->backup == NULL)
        {
          verr("csi: no memory for %u-byte backup buffer\n",
               (unsigned)dev->out_size);
          return -ENOMEM;
        }

      dev->backup_size = dev->out_size;
      esp_csi_cache_inval(dev->backup, dev->backup_size);
    }

  /* Frame geometry: CSI bridge pixel counters and ISP window */

  mipi_csi_brg_ll_set_intput_data_h_pixel_num(dev->hal.bridge_dev,
                                              width);
  mipi_csi_brg_ll_set_intput_data_v_row_num(dev->hal.bridge_dev,
                                            height);

#ifndef CONFIG_ESPRESSIF_MIPI_CSI_RAW_PASSTHROUGH
  esp_csi_isp_configure(dev, width, height);
#endif

  work_cancel(LPWORK, &dev->work);

  /* If the sensor is already streaming (pipeline restart), flush the
   * bridge and open the tap only at a frame boundary so the first DMA
   * block starts on the first byte of a frame.  Every following block
   * then stays aligned: blocks are exact frame size and the re-arm
   * happens inside vertical blanking.  (reopen=false: the tap is
   * opened under the spinlock below, right after the DMA is armed.)
   */

  esp_csi_bridge_resync(dev, false);

  flags = spin_lock_irqsave(&dev->lock);

  dev->cb          = callback;
  dev->cb_arg      = arg;
  dev->frames      = 0;
  dev->drops       = 0;
  dev->swaps       = 0;
  dev->swap_races  = 0;
  dev->overruns    = 0;
  dev->done        = NULL;
  dev->warned_unaligned = false;
  dev->logged_first = false;
  dev->sync_age    = 0;

  first        = dev->pending;
  dev->pending = NULL;
  dev->armed   = first;
  esp_csi_arm_dma(dev, first);

  /* Open the tap: from here the bridge pushes frames into the DMA */

  mipi_csi_brg_ll_enable(dev->hal.bridge_dev, true);
  dev->capturing = true;
  wd_start(&dev->poll_wdog, MSEC2TICK(10), esp_csi_poll, (wdparm_t)dev);

  spin_unlock_irqrestore(&dev->lock, flags);

  vinfo("csi: capture started %ux%u -> %p (%u bytes, %u beats)\n",
        width, height, first, (unsigned)dev->out_size,
        (unsigned)dev->dma_beats);
  return OK;
}

/****************************************************************************
 * Name: esp_csi_stop_capture
 *
 * Description:
 *   Stop streaming.  May be called from thread context or from inside
 *   the capture callback (LPWORK, with the framework spinlock held and
 *   interrupts disabled), so it must not block.
 *
 ****************************************************************************/

static int esp_csi_stop_capture(FAR struct imgdata_s *data)
{
  FAR struct esp_csi_dev_s *dev = (FAR struct esp_csi_dev_s *)data;
  uint32_t frames;
  uint32_t drops;
  uint32_t swaps;
  uint32_t races;
  uint32_t overruns;
  irqstate_t flags;

  if (!dev->initialized)
    {
      return -EPERM;
    }

  flags = spin_lock_irqsave(&dev->lock);

  if (!dev->capturing)
    {
      spin_unlock_irqrestore(&dev->lock, flags);
      return OK;
    }

  dev->capturing = false;
  wd_cancel(&dev->poll_wdog);
  dev->cb        = NULL;
  dev->cb_arg    = NULL;

  mipi_csi_brg_ll_enable(dev->hal.bridge_dev, false);
  dw_gdma_channel_enable_ctrl(dev->dma_chan, false);

  /* The disable above is only a request: the channel keeps writing
   * until outstanding AXI beats drain.  Wait for it so no buffer the
   * caller is about to free is still a DMA target when we return.
   */

  esp_csi_dma_wait_idle(dev);

  dev->armed   = NULL;
  dev->pending = NULL;
  dev->done    = NULL;

  frames   = dev->frames;
  drops    = dev->drops;
  swaps    = dev->swaps;
  races    = dev->swap_races;
  overruns = dev->overruns;

  spin_unlock_irqrestore(&dev->lock, flags);

  /* Non-blocking cancel: if the worker is already running it will find
   * done == NULL / cb == NULL and bail out.
   */

  work_cancel(LPWORK, &dev->work);

  vinfo("csi: capture stopped: frames=%u drops=%u swaps=%u races=%u "
        "overruns=%u\n",
        (unsigned)frames, (unsigned)drops, (unsigned)swaps,
        (unsigned)races, (unsigned)overruns);
  return OK;
}

/****************************************************************************
 * Name: esp_csi_alloc
 *
 * Description:
 *   Frame memory allocator used by the v4l2 framework for MMAP
 *   buffers.  Enforces data cache line alignment of address and size
 *   so that every buffer in the heap satisfies the DMA coherency
 *   requirements checked in set_buf().
 *
 ****************************************************************************/

static FAR void *esp_csi_alloc(FAR struct imgdata_s *data,
                               uint32_t align_size, uint32_t size)
{
  if (align_size < CSI_CACHE_LINE)
    {
      align_size = CSI_CACHE_LINE;
    }

  size = (size + CSI_CACHE_LINE - 1) & ~(uint32_t)(CSI_CACHE_LINE - 1);
  return kmm_memalign(align_size, size);
}

/****************************************************************************
 * Name: esp_csi_free
 ****************************************************************************/

static void esp_csi_free(FAR struct imgdata_s *data, FAR void *addr)
{
  kmm_free(addr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_mipi_csi_initialize
 *
 * Description:
 *   Latch the CSI link parameters and return the singleton imgdata
 *   lower half for registration by the board logic.  The hardware is
 *   not touched here; bring-up happens in the imgdata init operation
 *   when the video device is opened.
 *
 * Input Parameters:
 *   config - CSI link description (lanes, per-lane bit rate, DPHY LDO)
 *
 * Returned Value:
 *   Pointer to the imgdata lower half; NULL on invalid arguments.
 *
 ****************************************************************************/

FAR struct imgdata_s *esp_mipi_csi_initialize(
    FAR const struct esp_mipi_csi_config_s *config)
{
  FAR struct esp_csi_dev_s *dev = &g_csi_dev;

  if (config == NULL || config->data_lanes < 1 ||
      config->data_lanes > 2 || config->lane_rate_mbps == 0)
    {
      verr("csi: invalid link configuration\n");
      return NULL;
    }

  memcpy(&dev->config, config, sizeof(dev->config));
  spin_lock_init(&dev->lock);

  vinfo("csi: registered (%u lanes @%u Mbps%s)\n",
        config->data_lanes, (unsigned)config->lane_rate_mbps,
#ifdef CONFIG_ESPRESSIF_MIPI_CSI_RAW_PASSTHROUGH
        ", RAW passthrough"
#else
        ", ISP RAW8->RGB565"
#endif
        );

  return &dev->data;
}
