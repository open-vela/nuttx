/****************************************************************************
 * arch/arm/src/t113/t113_smhc.c
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
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/mutex.h>
#include <nuttx/mmcsd.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "t113_clk.h"
#include "t113_gpio.h"
#include "hardware/t113_smhc.h"
#include "hardware/t113_memorymap.h"
#include "hardware/t113_pinmap.h"
#include "t113_smhc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_T113_SMHC0) && !defined(CONFIG_T113_SMHC1) && \
    !defined(CONFIG_T113_SMHC2)
#  error "At least one CONFIG_T113_SMHCn must be enabled"
#endif

#if !defined(CONFIG_SCHED_WORKQUEUE) || !defined(CONFIG_SCHED_HPWORK)
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE and CONFIG_SCHED_HPWORK"
#endif

#if !defined(CONFIG_SDIO_BLOCKSETUP)
#  error "CONFIG_SDIO_BLOCKSETUP is mandatory for this driver"
#endif

/* The IDMAC encodes every "address" field (DLBA register,
 * descriptor.buf_addr_ptr1, descriptor.buf_addr_ptr2) as `phys >> 2`.
 * The controller internally re-shifts left by 2 to reconstruct the
 * 32-bit bus address.  Inputs to this macro must already be
 * 4-byte-aligned (smhc_dma_setup rejects misaligned buffers).
 */

#define SMHC_PHYS_TO_DESC_ADDR(p) \
        ((uint32_t)(((uintptr_t)(p)) >> 2))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct t113_smhc_config_s
{
  uintptr_t base;
  uint8_t   irq;
  uint8_t   bus;
  uint32_t  pinset_clk;
  uint32_t  pinset_cmd;
  uint32_t  pinset_d[4];
  int16_t   cd_pin;
  uint8_t   caps;
};

/* IDMAC descriptor: 16 bytes, word-aligned. */

struct t113_smhc_desc_s
{
  uint32_t ctrl;        /* DES0: OWN, ERR, CHAIN, FIRST, LAST, DIC */
  uint32_t buf_len;     /* DES1[12:0]: byte count, mult of 4, <=4 KiB    */
  uint32_t buf_addr;    /* DES2: physical buffer address                  */
  uint32_t next;        /* DES3: physical address of next descriptor     */
};

struct t113_smhc_dev_s
{
  struct sdio_dev_s        dev;    /* must be first */
  const struct t113_smhc_config_s *cfg;
  mutex_t                  lock;
  sem_t                    waitsem;
  struct wdog_s            waitwdog;
  sdio_eventset_t          waitevents;
  sdio_eventset_t          wkupevent;
  uint32_t                 waitints;     /* SMHC_INT_* mask serving waitevents */

  /* DMA descriptor ring + active transfer state (Task 3) */

  struct t113_smhc_desc_s *descs;        /* per-instance ring base */
  uint8_t                 *xfer_buffer;  /* in-flight DMA buffer    */
  size_t                   xfer_len;     /* in-flight DMA length    */
  bool                     xfer_recv;    /* true=RX, false=TX       */

  /* Media-change callback state (Task 4.2).  Populated by
   * t113_smhc_registercallback(); fired on hotplug via the HPWORK queue.
   */

  worker_t                 callback;     /* mmcsd_mediachange()        */
  void                    *cbarg;        /* mmcsd state pointer        */
  sdio_eventset_t          cbevents;     /* SDIOMEDIA_* arming bits    */
  struct work_s            cbwork;       /* HPWORK kicker              */
  sdio_statset_t           cdstatus;     /* last sampled CD status     */

  /* SDIO IO function IRQ dispatch (Task 5.3).  Index 0 is unused (CCCR
   * has no IRQ); 1..7 hold per-function callbacks installed by
   * t113_smhc_register_func_irq() and demux'd in t113_smhc_interrupt()
   * when SMHC_RINTSTS.SDIO_INT (bit 16) latches.
   */

  sdio_func_irq_t          func_irq[T113_SMHC_NUM_FUNCS];
  void                    *func_irq_arg[T113_SMHC_NUM_FUNCS];
  bool                     sdio_irq_enabled;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Mutual exclusion (if CONFIG_SDIO_MUXBUS) */

#ifdef CONFIG_SDIO_MUXBUS
static int            t113_smhc_lock(struct sdio_dev_s *dev, bool lock);
#endif

/* Initialization/setup */

static void           t113_smhc_reset(struct sdio_dev_s *dev);
static sdio_capset_t  t113_smhc_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t t113_smhc_status(struct sdio_dev_s *dev);
static void           t113_smhc_widebus(struct sdio_dev_s *dev, bool enable);
static void           t113_smhc_clock(struct sdio_dev_s *dev,
                                      enum sdio_clock_e rate);
static int            t113_smhc_attach(struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int            t113_smhc_sendcmd(struct sdio_dev_s *dev,
                                        uint32_t cmd, uint32_t arg);
static void           t113_smhc_blocksetup(struct sdio_dev_s *dev,
                                           unsigned int blocklen,
                                           unsigned int nblocks);
static int            t113_smhc_recvsetup(struct sdio_dev_s *dev,
                                          uint8_t *buffer, size_t nbytes);
static int            t113_smhc_sendsetup(struct sdio_dev_s *dev,
                                          const uint8_t *buffer,
                                          size_t nbytes);
static int            t113_smhc_cancel(struct sdio_dev_s *dev);

static int            t113_smhc_waitresponse(struct sdio_dev_s *dev,
                                             uint32_t cmd);
static int            t113_smhc_recv_r1(struct sdio_dev_s *dev, uint32_t cmd,
                                        uint32_t *r1);
static int            t113_smhc_recv_r2(struct sdio_dev_s *dev, uint32_t cmd,
                                        uint32_t r2[4]);
static int            t113_smhc_recv_r3(struct sdio_dev_s *dev, uint32_t cmd,
                                        uint32_t *r3);
static int            t113_smhc_recv_r4(struct sdio_dev_s *dev, uint32_t cmd,
                                        uint32_t *r4);
static int            t113_smhc_recv_r5(struct sdio_dev_s *dev, uint32_t cmd,
                                        uint32_t *r5);
static int            t113_smhc_recv_r6(struct sdio_dev_s *dev, uint32_t cmd,
                                        uint32_t *r6);
static int            t113_smhc_recv_r7(struct sdio_dev_s *dev, uint32_t cmd,
                                        uint32_t *r7);

/* Event/Callback support */

static void           t113_smhc_waitenable(struct sdio_dev_s *dev,
                                           sdio_eventset_t eventset,
                                           uint32_t timeout);
static sdio_eventset_t t113_smhc_eventwait(struct sdio_dev_s *dev);
static void           t113_smhc_callbackenable(struct sdio_dev_s *dev,
                                               sdio_eventset_t eventset);
static int            t113_smhc_registercallback(struct sdio_dev_s *dev,
                                                 worker_t callback,
                                                 void *arg);

/* DMA */

#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
static int            t113_smhc_dmapreflight(struct sdio_dev_s *dev,
                                             const uint8_t *buffer,
                                             size_t buflen);
#endif
static int            t113_smhc_dmarecvsetup(struct sdio_dev_s *dev,
                                             uint8_t *buffer, size_t buflen);
static int            t113_smhc_dmasendsetup(struct sdio_dev_s *dev,
                                             const uint8_t *buffer,
                                             size_t buflen);
#endif

/* Optional EXT-CSD hook */

static void           t113_smhc_gotextcsd(struct sdio_dev_s *dev,
                                          const uint8_t *buffer);

/* IRQ handler + watchdog timeout glue */

static int            t113_smhc_interrupt(int irq, void *context, void *arg);
static void           t113_smhc_eventtimeout(wdparm_t arg);
static void           t113_smhc_endwait(struct t113_smhc_dev_s *priv,
                                        sdio_eventset_t wkupevent);
static void           t113_smhc_callback(void *arg);

/* Private helper to wire up the sdio_dev_s method table at init time */

static void           t113_smhc_ops_init(struct sdio_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* IDMAC descriptor rings live in BSS aligned to a D-cache line (32 bytes on
 * Cortex-A7).  One ring per instance; sized by Kconfig.  Each descriptor is
 * 16 bytes, so up to 64 descs per ring keeps the per-bus arena <= 1 KiB.
 */

#ifdef CONFIG_T113_SMHC0
static const struct t113_smhc_config_s g_smhc0_config =
{
  .base       = T113_SMHC0_PADDR,
  .irq        = T113_IRQ_SMHC0,
  .bus        = 0,
  .pinset_clk = T113_SDC0_CLK,
  .pinset_cmd = T113_SDC0_CMD,
  .pinset_d   =
    {
      T113_SDC0_D0, T113_SDC0_D1, T113_SDC0_D2, T113_SDC0_D3
    },
  .cd_pin     = (int16_t)T113_SDC0_CD, /* PF6 input pull-up, active-low */
  .caps       = SDIO_CAPS_DMASUPPORTED | SDIO_CAPS_DMABEFOREWRITE |
                SDIO_CAPS_4BIT,
};

static struct t113_smhc_dev_s g_smhc0_dev;

static struct t113_smhc_desc_s
  g_smhc0_descs[CONFIG_T113_SMHC_DMA_DESC_COUNT]
  aligned_data(32);
#endif

#ifdef CONFIG_T113_SMHC1
static const struct t113_smhc_config_s g_smhc1_config =
{
  .base       = T113_SMHC1_PADDR,
  .irq        = T113_IRQ_SMHC1,
  .bus        = 1,
  .pinset_clk = T113_SDC1_CLK,
  .pinset_cmd = T113_SDC1_CMD,
  .pinset_d   =
    {
      T113_SDC1_D0, T113_SDC1_D1, T113_SDC1_D2, T113_SDC1_D3
    },
  .cd_pin     = -1,  /* SDIO WiFi: non-removable, no card-detect GPIO */
  .caps       = SDIO_CAPS_DMASUPPORTED | SDIO_CAPS_DMABEFOREWRITE |
                SDIO_CAPS_4BIT,
};

static struct t113_smhc_dev_s g_smhc1_dev;

static struct t113_smhc_desc_s
  g_smhc1_descs[CONFIG_T113_SMHC_DMA_DESC_COUNT]
  aligned_data(32);
#endif

#ifdef CONFIG_T113_SMHC2
/* NOTE: SMHC2 is unused on R528 HMI EVB4; kept here for completeness so
 * CONFIG_T113_SMHC2 builds cleanly on other boards.
 */

static const struct t113_smhc_config_s g_smhc2_config =
{
  .base       = T113_SMHC2_PADDR,
  .irq        = T113_IRQ_SMHC2,
  .bus        = 2,
  .pinset_clk = T113_SDC2_CLK,
  .pinset_cmd = T113_SDC2_CMD,
  .pinset_d   =
    {
      T113_SDC2_D0, T113_SDC2_D1, T113_SDC2_D2, T113_SDC2_D3
    },
  .cd_pin     = -1,  /* SMHC2 unwired on this board */
  .caps       = SDIO_CAPS_DMASUPPORTED | SDIO_CAPS_DMABEFOREWRITE |
                SDIO_CAPS_4BIT,
};

static struct t113_smhc_dev_s g_smhc2_dev;

static struct t113_smhc_desc_s
  g_smhc2_descs[CONFIG_T113_SMHC_DMA_DESC_COUNT]
  aligned_data(32);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: smhc_getreg / smhc_putreg
 *
 * Description:
 *   32-bit register access wrappers tied to the per-instance MMIO base.
 *
 ****************************************************************************/

static inline uint32_t smhc_getreg(struct t113_smhc_dev_s *priv,
                                   uint32_t offset)
{
  return getreg32(priv->cfg->base + offset);
}

static inline void smhc_putreg(struct t113_smhc_dev_s *priv,
                               uint32_t offset, uint32_t value)
{
  putreg32(value, priv->cfg->base + offset);
}

/****************************************************************************
 * Name: smhc_reset
 *
 * Description:
 *   Soft-reset the SMHC controller:
 *     1. Assert HW_RST | FIFO_RST | DMA_RST in SMHC_CTRL together with
 *        INT_ENB.  Poll until the three reset bits self-clear (timeout
 *        ~100us).
 *     2. Write 0xFFFFFFFF to SMHC_RINTSTS to W1C any latched events.
 *     3. Program SMHC_TMOUT to the default 0xFFFFFFFF (DTO=0x0FFFFFF,
 *        RTO=0xFF).
 *     4. Leave SMHC_INTMASK = 0; per-event masks are programmed by
 *        waitenable on a per-command basis.
 *
 ****************************************************************************/

static int smhc_reset(struct t113_smhc_dev_s *priv)
{
  uint32_t regval;
  int      retries;

  /* Disable any pending IRQ source masks before tearing down state. */

  smhc_putreg(priv, T113_SMHC_INTMASK_OFFSET, 0);

  /* Trigger HW + FIFO + DMA reset and pre-arm INT_ENB so the controller
   * latches IRQs once we mask them in.  Bits self-clear when the IP has
   * settled.
   */

  regval = T113_SMHC_CTRL_SOFT_RST | T113_SMHC_CTRL_FIFO_RST |
           T113_SMHC_CTRL_DMA_RST  | T113_SMHC_CTRL_INT_ENB;
  smhc_putreg(priv, T113_SMHC_CTRL_OFFSET, regval);

  for (retries = 1000; retries > 0; retries--)
    {
      regval = smhc_getreg(priv, T113_SMHC_CTRL_OFFSET);
      if ((regval & (T113_SMHC_CTRL_SOFT_RST |
                     T113_SMHC_CTRL_FIFO_RST |
                     T113_SMHC_CTRL_DMA_RST)) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (retries == 0)
    {
      mcerr("ERROR: SMHC%d reset timeout, CTRL=%08" PRIx32 "\n",
            priv->cfg->bus, regval);
      return -ETIMEDOUT;
    }

  /* Clear all latched raw interrupt status bits. */

  smhc_putreg(priv, T113_SMHC_RINTSTS_OFFSET, 0xffffffffu);
  smhc_putreg(priv, T113_SMHC_IDST_OFFSET,    0xffffffffu);

  /* Program response/data timeouts to the maximum the controller
   * supports: DTO=0x0FFFFFF (24 bits), RTO=0xFF (8 bits), encoded in
   * SMHC_TMOUT[31:8|7:0].
   */

  smhc_putreg(priv, T113_SMHC_TMOUT_OFFSET, 0xffffffffu);

  /* Bus-width back to 1-bit until widebus() promotes us. */

  smhc_putreg(priv, T113_SMHC_CTYPE_OFFSET, T113_SMHC_CTYPE_1BIT);

  /* INT_ENB is already set in the reset write above so the controller
   * will start asserting interrupts as soon as SMHC_INTMASK gates them
   * in.  Confirm by re-reading.
   */

  regval = smhc_getreg(priv, T113_SMHC_CTRL_OFFSET);
  if ((regval & T113_SMHC_CTRL_INT_ENB) == 0)
    {
      smhc_putreg(priv, T113_SMHC_CTRL_OFFSET,
                  regval | T113_SMHC_CTRL_INT_ENB);
    }

  mcinfo("SMHC%d: reset complete, RINT cleared, global IRQ enabled\n",
         priv->cfg->bus);
  return OK;
}

/****************************************************************************
 * Name: smhc_set_phase
 *
 * Description:
 *   Program DRV_DL (DAT/CMD output drive phase) and SAMP_DL (sample
 *   delay).  The Phase 1 speed envelope (DS / HS up to 50 MHz, SDR-only
 *   on 3.3 V) maps to a single combination -- 180 deg drive phase,
 *   sample delay 0 with SW_EN locked -- so we apply it unconditionally.
 *
 *   Called from both t113_smhc_initialize (so SELFTEST sees the same
 *   timing the production path uses) and t113_smhc_clock (so any
 *   subsequent rate change keeps the registers programmed).  Must run
 *   while CCLK is enabled and before the PRG_CLK sync pulse.
 *
 ****************************************************************************/

static void smhc_set_phase(struct t113_smhc_dev_s *priv)
{
  uint32_t regval;

  regval  = smhc_getreg(priv, T113_SMHC_DRV_DL_OFFSET);
  regval &= ~T113_SMHC_DRV_DL_PHASE_MASK;
  regval |= T113_SMHC_DRV_DL_PHASE_180;
  smhc_putreg(priv, T113_SMHC_DRV_DL_OFFSET, regval);

  regval  = smhc_getreg(priv, T113_SMHC_SAMP_DL_OFFSET);
  regval &= ~T113_SMHC_SAMP_DL_CFG_MASK;
  regval |= T113_SMHC_SAMP_DL_ENABLE;
  smhc_putreg(priv, T113_SMHC_SAMP_DL_OFFSET, regval);
}

/****************************************************************************
 * Name: smhc_dma_setup
 *
 * Description:
 *   Build the IDMAC descriptor chain covering [buffer, buffer+len) and
 *   program the controller for a single data-phase transfer.  Caller has
 *   already issued blocksetup() to set BLKSZ + BYTCNT and will issue
 *   sendcmd() with DATA_TRANS asserted right after.
 *
 *   Cache discipline (Cortex-A7 + PL310, 32-byte lines):
 *     - TX: clean buffer to point of coherency before handing to DMA so
 *       fresh writes are visible on the AXI side.
 *     - RX: invalidate buffer before DMA so any stale clean lines are
 *       dropped; CPU reads after completion will refill from RAM.
 *   Descriptor ring is itself in cacheable BSS so we clean it after
 *   building the chain (IDMAC fetches via AHB).
 *
 *   Layout of each descriptor:
 *     ctrl[31] OWN=1 (handed to DMA)
 *     ctrl[4]  CHAIN=1 (next field is link, not buffer)
 *     ctrl[3]  FIRST_FLAG on first
 *     ctrl[2]  LAST_FLAG  on last
 *     ctrl[1]  DIC=1 on non-last (only the last desc raises IRQ)
 *
 *   Buffer alignment: DES1[12:0] is mult-of-4, DES2 is word-aligned phys
 *   address.  Caller buffers are word-aligned (mmcsd block boundaries) and
 *   on T113 kernel virt == phys for DDR (no MMU re-mapping in sdcard
 *   defconfig).
 *
 *   For Phase-1 we limit to a single descriptor at 4 KiB max -- single-
 *   block (512 B) reads/writes only need one entry, multi-block (FAT
 *   cluster) up to 32 KiB needs <= 8 entries which still fits the
 *   16-entry default ring.
 *
 ****************************************************************************/

static int smhc_dma_setup(struct t113_smhc_dev_s *priv, uint8_t *buffer,
                          size_t len, bool is_recv)
{
  struct t113_smhc_desc_s *desc;
  size_t   chunk;
  size_t   remaining;
  uint32_t buf_phys;
  uint32_t ctrl;
  uint32_t regval;
  unsigned int idx;
  unsigned int total;
  int      retries;

  /* Inputs come from drivers/mmcsd in 512 B multiples; reject anything that
   * the IDMAC cannot DMA cleanly.
   */

  if (buffer == NULL || len == 0 || (len & 0x3u) != 0 ||
      ((uintptr_t)buffer & 0x3u) != 0)
    {
      return -EINVAL;
    }

  total = (len + T113_SMHC_DES1_SIZE_MAX - 1) / T113_SMHC_DES1_SIZE_MAX;
  if (total > CONFIG_T113_SMHC_DMA_DESC_COUNT)
    {
      mcerr("ERROR: SMHC%d xfer %zu B needs %u descs > %d\n",
            priv->cfg->bus, len, total,
            CONFIG_T113_SMHC_DMA_DESC_COUNT);
      return -E2BIG;
    }

  /* Stash the in-flight transfer so the IRQ handler / cancel path can do
   * the post-DMA invalidate (RX) and so cancel() can release the OWN bits.
   */

  priv->xfer_buffer = buffer;
  priv->xfer_len    = len;
  priv->xfer_recv   = is_recv;

  /* Cache pre-flight on the data buffer.  Order matters: RX must invalidate
   * BEFORE handing OWN to DMA so any dirty lines do not get evicted on top
   * of the DMA-deposited bytes.  TX must clean BEFORE handing OWN so the
   * IDMAC sees the latest data on the AHB side.
   */

  if (is_recv)
    {
      up_invalidate_dcache((uintptr_t)buffer, (uintptr_t)buffer + len);
    }
  else
    {
      up_clean_dcache((uintptr_t)buffer, (uintptr_t)buffer + len);
    }

  /* Build the descriptor chain.  T113 has linear DDR @ 0x40000000 with
   * kernel virt == phys (sdcard defconfig has no ARCH_ADDRENV); buffer
   * pointer is its own physical address.
   */

  buf_phys  = (uint32_t)(uintptr_t)buffer;
  remaining = len;

  for (idx = 0; idx < total; idx++)
    {
      desc = &priv->descs[idx];

      chunk = remaining < T113_SMHC_DES1_SIZE_MAX ?
              remaining : T113_SMHC_DES1_SIZE_MAX;

      ctrl = T113_SMHC_DES0_OWN | T113_SMHC_DES0_CHAIN;

      if (idx == 0)
        {
          ctrl |= T113_SMHC_DES0_FIRST;
        }

      if (idx == total - 1)
        {
          /* Last descriptor: terminate the ring, allow IRQ on
           * completion.  T113 IDMAC requires BOTH LD and ER on the
           * final descriptor in chain mode.
           */

          ctrl |= T113_SMHC_DES0_LAST | T113_SMHC_DES0_ER;
          desc->next = 0;
        }
      else
        {
          ctrl |= T113_SMHC_DES0_DIC;
          desc->next = SMHC_PHYS_TO_DESC_ADDR(&priv->descs[idx + 1]);
        }

      desc->buf_len  = chunk & T113_SMHC_DES1_SIZE_MASK;
      desc->buf_addr = SMHC_PHYS_TO_DESC_ADDR(buf_phys);
      desc->ctrl     = ctrl;

      buf_phys  += chunk;
      remaining -= chunk;
    }

  /* Clean the descriptor cache lines so the IDMAC fetches the values we
   * just wrote.  Span covers exactly the descs we touched.
   */

  up_clean_dcache((uintptr_t)priv->descs,
                  (uintptr_t)&priv->descs[total]);

  /* Engage IDMAC:
   *   1. CTRL.DMA_ENB=1
   *   2. CTRL.DMA_RST | FIFO_RST -> wait self-clear (drops stale FIFO data)
   *   3. IDMAC.SOFT_RST=1 -> wait self-clear
   *   4. IDMAC = FIX_BURST | ENB
   *   5. IDIE: enable RX/TX done + error/des-unavail summaries
   *   6. DLBA = phys(first descriptor)
   *   7. FIFOTH = (BSIZE_8 << 28) | (RX_TL=7 << 16) | TX_TL=248
   */

  regval  = smhc_getreg(priv, T113_SMHC_CTRL_OFFSET);
  regval |= T113_SMHC_CTRL_DMA_ENB;
  smhc_putreg(priv, T113_SMHC_CTRL_OFFSET, regval);

  regval |= T113_SMHC_CTRL_DMA_RST | T113_SMHC_CTRL_FIFO_RST;
  smhc_putreg(priv, T113_SMHC_CTRL_OFFSET, regval);

  for (retries = 1000; retries > 0; retries--)
    {
      regval = smhc_getreg(priv, T113_SMHC_CTRL_OFFSET);
      if ((regval & (T113_SMHC_CTRL_DMA_RST |
                     T113_SMHC_CTRL_FIFO_RST)) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (retries == 0)
    {
      mcerr("ERROR: SMHC%d FIFO/DMA reset timeout, CTRL=%08" PRIx32 "\n",
            priv->cfg->bus, regval);
      return -ETIMEDOUT;
    }

  smhc_putreg(priv, T113_SMHC_IDMAC_OFFSET, T113_SMHC_IDMAC_SOFT_RST);
  for (retries = 1000; retries > 0; retries--)
    {
      regval = smhc_getreg(priv, T113_SMHC_IDMAC_OFFSET);
      if ((regval & T113_SMHC_IDMAC_SOFT_RST) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (retries == 0)
    {
      mcerr("ERROR: SMHC%d IDMAC reset timeout, IDMAC=%08" PRIx32 "\n",
            priv->cfg->bus, regval);
      return -ETIMEDOUT;
    }

  /* Hand the IDMAC its descriptor list base address BEFORE flipping the
   * ENB bit.  T113 IDMAC starts a descriptor pre-fetch the moment ENB
   * goes high; with a stale DLBA it fetches a non-OWN descriptor and
   * raises DES_UNAVAIL before the data phase even begins (observed:
   * idst=0x10 with rint=0 immediately after blocksetup).
   */

  smhc_putreg(priv, T113_SMHC_DLBA_OFFSET,
              SMHC_PHYS_TO_DESC_ADDR(priv->descs));

  /* Clear stale IDST bits from any prior aborted transfer (W1C) before
   * unmasking IRQs.
   */

  smhc_putreg(priv, T113_SMHC_IDST_OFFSET, 0xffffffffu);

  /* Enable internal-DMA RX/TX done + error summaries.  Only the
   * direction-specific completion is unmasked (RX_INT for read, TX_INT
   * for write) to reduce IRQ pressure; the error sources stay gated
   * in for both directions.
   */

  regval = T113_SMHC_IDIE_FERR_INT_ENB | T113_SMHC_IDIE_DES_UNAVL_ENB |
           T113_SMHC_IDIE_ERR_SUM_ENB;
  if (is_recv)
    {
      regval |= T113_SMHC_IDIE_RX_INT_ENB;
    }
  else
    {
      regval |= T113_SMHC_IDIE_TX_INT_ENB;
    }

  smhc_putreg(priv, T113_SMHC_IDIE_OFFSET, regval);

  /* Now enable IDMAC with fixed-burst transfers.  DLBA + IDST + IDIE
   * are all in their correct steady state, so the first fetch lands on
   * a valid OWN=1 descriptor.
   */

  smhc_putreg(priv, T113_SMHC_IDMAC_OFFSET,
              T113_SMHC_IDMAC_FIX_BUST_CTL | T113_SMHC_IDMAC_ENB);

  /* Program FIFO water levels: BSIZE=8, RX_TL=7, TX_TL=248. */

  smhc_putreg(priv, T113_SMHC_FIFOTH_OFFSET,
              T113_SMHC_FIFOTH_BSIZE_8 |
              T113_SMHC_FIFOTH_RX_TL(7) |
              T113_SMHC_FIFOTH_TX_TL(248));

  return OK;
}

/****************************************************************************
 * Name: t113_smhc_endwait
 *
 * Description:
 *   Cancel the wait watchdog, disarm the IRQ-side mask, and post the
 *   waiting thread.  Always called from interrupt context (IRQ handler or
 *   watchdog callback).
 *
 ****************************************************************************/

static void t113_smhc_endwait(struct t113_smhc_dev_s *priv,
                              sdio_eventset_t wkupevent)
{
  wd_cancel(&priv->waitwdog);

  /* Mask IRQ sources we were waiting on so they cannot re-fire spuriously
   * before the next waitenable.  Preserve the SDIO IO IRQ bit so card-side
   * function IRQs continue to surface independently of command waits.
   */

  if (priv->waitints != 0)
    {
      smhc_putreg(priv, T113_SMHC_INTMASK_OFFSET,
                  priv->sdio_irq_enabled ? T113_SMHC_INT_SDIO : 0);
      priv->waitints = 0;
    }

  priv->wkupevent  = wkupevent;
  priv->waitevents = 0;

  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: t113_smhc_eventtimeout
 *
 * Description:
 *   Watchdog callback fired when waitenable's deadline expires before any
 *   IRQ-driven event lands.  Wakes the waiter with SDIOWAIT_TIMEOUT.
 *
 ****************************************************************************/

static void t113_smhc_eventtimeout(wdparm_t arg)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)arg;

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      mcerr("ERROR: SMHC%d wait timeout, waitevents=%02x\n",
            priv->cfg->bus, priv->waitevents);
      t113_smhc_endwait(priv, SDIOWAIT_TIMEOUT);
    }
}

/****************************************************************************
 * Name: t113_smhc_interrupt
 *
 * Description:
 *   SMHC GIC ISR.  Two distinct status registers are sampled per IRQ:
 *
 *     SMHC_RINTSTS -- command + data-side raw interrupts (CC, DTC, DCE,
 *                    DRTO, FU/FO, DEE, RTO, RCE, RE).
 *     SMHC_IDST    -- internal-DMA status (TX_INT, RX_INT, FATAL_BERR,
 *                    DES_UNAVAIL, ERR_FLAG_SUM, NIS, AIS).
 *
 *   Mapping to SDIOWAIT_* events:
 *
 *     CC                          -> CMDDONE | RESPONSEDONE
 *     DTC                         -> TRANSFERDONE
 *     IDST.TX_INT / IDST.RX_INT   -> TRANSFERDONE
 *     RTO                         -> TIMEOUT
 *     DRTO                        -> TIMEOUT
 *     RE / RCE / DCE / FU_FO /
 *       DEE / IDST.FATAL_BERR /
 *       IDST.DES_UNAVAIL          -> ERROR
 *
 *   We ack all RINT bits we sampled (W1C) regardless of whether the
 *   caller asked for them, so the next waitenable() begins on a clean
 *   register set; same for IDST bits we observed.  Any bits the caller
 *   did NOT arm are still ack'd to prevent latched state on the next
 *   round, matching how sam_sdmmc.c clears its IRQSTAT.
 *
 ****************************************************************************/

static int t113_smhc_interrupt(int irq, void *context, void *arg)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)arg;
  sdio_eventset_t  wkupevent = 0;
  uint32_t         rintsts;
  uint32_t         idst;
  uint32_t         pending;

  UNUSED(irq);
  UNUSED(context);

  rintsts = smhc_getreg(priv, T113_SMHC_RINTSTS_OFFSET);
  idst    = smhc_getreg(priv, T113_SMHC_IDST_OFFSET);

  if (rintsts == 0 && idst == 0)
    {
      return OK;
    }

  /* SDIO IO function IRQ dispatch.  The SMHC asserts a single SDIO_INT
   * bit aggregating all card-side function interrupts; the host has no
   * way to demux which function fired without reading CCCR.Int_Pending,
   * so we hand the event to function 1 (the canonical single-function
   * combo case) and let the consumer driver demux internally if needed.
   * Multi-function consumers that have registered other slots get fired
   * too -- mostly to give a future demux hook somewhere to land.  This
   * runs OUTSIDE the wkupevent path so a card IRQ raised while no
   * transfer is in flight still reaches its handler.
   */

  if ((rintsts & T113_SMHC_INT_SDIO) != 0)
    {
      unsigned int func;

      for (func = 1; func < T113_SMHC_NUM_FUNCS; func++)
        {
          if (priv->func_irq[func] != NULL)
            {
              priv->func_irq[func](priv->func_irq_arg[func]);
            }
        }
    }

  pending = rintsts & priv->waitints;

  /* Command-side translations ----------------------------------------- */

  if ((pending & T113_SMHC_INT_RTO) != 0)
    {
      wkupevent |= SDIOWAIT_TIMEOUT;
    }

  if ((pending & T113_SMHC_INT_DRTO) != 0)
    {
      wkupevent |= SDIOWAIT_TIMEOUT;
    }

  if ((pending & (T113_SMHC_INT_RE | T113_SMHC_INT_RCE | T113_SMHC_INT_DCE |
                  T113_SMHC_INT_FU_FO | T113_SMHC_INT_DEE)) != 0)
    {
      wkupevent |= SDIOWAIT_ERROR;
    }

  if ((pending & T113_SMHC_INT_CC) != 0)
    {
      /* CC fires after the response (or after the command line returns
       * to idle for no-response commands).  Map to both CMDDONE and
       * RESPONSEDONE so a caller waiting on either is satisfied.
       */

      wkupevent |= SDIOWAIT_RESPONSEDONE | SDIOWAIT_CMDDONE;
    }

  /* Data-side translation: DTC fires from RINT after the controller has
   * drained the FIFO; the IDMAC TX_INT/RX_INT fires after the descriptor
   * chain has fully transferred.  drivers/mmcsd waits on TRANSFERDONE so
   * either path satisfies it.
   */

  if ((pending & T113_SMHC_INT_DTC) != 0)
    {
      wkupevent |= SDIOWAIT_TRANSFERDONE;
    }

  /* IDMAC translations ------------------------------------------------- */

  if ((idst & (T113_SMHC_IDST_TX_INT | T113_SMHC_IDST_RX_INT)) != 0)
    {
      if ((priv->waitevents & SDIOWAIT_TRANSFERDONE) != 0)
        {
          wkupevent |= SDIOWAIT_TRANSFERDONE;
        }
    }

  if ((idst & (T113_SMHC_IDST_FATAL_BERR | T113_SMHC_IDST_DES_UNAVAIL |
               T113_SMHC_IDST_ERR_SUM)) != 0)
    {
      if ((priv->waitevents & SDIOWAIT_ERROR) != 0)
        {
          wkupevent |= SDIOWAIT_ERROR;
        }
    }

  /* Mask only the events the caller actually asked for so a stray DTC
   * during a command-only wait does not leak through.
   */

  wkupevent &= priv->waitevents;

  /* Post-flight RX dcache invalidate is deferred to t113_smhc_eventwait()
   * (caller / thread context) so this IRQ does not burn 200-500 us
   * walking a 256 KiB buffer.  priv->xfer_buffer/xfer_len/xfer_recv are
   * still valid until the next smhc_dma_setup().
   */

  /* W1C only the RINT bits the caller is actually waiting on.  Bits
   * outside priv->waitints (notably CC during a data transfer where
   * the wait is for TRANSFERDONE) must survive the IRQ so that the
   * subsequent t113_smhc_waitresponse busy-poll can observe them --
   * otherwise the poll burns its full 200 ms timeout per block and
   * mmcsd writes drop to ~14 KB/s.  IDST has no equivalent slow path,
   * so clear it fully.
   */

  smhc_putreg(priv, T113_SMHC_RINTSTS_OFFSET, rintsts & priv->waitints);
  smhc_putreg(priv, T113_SMHC_IDST_OFFSET, idst);

  if (wkupevent != 0)
    {
      t113_smhc_endwait(priv, wkupevent);
    }

  return OK;
}

/****************************************************************************
 * Name: t113_smhc_ops_init
 *
 * Description:
 *   Populate the sdio_dev_s method pointers for a single instance.
 *   NOTE: struct sdio_dev_s embeds its vtable inline (no separate ops
 *   struct), matching sam_sdmmc.c which initialises ops in the device
 *   aggregate.  Since we heap-zero the per-instance struct at init time we
 *   wire the callbacks up here instead of using a static const aggregate.
 *
 ****************************************************************************/

static void t113_smhc_ops_init(struct sdio_dev_s *dev)
{
#ifdef CONFIG_SDIO_MUXBUS
  dev->lock             = t113_smhc_lock;
#endif
  dev->reset            = t113_smhc_reset;
  dev->capabilities     = t113_smhc_capabilities;
  dev->status           = t113_smhc_status;
  dev->widebus          = t113_smhc_widebus;
  dev->clock            = t113_smhc_clock;
  dev->attach           = t113_smhc_attach;
  dev->sendcmd          = t113_smhc_sendcmd;
#ifdef CONFIG_SDIO_BLOCKSETUP
  dev->blocksetup       = t113_smhc_blocksetup;
#endif
  dev->recvsetup        = t113_smhc_recvsetup;
  dev->sendsetup        = t113_smhc_sendsetup;
  dev->cancel           = t113_smhc_cancel;
  dev->waitresponse     = t113_smhc_waitresponse;
  dev->recv_r1          = t113_smhc_recv_r1;
  dev->recv_r2          = t113_smhc_recv_r2;
  dev->recv_r3          = t113_smhc_recv_r3;
  dev->recv_r4          = t113_smhc_recv_r4;
  dev->recv_r5          = t113_smhc_recv_r5;
  dev->recv_r6          = t113_smhc_recv_r6;
  dev->recv_r7          = t113_smhc_recv_r7;
  dev->waitenable       = t113_smhc_waitenable;
  dev->eventwait        = t113_smhc_eventwait;
  dev->callbackenable   = t113_smhc_callbackenable;
  dev->registercallback = t113_smhc_registercallback;
#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
  dev->dmapreflight     = t113_smhc_dmapreflight;
#endif
  dev->dmarecvsetup     = t113_smhc_dmarecvsetup;
  dev->dmasendsetup     = t113_smhc_dmasendsetup;
#endif
  dev->gotextcsd        = t113_smhc_gotextcsd;
}

/****************************************************************************
 * Name: t113_smhc_lock
 ****************************************************************************/

#ifdef CONFIG_SDIO_MUXBUS
static int t113_smhc_lock(struct sdio_dev_s *dev, bool lock)
{
  /* T113 SMHC0/1/2 each drive an independent SD/SDIO bus.  No bus
   * sharing on this SoC, so MUXBUS lock is a no-op.
   */

  UNUSED(dev);
  UNUSED(lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: t113_smhc_reset
 ****************************************************************************/

static void t113_smhc_reset(struct sdio_dev_s *dev)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;

  smhc_reset(priv);
}

/****************************************************************************
 * Name: t113_smhc_capabilities
 *
 * Description:
 *   Report static host-controller capabilities to the mmcsd protocol layer.
 *   Set per-instance in cfg->caps so SMHC0/1/2 can advertise different
 *   feature sets without rebuilding the driver.
 *
 ****************************************************************************/

static sdio_capset_t t113_smhc_capabilities(struct sdio_dev_s *dev)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;

  return priv->cfg->caps;
}

/****************************************************************************
 * Name: t113_smhc_status
 *
 * Description:
 *   Sample the current card presence by reading the configured CD GPIO.
 *   Allwinner microSD sockets pull the CD line LOW when a card is fully
 *   seated in the slot (the carrier closes the switch to ground via a 1K
 *   pull-up to VCC-CARD).  Instances without a wired CD pin (cd_pin < 0,
 *   e.g. embedded SDIO WiFi) report present.
 *
 ****************************************************************************/

static sdio_statset_t t113_smhc_status(struct sdio_dev_s *dev)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  sdio_statset_t status = 0;

  if (priv->cfg->cd_pin < 0)
    {
      /* No CD GPIO -- caller treats the slot as always-present (eMMC,
       * embedded SDIO).
       */

      status = SDIO_STATUS_PRESENT;
    }
  else if (!t113_gpio_read((uint16_t)priv->cfg->cd_pin))
    {
      status = SDIO_STATUS_PRESENT;
    }

  priv->cdstatus = status;
  return status;
}

/****************************************************************************
 * Name: t113_smhc_widebus
 *
 * Description:
 *   Program SMHC_CTYPE for the requested bus width.  SD cards switch from
 *   1-bit (post-reset) to 4-bit during initialisation once ACMD6 has
 *   succeeded; mmcsd_widebus() then calls SDIO_WIDEBUS(true).
 *
 ****************************************************************************/

static void t113_smhc_widebus(struct sdio_dev_s *dev, bool enable)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;

  smhc_putreg(priv, T113_SMHC_CTYPE_OFFSET,
              enable ? T113_SMHC_CTYPE_4BIT : T113_SMHC_CTYPE_1BIT);

  mcinfo("SMHC%d: bus width = %d-bit\n",
         priv->cfg->bus, enable ? 4 : 1);
}

/****************************************************************************
 * Name: smhc_update_clock
 *
 * Description:
 *   Issue a SMHC PRG_CLK synchronisation pulse so the controller latches
 *   any pending divider changes onto the SD bus.  Sequence: assert
 *   MASK_DATA0, load CMD = CMD_LOAD | PRG_CLK | WAIT_PRE_OVER, poll for
 *   CMD_LOAD to self-clear (controller-side acknowledgement), drop
 *   MASK_DATA0.  RINT bits set during the pulse are W1C-cleared so the
 *   next real command sees a clean status register.
 *
 *   Returns OK on success or -ETIMEDOUT if CMD_LOAD does not self-clear
 *   in 1 ms (an order of magnitude past the controller's worst case).
 *
 ****************************************************************************/

static int smhc_update_clock(struct t113_smhc_dev_s *priv)
{
  uint32_t regval;
  int      retries;

  /* MASK_DATA0 prevents a glitch on D0 from being interpreted as a busy
   * indication while we issue the sync pulse.
   */

  regval  = smhc_getreg(priv, T113_SMHC_CLKDIV_OFFSET);
  regval |= T113_SMHC_CLKDIV_MASK_DATA0;
  smhc_putreg(priv, T113_SMHC_CLKDIV_OFFSET, regval);

  /* Issue the sync pulse.  CMD_LOAD self-clears once the controller has
   * applied the divider.
   */

  smhc_putreg(priv, T113_SMHC_CMD_OFFSET,
              T113_SMHC_CMD_CMD_LOAD | T113_SMHC_CMD_PRG_CLK |
              T113_SMHC_CMD_WAIT_PRE_OVER);

  for (retries = 1000; retries > 0; retries--)
    {
      regval = smhc_getreg(priv, T113_SMHC_CMD_OFFSET);
      if ((regval & T113_SMHC_CMD_CMD_LOAD) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (retries == 0)
    {
      mcerr("ERROR: SMHC%d PRG_CLK pulse timeout, CMD=%08" PRIx32 "\n",
            priv->cfg->bus, regval);
      return -ETIMEDOUT;
    }

  /* Clear any RINT bits the pulse may have raised so a subsequent real
   * waitenable starts on a clean slate.
   */

  smhc_putreg(priv, T113_SMHC_RINTSTS_OFFSET, 0xffffffffu);

  /* Drop MASK_DATA0; D0 returns to its normal busy-detect role. */

  regval  = smhc_getreg(priv, T113_SMHC_CLKDIV_OFFSET);
  regval &= ~T113_SMHC_CLKDIV_MASK_DATA0;
  smhc_putreg(priv, T113_SMHC_CLKDIV_OFFSET, regval);

  return OK;
}

/****************************************************************************
 * Name: t113_smhc_clock
 *
 * Description:
 *   Reprogram the SDCLK rate.  drivers/mmcsd calls this at four points:
 *     CLOCK_IDMODE            -- 400 kHz init clock (CMD0 / CMD8 / ACMD41)
 *     CLOCK_MMC_TRANSFER      -- 25 MHz default-speed for eMMC
 *     CLOCK_SD_TRANSFER_1BIT  -- 25 MHz default-speed, 1-bit
 *     CLOCK_SD_TRANSFER_4BIT  -- 25 MHz default-speed, 4-bit
 *
 *   Sequence:
 *     1. Gate CCLK off (clear CCLK_ENB) so the SoC clock tree can be
 *        retuned without glitches on the SD bus.
 *     2. Reprogram the SMHCn module clock via t113_smhc_clk_enable.
 *     3. Re-enable CCLK and apply the SoC-internal divider in CLKDIV
 *        (we leave it at 0 since the entire divider already lives in
 *        the CCU CLK_REG; CLKDIV.DIV is for the optional fine divider
 *        on DDR-mode parts which T113 does not need at default speed).
 *     4. Send the PRG_CLK sync pulse so the new clock propagates.
 *
 ****************************************************************************/

static void t113_smhc_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  uint32_t freq_hz;
  uint32_t regval;
  int ret;

  switch (rate)
    {
      case CLOCK_SDIO_DISABLED:

        /* mmcsd_sdio.c calls SDIO_CLOCK(CLOCK_SDIO_DISABLED) before
         * retuning to gate the SD bus clock off cleanly.  Clear
         * CCLK_ENB and sync the change, then return without
         * re-enabling.
         */

        regval  = smhc_getreg(priv, T113_SMHC_CLKDIV_OFFSET);
        regval &= ~T113_SMHC_CLKDIV_CCLK_ENB;
        smhc_putreg(priv, T113_SMHC_CLKDIV_OFFSET, regval);
        smhc_update_clock(priv);
        mcinfo("SMHC%d: SDCLK disabled\n", priv->cfg->bus);
        return;

      case CLOCK_IDMODE:
        freq_hz = 400000u;
        break;

      case CLOCK_MMC_TRANSFER:
        freq_hz = 25000000u;
        break;

      case CLOCK_SD_TRANSFER_1BIT:
      case CLOCK_SD_TRANSFER_4BIT:

        /* SD data-phase SDCLK comes from CONFIG_T113_SMHC_SDCLK_HZ.
         * Default 50 MHz is HS-SDR25 in-spec at 3.3V.  Higher values
         * overclock out-of-spec; see Kconfig help.
         */

        freq_hz = CONFIG_T113_SMHC_SDCLK_HZ;
        break;

      default:
        mcerr("ERROR: unknown rate %d\n", (int)rate);
        return;
    }

  /* Step 1: gate CCLK while we retune. */

  regval  = smhc_getreg(priv, T113_SMHC_CLKDIV_OFFSET);
  regval &= ~T113_SMHC_CLKDIV_CCLK_ENB;
  smhc_putreg(priv, T113_SMHC_CLKDIV_OFFSET, regval);

  ret = smhc_update_clock(priv);
  if (ret < 0)
    {
      mcerr("ERROR: SMHC%d CCLK gate sync failed: %d\n",
            priv->cfg->bus, ret);
      return;
    }

  /* Step 2: reprogram the SMHCn module clock.  This drives bus gate +
   * reset + parent + divider in one shot (idempotent across re-entry).
   */

  ret = t113_smhc_clk_enable(priv->cfg->bus, freq_hz);
  if (ret < 0)
    {
      mcerr("ERROR: SMHC%d clk_enable(%lu) failed: %d\n",
            priv->cfg->bus, (unsigned long)freq_hz, ret);
      return;
    }

  /* Step 3: re-enable CCLK with CLKDIV.DIV=0 (use the CCU divider only). */

  smhc_putreg(priv, T113_SMHC_CLKDIV_OFFSET, T113_SMHC_CLKDIV_CCLK_ENB);

  /* Step 3a: program output drive phase + sample delay for the new
   * clock.  Phase table:
   *
   *   DS / HSSDR  (any rate)       odly = 180 deg (1)   sdly = 0  SW_EN
   *   HSDDR 4-bit (50 MHz)         odly =  90 deg (0)   sdly = 0xe
   *   HS200 / SDR104 (>=50 MHz)    odly =  90 deg (0)   sdly varies
   *
   * We currently only enter SDR HS up to 50 MHz so 180 deg + sdly=0 is
   * the one combination that matters.  Without these writes, the chip
   * uses default-reset DAT_PH=0/SAMP_EN=0 path, which adds ~40 ns/byte
   * of fixed inter-block stall (SMHC_LEADS L9: SDCLK doubling only
   * yields 1.4x throughput).
   */

  smhc_set_phase(priv);

  /* Step 4: propagate via PRG_CLK pulse so the card sees the new clock. */

  ret = smhc_update_clock(priv);
  if (ret < 0)
    {
      mcerr("ERROR: SMHC%d PRG_CLK pulse failed: %d\n",
            priv->cfg->bus, ret);
      return;
    }

  mcinfo("SMHC%d: SDCLK = %lu Hz\n",
         priv->cfg->bus, (unsigned long)freq_hz);
}

/****************************************************************************
 * Name: t113_smhc_attach
 ****************************************************************************/

static int t113_smhc_attach(struct sdio_dev_s *dev)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  int ret;

  ret = irq_attach(priv->cfg->irq, t113_smhc_interrupt, priv);
  if (ret < 0)
    {
      mcerr("ERROR: irq_attach(%d) failed: %d\n", priv->cfg->irq, ret);
      return ret;
    }

  up_enable_irq(priv->cfg->irq);
  return OK;
}

/****************************************************************************
 * Name: t113_smhc_sendcmd
 *
 * Description:
 *   Encode the NuttX cmd word into SMHC_CMD register bits, write SMHC_CMDARG
 *   and trigger the controller via CMD_LOAD.  Handles command-only,
 *   single-block and multi-block data transfers (CMD17/CMD18/CMD24/CMD25)
 *   plus the explicit STOP_TRANSMISSION (CMD12).
 *
 *   AUTO_STOP policy: NuttX drivers/mmcsd/mmcsd_sdio.c always issues CMD12
 *   explicitly through mmcsd_stoptransmission() after multi-block reads /
 *   writes (see mmcsd_sdio.c:1820 and :2256).  We therefore never set the
 *   SMHC_CMD AUTO_STOP bit (bit 12) for CMD18/CMD25.  For CMD12 itself,
 *   the NuttX MMCSD_STOPXFR flag maps to STOP_ABT_CMD (bit 14) which
 *   tells the controller "this command aborts the in-flight data
 *   transfer".
 *
 *   We also program SMHC_A12A=0xffff to disable the auto-CMD12
 *   sequencer; if the auto path were inadvertently armed via a stale
 *   register write the controller would issue a phantom CMD12.
 *
 *   Mapping is patterned after sam_sdmmc.c::sam_sendcmd.
 *
 ****************************************************************************/

static int t113_smhc_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t arg)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  uint32_t cmdidx;
  uint32_t regval;

  cmdidx = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval = T113_SMHC_CMD_IDX(cmdidx) | T113_SMHC_CMD_CMD_LOAD |
           T113_SMHC_CMD_WAIT_PRE_OVER;

  /* Response shaping.  R1/R1b/R5/R6 are short with CRC; R2 is long with
   * CRC; R3/R4/R7 are short, R3/R4 carry no valid CRC (OCR responses).
   * For R7 the SD spec defines CRC validity; we leave CHK_RESP_CRC
   * asserted to match SAMA5 behaviour.
   */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
      case MMCSD_NO_RESPONSE:
        break;

      case MMCSD_R1_RESPONSE:
      case MMCSD_R1B_RESPONSE:
      case MMCSD_R5_RESPONSE:
      case MMCSD_R6_RESPONSE:
      case MMCSD_R7_RESPONSE:
        regval |= T113_SMHC_CMD_RESP_RCV | T113_SMHC_CMD_CHK_RESP_CRC;
        break;

      case MMCSD_R2_RESPONSE:
        regval |= T113_SMHC_CMD_RESP_RCV | T113_SMHC_CMD_LONG_RESP |
                  T113_SMHC_CMD_CHK_RESP_CRC;
        break;

      case MMCSD_R3_RESPONSE:
      case MMCSD_R4_RESPONSE:
        regval |= T113_SMHC_CMD_RESP_RCV;
        break;

      default:
        mcerr("ERROR: bad response type cmd=%08" PRIx32 "\n", cmd);
        return -EINVAL;
    }

  /* Data phase encoding.  CMD17/CMD18/CMD24/CMD25 (and SDIO data CMD53)
   * carry MMCSD_DATAXFR; multi-block CMD18/CMD25 additionally carry
   * MMCSD_MULTIBLOCK but the controller does not need a separate bit for
   * that -- BLKSIZ + BYTCNT (programmed by t113_smhc_blocksetup) tell the
   * IP how many bytes to move.  TRANS_WRITE selects host->card direction.
   */

  if ((cmd & MMCSD_DATAXFR_MASK) != MMCSD_NODATAXFR)
    {
      regval |= T113_SMHC_CMD_DATA_TRANS;
      if ((cmd & MMCSD_WRXFR) != 0)
        {
          regval |= T113_SMHC_CMD_TRANS_WRITE;
        }
    }

  /* MMCSD_STOPXFR is set on CMD12 STOP_TRANSMISSION (and SD_ACMD52ABRT).
   * That's the "abort the in-flight data transfer" semantic which maps to
   * SMHC_CMD.STOP_ABT_CMD bit 14, NOT to the auto-CMD12 bit 12.
   */

  if ((cmd & MMCSD_STOPXFR) != 0)
    {
      regval |= T113_SMHC_CMD_STOP_ABT_CMD;
    }

  /* Clear stale RINT bits that match the events we are about to wait on
   * so a previous CMD's interrupt does not race.
   */

  smhc_putreg(priv, T113_SMHC_RINTSTS_OFFSET,
              T113_SMHC_INT_CC | T113_SMHC_INT_RE |
              T113_SMHC_INT_RCE | T113_SMHC_INT_RTO);

  /* Disable the auto-CMD12 sequencer (we issue CMD12 explicitly). */

  smhc_putreg(priv, T113_SMHC_A12A_OFFSET, 0xffffu);

  smhc_putreg(priv, T113_SMHC_CMDARG_OFFSET, arg);
  smhc_putreg(priv, T113_SMHC_CMD_OFFSET, regval);
  return OK;
}

/****************************************************************************
 * Name: t113_smhc_blocksetup
 *
 * Description:
 *   Program SMHC_BLKSIZ + SMHC_BYTCNT for the upcoming data-phase
 *   transfer.  drivers/mmcsd calls this before each recvsetup /
 *   sendsetup pair (it is mandatory under CONFIG_SDIO_BLOCKSETUP).
 *
 ****************************************************************************/

static void t113_smhc_blocksetup(struct sdio_dev_s *dev,
                                 unsigned int blocklen,
                                 unsigned int nblocks)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;

  smhc_putreg(priv, T113_SMHC_BLKSIZ_OFFSET,
              blocklen & T113_SMHC_BLKSIZ_MASK);
  smhc_putreg(priv, T113_SMHC_BYTCNT_OFFSET,
              (uint32_t)blocklen * (uint32_t)nblocks);
}

/****************************************************************************
 * Name: t113_smhc_recvsetup
 *
 * Description:
 *   Configure the data path for an upcoming card-to-host transfer.
 *   T113 SMHC is DMA-only (no PIO data path is wired up in P1), so this
 *   is an alias for the DMA receive setup.  Keeping the dual entry
 *   point lets drivers/mmcsd dispatch through whichever vtable slot
 *   matches its CONFIG_SDIO_DMA build state.
 *
 ****************************************************************************/

static int t113_smhc_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                               size_t nbytes)
{
  return t113_smhc_dmarecvsetup(dev, buffer, nbytes);
}

/****************************************************************************
 * Name: t113_smhc_sendsetup
 *
 * Description:
 *   Configure the data path for an upcoming host-to-card transfer.
 *   See t113_smhc_recvsetup() above; T113 always takes the DMA path.
 *
 ****************************************************************************/

static int t113_smhc_sendsetup(struct sdio_dev_s *dev,
                               const uint8_t *buffer, size_t nbytes)
{
  return t113_smhc_dmasendsetup(dev, buffer, nbytes);
}

/****************************************************************************
 * Name: t113_smhc_cancel
 *
 * Description:
 *   Abort any in-flight DMA transfer and post SDIOWAIT_ERROR to a
 *   waiter:
 *     1. Disable IDIE.
 *     2. SOFT_RST IDMAC + clear DMAC.
 *     3. Reset DMA + FIFO via CTRL.
 *     4. Drop OWN bits in our descriptor ring so a stray refetch
 *        immediately surfaces DES_UNAVAIL (defensive).
 *     5. Wake any pending eventwait so the upper layer fails fast.
 *
 ****************************************************************************/

static int t113_smhc_cancel(struct sdio_dev_s *dev)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  irqstate_t flags;
  uint32_t   regval;
  unsigned int i;

  flags = enter_critical_section();

  /* Disable IDMAC interrupt sources, soft-reset and disable IDMAC. */

  smhc_putreg(priv, T113_SMHC_IDIE_OFFSET, 0);
  smhc_putreg(priv, T113_SMHC_IDMAC_OFFSET, T113_SMHC_IDMAC_SOFT_RST);
  smhc_putreg(priv, T113_SMHC_IDMAC_OFFSET, 0);

  /* Reset DMA and FIFO via CTRL.  Bits self-clear; we tolerate a busy
   * controller silently here (cancel is best-effort).
   */

  regval  = smhc_getreg(priv, T113_SMHC_CTRL_OFFSET);
  regval |= T113_SMHC_CTRL_DMA_RST | T113_SMHC_CTRL_FIFO_RST;
  smhc_putreg(priv, T113_SMHC_CTRL_OFFSET, regval);

  /* Drop OWN bits in the ring so the IDMAC cannot replay the chain on
   * the next descriptor fetch.
   */

  if (priv->descs != NULL)
    {
      for (i = 0; i < CONFIG_T113_SMHC_DMA_DESC_COUNT; i++)
        {
          priv->descs[i].ctrl &= ~T113_SMHC_DES0_OWN;
        }

      up_clean_dcache((uintptr_t)priv->descs,
                      (uintptr_t)&priv->descs[
                        CONFIG_T113_SMHC_DMA_DESC_COUNT]);
    }

  /* Clear pending IDST + RINT bits so the next eventwait starts fresh. */

  smhc_putreg(priv, T113_SMHC_IDST_OFFSET, 0xffffffffu);
  smhc_putreg(priv, T113_SMHC_RINTSTS_OFFSET, 0xffffffffu);

  priv->xfer_buffer = NULL;
  priv->xfer_len    = 0;

  /* Wake any pending waiter with ERROR.  endwait clears waitints/wkupevent
   * book-keeping; no-op if nothing is armed.
   */

  if (priv->waitevents != 0)
    {
      t113_smhc_endwait(priv, SDIOWAIT_ERROR);
    }

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: t113_smhc_waitresponse
 *
 * Description:
 *   Poll the raw interrupt-status register for the response-complete bit
 *   (or an error bit) corresponding to the last issued command.  Mirrors
 *   sam_waitresponse() shape: busy-poll until CC fires, then map error
 *   bits to errno.  Caller-driven recv_rN() reads RESP0..3 afterwards.
 *
 *   For commands without a response (CMD0, CMD4, CMD15), we still wait on
 *   CC since the controller asserts it once the command line returns to
 *   idle.
 *
 ****************************************************************************/

static int t113_smhc_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  clock_t   start;
  clock_t   elapsed;
  clock_t   timeout;
  uint32_t  rintsts;
  uint32_t  errmask;
  int       ret = OK;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
      case MMCSD_NO_RESPONSE:
        timeout = MSEC2TICK(50);
        errmask = 0;
        break;

      case MMCSD_R1_RESPONSE:
      case MMCSD_R1B_RESPONSE:
      case MMCSD_R2_RESPONSE:
      case MMCSD_R5_RESPONSE:
      case MMCSD_R6_RESPONSE:
        timeout = MSEC2TICK(200);
        errmask = T113_SMHC_INT_RTO | T113_SMHC_INT_RCE | T113_SMHC_INT_RE;
        break;

      case MMCSD_R3_RESPONSE:
      case MMCSD_R4_RESPONSE:
      case MMCSD_R7_RESPONSE:
        /* OCR / SDIO-OCR responses carry no valid CRC; only timeout is
         * fatal.  CMD8 (R7) is technically CRC-checked but ACMD41 (R3)
         * is not, and we let recv_r* discriminate.
         */

        timeout = MSEC2TICK(200);
        errmask = T113_SMHC_INT_RTO | T113_SMHC_INT_RE;
        break;

      default:
        return -EINVAL;
    }

  start = clock_systime_ticks();
  for (; ; )
    {
      rintsts = smhc_getreg(priv, T113_SMHC_RINTSTS_OFFSET);
      if ((rintsts & errmask) != 0)
        {
          mcerr("ERROR: cmd=%08" PRIx32 " RINT=%08" PRIx32 "\n",
                cmd, rintsts);
          ret = (rintsts & T113_SMHC_INT_RTO) ? -ETIMEDOUT : -EIO;
          break;
        }

      if ((rintsts & T113_SMHC_INT_CC) != 0)
        {
          break;
        }

      elapsed = clock_systime_ticks() - start;
      if (elapsed >= timeout)
        {
          mcerr("ERROR: cmd=%08" PRIx32 " timeout RINT=%08" PRIx32 "\n",
                cmd, rintsts);
          ret = -ETIMEDOUT;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: t113_smhc_check_short
 *
 * Description:
 *   Common error decoding for short (48-bit) responses.  Returns OK when
 *   the latched RINT bits indicate clean reception, otherwise maps the
 *   first-set error to errno.  Caller is responsible for clearing the
 *   handled error/CC bits in RINT.
 *
 ****************************************************************************/

static int t113_smhc_check_short(struct t113_smhc_dev_s *priv,
                                 bool check_crc)
{
  uint32_t rintsts = smhc_getreg(priv, T113_SMHC_RINTSTS_OFFSET);

  if ((rintsts & T113_SMHC_INT_RTO) != 0)
    {
      mcerr("ERROR: response timeout RINT=%08" PRIx32 "\n", rintsts);
      return -ETIMEDOUT;
    }

  if (check_crc && (rintsts & T113_SMHC_INT_RCE) != 0)
    {
      mcerr("ERROR: response CRC error RINT=%08" PRIx32 "\n", rintsts);
      return -EIO;
    }

  if ((rintsts & T113_SMHC_INT_RE) != 0)
    {
      mcerr("ERROR: response error RINT=%08" PRIx32 "\n", rintsts);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: t113_smhc_recv_r1
 *
 * Description:
 *   R1 / R1B 48-bit response.  Card status returned in RESP0 by the
 *   Allwinner SMHC (raw 32 bits of the [39:8] field after the controller
 *   strips the start, transmission and CRC7 wrappers).
 *
 ****************************************************************************/

static int t113_smhc_recv_r1(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t *r1)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (r1 == NULL)
    {
      return -EINVAL;
    }

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1B_RESPONSE)
    {
      mcerr("ERROR: wrong response type cmd=%08" PRIx32 "\n", cmd);
      return -EINVAL;
    }
#endif

  ret = t113_smhc_check_short(priv, true);
  *r1 = smhc_getreg(priv, T113_SMHC_RESP0_OFFSET);
  return ret;
}

/****************************************************************************
 * Name: t113_smhc_recv_r2
 *
 * Description:
 *   R2 long (136-bit) response: CID / CSD register.  The controller
 *   stores the 127:0 payload directly in RESP0..3, with the MSW
 *   (bits 127:96) in RESP3.
 *
 ****************************************************************************/

static int t113_smhc_recv_r2(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t r2[4])
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (r2 == NULL)
    {
      return -EINVAL;
    }

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R2_RESPONSE)
    {
      mcerr("ERROR: wrong response type cmd=%08" PRIx32 "\n", cmd);
      return -EINVAL;
    }
#endif

  ret = t113_smhc_check_short(priv, true);

  r2[0] = smhc_getreg(priv, T113_SMHC_RESP3_OFFSET);
  r2[1] = smhc_getreg(priv, T113_SMHC_RESP2_OFFSET);
  r2[2] = smhc_getreg(priv, T113_SMHC_RESP1_OFFSET);
  r2[3] = smhc_getreg(priv, T113_SMHC_RESP0_OFFSET);
  return ret;
}

/****************************************************************************
 * Name: t113_smhc_recv_r3
 *
 * Description:
 *   R3 OCR response (no CRC).  Returned in RESP0.
 *
 ****************************************************************************/

static int t113_smhc_recv_r3(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t *r3)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (r3 == NULL)
    {
      return -EINVAL;
    }

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE)
    {
      mcerr("ERROR: wrong response type cmd=%08" PRIx32 "\n", cmd);
      return -EINVAL;
    }
#endif

  ret = t113_smhc_check_short(priv, false);
  *r3 = smhc_getreg(priv, T113_SMHC_RESP0_OFFSET);
  return ret;
}

/****************************************************************************
 * Name: t113_smhc_recv_r4
 *
 * Description:
 *   R4 SDIO OCR response (no CRC).  Returned in RESP0.
 *
 ****************************************************************************/

static int t113_smhc_recv_r4(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t *r4)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (r4 == NULL)
    {
      return -EINVAL;
    }

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R4_RESPONSE)
    {
      mcerr("ERROR: wrong response type cmd=%08" PRIx32 "\n", cmd);
      return -EINVAL;
    }
#endif

  ret = t113_smhc_check_short(priv, false);
  *r4 = smhc_getreg(priv, T113_SMHC_RESP0_OFFSET);
  return ret;
}

/****************************************************************************
 * Name: t113_smhc_recv_r5
 *
 * Description:
 *   R5 SDIO CMD52/53 response.  CRC checked.  Returned in RESP0.
 *
 ****************************************************************************/

static int t113_smhc_recv_r5(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t *r5)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (r5 == NULL)
    {
      return -EINVAL;
    }

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R5_RESPONSE)
    {
      mcerr("ERROR: wrong response type cmd=%08" PRIx32 "\n", cmd);
      return -EINVAL;
    }
#endif

  ret = t113_smhc_check_short(priv, true);
  *r5 = smhc_getreg(priv, T113_SMHC_RESP0_OFFSET);
  return ret;
}

/****************************************************************************
 * Name: t113_smhc_recv_r6
 *
 * Description:
 *   R6 published-RCA response (CRC-checked).  Returned in RESP0.
 *
 ****************************************************************************/

static int t113_smhc_recv_r6(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t *r6)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (r6 == NULL)
    {
      return -EINVAL;
    }

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R6_RESPONSE)
    {
      mcerr("ERROR: wrong response type cmd=%08" PRIx32 "\n", cmd);
      return -EINVAL;
    }
#endif

  ret = t113_smhc_check_short(priv, true);
  *r6 = smhc_getreg(priv, T113_SMHC_RESP0_OFFSET);
  return ret;
}

/****************************************************************************
 * Name: t113_smhc_recv_r7
 *
 * Description:
 *   R7 CMD8 voltage echo response (CRC-checked).  Returned in RESP0.
 *
 ****************************************************************************/

static int t113_smhc_recv_r7(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t *r7)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (r7 == NULL)
    {
      return -EINVAL;
    }

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      mcerr("ERROR: wrong response type cmd=%08" PRIx32 "\n", cmd);
      return -EINVAL;
    }
#endif

  ret = t113_smhc_check_short(priv, true);
  *r7 = smhc_getreg(priv, T113_SMHC_RESP0_OFFSET);
  return ret;
}

/****************************************************************************
 * Name: t113_smhc_waitenable
 ****************************************************************************/

static void t113_smhc_waitenable(struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset, uint32_t timeout)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  uint32_t mask = 0;

  /* Drop any leftover IRQ mask from a previous wait, but preserve the
   * SDIO IO IRQ bit if a Phase-2 consumer has armed card-IRQ delivery.
   */

  smhc_putreg(priv, T113_SMHC_INTMASK_OFFSET,
              priv->sdio_irq_enabled ? T113_SMHC_INT_SDIO : 0);

  /* Drop any stale RINT/IDST status from a prior transfer so the
   * waitresponse busy-poll cannot be tricked by a leftover CC.
   */

  smhc_putreg(priv, T113_SMHC_RINTSTS_OFFSET, 0xffffffffu);
  smhc_putreg(priv, T113_SMHC_IDST_OFFSET, 0xffffffffu);

  priv->waitints   = 0;
  priv->wkupevent  = 0;
  priv->waitevents = eventset;

  /* Translate eventset -> SMHC INTMASK bits.  CMDDONE / RESPONSEDONE both
   * map to CC; the timeout-on-response (RTO) is always armed when the
   * caller cares about either response or timeout, so we can map an RTO
   * IRQ into either SDIOWAIT_TIMEOUT or SDIOWAIT_ERROR depending on what
   * the caller asked for.
   */

  if ((eventset & (SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE)) != 0)
    {
      mask |= T113_SMHC_INT_CC | T113_SMHC_INT_RTO | T113_SMHC_INT_RCE |
              T113_SMHC_INT_RE;
    }

  if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
    {
      /* Data-side IRQs.  Wired now so the bits are not lost; the IRQ
       * handler maps them in Task 3 once the DMA path lands.
       */

      mask |= T113_SMHC_INT_DTC | T113_SMHC_INT_DCE | T113_SMHC_INT_DRTO |
              T113_SMHC_INT_FU_FO | T113_SMHC_INT_DEE;
    }

  if ((eventset & SDIOWAIT_TIMEOUT) != 0)
    {
      mask |= T113_SMHC_INT_RTO | T113_SMHC_INT_DRTO;
    }

  if ((eventset & SDIOWAIT_ERROR) != 0)
    {
      mask |= T113_SMHC_INT_RE | T113_SMHC_INT_RCE | T113_SMHC_INT_DCE |
              T113_SMHC_INT_FU_FO | T113_SMHC_INT_DEE;
    }

  priv->waitints = mask;
  if (priv->sdio_irq_enabled)
    {
      mask |= T113_SMHC_INT_SDIO;
    }

  smhc_putreg(priv, T113_SMHC_INTMASK_OFFSET, mask);

  /* Software watchdog backstop.  Mirrors sam_waitenable: a zero timeout
   * with SDIOWAIT_TIMEOUT armed counts as already-fired.
   */

  if ((eventset & SDIOWAIT_TIMEOUT) != 0)
    {
      if (timeout == 0)
        {
          priv->wkupevent  = SDIOWAIT_TIMEOUT;
          priv->waitevents = 0;
          return;
        }

      wd_start(&priv->waitwdog, MSEC2TICK(timeout),
               t113_smhc_eventtimeout, (wdparm_t)priv);
    }
}

/****************************************************************************
 * Name: t113_smhc_eventwait
 ****************************************************************************/

static sdio_eventset_t t113_smhc_eventwait(struct sdio_dev_s *dev)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  sdio_eventset_t wkupevent;

  /* Race-free: if the IRQ already fired before we got here, wkupevent
   * will be non-zero and the semaphore will already be posted; otherwise
   * we sleep until the IRQ posts.
   */

  for (; ; )
    {
      nxsem_wait_uninterruptible(&priv->waitsem);
      wkupevent = priv->wkupevent;
      if (wkupevent != 0)
        {
          break;
        }
    }

  /* Post-flight RX dcache invalidate, deferred from the IRQ handler.
   * Mirrors the pattern in sam_sdmmc.c (pre-flight in setup, post-flight
   * after TRANSFERDONE) but pays the 256 KiB cache walk on the waiting
   * thread instead of in IRQ context.  Skip on TX, on cancel/error
   * paths (xfer_buffer cleared by smhc_dma_cancel), and on command-only
   * waits (xfer_recv false).
   */

  if ((wkupevent & SDIOWAIT_TRANSFERDONE) != 0 &&
      priv->xfer_recv && priv->xfer_buffer != NULL && priv->xfer_len > 0)
    {
      up_invalidate_dcache((uintptr_t)priv->xfer_buffer,
                           (uintptr_t)priv->xfer_buffer + priv->xfer_len);
    }

  /* Disable any lingering IRQ mask -- waitenable will re-arm next round.
   * Preserve SDIO IO IRQ if armed so card-side IRQs keep flowing between
   * transfers.
   */

  smhc_putreg(priv, T113_SMHC_INTMASK_OFFSET,
              priv->sdio_irq_enabled ? T113_SMHC_INT_SDIO : 0);
  priv->waitints   = 0;
  priv->wkupevent  = 0;

  return wkupevent;
}

/****************************************************************************
 * Name: t113_smhc_callback
 *
 * Description:
 *   Worker invoked from t113_smhc_callbackenable (and, in a future
 *   port, from a CD-pin EINT) to forward an SDIOMEDIA_INSERTED or
 *   SDIOMEDIA_EJECTED event to the mmcsd layer.  Mirrors sam_callback()
 *   in sam_sdmmc.c: the function disarms further events on each fire so
 *   the registered worker (mmcsd_mediachange) is responsible for
 *   re-enabling them via SDIO_CALLBACKENABLE().
 *
 *   Always queued onto HPWORK so the registered handler runs in thread
 *   context, never IRQ context.
 *
 ****************************************************************************/

static void t113_smhc_callback(void *arg)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)arg;
  sdio_statset_t  status;

  if (priv->callback == NULL)
    {
      return;
    }

  status = t113_smhc_status(&priv->dev);

  /* Filter against the events the upper layer is currently armed for.
   * A media-present transition only fires if INSERTED is enabled, etc.
   */

  if ((status & SDIO_STATUS_PRESENT) != 0)
    {
      if ((priv->cbevents & SDIOMEDIA_INSERTED) == 0)
        {
          return;
        }
    }
  else
    {
      if ((priv->cbevents & SDIOMEDIA_EJECTED) == 0)
        {
          return;
        }
    }

  /* Disarm before firing -- the handler must call SDIO_CALLBACKENABLE
   * to re-arm.
   */

  priv->cbevents = 0;

  if (up_interrupt_context())
    {
      mcinfo("Queue cb=%p arg=%p\n", priv->callback, priv->cbarg);
      work_queue(HPWORK, &priv->cbwork, priv->callback, priv->cbarg, 0);
    }
  else
    {
      mcinfo("Direct cb=%p arg=%p\n", priv->callback, priv->cbarg);
      priv->callback(priv->cbarg);
    }
}

/****************************************************************************
 * Name: t113_smhc_callbackenable
 *
 * Description:
 *   Arm or disarm media-change callbacks.  After arming, fire the worker
 *   immediately so the upper layer re-syncs to the current CD state in
 *   case a transition was missed (e.g. card removed before the upper layer
 *   armed EJECTED).
 *
 ****************************************************************************/

static void t113_smhc_callbackenable(struct sdio_dev_s *dev,
                                     sdio_eventset_t eventset)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;

  mcinfo("eventset=%02x\n", eventset);
  priv->cbevents = eventset;
  t113_smhc_callback(priv);
}

/****************************************************************************
 * Name: t113_smhc_registercallback
 *
 * Description:
 *   Record the mmcsd_mediachange() worker and its private state pointer.
 *   The mmcsd layer registers itself once at probe time; we keep the
 *   pointer for the lifetime of the device.
 *
 *   Note (P1 limitation): the T113 GPIO layer in this tree does not yet
 *   expose an EINT API, so card-detect transitions are surfaced by the
 *   mmcsd layer's own polling cycle through SDIO_STATUS / SDIO_PRESENT.
 *   When a CD-pin EINT helper lands, it can call t113_smhc_callback()
 *   directly from the rising/falling edge ISR with no other change.
 *
 ****************************************************************************/

static int t113_smhc_registercallback(struct sdio_dev_s *dev,
                                      worker_t callback, void *arg)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;

  mcinfo("Register cb=%p arg=%p\n", callback, arg);
  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
/****************************************************************************
 * Name: t113_smhc_dmapreflight
 ****************************************************************************/

static int t113_smhc_dmapreflight(struct sdio_dev_s *dev,
                                  const uint8_t *buffer, size_t buflen)
{
  UNUSED(dev);

  /* IDMAC requires 4-byte aligned buffer + length.  smhc_dma_setup
   * enforces the same predicate; mirror it here so the upper half can
   * fall back to PIO before committing the card to the transfer.
   * (T113 has no PIO data path implemented; recvsetup aliases
   * dmarecvsetup.  Returning OK here therefore both validates the
   * request and signals that the DMA path will accept it.)
   */

  if (buffer == NULL || buflen == 0 ||
      ((uintptr_t)buffer & 0x3u) != 0 || (buflen & 0x3u) != 0)
    {
      return -EINVAL;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: t113_smhc_dmarecvsetup
 *
 * Description:
 *   Build the IDMAC descriptor chain for a card-to-host (RX) transfer
 *   and engage the controller.  drivers/mmcsd calls this BEFORE the
 *   data-phase sendcmd; the actual transfer kicks off once SMHC_CMD's
 *   DATA_TRANS bit is set by t113_smhc_sendcmd.
 *
 ****************************************************************************/

static int t113_smhc_dmarecvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                                  size_t buflen)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;

  return smhc_dma_setup(priv, buffer, buflen, true);
}

/****************************************************************************
 * Name: t113_smhc_dmasendsetup
 *
 * Description:
 *   Build the IDMAC descriptor chain for a host-to-card (TX) transfer.
 *   Buffer is const-correct on the SDIO API but the IDMAC reads from it,
 *   so we cast away const to share smhc_dma_setup() with the RX path.
 *
 ****************************************************************************/

static int t113_smhc_dmasendsetup(struct sdio_dev_s *dev,
                                  const uint8_t *buffer, size_t buflen)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;

  return smhc_dma_setup(priv, (uint8_t *)buffer, buflen, false);
}
#endif /* CONFIG_SDIO_DMA */

/****************************************************************************
 * Name: t113_smhc_gotextcsd
 ****************************************************************************/

static void t113_smhc_gotextcsd(struct sdio_dev_s *dev,
                                const uint8_t *buffer)
{
  /* eMMC EXT_CSD-arrived hook (HS200/HS400 tuning).  Phase 1 is SD
   * memory only; nothing to do.
   */

  UNUSED(dev);
  UNUSED(buffer);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_smhc_register_func_irq
 *
 * Description:
 *   Install or remove a per-function SDIO IRQ callback.  See header doc
 *   for semantics.  Cb=NULL deregisters.
 *
 ****************************************************************************/

int t113_smhc_register_func_irq(struct sdio_dev_s *dev, uint8_t func,
                                sdio_func_irq_t cb, void *arg)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  irqstate_t flags;

  if (priv == NULL || func == 0 || func >= T113_SMHC_NUM_FUNCS)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  if (cb != NULL && priv->func_irq[func] != NULL)
    {
      leave_critical_section(flags);
      return -EBUSY;
    }

  priv->func_irq[func]     = cb;
  priv->func_irq_arg[func] = arg;

  leave_critical_section(flags);
  mcinfo("SMHC%d: func %u IRQ %s\n", priv->cfg->bus, func,
         cb != NULL ? "registered" : "deregistered");
  return OK;
}

/****************************************************************************
 * Name: t113_smhc_enable_func_irq
 *
 * Description:
 *   Enable / disable host-side SDIO IO interrupt dispatch.  Toggles the
 *   SDIO_INT bit in SMHC_INTMASK without touching the command/data wait
 *   mask -- card-IRQ delivery is independent of waitenable() arming.
 *
 ****************************************************************************/

int t113_smhc_enable_func_irq(struct sdio_dev_s *dev, uint8_t func,
                              bool enable)
{
  struct t113_smhc_dev_s *priv = (struct t113_smhc_dev_s *)dev;
  irqstate_t flags;
  uint32_t   regval;

  if (priv == NULL || func == 0 || func >= T113_SMHC_NUM_FUNCS)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  regval = smhc_getreg(priv, T113_SMHC_INTMASK_OFFSET);

  if (enable)
    {
      regval |= T113_SMHC_INT_SDIO;
      priv->sdio_irq_enabled = true;
    }
  else
    {
      regval &= ~T113_SMHC_INT_SDIO;
      priv->sdio_irq_enabled = false;
    }

  smhc_putreg(priv, T113_SMHC_INTMASK_OFFSET, regval);

  /* Clear any latched SDIO_INT so the next assertion is a fresh edge. */

  smhc_putreg(priv, T113_SMHC_RINTSTS_OFFSET, T113_SMHC_INT_SDIO);

  leave_critical_section(flags);
  mcinfo("SMHC%d: SDIO IRQ %s (func=%u)\n", priv->cfg->bus,
         enable ? "enabled" : "disabled", func);
  return OK;
}

/****************************************************************************
 * Name: t113_smhc_initialize
 *
 * Description:
 *   Initialize and return an SDIO device instance for the requested slot.
 *
 * Input Parameters:
 *   slotno - SMHC controller index (0/1/2)
 *
 * Returned Value:
 *   Pointer to sdio_dev_s on success; NULL on invalid slot.
 *
 ****************************************************************************/

struct sdio_dev_s *t113_smhc_initialize(int slotno)
{
  struct t113_smhc_dev_s *priv;
  const struct t113_smhc_config_s *cfg;

  struct t113_smhc_desc_s *descs;

  switch (slotno)
    {
#ifdef CONFIG_T113_SMHC0
      case 0:
        priv  = &g_smhc0_dev;
        cfg   = &g_smhc0_config;
        descs = g_smhc0_descs;
        break;
#endif
#ifdef CONFIG_T113_SMHC1
      case 1:
        priv  = &g_smhc1_dev;
        cfg   = &g_smhc1_config;
        descs = g_smhc1_descs;
        break;
#endif
#ifdef CONFIG_T113_SMHC2
      case 2:
        priv  = &g_smhc2_dev;
        cfg   = &g_smhc2_config;
        descs = g_smhc2_descs;
        break;
#endif
      default:
        return NULL;
    }

  memset(priv, 0, sizeof(*priv));
  priv->cfg   = cfg;
  priv->descs = descs;
  t113_smhc_ops_init(&priv->dev);

  nxmutex_init(&priv->lock);
  nxsem_init(&priv->waitsem, 0, 0);

  /* Apply pinmux for CLK / CMD / D0..D3.  pinset_* are stored as 32-bit
   * for forward compatibility but the encoder fits in 16 bits today; cast
   * matches t113_can.c / t113_i2c.c usage.
   */

  t113_gpio_config((uint16_t)cfg->pinset_clk);
  t113_gpio_config((uint16_t)cfg->pinset_cmd);
  t113_gpio_config((uint16_t)cfg->pinset_d[0]);
  t113_gpio_config((uint16_t)cfg->pinset_d[1]);
  t113_gpio_config((uint16_t)cfg->pinset_d[2]);
  t113_gpio_config((uint16_t)cfg->pinset_d[3]);

  /* Card-detect: configure the pin as input + pull-up (matches encoded
   * pinset).  We sample it here so cdstatus reflects boot-time presence
   * and mmcsd_probe() picks the right path on its first cycle.  EINT
   * routing is deferred until the T113 GPIO layer grows an EINT API;
   * meanwhile mmcsd's polling cycle drives status() and surfaces hotplug.
   */

  if (cfg->cd_pin >= 0)
    {
      t113_gpio_config((uint16_t)cfg->cd_pin);
    }

  t113_smhc_status(&priv->dev);

  /* Bring SMHCn out of reset and program a 25 MHz module clock. */

  if (t113_smhc_clk_enable(cfg->bus, 25000000) < 0)
    {
      mcerr("ERROR: t113_smhc_clk_enable(%d) failed\n", cfg->bus);
      return NULL;
    }

  /* Soft-reset the host controller. */

  if (smhc_reset(priv) < 0)
    {
      mcerr("ERROR: SMHC%d reset failed\n", cfg->bus);
      return NULL;
    }

  /* Program output-drive / sample-delay phases now so any code that
   * issues commands before mmcsd_sdinitialize calls SDIO_CLOCK (most
   * notably the SELFTEST block below, when enabled) sees the same
   * timing the production path uses.
   */

  smhc_set_phase(priv);

  mcinfo("SMHC%d: enabled @ 25MHz, pinmux applied\n", slotno);

#ifdef CONFIG_T113_SMHC_SELFTEST
  if (slotno == 0)
    {
      /* Attach the IRQ early so the selftest can use the same
       * waitenable / eventwait machinery the upper half will use.
       * SDIO_ATTACH() called later from mmcsd_slotinitialize() will
       * just rebind to the same handler.
       */

      if (t113_smhc_attach(&priv->dev) == OK)
        {
          uint32_t r;
          int      i;

          /* CMD0 -- GO_IDLE_STATE, no response */

          t113_smhc_waitenable(&priv->dev,
                               SDIOWAIT_CMDDONE | SDIOWAIT_TIMEOUT, 100);
          t113_smhc_sendcmd(&priv->dev, MMCSD_CMD0, 0);
          t113_smhc_eventwait(&priv->dev);
          mcinfo("CMD0 done\n");

          /* CMD8 -- SEND_IF_COND, expects R7 echo of arg 0x000001AA */

          t113_smhc_waitenable(&priv->dev,
                               SDIOWAIT_RESPONSEDONE |
                               SDIOWAIT_TIMEOUT, 100);
          t113_smhc_sendcmd(&priv->dev, SD_CMD8, 0x000001aa);
          t113_smhc_eventwait(&priv->dev);
          r = 0;
          t113_smhc_recv_r7(&priv->dev, SD_CMD8, &r);
          mcinfo("CMD8 response: 0x%08" PRIx32 " (expect 0x1AA)\n", r);

          /* ACMD41 loop -- wait for OCR bit 31 (init done) */

          for (i = 0; i < 100; i++)
            {
              t113_smhc_waitenable(&priv->dev,
                                   SDIOWAIT_RESPONSEDONE |
                                   SDIOWAIT_TIMEOUT, 100);
              t113_smhc_sendcmd(&priv->dev, SD_CMD55, 0);
              t113_smhc_eventwait(&priv->dev);
              t113_smhc_recv_r1(&priv->dev, SD_CMD55, &r);

              t113_smhc_waitenable(&priv->dev,
                                   SDIOWAIT_RESPONSEDONE |
                                   SDIOWAIT_TIMEOUT, 100);
              t113_smhc_sendcmd(&priv->dev, SD_ACMD41, 0x40ff8000);
              t113_smhc_eventwait(&priv->dev);
              t113_smhc_recv_r3(&priv->dev, SD_ACMD41, &r);

              if ((r & 0x80000000) != 0)
                {
                  mcinfo("ACMD41 ready: 0x%08" PRIx32 "\n", r);
                  break;
                }

              up_mdelay(10);
            }

          /* M3 path: enumerate to TRAN state, set blocklen, read MBR LBA 0
           * and dump first/last 16 bytes.  Foreman cross-checks the bytes
           * against `dd if=/dev/sdX bs=512 count=1` on Linux.  Buffer is
           * D-cache-line aligned so smhc_dma_setup() can do clean cache
           * pre-flight without false sharing.
           */

          {
            static uint8_t mbr_buf[512] aligned_data(32);
            uint32_t       cid[4];
            uint32_t       rca = 0;
            uint32_t       evt;

            /* CMD2: ALL_SEND_CID (R2 long, broadcast). */

            t113_smhc_waitenable(&priv->dev,
                                 SDIOWAIT_RESPONSEDONE |
                                 SDIOWAIT_TIMEOUT, 200);
            t113_smhc_sendcmd(&priv->dev, MMCSD_CMD2, 0);
            t113_smhc_eventwait(&priv->dev);
            if (t113_smhc_recv_r2(&priv->dev, MMCSD_CMD2, cid) == OK)
              {
                mcinfo("CMD2 CID[3..0]: %08" PRIx32 " %08" PRIx32
                       " %08" PRIx32 " %08" PRIx32 "\n",
                       cid[0], cid[1], cid[2], cid[3]);
              }

            /* CMD3: SEND_RELATIVE_ADDR (R6, RCA in upper 16 bits). */

            t113_smhc_waitenable(&priv->dev,
                                 SDIOWAIT_RESPONSEDONE |
                                 SDIOWAIT_TIMEOUT, 200);
            t113_smhc_sendcmd(&priv->dev, SD_CMD3, 0);
            t113_smhc_eventwait(&priv->dev);
            t113_smhc_recv_r6(&priv->dev, SD_CMD3, &r);
            rca = r & 0xffff0000u;
            mcinfo("CMD3 RCA: 0x%04" PRIx32 "\n", rca >> 16);

            /* CMD7: SELECT (R1b). */

            t113_smhc_waitenable(&priv->dev,
                                 SDIOWAIT_RESPONSEDONE |
                                 SDIOWAIT_TIMEOUT, 200);
            t113_smhc_sendcmd(&priv->dev, MMCSD_CMD7S, rca);
            t113_smhc_eventwait(&priv->dev);

            /* CMD16: SET_BLOCKLEN(512). */

            t113_smhc_waitenable(&priv->dev,
                                 SDIOWAIT_RESPONSEDONE |
                                 SDIOWAIT_TIMEOUT, 200);
            t113_smhc_sendcmd(&priv->dev, MMCSD_CMD16, 512);
            t113_smhc_eventwait(&priv->dev);

            /* CMD17 READ_SINGLE_BLOCK at LBA 0 (MBR).  blocksetup ->
             * dmarecvsetup -> waitenable(TRANSFERDONE) -> sendcmd kicks
             * off the data phase which the IRQ completes via
             * IDST.RX_INT or RINT.DTC.
             */

            t113_smhc_blocksetup(&priv->dev, 512, 1);
            if (t113_smhc_dmarecvsetup(&priv->dev, mbr_buf, 512) == OK)
              {
                t113_smhc_waitenable(&priv->dev,
                                     SDIOWAIT_TRANSFERDONE |
                                     SDIOWAIT_TIMEOUT |
                                     SDIOWAIT_ERROR, 5000);
                t113_smhc_sendcmd(&priv->dev, MMCSD_CMD17, 0);
                evt = (uint32_t)t113_smhc_eventwait(&priv->dev);
                if ((evt & SDIOWAIT_TRANSFERDONE) != 0)
                  {
                    mcinfo("MBR LBA0 read OK\n");
                    mcinfo("MBR[0..15]:    %02x %02x %02x %02x %02x %02x "
                           "%02x %02x %02x %02x %02x %02x %02x %02x %02x "
                           "%02x\n",
                           mbr_buf[0], mbr_buf[1], mbr_buf[2], mbr_buf[3],
                           mbr_buf[4], mbr_buf[5], mbr_buf[6], mbr_buf[7],
                           mbr_buf[8], mbr_buf[9], mbr_buf[10], mbr_buf[11],
                           mbr_buf[12], mbr_buf[13], mbr_buf[14],
                           mbr_buf[15]);
                    mcinfo("MBR[496..511]: %02x %02x %02x %02x %02x %02x "
                           "%02x %02x %02x %02x %02x %02x %02x %02x %02x "
                           "%02x (expect ... 55 aa)\n",
                           mbr_buf[496], mbr_buf[497], mbr_buf[498],
                           mbr_buf[499], mbr_buf[500], mbr_buf[501],
                           mbr_buf[502], mbr_buf[503], mbr_buf[504],
                           mbr_buf[505], mbr_buf[506], mbr_buf[507],
                           mbr_buf[508], mbr_buf[509], mbr_buf[510],
                           mbr_buf[511]);
                  }
                else
                  {
                    mcerr("ERROR: MBR LBA0 read failed evt=0x%08" PRIx32
                          "\n", evt);
                  }
              }
            else
              {
                mcerr("ERROR: dmarecvsetup failed\n");
              }
          }

#ifdef CONFIG_T113_SMHC_SELFTEST_SDIO
          {
            /* Driver-side smoke for the SDIO protocol path.  No SDIO device
             * is wired to SMHC0 (it sits on a microSD socket), so every
             * CMD52/CMD53 issued here is expected to time out.  The test
             * passes as long as eventwait() returns control to us in
             * finite time -- i.e. the controller does not hang on a
             * non-responsive bus.  Phase-2 (RTL8723DS WiFi over SMHC1)
             * exercises the same code paths against a real SDIO target.
             *
             * Argument layout (SD Specifications Part E1, SDIO Simplified
             * Spec):
             *   CMD52 IO_RW_DIRECT
             *     [31]    RW (0=read, 1=write)
             *     [30:28] FUNC
             *     [27]    RAW
             *     [26]    rsv
             *     [25:9]  REG_ADDR
             *     [8]     rsv
             *     [7:0]   DATA
             *   CMD53 IO_RW_EXTENDED
             *     [31]    RW
             *     [30:28] FUNC
             *     [27]    BLOCK_MODE (0=byte, 1=block)
             *     [26]    OP_CODE    (0=fixed addr, 1=incrementing)
             *     [25:9]  REG_ADDR
             *     [8:0]   COUNT
             */

            static uint8_t cmd53_buf[4] aligned_data(32);
            uint32_t       cmd52_arg;
            uint32_t       cmd53_arg;
            uint32_t       evt;
            uint32_t       r5;

            /* CMD52 read F0 / CCCR addr 0x00 (CCCR_REVISION).  No card
             * means RTO; we just verify the call returns.
             */

            cmd52_arg = (0u << 31) | (0u << 28) | (0u << 27) |
                        ((uint32_t)0x00 << 9) | 0u;
            t113_smhc_waitenable(&priv->dev,
                                 SDIOWAIT_RESPONSEDONE |
                                 SDIOWAIT_TIMEOUT |
                                 SDIOWAIT_ERROR, 100);
            t113_smhc_sendcmd(&priv->dev, SDIO_CMD52, cmd52_arg);
            evt = (uint32_t)t113_smhc_eventwait(&priv->dev);
            mcinfo("CMD52 F0/0x00: evt=0x%08" PRIx32 "\n", evt);
            if ((evt & SDIOWAIT_RESPONSEDONE) != 0)
              {
                r5 = 0;
                t113_smhc_recv_r5(&priv->dev, SDIO_CMD52, &r5);
                mcinfo("CMD52 R5: 0x%08" PRIx32 "\n", r5);
              }

            /* CMD52 read F7 / CCCR addr 0x00.  Function 7 is reserved on
             * SDIO; expect timeout (or out-of-range R5 flag if a card
             * happens to respond -- it will not, here).
             */

            cmd52_arg = (0u << 31) | (7u << 28) | (0u << 27) |
                        ((uint32_t)0x00 << 9) | 0u;
            t113_smhc_waitenable(&priv->dev,
                                 SDIOWAIT_RESPONSEDONE |
                                 SDIOWAIT_TIMEOUT |
                                 SDIOWAIT_ERROR, 100);
            t113_smhc_sendcmd(&priv->dev, SDIO_CMD52, cmd52_arg);
            evt = (uint32_t)t113_smhc_eventwait(&priv->dev);
            mcinfo("CMD52 F7/0x00: evt=0x%08" PRIx32
                   " (expect timeout)\n", evt);

            /* CMD53 byte-mode 4-byte read from F0 CCCR.  Same expectation:
             * the data phase must not hang -- eventwait must return.
             */

            cmd53_arg = (0u << 31) | (0u << 28) | (0u << 27) |
                        (1u << 26) | ((uint32_t)0x00 << 9) | 4u;
            t113_smhc_blocksetup(&priv->dev, 4, 1);
            if (t113_smhc_dmarecvsetup(&priv->dev, cmd53_buf, 4) == OK)
              {
                t113_smhc_waitenable(&priv->dev,
                                     SDIOWAIT_TRANSFERDONE |
                                     SDIOWAIT_TIMEOUT |
                                     SDIOWAIT_ERROR, 100);
                t113_smhc_sendcmd(&priv->dev, SDIO_CMD53RD, cmd53_arg);
                evt = (uint32_t)t113_smhc_eventwait(&priv->dev);
                mcinfo("CMD53 F0/4byte: evt=0x%08" PRIx32
                       " (expect timeout, must not hang)\n", evt);
              }

            /* Cancel any in-flight DMA so the next driver call starts
             * from a clean state -- selftest is best-effort and we do not
             * want a dangling descriptor chain after the timeout.
             */

            t113_smhc_cancel(&priv->dev);
          }
#endif /* CONFIG_T113_SMHC_SELFTEST_SDIO */
        }
    }
#endif

  return &priv->dev;
}
