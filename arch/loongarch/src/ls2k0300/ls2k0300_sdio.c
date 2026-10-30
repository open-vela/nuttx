/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_sdio.c
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

/* SD/SDIO host controller driver for the LS2K0300 "loongson,ls-mmc" SDIO
 * block.  The controller is described in chapter 15 of the LS2K0300 user
 * manual; the register/DMA programming sequences below follow the U-Boot
 * driver (drivers/mmc/ls_mmc.c) and the Linux driver (drivers/mmc/host/
 * ls2kmci.c), which are both known to drive this controller.
 *
 * The driver implements the NuttX SDIO interface (include/nuttx/sdio.h) and
 * is registered with the generic MMC/SD core through mmcsd_slotinitialize().
 * All transfers are driven with the DMA engine that lives inside the SDIO
 * block: the controller owns two DMA request generators, one per direction,
 * programmed through a descriptor that is fetched from memory.
 *
 * The driver polls the SDI_INT_MASK status register instead of using the
 * controller interrupt: the MMCSd core issues one command at a time and
 * waits for each one to complete, so a polling implementation is both
 * simpler and fully sufficient.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/compiler.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_sdio.h"

/* Write back and invalidate the D-cache lines that cover a buffer, so that
 * the DMA engine and the CPU see the same data.  Provided by
 * ls2k0300_dma_cache.S.
 */

extern void ls2k0300_dma_flush(FAR void *addr, size_t len);

#ifdef CONFIG_LS2K0300_SDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Which controller carries the card: SDIO1 is the 4 bit SD card socket,
 * SDIO0 is the 8 bit eMMC interface.
 */

#ifdef CONFIG_LS2K0300_SDIO1
#  define LS2K0300_SDIO_PHYSBASE LS2K0300_SDIO1_PHYS
#  define LS2K0300_SDIO_BASE     LS2K0300_SDIO1_BASE
#else
#  define LS2K0300_SDIO_PHYSBASE LS2K0300_SDIO0_PHYS
#  define LS2K0300_SDIO_BASE     LS2K0300_SDIO0_BASE
#endif

/* Identification stage clock, the SD specification limits it to 400 kHz */

#define LS2K0300_SDIO_ID_CLOCK   400

/* Maximum SD clock in kHz once the card has been identified.  The
 * controller divides its module clock by an 8 bit prescaler, so the
 * achievable rate is clkfreq / psc with 1 <= psc <= 255.
 */

#ifndef CONFIG_LS2K0300_SDIO_MAXFREQ
#  define CONFIG_LS2K0300_SDIO_MAXFREQ 25000000
#endif

/* Polling limits.  A command must always complete quickly, the data phase
 * timeout is supplied by the MMCSd core.
 */

#define LS2K0300_SDIO_CMD_TMO_MS 1000

/* The status register is polled in two phases: a tight loop that costs no
 * scheduling latency for the usual case, then 1 ms sleeps so that a long
 * busy (R1b) does not hog the CPU.
 */

#define LS2K0300_SDIO_BUSY_US    2000
#define LS2K0300_SDIO_IDLE_US    1000

/* Data timeout counter, 24 bit, in divided SD clocks */

#define LS2K0300_SDIO_DTIM_VALUE  0x00ffffff

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ls2k0300_sdio_s
{
  struct sdio_dev_s dev;          /* Must be first: the MMCSd core casts */
  uintptr_t base;                 /* Uncached register base             */
  uint32_t  physbase;             /* Physical register base (DMA)       */
  uint32_t  clkfreq;              /* Controller module clock in Hz      */
  uint32_t  prescaler;            /* Programmed SD clock divider        */
  bool      widebus;              /* 4 bit bus requested                */
  bool      xferpending;          /* A data phase is armed              */

  /* Block geometry, reported by SDIO_BLOCKSETUP before every transfer */

  uint32_t  blocksize;
  uint32_t  nblocks;

  /* Data phase state */

  uint8_t  *buffer;
  size_t    buflen;
  bool      receiving;

  /* Event wait state */

  sdio_eventset_t waitevents;
  uint32_t  wtimeoutms;

  /* SDI_INT_MASK snapshot taken when the command completed */

  uint32_t  cmdstatus;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ls2k0300_sdio_softreset(FAR struct ls2k0300_sdio_s *priv);
static int  ls2k0300_sdio_setclock(FAR struct ls2k0300_sdio_s *priv,
                                   uint32_t ratekhz);

/* SDIO interface methods */

#ifdef CONFIG_SDIO_MUXBUS
static int  ls2k0300_sdio_lock(FAR struct sdio_dev_s *dev, bool lock);
#endif
static void ls2k0300_sdio_reset(FAR struct sdio_dev_s *dev);
static sdio_capset_t ls2k0300_sdio_capabilities(FAR struct sdio_dev_s *dev);
static sdio_statset_t ls2k0300_sdio_status(FAR struct sdio_dev_s *dev);
static void ls2k0300_sdio_widebus(FAR struct sdio_dev_s *dev, bool enable);
static void ls2k0300_sdio_clock(FAR struct sdio_dev_s *dev,
                                enum sdio_clock_e rate);
static int  ls2k0300_sdio_attach(FAR struct sdio_dev_s *dev);
static int  ls2k0300_sdio_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd,
                                  uint32_t arg);
#ifdef CONFIG_SDIO_BLOCKSETUP
static void ls2k0300_sdio_blocksetup(FAR struct sdio_dev_s *dev,
                                     unsigned int blocklen,
                                     unsigned int nblocks);
#endif
static int  ls2k0300_sdio_recvsetup(FAR struct sdio_dev_s *dev,
                                    FAR uint8_t *buffer, size_t nbytes);
static int  ls2k0300_sdio_sendsetup(FAR struct sdio_dev_s *dev,
                                    FAR const uint8_t *buffer,
                                    size_t nbytes);
static int  ls2k0300_sdio_cancel(FAR struct sdio_dev_s *dev);
static int  ls2k0300_sdio_waitresponse(FAR struct sdio_dev_s *dev,
                                       uint32_t cmd);
static int  ls2k0300_sdio_recvshortcrc(FAR struct sdio_dev_s *dev,
                                       uint32_t cmd, FAR uint32_t *rshort);
static int  ls2k0300_sdio_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd,
                                   FAR uint32_t rlong[4]);
static int  ls2k0300_sdio_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd,
                                    FAR uint32_t *rshort);
static void ls2k0300_sdio_waitenable(FAR struct sdio_dev_s *dev,
                                     sdio_eventset_t eventset,
                                     uint32_t timeout);
static sdio_eventset_t ls2k0300_sdio_eventwait(FAR struct sdio_dev_s *dev);
static void ls2k0300_sdio_callbackenable(FAR struct sdio_dev_s *dev,
                                         sdio_eventset_t eventset);
#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
static int  ls2k0300_sdio_registercallback(FAR struct sdio_dev_s *dev,
                                           worker_t callback, FAR void *arg);
#endif

/* Data transfer */

static int  ls2k0300_sdio_datasetup(FAR struct ls2k0300_sdio_s *priv);
static void ls2k0300_sdio_datastop(FAR struct ls2k0300_sdio_s *priv);

/* Status polling */

static int  ls2k0300_sdio_pollstat(FAR struct ls2k0300_sdio_s *priv,
                                   uint32_t mask, uint32_t timeoutms,
                                   FAR uint32_t *status);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The DMA descriptor must be 32 byte aligned: the low 5 bits of the address
 * written to the DMA order register carry flags.  Only one transfer is in
 * flight at a time (the MMCSd core serialises the bus), so a single
 * descriptor is enough.
 */

static struct ls2k0300_sdio_dmadesc_s g_sdiodma
  aligned_data(LS2K0300_DMA_ORDER_ALIGN);

static struct ls2k0300_sdio_s g_sdiodev =
{
  .dev =
  {
#ifdef CONFIG_SDIO_MUXBUS
    .lock             = ls2k0300_sdio_lock,
#endif
    .reset            = ls2k0300_sdio_reset,
    .capabilities     = ls2k0300_sdio_capabilities,
    .status           = ls2k0300_sdio_status,
    .widebus          = ls2k0300_sdio_widebus,
    .clock            = ls2k0300_sdio_clock,
    .attach           = ls2k0300_sdio_attach,

    .sendcmd          = ls2k0300_sdio_sendcmd,
#ifdef CONFIG_SDIO_BLOCKSETUP
    .blocksetup       = ls2k0300_sdio_blocksetup,
#endif
    .recvsetup        = ls2k0300_sdio_recvsetup,
    .sendsetup        = ls2k0300_sdio_sendsetup,
    .cancel           = ls2k0300_sdio_cancel,

    .waitresponse     = ls2k0300_sdio_waitresponse,
    .recv_r1          = ls2k0300_sdio_recvshortcrc,
    .recv_r2          = ls2k0300_sdio_recvlong,
    .recv_r3          = ls2k0300_sdio_recvshort,
    .recv_r4          = ls2k0300_sdio_recvshort,
    .recv_r5          = ls2k0300_sdio_recvshortcrc,
    .recv_r6          = ls2k0300_sdio_recvshortcrc,
    .recv_r7          = ls2k0300_sdio_recvshort,

    .waitenable       = ls2k0300_sdio_waitenable,
    .eventwait        = ls2k0300_sdio_eventwait,
    .callbackenable   = ls2k0300_sdio_callbackenable,
#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
    .registercallback = ls2k0300_sdio_registercallback,
#endif

#ifdef CONFIG_SDIO_DMA
    .dmarecvsetup     = ls2k0300_sdio_recvsetup,
    .dmasendsetup     = ls2k0300_sdio_sendsetup,
#endif
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ls2k0300_sdio_getreg / ls2k0300_sdio_putreg
 *
 * Description:
 *   32 bit access to the controller registers.  The register block is
 *   mapped uncached so no cache maintenance is needed for the registers.
 *
 ****************************************************************************/

static inline uint32_t ls2k0300_sdio_getreg(FAR struct ls2k0300_sdio_s *priv,
                                            uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void ls2k0300_sdio_putreg(FAR struct ls2k0300_sdio_s *priv,
                                        uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: ls2k0300_sdio_pollstat
 *
 * Description:
 *   Poll SDI_INT_MASK until any bit in mask is set or the timeout expires.
 *   The status register is a set of write-1-to-clear latches, so reading it
 *   is a non destructive operation.
 *
 * Returned Value:
 *   OK when a bit in mask was seen, -ETIMEDOUT otherwise.  The register
 *   snapshot is always returned in *status.
 *
 ****************************************************************************/

static int ls2k0300_sdio_pollstat(FAR struct ls2k0300_sdio_s *priv,
                                  uint32_t mask, uint32_t timeoutms,
                                  FAR uint32_t *status)
{
  uint32_t val;
  uint32_t busyus = 0;

  DEBUGASSERT(status != NULL);

  for (; ; )
    {
      val = ls2k0300_sdio_getreg(priv, LS2K0300_SDIO_INTSTA);
      if ((val & mask) != 0)
        {
          *status = val;
          return OK;
        }

      if (busyus < LS2K0300_SDIO_BUSY_US)
        {
          /* Still in the cheap polling window */

          up_udelay(10);
          busyus += 10;
        }
      else if (timeoutms == 0)
        {
          *status = val;
          return -ETIMEDOUT;
        }
      else
        {
          /* Hand the CPU back: the remaining wait can be long (an R1b
           * command holds the data line busy until the card has finished
           * programming).
           */

          nxsig_usleep(LS2K0300_SDIO_IDLE_US);
          if (timeoutms <= (LS2K0300_SDIO_IDLE_US / 1000))
            {
              *status = val;
              return -ETIMEDOUT;
            }

          timeoutms -= (LS2K0300_SDIO_IDLE_US / 1000);
        }
    }
}

/****************************************************************************
 * Name: ls2k0300_sdio_setclock
 *
 * Description:
 *   Program the SD clock divider.  The output rate is
 *   clkfreq / prescaler with 1 <= prescaler <= 255, and the sample point
 *   is moved to the falling edge of the SD clock (SDI_PRE bit 31) as both
 *   reference drivers do.
 *
 ****************************************************************************/

static int ls2k0300_sdio_setclock(FAR struct ls2k0300_sdio_s *priv,
                                  uint32_t ratekhz)
{
  uint32_t prescaler;

  if (ratekhz == 0)
    {
      /* Gate the SD clock */

      ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_CON, 0);
      return OK;
    }

  /* Find the smallest divisor that does not exceed the requested rate */

  for (prescaler = 1; prescaler < 255; prescaler++)
    {
      if ((priv->clkfreq / 1000) / prescaler <= ratekhz)
        {
          break;
        }
    }

  if (prescaler > 255)
    {
      prescaler = 255;
    }

  priv->prescaler = prescaler;

  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_PRE,
                       prescaler | SDIO_PRE_REVCLOCK);
  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_CON, SDIO_CON_ENCLK);

  finfo("SDIO clock: %lu kHz (requested %lu kHz)\n",
        (unsigned long)((priv->clkfreq / 1000) / prescaler),
        (unsigned long)ratekhz);
  return OK;
}

/****************************************************************************
 * Name: ls2k0300_sdio_softreset
 *
 * Description:
 *   Reset the controller and restore the default (identification) state.
 *
 ****************************************************************************/

static void ls2k0300_sdio_softreset(FAR struct ls2k0300_sdio_s *priv)
{
  uint32_t regval;
  int i;

  /* Stop any transfer in flight and mask the controller interrupts.
   * SDI_INT_MASK is a write-1-to-clear register.
   */

  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_INTEN, 0);
  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_DATCON, 0);
  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_BSIZE, 0);
  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_INTSTA, SDIO_INT_ALL);

  /* Request a module reset.  The bit self clears when the reset is done. */

  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_CON, SDIO_CON_SOFTRST);
  for (i = 0; i < 1000; i++)
    {
      regval = ls2k0300_sdio_getreg(priv, LS2K0300_SDIO_CON);
      if ((regval & SDIO_CON_SOFTRST) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  if ((regval & SDIO_CON_SOFTRST) != 0)
    {
      mcerr("ERROR: SDIO reset timeout\n");
    }

  /* Back to the identification clock and enable the status bits.  The
   * status latches are only useful to this polling driver; the controller
   * interrupt itself is never routed to the interrupt controller.
   */

  ls2k0300_sdio_setclock(priv, LS2K0300_SDIO_ID_CLOCK);
  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_INTEN, SDIO_INTEN_ALL);

  priv->xferpending = false;
  priv->cmdstatus   = 0;
}

/****************************************************************************
 * Name: ls2k0300_sdio_datasetup
 *
 * Description:
 *   Arm the data phase.  Both reference drivers program the block size,
 *   the data timeout and SDI_DAT_CON before the data command is issued,
 *   then hand the buffer over to the built-in DMA engine.
 *
 *   The descriptor and the buffer are written back from the D-cache before
 *   the DMA is started, so the DMA engine sees the same bytes as the CPU.
 *
 ****************************************************************************/

static int ls2k0300_sdio_datasetup(FAR struct ls2k0300_sdio_s *priv)
{
  FAR struct ls2k0300_sdio_dmadesc_s *desc = &g_sdiodma;
  uintptr_t orderreg;
  uint32_t datcon;
  uint32_t saddr;
  uint32_t order;
  uint32_t nblocks;
  uint32_t blocksize;

  DEBUGASSERT(priv->buffer != NULL && priv->buflen > 0);

  /* The descriptor moves 32 bit words and the block counter of
   * SDI_DAT_CON is 12 bit wide.
   */

  if (((uintptr_t)priv->buffer & 3) != 0 || (priv->buflen & 3) != 0)
    {
      mcerr("ERROR: unaligned DMA buffer %p len %zu\n",
            priv->buffer, priv->buflen);
      return -EINVAL;
    }

  nblocks   = priv->nblocks;
  blocksize = priv->blocksize;

  if (nblocks == 0 || blocksize == 0 ||
      (size_t)nblocks * blocksize != priv->buflen)
    {
      /* SDIO_BLOCKSETUP did not describe this transfer: fall back to a
       * plain block split of the buffer.
       */

      blocksize = priv->buflen >= 512 ? 512 : priv->buflen;
      nblocks   = priv->buflen / blocksize;
      if (nblocks == 0)
        {
          nblocks = 1;
        }
    }

  if (nblocks == 0 || nblocks > LS2K0300_SDIO_MAXBLOCKS ||
      blocksize == 0 || blocksize > 4095)
    {
      mcerr("ERROR: bad geometry size=%" PRIu32 " blocks=%" PRIu32 "\n",
            blocksize, nblocks);
      return -EINVAL;
    }

  /* Memory address as seen by the DMA masters: the cached and uncached
   * aliases of the same physical page share the low address bits, so the
   * physical address is simply the low half of the pointer.
   */

  saddr = (uint32_t)(uintptr_t)priv->buffer;

  desc->order_addr    = 0;          /* No further descriptor */
  desc->saddr         = saddr;
  desc->daddr         = priv->physbase + LS2K0300_SDIO_WRDAT;
  desc->length        = priv->buflen / 4;
  desc->step_length   = 0;          /* Single step */
  desc->step_times    = 1;
  desc->cmd           = LS2K0300_DMA_CMD_INTMSK |
                        (priv->receiving ? 0 : LS2K0300_DMA_CMD_WRITE);
  desc->reserved      = 0;
  desc->order_addr_hi = 0;
  desc->saddr_hi      = 0;

  ls2k0300_dma_flush(desc, sizeof(*desc));
  ls2k0300_dma_flush(priv->buffer, priv->buflen);

  /* Data path: block size, then the timeout counter, then SDI_DAT_CON.
   * Writing SDI_DAT_CON with DTST set starts the data phase as soon as the
   * card answers the command that has just been (or is about to be) sent.
   */

  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_BSIZE, blocksize);
  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_DTIMER, LS2K0300_SDIO_DTIM_VALUE);

  datcon = nblocks & SDIO_DATCON_BLKNUM_MASK;
  if (priv->widebus)
    {
      datcon |= SDIO_DATCON_WIDEMODE;
    }

  datcon |= SDIO_DATCON_DTST | SDIO_DATCON_DMAEN;
  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_DATCON, datcon);

  /* Start the DMA.  The order register holds the descriptor address with
   * the start flag in bit 3; the high half is written first so that the
   * write to the low half (which arms the engine) is the last one.  This
   * is the sequence used by the Linux driver.
   */

  orderreg = priv->base +
             (priv->receiving ? LS2K0300_SDIO_RDMA_OFFSET
                              : LS2K0300_SDIO_WDMA_OFFSET);

  order = ((uint32_t)(uintptr_t)desc & ~(LS2K0300_DMA_ORDER_ALIGN - 1)) |
          LS2K0300_DMA_ORDER_START;

  putreg32(0, orderreg + 4);
  putreg32(order, orderreg);

  priv->xferpending = true;
  return OK;
}

/****************************************************************************
 * Name: ls2k0300_sdio_datastop
 *
 * Description:
 *   Tear the data path down and clear the transfer status latches.
 *
 ****************************************************************************/

static void ls2k0300_sdio_datastop(FAR struct ls2k0300_sdio_s *priv)
{
  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_DATCON, 0);
  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_BSIZE, 0);
  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_INTSTA, SDIO_INT_ALL);
  priv->xferpending = false;
}

/****************************************************************************
 * Name: ls2k0300_sdio_lock
 *
 * Description:
 *   Bus locking is not needed: the MMCSd core serialises access to the
 *   slot and nothing else shares this controller.
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_MUXBUS
static int ls2k0300_sdio_lock(FAR struct sdio_dev_s *dev, bool lock)
{
  return OK;
}
#endif

/****************************************************************************
 * Name: ls2k0300_sdio_reset
 *
 * Description:
 *   Reset the SDIO interface.  Called by the MMCSd core before the card
 *   identification sequence.
 *
 ****************************************************************************/

static void ls2k0300_sdio_reset(FAR struct sdio_dev_s *dev)
{
  FAR struct ls2k0300_sdio_s *priv = (FAR struct ls2k0300_sdio_s *)dev;

  finfo("Resetting the SDIO interface\n");
  ls2k0300_sdio_softreset(priv);
}

/****************************************************************************
 * Name: ls2k0300_sdio_capabilities
 *
 * Description:
 *   Report the capabilities and limitations of the controller.
 *
 ****************************************************************************/

static sdio_capset_t ls2k0300_sdio_capabilities(FAR struct sdio_dev_s *dev)
{
  sdio_capset_t caps = SDIO_CAPS_DMASUPPORTED | SDIO_CAPS_4BIT;

#ifdef CONFIG_LS2K0300_SDIO0
  /* SDIO0 has the full 8 bit data bus wired to the eMMC device */

  caps |= SDIO_CAPS_8BIT;
#endif

  return caps;
}

/****************************************************************************
 * Name: ls2k0300_sdio_status
 *
 * Description:
 *   There is no card detect line routed to the controller and the slot is
 *   soldered on the board, so the medium is always reported present.
 *
 ****************************************************************************/

static sdio_statset_t ls2k0300_sdio_status(FAR struct sdio_dev_s *dev)
{
  return SDIO_STATUS_PRESENT;
}

/****************************************************************************
 * Name: ls2k0300_sdio_widebus
 *
 * Description:
 *   Select 1 bit or 4 bit bus operation.
 *
 ****************************************************************************/

static void ls2k0300_sdio_widebus(FAR struct sdio_dev_s *dev, bool enable)
{
  FAR struct ls2k0300_sdio_s *priv = (FAR struct ls2k0300_sdio_s *)dev;

  finfo("widebus=%d\n", enable);
  priv->widebus = enable;
}

/****************************************************************************
 * Name: ls2k0300_sdio_clock
 *
 * Description:
 *   Enable/disable SDIO clocking and select the operating frequency.
 *
 ****************************************************************************/

static void ls2k0300_sdio_clock(FAR struct sdio_dev_s *dev,
                                enum sdio_clock_e rate)
{
  FAR struct ls2k0300_sdio_s *priv = (FAR struct ls2k0300_sdio_s *)dev;
  uint32_t ratekhz;

  switch (rate)
    {
      case CLOCK_SDIO_DISABLED:
        ratekhz = 0;
        break;

      case CLOCK_IDMODE:
        ratekhz = LS2K0300_SDIO_ID_CLOCK;
        break;

      case CLOCK_MMC_TRANSFER:
      case CLOCK_SD_TRANSFER_1BIT:
      case CLOCK_SD_TRANSFER_4BIT:
      default:
        ratekhz = CONFIG_LS2K0300_SDIO_MAXFREQ / 1000;
        break;
    }

  ls2k0300_sdio_setclock(priv, ratekhz);
}

/****************************************************************************
 * Name: ls2k0300_sdio_attach
 *
 * Description:
 *   There is no card detect interrupt; nothing to attach.
 *
 ****************************************************************************/

static int ls2k0300_sdio_attach(FAR struct sdio_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: ls2k0300_sdio_sendcmd
 *
 * Description:
 *   Send a command and wait for the command phase to complete.  The
 *   command encoding used by the MMCSd core carries the response type and
 *   the data transfer direction (see include/nuttx/sdio.h).
 *
 ****************************************************************************/

static int ls2k0300_sdio_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd,
                                 uint32_t arg)
{
  FAR struct ls2k0300_sdio_s *priv = (FAR struct ls2k0300_sdio_s *)dev;
  uint32_t resptype = cmd & MMCSD_RESPONSE_MASK;
  uint32_t cmdindex = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  uint32_t status;
  uint32_t cmcon;
  int ret;

  finfo("cmd=%02" PRIx32 " arg=%08" PRIx32 " data=%d\n",
        cmdindex, arg, (cmd & MMCSD_DATAXFR_MASK) != 0);

  priv->cmdstatus = 0;

  /* Drop the status latches left over from the previous command */

  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_INTSTA, SDIO_INT_ALL);

  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_CMDARG, arg);

  cmcon = (cmdindex & SDIO_CMDCON_INDEX_MASK) |
          SDIO_CMDCON_SENDERHOST | SDIO_CMDCON_START;

  if (resptype != MMCSD_NO_RESPONSE)
    {
      cmcon |= SDIO_CMDCON_WAITRSP;
    }

  if (resptype == MMCSD_R2_RESPONSE)
    {
      /* The reference driver notes that enabling the CRC check for a 136
       * bit response makes the controller misbehave.
       */

      cmcon |= SDIO_CMDCON_LONGRSP;
    }
  else if (resptype != MMCSD_NO_RESPONSE &&
           resptype != MMCSD_R3_RESPONSE &&
           resptype != MMCSD_R4_RESPONSE)
    {
      /* R3 and R4 carry no CRC field in the response, so checking it would
       * report a spurious error (this is what U-Boot does with its
       * MMC_RSP_CRC flag).
       */

      cmcon |= SDIO_CMDCON_CHECKCRC;
    }

  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_CMDCON, cmcon);

  ret = ls2k0300_sdio_pollstat(priv, SDIO_INT_CMDMASK,
                               LS2K0300_SDIO_CMD_TMO_MS, &status);
  priv->cmdstatus = status;

  /* Acknowledge the command status latches only: a data phase armed by
   * recvsetup()/sendsetup() may already have finished and its status bit
   * must survive until eventwait() looks at it.
   */

  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_INTSTA,
                       status & SDIO_INT_CMDMASK);

  if (ret < 0)
    {
      mcerr("ERROR: CMD%" PRIu32 " timeout, status=%08" PRIx32 "\n",
            cmdindex, status);
      return -ETIMEDOUT;
    }

  return OK;
}

/****************************************************************************
 * Name: ls2k0300_sdio_blocksetup
 *
 * Description:
 *   Report the block size and the block count of the next transfer.  The
 *   controller needs both before the data phase is started.
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_BLOCKSETUP
static void ls2k0300_sdio_blocksetup(FAR struct sdio_dev_s *dev,
                                     unsigned int blocklen,
                                     unsigned int nblocks)
{
  FAR struct ls2k0300_sdio_s *priv = (FAR struct ls2k0300_sdio_s *)dev;

  finfo("blocklen=%u nblocks=%u\n", blocklen, nblocks);
  priv->blocksize = blocklen;
  priv->nblocks   = nblocks;
}
#endif

/****************************************************************************
 * Name: ls2k0300_sdio_recvsetup / ls2k0300_sdio_sendsetup
 *
 * Description:
 *   Set up the DMA and the data path for the transfer that the command
 *   that follows is going to trigger.
 *
 ****************************************************************************/

static int ls2k0300_sdio_recvsetup(FAR struct sdio_dev_s *dev,
                                   FAR uint8_t *buffer, size_t nbytes)
{
  FAR struct ls2k0300_sdio_s *priv = (FAR struct ls2k0300_sdio_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);

  priv->buffer    = buffer;
  priv->buflen    = nbytes;
  priv->receiving = true;

  return ls2k0300_sdio_datasetup(priv);
}

static int ls2k0300_sdio_sendsetup(FAR struct sdio_dev_s *dev,
                                   FAR const uint8_t *buffer, size_t nbytes)
{
  FAR struct ls2k0300_sdio_s *priv = (FAR struct ls2k0300_sdio_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);

  priv->buffer    = (FAR uint8_t *)(uintptr_t)buffer;
  priv->buflen    = nbytes;
  priv->receiving = false;

  return ls2k0300_sdio_datasetup(priv);
}

/****************************************************************************
 * Name: ls2k0300_sdio_cancel
 *
 * Description:
 *   Cancel the data transfer in progress.
 *
 ****************************************************************************/

static int ls2k0300_sdio_cancel(FAR struct sdio_dev_s *dev)
{
  FAR struct ls2k0300_sdio_s *priv = (FAR struct ls2k0300_sdio_s *)dev;

  mcerr("Cancelling the transfer\n");
  ls2k0300_sdio_datastop(priv);
  return OK;
}

/****************************************************************************
 * Name: ls2k0300_sdio_waitresponse
 *
 * Description:
 *   Wait for the response to the last command.  The command phase has
 *   already been polled to completion in ls2k0300_sdio_sendcmd(), so this
 *   only classifies the outcome.
 *
 ****************************************************************************/

static int ls2k0300_sdio_waitresponse(FAR struct sdio_dev_s *dev,
                                      uint32_t cmd)
{
  FAR struct ls2k0300_sdio_s *priv = (FAR struct ls2k0300_sdio_s *)dev;
  uint32_t resptype = cmd & MMCSD_RESPONSE_MASK;
  uint32_t status = priv->cmdstatus;

  if ((status & SDIO_INT_CMDTOUT) != 0)
    {
      mcerr("ERROR: response timeout status=%08" PRIx32 "\n", status);
      return -ETIMEDOUT;
    }

  if ((status & SDIO_INT_RSPCRC) != 0)
    {
      /* A CRC error is only fatal for the 48 bit responses: the CRC of a
       * 136 bit response is not checked by the card specification and the
       * controller reports spurious errors for some cards.
       */

      if (resptype != MMCSD_R2_RESPONSE)
        {
          mcerr("ERROR: response CRC error status=%08" PRIx32 "\n", status);
          return -EIO;
        }
    }

  if ((status & (SDIO_INT_CMDFIN | SDIO_INT_R1BFIN)) == 0)
    {
      mcerr("ERROR: no response status=%08" PRIx32 "\n", status);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: ls2k0300_sdio_recvshortcrc / ls2k0300_sdio_recvlong /
 *       ls2k0300_sdio_recvshort
 *
 * Description:
 *   Return the response of the last command.  The controller stores a 48
 *   bit response in SDI_RSP0 and a 136 bit response in SDI_RSP0..3, with
 *   SDI_RSP0 holding the most significant word.
 *
 ****************************************************************************/

static int ls2k0300_sdio_recvshortcrc(FAR struct sdio_dev_s *dev,
                                      uint32_t cmd, FAR uint32_t *rshort)
{
  FAR struct ls2k0300_sdio_s *priv = (FAR struct ls2k0300_sdio_s *)dev;
  uint32_t status = priv->cmdstatus;

  if ((status & SDIO_INT_CMDTOUT) != 0)
    {
      return -ETIMEDOUT;
    }

  if ((status & SDIO_INT_RSPCRC) != 0)
    {
      return -EIO;
    }

  if (rshort != NULL)
    {
      *rshort = ls2k0300_sdio_getreg(priv, LS2K0300_SDIO_RSP0);
    }

  return OK;
}

static int ls2k0300_sdio_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd,
                                  FAR uint32_t rlong[4])
{
  FAR struct ls2k0300_sdio_s *priv = (FAR struct ls2k0300_sdio_s *)dev;
  uint32_t status = priv->cmdstatus;

  if ((status & SDIO_INT_CMDTOUT) != 0)
    {
      return -ETIMEDOUT;
    }

  if (rlong != NULL)
    {
      rlong[0] = ls2k0300_sdio_getreg(priv, LS2K0300_SDIO_RSP0);
      rlong[1] = ls2k0300_sdio_getreg(priv, LS2K0300_SDIO_RSP1);
      rlong[2] = ls2k0300_sdio_getreg(priv, LS2K0300_SDIO_RSP2);
      rlong[3] = ls2k0300_sdio_getreg(priv, LS2K0300_SDIO_RSP3);
    }

  return OK;
}

static int ls2k0300_sdio_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd,
                                   FAR uint32_t *rshort)
{
  FAR struct ls2k0300_sdio_s *priv = (FAR struct ls2k0300_sdio_s *)dev;
  uint32_t status = priv->cmdstatus;

  if ((status & SDIO_INT_CMDTOUT) != 0)
    {
      return -ETIMEDOUT;
    }

  if (rshort != NULL)
    {
      *rshort = ls2k0300_sdio_getreg(priv, LS2K0300_SDIO_RSP0);
    }

  return OK;
}

/****************************************************************************
 * Name: ls2k0300_sdio_waitenable
 *
 * Description:
 *   Enable/disable of event set used in the following eventwait call.
 *
 ****************************************************************************/

static void ls2k0300_sdio_waitenable(FAR struct sdio_dev_s *dev,
                                     sdio_eventset_t eventset,
                                     uint32_t timeout)
{
  FAR struct ls2k0300_sdio_s *priv = (FAR struct ls2k0300_sdio_s *)dev;

  priv->waitevents = eventset;
  priv->wtimeoutms = timeout;
}

/****************************************************************************
 * Name: ls2k0300_sdio_eventwait
 *
 * Description:
 *   Wait for the armed event.  Only the completion of a data phase is
 *   asynchronous: the command phase is already complete when this is
 *   called.
 *
 ****************************************************************************/

static sdio_eventset_t ls2k0300_sdio_eventwait(FAR struct sdio_dev_s *dev)
{
  FAR struct ls2k0300_sdio_s *priv = (FAR struct ls2k0300_sdio_s *)dev;
  sdio_eventset_t events = 0;
  uint32_t status;
  uint32_t timeoutms;
  int ret;

  if (!priv->xferpending)
    {
      /* Command only: nothing left to wait for */

      return (priv->waitevents & (SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE))
             ? SDIOWAIT_CMDDONE : SDIOWAIT_TRANSFERDONE;
    }

  timeoutms = priv->wtimeoutms != 0 ? priv->wtimeoutms
                                    : LS2K0300_SDIO_CMD_TMO_MS;

  ret = ls2k0300_sdio_pollstat(priv, SDIO_INT_DATAMASK, timeoutms, &status);
  ls2k0300_sdio_putreg(priv, LS2K0300_SDIO_INTSTA,
                       status & SDIO_INT_DATAMASK);

  priv->xferpending = false;

  if (ret < 0)
    {
      mcerr("ERROR: data timeout status=%08" PRIx32 "\n", status);
      events = SDIOWAIT_TIMEOUT;
    }
  else if ((status & SDIO_INT_DATAERR) != 0)
    {
      mcerr("ERROR: data error status=%08" PRIx32 "\n", status);
      events = ((status & SDIO_INT_DATTOUT) != 0) ? SDIOWAIT_TIMEOUT
                                                  : SDIOWAIT_ERROR;
    }
  else
    {
      events = SDIOWAIT_TRANSFERDONE;
    }

  return events;
}

/****************************************************************************
 * Name: ls2k0300_sdio_callbackenable
 *
 * Description:
 *   Media insertion/removal events are not supported: the slot has no card
 *   detect line.
 *
 ****************************************************************************/

static void ls2k0300_sdio_callbackenable(FAR struct sdio_dev_s *dev,
                                         sdio_eventset_t eventset)
{
}

/****************************************************************************
 * Name: ls2k0300_sdio_registercallback
 *
 * Description:
 *   Media insertion/removal events are not supported.
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
static int ls2k0300_sdio_registercallback(FAR struct sdio_dev_s *dev,
                                          worker_t callback, FAR void *arg)
{
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ls2k0300_sdio_initialize
 *
 * Description:
 *   Initialize the SDIO controller and register it with the MMC/SD core,
 *   which probes the card and creates /dev/mmcsd0.
 *
 * Returned Value:
 *   A pointer to the SDIO interface on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct sdio_dev_s *ls2k0300_sdio_initialize(void)
{
  FAR struct ls2k0300_sdio_s *priv = &g_sdiodev;
  int ret;

  priv->base      = LS2K0300_SDIO_BASE;
  priv->physbase  = LS2K0300_SDIO_PHYSBASE;
  priv->clkfreq   = CONFIG_LS2K0300_SDIO_CLKFREQ;
  priv->widebus   = false;
  priv->blocksize = 512;
  priv->nblocks   = 1;

  finfo("SDIO at %p (phys 0x%08" PRIx32 ") clk=%" PRIu32 "Hz\n",
        (FAR void *)priv->base, priv->physbase, priv->clkfreq);

  ls2k0300_sdio_softreset(priv);

  ret = mmcsd_slotinitialize(0, &priv->dev);
  if (ret < 0)
    {
      mcerr("ERROR: mmcsd_slotinitialize failed: %d\n", ret);
      return NULL;
    }

  return &priv->dev;
}

#endif /* CONFIG_LS2K0300_SDIO */
