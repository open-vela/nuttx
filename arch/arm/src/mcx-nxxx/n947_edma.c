/****************************************************************************
 * arch/arm/src/mcx-nxxx/n947_edma.c
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
#include <assert.h>
#include <debug.h>
#include <syslog.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/clock.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "chip.h"
#include "nxxx_clockconfig.h"
#include "n947_edma.h"

#include "hardware/nxxx_clock.h"
#include "hardware/n947/n947_dmamux.h"
#include "hardware/n947/n947_edma.h"

#ifdef CONFIG_N947_EDMA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define N947_DMA_IDLE        0
#define N947_DMA_CONFIGURED  1
#define N947_DMA_ACTIVE      2

#define N947_EDMA_SELFTEST_BUFLEN       64
#define N947_EDMA_SELFTEST_ALIGN        32
#define N947_EDMA_SELFTEST_TIMEOUT      MSEC2TICK(100)

#define N947_INPUTMUX_DMA0_REQ_ENABLE_OFFSET(n) (0x700 + ((n) * 0x10))
#define N947_INPUTMUX_DMA_REQ_GROUP(s)          ((unsigned int)(s) >> 5)
#define N947_INPUTMUX_DMA_REQ_BIT(s)            (1u << ((s) & 31))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct n947_dmach_s
{
  uintptr_t base;               /* DMA controller base */
  uint32_t  reqsrc;             /* CH_MUX source value */
  uint32_t  flags;              /* EDMA_CONFIG_* flags */
  uint32_t  nbytes;             /* Saved initial minor loop byte count */
  edma_callback_t callback;     /* Client callback */
  void     *arg;                /* Client callback argument */
  uint8_t   controller;         /* DMA controller number */
  uint8_t   chan;               /* Channel number within the controller */
  uint8_t   irq;                /* Channel IRQ */
  uint8_t   state;              /* NXXX_DMA_* */
  bool      inuse;              /* Channel is allocated */
};

struct n947_edma_controller_s
{
  uintptr_t base;                       /* DMA controller base */
  struct clock_gate_reg_s clock_gate;   /* DMA clock gate */
  uint8_t irqbase;                      /* First channel IRQ */
  struct n947_dmach_s dmach[N947_EDMA_NCHANNELS];
};

struct n947_edma_s
{
  spinlock_t lock;
  bool initialized;
  struct n947_edma_controller_s ctrl[N947_EDMA_NCONTROLLERS];
};

#ifdef CONFIG_N947_EDMA_SELFTEST
struct n947_edma_selftest_s
{
  sem_t sem;
  volatile bool done;
  int result;
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct n947_edma_s g_edma =
{
  .lock = SP_UNLOCKED,
  .ctrl =
  {
    {
      .base       = NXXX_DMA0_BASE,
      .clock_gate = CLOCK_GATE_DMA0,
      .irqbase    = NXXX_IRQ_EDMA_0_CH0,
    },
    {
      .base       = NXXX_DMA1_BASE,
      .clock_gate = CLOCK_GATE_DMA1,
      .irqbase    = NXXX_IRQ_EDMA_1_CH0,
    },
  },
};

#ifdef CONFIG_N947_EDMA_SELFTEST
static uint8_t g_edma_selftest_src[N947_EDMA_SELFTEST_BUFLEN]
  aligned_data(N947_EDMA_SELFTEST_ALIGN);
static uint8_t g_edma_selftest_dst[N947_EDMA_SELFTEST_BUFLEN]
  aligned_data(N947_EDMA_SELFTEST_ALIGN);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uintptr_t n947_edma_chbase(struct n947_dmach_s *dmach)
{
  return N947_EDMA_CH_BASE(dmach->base, dmach->chan);
}

static inline void n947_edma_cleardone(struct n947_dmach_s *dmach)
{
  /* CH_CSR[DONE] is write-one-to-clear.  Avoid accidentally writing DONE
   * when setting other CH_CSR bits unless clearing it is intended.
   */

  putreg32(getreg32(N947_EDMA_CH_CSR(dmach->base, dmach->chan)) |
           EDMA_CH_CSR_DONE,
           N947_EDMA_CH_CSR(dmach->base, dmach->chan));
}

static inline void n947_edma_clearint(struct n947_dmach_s *dmach)
{
  putreg32(getreg32(N947_EDMA_CH_INT(dmach->base, dmach->chan)) |
           EDMA_CH_INT_INT,
           N947_EDMA_CH_INT(dmach->base, dmach->chan));
}

static inline void n947_edma_clearerr(struct n947_dmach_s *dmach)
{
  putreg32(getreg32(N947_EDMA_CH_ES(dmach->base, dmach->chan)) |
           EDMA_CH_ES_ERR,
           N947_EDMA_CH_ES(dmach->base, dmach->chan));
}

static void n947_edma_set_chcsr(struct n947_dmach_s *dmach,
                                uint32_t clearbits, uint32_t setbits)
{
  uintptr_t regaddr = N947_EDMA_CH_CSR(dmach->base, dmach->chan);
  uint32_t regval;

  regval  = getreg32(regaddr);
  regval &= ~EDMA_CH_CSR_DONE;
  regval &= ~clearbits;
  regval |= setbits;
  putreg32(regval, regaddr);
}

static void n947_edma_restore_reqsrc(struct n947_dmach_s *dmach)
{
  unsigned int group;

  if (dmach->reqsrc == DMA_REQUEST_MUXSOFTWARE)
    {
      return;
    }

  putreg32(EDMA_CH_MUX_SRC(dmach->reqsrc),
           N947_EDMA_CH_MUX(dmach->base, dmach->chan));

#ifdef NXXX_INPUTMUX0_BASE
  /* MCX Nxxx gates each DMA request source in INPUTMUX as well as in the
   * per-channel CH_MUX field.  Reassert both before enabling ERQ; otherwise
   * a valid CH_MUX can still leave HRS low and starve a peripheral FIFO.
   */

  if (dmach->controller == 0 && dmach->reqsrc < 128)
    {
      group = N947_INPUTMUX_DMA_REQ_GROUP(dmach->reqsrc);
      modifyreg32(NXXX_INPUTMUX0_BASE +
                  N947_INPUTMUX_DMA0_REQ_ENABLE_OFFSET(group),
                  0, N947_INPUTMUX_DMA_REQ_BIT(dmach->reqsrc));
    }
#endif
}

static void n947_edma_disable(struct n947_dmach_s *dmach)
{
  n947_edma_set_chcsr(dmach,
                      EDMA_CH_CSR_ERQ | EDMA_CH_CSR_EARQ |
                      EDMA_CH_CSR_EEI,
                      0);

  putreg16(0, N947_EDMA_TCD_CSR(dmach->base, dmach->chan));
}

static void n947_edma_reset_channel(struct n947_dmach_s *dmach)
{
  uintptr_t chbase = n947_edma_chbase(dmach);

  n947_edma_disable(dmach);
  n947_edma_clearint(dmach);
  n947_edma_clearerr(dmach);
  n947_edma_cleardone(dmach);

  /* Keep DMA bus transactions non-secure/user by default.  The MCX Nxxx
   * port uses the non-secure peripheral aliases in nxxx_memorymap.h.
   */

  putreg32(0, chbase + N947_EDMA_CH_SBR_OFFSET);

  putreg32(0, chbase + N947_EDMA_TCD_SADDR_OFFSET);
  putreg16(0, chbase + N947_EDMA_TCD_SOFF_OFFSET);
  putreg16(0, chbase + N947_EDMA_TCD_ATTR_OFFSET);
  putreg32(0, chbase + N947_EDMA_TCD_NBYTES_MLOFFNO_OFFSET);
  putreg32(0, chbase + N947_EDMA_TCD_SLAST_SDA_OFFSET);
  putreg32(0, chbase + N947_EDMA_TCD_DADDR_OFFSET);
  putreg16(0, chbase + N947_EDMA_TCD_DOFF_OFFSET);
  putreg16(0, chbase + N947_EDMA_TCD_CITER_ELINKNO_OFFSET);
  putreg32(0, chbase + N947_EDMA_TCD_DLAST_SGA_OFFSET);
  putreg16(0, chbase + N947_EDMA_TCD_CSR_OFFSET);
  putreg16(0, chbase + N947_EDMA_TCD_BITER_ELINKNO_OFFSET);
}

static void n947_edma_finish(struct n947_dmach_s *dmach, int result,
                             bool callback)
{
  edma_callback_t cb;
  void *arg;
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_edma.lock);

  n947_edma_disable(dmach);
  n947_edma_clearint(dmach);
  n947_edma_clearerr(dmach);
  n947_edma_cleardone(dmach);

  cb             = dmach->callback;
  arg            = dmach->arg;
  dmach->callback = NULL;
  dmach->arg      = NULL;
  dmach->state    = N947_DMA_IDLE;

  spin_unlock_irqrestore(&g_edma.lock, flags);

  if (callback && cb != NULL)
    {
      cb((DMACH_HANDLE)dmach, arg, true, result);
    }
}

static int n947_edma_interrupt(int irq, void *context, void *arg)
{
  struct n947_dmach_s *dmach = (struct n947_dmach_s *)arg;
  edma_callback_t cb;
  void *cbarg;
  uint32_t chcsr;
  uint32_t ches;
  uint32_t chint;
  uint32_t mux;
  bool done;

  (void)irq;
  (void)context;

  DEBUGASSERT(dmach != NULL);

  chint = getreg32(N947_EDMA_CH_INT(dmach->base, dmach->chan));
  ches  = getreg32(N947_EDMA_CH_ES(dmach->base, dmach->chan));

  if ((chint & EDMA_CH_INT_INT) == 0 && (ches & EDMA_CH_ES_ERR) == 0)
    {
      return OK;
    }

  /* DMA3 can drop CH_MUX while a cyclic scatter-gather channel remains
   * enabled (ERQ/ESG and the next TCD are still intact).  This was observed
   * on MCXN947 SAI1 TX after several major loops: no channel error is
   * raised, but HRS falls to zero and the peripheral FIFO eventually
   * underruns.
   *
   * Reassert the allocated request source at the major-loop boundary before
   * clearing the interrupt or calling the client.  The comparison makes the
   * normal path read-only, and restoring it here leaves a complete alternate
   * TCD/FIFO interval before the next boundary.
   */

  if (dmach->reqsrc != DMA_REQUEST_MUXSOFTWARE)
    {
      mux = getreg32(N947_EDMA_CH_MUX(dmach->base, dmach->chan)) &
            EDMA_CH_MUX_SRC_MASK;
      if (mux != EDMA_CH_MUX_SRC(dmach->reqsrc))
        {
          n947_edma_restore_reqsrc(dmach);
        }
    }

  n947_edma_clearint(dmach);

  if ((ches & EDMA_CH_ES_ERR) != 0)
    {
      dmaerr("eDMA%u CH%u error: CH_ES=%08" PRIx32 "\n",
             dmach->controller, dmach->chan, ches);
      n947_edma_clearerr(dmach);
      n947_edma_finish(dmach, -EIO, true);
      return OK;
    }

  chcsr = getreg32(N947_EDMA_CH_CSR(dmach->base, dmach->chan));
  done  = (chcsr & EDMA_CH_CSR_DONE) != 0;

  if (done)
    {
      n947_edma_cleardone(dmach);
    }

  if (done &&
      (dmach->flags & (EDMA_CONFIG_LOOP_MASK |
                       EDMA_CONFIG_SCATTERGATHER)) == 0)
    {
      n947_edma_finish(dmach, OK, true);
      return OK;
    }

  cb    = dmach->callback;
  cbarg = dmach->arg;

  if (cb != NULL)
    {
      cb((DMACH_HANDLE)dmach, cbarg, done, OK);
    }

  return OK;
}

#ifdef CONFIG_N947_EDMA_SELFTEST
static void n947_edma_selftest_callback(DMACH_HANDLE handle, void *arg,
                                        bool done, int result)
{
  struct n947_edma_selftest_s *test =
    (struct n947_edma_selftest_s *)arg;

  (void)handle;

  test->done   = done;
  test->result = result;
  nxsem_post(&test->sem);
}

/* Print the full channel/TCD state so a failed selftest tells us WHERE it
 * stopped: never-activated (CITER untouched, dst unchanged), moved-but-no-
 * interrupt (DONE/INT set, dst filled), or bus error (CH_ES).  syslog, not
 * dmaerr: DEBUG_DMA is normally off.
 */

static void n947_edma_selftest_dump(struct n947_dmach_s *dmach)
{
  uintptr_t base   = dmach->base;
  uintptr_t chbase = n947_edma_chbase(dmach);

  syslog(LOG_ERR, "eDMA dump: MP_CSR=%08lx MP_ES=%08lx MP_INT=%08lx"
         " MP_HRS=%08lx\n",
         (unsigned long)getreg32(N947_EDMA_MP_CSR(base)),
         (unsigned long)getreg32(N947_EDMA_MP_ES(base)),
         (unsigned long)getreg32(N947_EDMA_MP_INT(base)),
         (unsigned long)getreg32(N947_EDMA_MP_HRS(base)));
  syslog(LOG_ERR, "eDMA dump: CH%u CSR=%08lx ES=%08lx INT=%08lx SBR=%08lx"
         " PRI=%08lx MUX=%08lx\n", dmach->chan,
         (unsigned long)getreg32(chbase + N947_EDMA_CH_CSR_OFFSET),
         (unsigned long)getreg32(chbase + N947_EDMA_CH_ES_OFFSET),
         (unsigned long)getreg32(chbase + N947_EDMA_CH_INT_OFFSET),
         (unsigned long)getreg32(chbase + N947_EDMA_CH_SBR_OFFSET),
         (unsigned long)getreg32(chbase + N947_EDMA_CH_PRI_OFFSET),
         (unsigned long)getreg32(chbase + N947_EDMA_CH_MUX_OFFSET));
  syslog(LOG_ERR, "eDMA dump: SADDR=%08lx DADDR=%08lx NBYTES=%08lx"
         " SOFF=%04x DOFF=%04x ATTR=%04x\n",
         (unsigned long)getreg32(chbase + N947_EDMA_TCD_SADDR_OFFSET),
         (unsigned long)getreg32(chbase + N947_EDMA_TCD_DADDR_OFFSET),
         (unsigned long)getreg32(
           chbase + N947_EDMA_TCD_NBYTES_MLOFFNO_OFFSET),
         getreg16(chbase + N947_EDMA_TCD_SOFF_OFFSET),
         getreg16(chbase + N947_EDMA_TCD_DOFF_OFFSET),
         getreg16(chbase + N947_EDMA_TCD_ATTR_OFFSET));
  syslog(LOG_ERR, "eDMA dump: CITER=%04x BITER=%04x TCD_CSR=%04x"
         " dst[0..3]=%02x %02x %02x %02x\n",
         getreg16(chbase + N947_EDMA_TCD_CITER_ELINKNO_OFFSET),
         getreg16(chbase + N947_EDMA_TCD_BITER_ELINKNO_OFFSET),
         getreg16(chbase + N947_EDMA_TCD_CSR_OFFSET),
         g_edma_selftest_dst[0], g_edma_selftest_dst[1],
         g_edma_selftest_dst[2], g_edma_selftest_dst[3]);
}

static int n947_edma_selftest_controller(unsigned int controller)
{
  struct n947_edma_selftest_s test;
  struct n947_edma_xfrconfig_s config;
  DMACH_HANDLE handle;
  unsigned int i;
  int ret;

  DEBUGASSERT(controller < N947_EDMA_NCONTROLLERS);

  for (i = 0; i < N947_EDMA_SELFTEST_BUFLEN; i++)
    {
      g_edma_selftest_src[i] = (uint8_t)(0xa5 ^ i);
      g_edma_selftest_dst[i] = (uint8_t)0x5a;
    }

  test.done   = false;
  test.result = -EINPROGRESS;

  ret = nxsem_init(&test.sem, 0, 0);
  if (ret < 0)
    {
      return ret;
    }

  handle = n947_dmach_alloc(N947_DMA_REQUEST(controller,
                                            DMA_REQUEST_MUXSOFTWARE),
                            0);
  if (handle == NULL)
    {
      ret = -ENODEV;
      goto out_destroy_sem;
    }

  /* One TCD_CSR.START write activates exactly ONE minor loop.  A software-
   * triggered mem-to-mem transfer must therefore carry the whole payload in
   * a single minor loop (nbytes = total, iter = 1); with nbytes=1/iter=64
   * only the first byte moves and the major loop never completes.
   */

  memset(&config, 0, sizeof(config));
  config.saddr  = (uintptr_t)g_edma_selftest_src;
  config.daddr  = (uintptr_t)g_edma_selftest_dst;
  config.soff   = 1;
  config.doff   = 1;
  config.iter   = 1;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = EDMA_8BIT;
  config.dsize  = EDMA_8BIT;
  config.nbytes = N947_EDMA_SELFTEST_BUFLEN;

  ret = n947_dmach_xfrsetup(handle, &config);
  if (ret < 0)
    {
      goto out_free_channel;
    }

  up_clean_dcache((uintptr_t)g_edma_selftest_src,
                  (uintptr_t)g_edma_selftest_src +
                  N947_EDMA_SELFTEST_BUFLEN);
  up_invalidate_dcache((uintptr_t)g_edma_selftest_dst,
                       (uintptr_t)g_edma_selftest_dst +
                       N947_EDMA_SELFTEST_BUFLEN);

  ret = n947_dmach_start(handle, n947_edma_selftest_callback, &test);
  if (ret < 0)
    {
      goto out_free_channel;
    }

  ret = nxsem_tickwait_uninterruptible(&test.sem,
                                       N947_EDMA_SELFTEST_TIMEOUT);
  if (ret < 0)
    {
      dmaerr("eDMA%u selftest timeout: %d\n", controller, ret);
      n947_edma_selftest_dump((struct n947_dmach_s *)handle);
      goto out_free_channel;
    }

  up_invalidate_dcache((uintptr_t)g_edma_selftest_dst,
                       (uintptr_t)g_edma_selftest_dst +
                       N947_EDMA_SELFTEST_BUFLEN);

  if (!test.done)
    {
      ret = -EIO;
      dmaerr("eDMA%u selftest callback before DONE\n", controller);
      goto out_free_channel;
    }

  if (test.result < 0)
    {
      ret = test.result;
      dmaerr("eDMA%u selftest callback failed: %d\n", controller, ret);
      goto out_free_channel;
    }

  if (n947_dmach_idle(handle) != 0)
    {
      ret = -EIO;
      dmaerr("eDMA%u selftest channel still active\n", controller);
      goto out_free_channel;
    }

  if (memcmp(g_edma_selftest_src, g_edma_selftest_dst,
             N947_EDMA_SELFTEST_BUFLEN) != 0)
    {
      ret = -EFAULT;
      dmaerr("eDMA%u selftest data mismatch\n", controller);
      goto out_free_channel;
    }

  dmainfo("eDMA%u selftest ok\n", controller);
  ret = OK;

out_free_channel:
  n947_dmach_free(handle);

out_destroy_sem:
  nxsem_destroy(&test.sem);
  return ret;
}
#endif

static void n947_edma_initialize_once(void)
{
  irqstate_t flags;
  unsigned int controller;
  unsigned int chan;

  flags = spin_lock_irqsave(&g_edma.lock);

  if (g_edma.initialized)
    {
      spin_unlock_irqrestore(&g_edma.lock, flags);
      return;
    }

  g_edma.initialized = true;
  spin_unlock_irqrestore(&g_edma.lock, flags);

#ifdef CLOCK_GATE_INPUTMUX
  nxxx_set_clock_gate(CLOCK_GATE_INPUTMUX, true);
#endif

  for (controller = 0; controller < N947_EDMA_NCONTROLLERS; controller++)
    {
      struct n947_edma_controller_s *ctrl = &g_edma.ctrl[controller];
      uint32_t regval;

      nxxx_set_clock_gate(ctrl->clock_gate, true);

      regval = getreg32(N947_EDMA_MP_CSR(ctrl->base));
      regval &= ~(EDMA_MP_CSR_HALT | EDMA_MP_CSR_CX | EDMA_MP_CSR_ECX);
      regval |= EDMA_MP_CSR_ERCA;
      putreg32(regval, N947_EDMA_MP_CSR(ctrl->base));

      for (chan = 0; chan < N947_EDMA_NCHANNELS; chan++)
        {
          struct n947_dmach_s *dmach = &ctrl->dmach[chan];

          dmach->base       = ctrl->base;
          dmach->controller = controller;
          dmach->chan       = chan;
          dmach->irq        = ctrl->irqbase + chan;
          dmach->state      = N947_DMA_IDLE;

          n947_edma_reset_channel(dmach);
          putreg32(0, N947_EDMA_CH_MUX(ctrl->base, chan));
          putreg32(EDMA_CH_PRI_APL(chan & 7),
                   N947_EDMA_CH_PRI(ctrl->base, chan));

          irq_attach(dmach->irq, n947_edma_interrupt, dmach);
          up_enable_irq(dmach->irq);
        }
    }
}

static int n947_edma_program_tcd(struct n947_dmach_s *dmach,
                                 const struct n947_edma_xfrconfig_s *config)
{
  uintptr_t chbase = n947_edma_chbase(dmach);
  uint16_t attr;
  uint16_t csr;
  uint16_t iter;
  uint32_t nbytes;
  int32_t slast;
  int32_t dlast;
  uint64_t majorbytes;

  if (config->iter == 0 || config->nbytes == 0)
    {
      return -EINVAL;
    }

  if (config->iter > EDMA_TCD_CITER_ELINKNO_CITER_MASK ||
      (config->nbytes & ~EDMA_TCD_NBYTES_MLOFFNO_NBYTES_MASK) != 0 ||
      config->ssize > EDMA_64BYTE ||
      config->dsize > EDMA_64BYTE)
    {
      return -EINVAL;
    }

  if ((config->flags & EDMA_CONFIG_LOOP_MASK) == EDMA_CONFIG_LOOP_MASK)
    {
      return -EINVAL;
    }

  if ((config->flags & EDMA_CONFIG_LINKTYPE_MASK) !=
      EDMA_CONFIG_LINKTYPE_LINKNONE)
    {
      return -ENOSYS;
    }

  majorbytes = (uint64_t)config->nbytes * (uint64_t)config->iter;
  if ((config->flags & EDMA_CONFIG_LOOP_MASK) != 0 &&
      majorbytes > INT32_MAX)
    {
      return -E2BIG;
    }

  iter = config->iter & EDMA_TCD_CITER_ELINKNO_CITER_MASK;
  nbytes = EDMA_TCD_NBYTES_MLOFFNO_NBYTES(config->nbytes);

  attr = EDMA_TCD_ATTR_SSIZE(config->ssize) |
         EDMA_TCD_ATTR_DSIZE(config->dsize);

  slast = (config->flags & EDMA_CONFIG_LOOPSRC) != 0 ?
          -(int32_t)majorbytes : 0;
  dlast = (config->flags & EDMA_CONFIG_LOOPDEST) != 0 ?
          -(int32_t)majorbytes : 0;

  csr = 0;
  if ((config->flags & EDMA_CONFIG_LOOP_MASK) == 0)
    {
      csr |= EDMA_TCD_CSR_DREQ;
    }

  if ((config->flags & EDMA_CONFIG_INTHALF) != 0)
    {
      csr |= EDMA_TCD_CSR_INTHALF;
    }

  /* The existing UART/I2C users expect the supplied callback to run when a
   * normal one-shot transfer finishes even if EDMA_CONFIG_INTMAJOR was not
   * explicitly set.
   */

  csr |= EDMA_TCD_CSR_INTMAJOR;

  n947_edma_disable(dmach);
  n947_edma_clearint(dmach);
  n947_edma_clearerr(dmach);
  n947_edma_cleardone(dmach);

  /* Clear TCD_CSR before installing a new descriptor.  This mirrors the NXP
   * SDK sequence and avoids stale DONE/ESG state affecting the next setup.
   */

  putreg16(0, chbase + N947_EDMA_TCD_CSR_OFFSET);

  putreg32((uint32_t)config->saddr, chbase + N947_EDMA_TCD_SADDR_OFFSET);
  putreg16((uint16_t)config->soff, chbase + N947_EDMA_TCD_SOFF_OFFSET);
  putreg16(attr, chbase + N947_EDMA_TCD_ATTR_OFFSET);
  putreg32(nbytes, chbase + N947_EDMA_TCD_NBYTES_MLOFFNO_OFFSET);
  putreg32((uint32_t)slast, chbase + N947_EDMA_TCD_SLAST_SDA_OFFSET);
  putreg32((uint32_t)config->daddr, chbase + N947_EDMA_TCD_DADDR_OFFSET);
  putreg16((uint16_t)config->doff, chbase + N947_EDMA_TCD_DOFF_OFFSET);
  putreg16(EDMA_TCD_CITER_ELINKNO_CITER(iter),
           chbase + N947_EDMA_TCD_CITER_ELINKNO_OFFSET);
  putreg32((uint32_t)dlast, chbase + N947_EDMA_TCD_DLAST_SGA_OFFSET);
  putreg16(csr, chbase + N947_EDMA_TCD_CSR_OFFSET);
  putreg16(EDMA_TCD_BITER_ELINKNO_BITER(iter),
           chbase + N947_EDMA_TCD_BITER_ELINKNO_OFFSET);

  dmach->flags  = config->flags;
  dmach->nbytes = config->nbytes;
  return OK;
}

static int n947_edma_build_tcd(
  const struct n947_edma_xfrconfig_s *config,
  uintptr_t next, struct n947_edma_tcd_s *tcd)
{
  uint64_t majorbytes;
  uint16_t csr;

  if (config == NULL || tcd == NULL ||
      config->iter == 0 || config->nbytes == 0)
    {
      return -EINVAL;
    }

  if (config->iter > EDMA_TCD_CITER_ELINKNO_CITER_MASK ||
      (config->nbytes & ~EDMA_TCD_NBYTES_MLOFFNO_NBYTES_MASK) != 0 ||
      config->ssize > EDMA_64BYTE ||
      config->dsize > EDMA_64BYTE ||
      (config->flags & EDMA_CONFIG_LINKTYPE_MASK) !=
        EDMA_CONFIG_LINKTYPE_LINKNONE)
    {
      return -EINVAL;
    }

  /* DLAST_SGA is occupied by the next descriptor address in scatter-gather
   * mode, so destination looping cannot be combined with a link.
   */

  if (next != 0 &&
      (config->flags & EDMA_CONFIG_LOOPDEST) != 0)
    {
      return -EINVAL;
    }

  if (next != 0 && (next & 31) != 0)
    {
      return -EINVAL;
    }

  majorbytes = (uint64_t)config->nbytes * config->iter;
  if ((config->flags & EDMA_CONFIG_LOOP_MASK) != 0 &&
      majorbytes > INT32_MAX)
    {
      return -E2BIG;
    }

  memset(tcd, 0, sizeof(*tcd));
  tcd->saddr  = (uint32_t)config->saddr;
  tcd->soff   = (uint16_t)config->soff;
  tcd->attr   = EDMA_TCD_ATTR_SSIZE(config->ssize) |
                EDMA_TCD_ATTR_DSIZE(config->dsize);
  tcd->nbytes = EDMA_TCD_NBYTES_MLOFFNO_NBYTES(config->nbytes);
  tcd->slast  = (config->flags & EDMA_CONFIG_LOOPSRC) != 0 ?
                (uint32_t)-(int32_t)majorbytes : 0;
  tcd->daddr  = (uint32_t)config->daddr;
  tcd->doff   = (uint16_t)config->doff;
  tcd->citer  = EDMA_TCD_CITER_ELINKNO_CITER(config->iter);
  tcd->biter  = EDMA_TCD_BITER_ELINKNO_BITER(config->iter);

  csr = EDMA_TCD_CSR_INTMAJOR;
  if ((config->flags & EDMA_CONFIG_INTHALF) != 0)
    {
      csr |= EDMA_TCD_CSR_INTHALF;
    }

  if (next != 0)
    {
      tcd->dlast_sga = (uint32_t)next;
      csr |= EDMA_TCD_CSR_ESG;
    }
  else
    {
      tcd->dlast_sga =
        (config->flags & EDMA_CONFIG_LOOPDEST) != 0 ?
        (uint32_t)-(int32_t)majorbytes : 0;
      if ((config->flags & EDMA_CONFIG_LOOP_MASK) == 0)
        {
          csr |= EDMA_TCD_CSR_DREQ;
        }
    }

  tcd->csr = csr;
  return OK;
}

static void n947_edma_install_tcd(struct n947_dmach_s *dmach,
                                  const struct n947_edma_tcd_s *tcd)
{
  uintptr_t chbase = n947_edma_chbase(dmach);

  /* MCUXpresso clears CSR before installing ESG because a stale DONE bit
   * prevents the hardware from accepting the scatter-gather enable bit.
   */

  putreg16(0, chbase + N947_EDMA_TCD_CSR_OFFSET);
  putreg32(tcd->saddr, chbase + N947_EDMA_TCD_SADDR_OFFSET);
  putreg16(tcd->soff, chbase + N947_EDMA_TCD_SOFF_OFFSET);
  putreg16(tcd->attr, chbase + N947_EDMA_TCD_ATTR_OFFSET);
  putreg32(tcd->nbytes, chbase + N947_EDMA_TCD_NBYTES_MLOFFNO_OFFSET);
  putreg32(tcd->slast, chbase + N947_EDMA_TCD_SLAST_SDA_OFFSET);
  putreg32(tcd->daddr, chbase + N947_EDMA_TCD_DADDR_OFFSET);
  putreg16(tcd->doff, chbase + N947_EDMA_TCD_DOFF_OFFSET);
  putreg16(tcd->citer, chbase + N947_EDMA_TCD_CITER_ELINKNO_OFFSET);
  putreg32(tcd->dlast_sga, chbase + N947_EDMA_TCD_DLAST_SGA_OFFSET);
  putreg16(tcd->csr, chbase + N947_EDMA_TCD_CSR_OFFSET);
  putreg16(tcd->biter, chbase + N947_EDMA_TCD_BITER_ELINKNO_OFFSET);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void weak_function arm_dma_initialize(void)
{
  n947_edma_initialize_once();
}

DMACH_HANDLE n947_dmach_alloc(uint32_t reqsrc, uint8_t priority)
{
  struct n947_dmach_s *dmach = NULL;
  uint32_t source = N947_DMA_REQUEST_SOURCE(reqsrc);
  unsigned int first_controller;
  unsigned int last_controller;
  unsigned int controller;
  unsigned int chan;
  irqstate_t flags;

  if ((reqsrc & ~(N947_DMA_REQUEST_CONTROLLER_MASK |
                  N947_DMA_REQUEST_SOURCE_MASK)) != 0)
    {
      return NULL;
    }

  n947_edma_initialize_once();

  if ((reqsrc & N947_DMA_REQUEST_CONTROLLER_MASK) != 0)
    {
      first_controller = N947_DMA_REQUEST_CONTROLLER(reqsrc);
      last_controller  = first_controller;
    }
  else
    {
      first_controller = 0;
      last_controller  = N947_EDMA_NCONTROLLERS - 1;
    }

  flags = spin_lock_irqsave(&g_edma.lock);

  for (controller = first_controller;
       controller <= last_controller && dmach == NULL;
       controller++)
    {
      for (chan = 0; chan < N947_EDMA_NCHANNELS; chan++)
        {
          struct n947_dmach_s *candidate;

          candidate = &g_edma.ctrl[controller].dmach[chan];

          if (!candidate->inuse)
            {
              dmach           = candidate;
              dmach->inuse    = true;
              dmach->state    = N947_DMA_IDLE;
              dmach->reqsrc   = source;
              dmach->callback = NULL;
              dmach->arg      = NULL;
              dmach->flags    = 0;
              dmach->nbytes   = 0;
              break;
            }
        }
    }

  spin_unlock_irqrestore(&g_edma.lock, flags);

  if (dmach == NULL)
    {
      dmaerr("No free eDMA channel for request %" PRIu32 "\n", source);
      return NULL;
    }

  n947_edma_reset_channel(dmach);
  n947_edma_restore_reqsrc(dmach);
  putreg32(EDMA_CH_PRI_APL(priority & 7),
           N947_EDMA_CH_PRI(dmach->base, dmach->chan));

  dmainfo("eDMA%u CH%u req=%" PRIu32 "\n",
          dmach->controller, dmach->chan, source);
  return (DMACH_HANDLE)dmach;
}

void n947_dmach_free(DMACH_HANDLE handle)
{
  struct n947_dmach_s *dmach = (struct n947_dmach_s *)handle;
  irqstate_t flags;

  DEBUGASSERT(dmach != NULL);

  n947_edma_finish(dmach, -EINTR, false);
  putreg32(0, N947_EDMA_CH_MUX(dmach->base, dmach->chan));

  flags = spin_lock_irqsave(&g_edma.lock);
  dmach->inuse = false;
  dmach->reqsrc = 0;
  spin_unlock_irqrestore(&g_edma.lock, flags);
}

int n947_dmach_xfrsetup(DMACH_HANDLE handle,
                        const struct n947_edma_xfrconfig_s *config)
{
  struct n947_dmach_s *dmach = (struct n947_dmach_s *)handle;
  int ret;

  DEBUGASSERT(dmach != NULL && config != NULL && dmach->inuse);

  if (dmach->state == N947_DMA_ACTIVE)
    {
      return -EBUSY;
    }

  ret = n947_edma_program_tcd(dmach, config);
  if (ret < 0)
    {
      return ret;
    }

  n947_edma_restore_reqsrc(dmach);

  dmach->state = N947_DMA_CONFIGURED;
  return OK;
}

int n947_dmach_sgsetup(DMACH_HANDLE handle,
                       const struct n947_edma_xfrconfig_s *configs,
                       unsigned int count,
                       struct n947_edma_tcd_s *tcds)
{
  struct n947_dmach_s *dmach = (struct n947_dmach_s *)handle;
  unsigned int i;
  int ret;

  DEBUGASSERT(dmach != NULL && dmach->inuse);

  if (configs == NULL || tcds == NULL || count < 2 ||
      ((uintptr_t)tcds & 31) != 0)
    {
      return -EINVAL;
    }

  if (dmach->state == N947_DMA_ACTIVE)
    {
      return -EBUSY;
    }

  for (i = 0; i < count; i++)
    {
      uintptr_t next = (uintptr_t)&tcds[(i + 1) % count];

      ret = n947_edma_build_tcd(&configs[i], next, &tcds[i]);
      if (ret < 0)
        {
          return ret;
        }
    }

  up_clean_dcache((uintptr_t)tcds,
                  (uintptr_t)tcds + count * sizeof(*tcds));

  n947_edma_disable(dmach);
  n947_edma_clearint(dmach);
  n947_edma_clearerr(dmach);
  n947_edma_cleardone(dmach);
  n947_edma_install_tcd(dmach, &tcds[0]);

  n947_edma_restore_reqsrc(dmach);

  dmach->flags  = EDMA_CONFIG_SCATTERGATHER;
  dmach->nbytes = configs[0].nbytes;
  dmach->state  = N947_DMA_CONFIGURED;
  return OK;
}

int n947_dmach_start(DMACH_HANDLE handle,
                     edma_callback_t callback, void *arg)
{
  struct n947_dmach_s *dmach = (struct n947_dmach_s *)handle;
  irqstate_t flags;

  DEBUGASSERT(dmach != NULL && dmach->inuse);

  if (dmach->state != N947_DMA_CONFIGURED &&
      dmach->state != N947_DMA_ACTIVE)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&g_edma.lock);
  dmach->callback = callback;
  dmach->arg      = arg;
  dmach->state    = N947_DMA_ACTIVE;
  spin_unlock_irqrestore(&g_edma.lock, flags);

  n947_edma_clearint(dmach);
  n947_edma_clearerr(dmach);
  n947_edma_cleardone(dmach);

  if (dmach->reqsrc == DMA_REQUEST_MUXSOFTWARE)
    {
      /* Software trigger: no hardware request line exists (CH_MUX = 0),
       * so ERQ is meaningless here.  TCD_CSR.START alone activates the
       * channel for one minor loop.  Keep EEI so bus errors still raise
       * the error interrupt.
       */

      n947_edma_set_chcsr(dmach, EDMA_CH_CSR_ERQ | EDMA_CH_CSR_EARQ,
                          EDMA_CH_CSR_EEI);
      putreg16(getreg16(N947_EDMA_TCD_CSR(dmach->base, dmach->chan)) |
               EDMA_TCD_CSR_START,
               N947_EDMA_TCD_CSR(dmach->base, dmach->chan));
    }
  else
    {
      n947_edma_restore_reqsrc(dmach);
      n947_edma_set_chcsr(dmach, 0,
                          EDMA_CH_CSR_EEI | EDMA_CH_CSR_ERQ |
                          EDMA_CH_CSR_EARQ);
    }

  return OK;
}

void n947_dmach_stop(DMACH_HANDLE handle)
{
  struct n947_dmach_s *dmach = (struct n947_dmach_s *)handle;

  DEBUGASSERT(dmach != NULL);
  n947_edma_finish(dmach, -EINTR, false);
}

unsigned int n947_dmach_getcount(DMACH_HANDLE handle)
{
  struct n947_dmach_s *dmach = (struct n947_dmach_s *)handle;
  uint32_t chcsr;
  uint16_t citer;

  DEBUGASSERT(dmach != NULL);

  chcsr = getreg32(N947_EDMA_CH_CSR(dmach->base, dmach->chan));
  if ((chcsr & EDMA_CH_CSR_DONE) != 0)
    {
      return 0;
    }

  citer = getreg16(N947_EDMA_TCD_CITER_ELINKNO(dmach->base, dmach->chan));
  citer &= EDMA_TCD_CITER_ELINKNO_CITER_MASK;

  return (unsigned int)citer;
}

void n947_dmach_dump(DMACH_HANDLE handle)
{
  struct n947_dmach_s *dmach = (struct n947_dmach_s *)handle;
  uintptr_t chbase;

  if (dmach == NULL)
    {
      return;
    }

  chbase = n947_edma_chbase(dmach);
  syslog(LOG_ERR,
         "eDMA%u CH%u: MP_CSR=%08lx HRS=%08lx CH_CSR=%08lx "
         "ES=%08lx INT=%08lx MUX=%08lx\n",
         dmach->controller, dmach->chan,
         (unsigned long)getreg32(N947_EDMA_MP_CSR(dmach->base)),
         (unsigned long)getreg32(N947_EDMA_MP_HRS(dmach->base)),
         (unsigned long)getreg32(chbase + N947_EDMA_CH_CSR_OFFSET),
         (unsigned long)getreg32(chbase + N947_EDMA_CH_ES_OFFSET),
         (unsigned long)getreg32(chbase + N947_EDMA_CH_INT_OFFSET),
         (unsigned long)getreg32(chbase + N947_EDMA_CH_MUX_OFFSET));
  syslog(LOG_ERR,
         "eDMA%u CH%u: SADDR=%08lx DADDR=%08lx CITER=%04x "
         "BITER=%04x TCD_CSR=%04x DLAST_SGA=%08lx\n",
         dmach->controller, dmach->chan,
         (unsigned long)getreg32(chbase + N947_EDMA_TCD_SADDR_OFFSET),
         (unsigned long)getreg32(chbase + N947_EDMA_TCD_DADDR_OFFSET),
         getreg16(chbase + N947_EDMA_TCD_CITER_ELINKNO_OFFSET),
         getreg16(chbase + N947_EDMA_TCD_BITER_ELINKNO_OFFSET),
         getreg16(chbase + N947_EDMA_TCD_CSR_OFFSET),
         (unsigned long)getreg32(chbase +
                                 N947_EDMA_TCD_DLAST_SGA_OFFSET));
}

unsigned int n947_dmach_idle(DMACH_HANDLE handle)
{
  struct n947_dmach_s *dmach = (struct n947_dmach_s *)handle;

  DEBUGASSERT(dmach != NULL);
  return dmach->state == N947_DMA_IDLE ? 0 : 1;
}

#ifdef CONFIG_N947_EDMA_SELFTEST
int n947_edma_selftest(void)
{
  unsigned int controller;
  int ret;

  for (controller = 0; controller < N947_EDMA_NCONTROLLERS; controller++)
    {
      ret = n947_edma_selftest_controller(controller);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}
#endif

#endif /* CONFIG_N947_EDMA */
