/****************************************************************************
 * arch/arm/src/stm32h7/stm32_sai.c
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
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/cache.h>
#include <nuttx/wdog.h>
#include <nuttx/queue.h>
#include <nuttx/wqueue.h>
#include <nuttx/mutex.h>
#include <nuttx/spinlock.h>
#include <nuttx/semaphore.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_dma.h"
#include "stm32_gpio.h"
#include "stm32_sai.h"
#include "stm32_rcc.h"

#ifdef CONFIG_STM32H7_SAI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO required by this driver
#endif

#ifndef CONFIG_I2S
#  error CONFIG_I2S required by this driver
#endif

#ifdef CONFIG_STM32H7_SAI_POLLING
#  error "Polling SAI not yet supported"
#endif

#ifdef CONFIG_STM32H7_SAI_INTERRUPTS
#  error "Interrupt driven SAI not yet supported"
#endif

#ifndef CONFIG_STM32H7_SAI_DEFAULT_SAMPLERATE
#  define CONFIG_STM32H7_SAI_DEFAULT_SAMPLERATE  (48000)
#endif

#ifndef CONFIG_STM32H7_SAI_DEFAULT_DATALEN
#  define CONFIG_STM32H7_SAI_DEFAULT_DATALEN     (16)
#endif

#ifndef CONFIG_STM32H7_SAI_MAXINFLIGHT
#  define CONFIG_STM32H7_SAI_MAXINFLIGHT         (16)
#endif

/* The kernel clock is derived from PLL2P.  The board header must state what
 * that frequency is; inferring it from the raw divider fields would duplicate
 * the PLL arithmetic that board.h already performs.
 */

#ifdef CONFIG_STM32H7_SAI
#  ifndef STM32_PLL2P_FREQUENCY
#    error "Please define STM32_PLL2P_FREQUENCY in board.h"
#  endif
#  define SAI_KERNEL_FREQUENCY  STM32_PLL2P_FREQUENCY
#endif

/* SAI DMA priority */

#ifdef CONFIG_STM32H7_SAI_DMAPRIO
#  define SAI_DMA_PRIO       CONFIG_STM32H7_SAI_DMAPRIO
#else
#  define SAI_DMA_PRIO       DMA_SCR_PRIVERYHI
#endif

#if (SAI_DMA_PRIO & ~DMA_SCR_PL_MASK) != 0
#  error "Illegal value for CONFIG_STM32H7_SAI_DMAPRIO"
#endif

/* DMA transfer configuration.  The transfer width follows the block's data
 * width: a 16-bit block moves one half-word per request, a 32-bit block one
 * word.  Memory increments, the peripheral address does not.
 *
 * The width matters for PDM: ST's reference configuration for this board
 * reads the PDM bitstream as 16-bit half-words, so the DMA must be sized to
 * match or every other half-word is skipped.
 *
 * The controller runs in direct mode, matching ST's configuration for this
 * board's capture path: no FIFO, single-beat transfers on both sides.  With
 * FIFO mode disabled the burst fields must stay at single, because a burst
 * longer than the peripheral's request granularity raises a transfer error.
 * PFCTRL stays clear: the SAI block drives the request line as a normal DMA
 * request, not as a flow-control handshake.
 */

#define SAI_RXDMA16_CONFIG   (DMA_SCR_DIR_P2M | \
                              DMA_SCR_MINC | SAI_DMA_PRIO | \
                              DMA_SCR_PSIZE_16BITS | DMA_SCR_MSIZE_16BITS | \
                              DMA_SCR_PBURST_SINGLE | DMA_SCR_MBURST_SINGLE)

/* Byte-wide receive, used for PDM.  The PDM interface hands one bitstream
 * byte per request: the data register's upper half repeats the lower one
 * when read as a half-word, which shows up as adjacent identical bytes in
 * the captured stream.
 */

#define SAI_RXDMA8_CONFIG    (DMA_SCR_DIR_P2M | \
                              DMA_SCR_MINC | SAI_DMA_PRIO | \
                              DMA_SCR_PSIZE_8BITS | DMA_SCR_MSIZE_8BITS | \
                              DMA_SCR_PBURST_SINGLE | DMA_SCR_MBURST_SINGLE)

#define SAI_RXDMA32_CONFIG   (DMA_SCR_DIR_P2M | \
                              DMA_SCR_MINC | SAI_DMA_PRIO | \
                              DMA_SCR_PSIZE_32BITS | DMA_SCR_MSIZE_32BITS | \
                              DMA_SCR_PBURST_SINGLE | DMA_SCR_MBURST_SINGLE)

#define SAI_TXDMA16_CONFIG   (DMA_SCR_DIR_M2P | \
                              DMA_SCR_MINC | SAI_DMA_PRIO | \
                              DMA_SCR_PSIZE_16BITS | DMA_SCR_MSIZE_16BITS | \
                              DMA_SCR_PBURST_SINGLE | DMA_SCR_MBURST_SINGLE)

#define SAI_TXDMA32_CONFIG   (DMA_SCR_DIR_M2P | \
                              DMA_SCR_MINC | SAI_DMA_PRIO | \
                              DMA_SCR_PSIZE_32BITS | DMA_SCR_MSIZE_32BITS | \
                              DMA_SCR_PBURST_SINGLE | DMA_SCR_MBURST_SINGLE)

/* Master clock divider arithmetic.  The kernel clock reaches the audio
 * interface through one divider, and NODIV decides what that divider drives:
 *
 *   NODIV = 0   MCLK = CK / MCKDIV, and the frame rate follows from the
 *               fixed 256-cycle master clock per sample:
 *
 *                 FS = CK / (MCKDIV * 256)          for OSR = 0
 *                 FS = CK / (MCKDIV * 256 * 2)      for OSR = 1
 *
 *   NODIV = 1   the same divider feeds the bit clock directly, and the frame
 *               rate follows from the frame length instead:
 *
 *                 FS = CK / (MCKDIV * FRL)
 *
 * PDM uses the NODIV = 1 form.  Its reference configuration asks for a frame
 * rate of eight times the sample rate because the frame is 32 bits wide, so
 * the product is 256 * FS in either case and the divider expression collapses
 * to the same number.  What differs is which register field the divider
 * lands in, and getting that wrong leaves the microphone with no bit clock
 * at all.
 *
 * Rounding is done in tenths of a hertz to keep the significant digits, the
 * same technique the ST HAL uses.  A divider below 1 is clamped to 1: the
 * resulting frame rate is then slow but well defined, which is preferable to
 * writing zero into the field (the encoding is 1-based).
 */

#define SAI_MCKDIV_MIN       (1)
#define SAI_MCKDIV_MAX       (63)

/* PDM frame geometry, taken from the reference configuration ST ships for
 * this board's microphone path: frame length 32, two slots, one active,
 * 16-bit data.  The PDM interface derives its bitstream clock from the master
 * clock, so a 32-bit frame turns MCLK into the 1.024 MHz sample stream the
 * microphone produces, and 64 decimation steps bring that back to 16 kHz.
 */

#define SAI_PDM_FRAME_LENGTH      (32)
#define SAI_PDM_SLOT_COUNT        (2)
#define SAI_PDM_ACTIVE_FRAME_LEN  (1)

/* Microphone pairs declared to the PDM interface, and the slot enable mask.
 * The reference configuration declares two pairs and enables slot 1; slot 0
 * stays off even though the frame has two slots.
 */

#define SAI_PDM_MIC_PAIRS         (2)
#define SAI_PDM_SLOT_ENABLE       (0x0002)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2S buffer container */

struct sai_buffer_s
{
  struct sai_buffer_s *flink;  /* Supports a singly linked list */
  i2s_callback_t callback;     /* Function to call when the transfer completes */
  uint32_t timeout;            /* The timeout value to use with transfers */
  void *arg;                   /* The argument to be returned with the callback */
  struct ap_buffer_s *apb;     /* The audio buffer */
  int result;                  /* The result of the transfer */
};

/* The state of one SAI peripheral sub-block */

struct stm32h7_sai_s
{
  struct i2s_dev_s dev;        /* Externally visible I2S interface */

  /* Callback for changes in sample rate */

  stm32_sai_sampleratecb_t sampleratecb;
  uintptr_t base;              /* SAI block register base address */
  uintptr_t pdmbase;           /* SAI global register base (PDMCR/PDMDLY) */
  uintptr_t gcrbase;           /* SAI global register base (GCR) */
  uint32_t apbclkbit;          /* RCC APB enable bit for this SAI instance */
  uint32_t ccipreg;            /* RCC kernel clock selection register */
  uint32_t ccipsrc;            /* Kernel clock source for this sub-block */
  uint32_t ccipsrcmask;        /* Mask covering the source field */
  uint32_t irq;                /* Peripheral interrupt (unused so far) */
  mutex_t lock;                /* Assures mutually exclusive access to SAI */
  uint32_t frequency;          /* SAI kernel clock frequency */
  uint32_t dmamap;             /* DMAMUX request for this sub-block */
  uint32_t dmamux_ccr;         /* DMAMUX request number, for diagnosis */
  bool pdm;                    /* True: PDM microphone mode */
  uint8_t or;                  /* Oversampling ratio flag */
#ifdef CONFIG_STM32H7_SAI_DMA
  DMA_HANDLE dma;              /* DMA channel handle */
  uint32_t dma_ccr;            /* DMA control register */
  uint32_t dma_scr;            /* DMA stream config as programmed */
  uint32_t dma_sndtr;          /* DMA transfer count as programmed */
  uint32_t dma_spar;           /* DMA peripheral address as programmed */
  uint32_t dma_sm0ar;          /* DMA memory address as programmed */
#endif
  uint8_t datalen;             /* Data width in bits */
  uint32_t samplerate;         /* Data sample rate */
  uint8_t rxenab:1;            /* True: RX transfers enabled */
  uint8_t txenab:1;            /* True: TX transfers enabled */
  struct wdog_s dog;           /* Watchdog that handles timeouts */

  /* Snapshot taken when a transfer ends.  SAI reports overrun and a wrong
   * clock configuration through bits that DMA does not see, so a bare -EIO
   * from the DMA callback is not enough to tell the two apart.  These are
   * written from interrupt level and read afterwards for diagnosis.
   */

  uint8_t last_isr;            /* DMA interrupt status at completion */
  uint32_t last_sr;            /* SAI status register */
  uint32_t last_cr1;           /* SAI configuration register 1 */
  uint32_t last_frcr;          /* SAI frame configuration register */
  uint32_t last_slotr;         /* SAI slot register */
  uint32_t last_pdmcr;         /* SAI PDM control register (PDM only) */
  uint32_t last_dma_scr;       /* DMA stream configuration at start */
  uint32_t last_dma_sndtr;     /* DMA transfer count at start */
  uint32_t last_dma_spar;      /* DMA peripheral address at start */
  uint32_t last_dma_sm0ar;     /* DMA memory address at start */
  uint32_t last_dmamux_ccr;    /* DMAMUX request number */
  sq_queue_t pend;             /* A queue of pending transfers */
  sq_queue_t act;              /* A queue of active transfers */
  sq_queue_t done;             /* A queue of completed transfers */
  struct work_s work;          /* Supports worker thread operations */

  /* Pre-allocated pool of buffer containers */

  sem_t bufsem;                   /* Buffer wait semaphore */
  struct sai_buffer_s *freelist;  /* A list a free buffer containers */
  struct sai_buffer_s containers[CONFIG_STM32H7_SAI_MAXINFLIGHT];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Buffer container helpers */

static struct sai_buffer_s *
                sai_buf_allocate(struct stm32h7_sai_s *priv);
static void     sai_buf_free(struct stm32h7_sai_s *priv,
                  struct sai_buffer_s *bfcontainer);
static void     sai_buf_initialize(struct stm32h7_sai_s *priv);

/* DMA support */

#ifdef CONFIG_STM32H7_SAI_DMA
static void     sai_schedule(struct stm32h7_sai_s *priv, int result);
static void     sai_dma_callback(DMA_HANDLE handle, uint8_t isr, void *arg);
#endif

/* I2S methods */

static uint32_t sai_samplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t sai_datawidth(struct i2s_dev_s *dev, int bits);
static int      sai_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                  i2s_callback_t callback, void *arg, uint32_t timeout);
static int      sai_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                  i2s_callback_t callback, void *arg,
                  uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2S device operations */

static const struct i2s_ops_s g_i2sops =
{
  /* Receiver methods */

  .i2s_rxsamplerate = sai_samplerate,
  .i2s_rxdatawidth  = sai_datawidth,
  .i2s_receive      = sai_receive,

  /* Transmitter methods */

  .i2s_txsamplerate = sai_samplerate,
  .i2s_txdatawidth  = sai_datawidth,
  .i2s_send         = sai_send,
};

/* Per-block description.  Only the blocks listed in the configuration get
 * an instance; the rest reference missing GPIO macros and are compiled out.
 */

#ifdef CONFIG_STM32H7_SAI1_A
static struct stm32h7_sai_s g_sai1a_priv =
{
  .dev.ops     = &g_i2sops,
  .base        = STM32H7_SAI1_A_BASE,
  .pdmbase     = STM32_SAI1_BASE,
  .gcrbase     = STM32H7_SAI1_GCR,
  .apbclkbit   = RCC_APB2ENR_SAI1EN,
  .ccipreg     = STM32_RCC_D2CCIP1R,
  .ccipsrc     = RCC_D2CCIP1R_SAI1SEL_PLL2,
  .ccipsrcmask = RCC_D2CCIP1R_SAI1SEL_MASK,
  .irq         = STM32_IRQ_SAI1,
  .lock        = NXMUTEX_INITIALIZER,
  .frequency   = SAI_KERNEL_FREQUENCY,
  .dmamap      = DMAMAP_DMA12_SAI1A_0,
  .pdm         = true,
  .datalen     = CONFIG_STM32H7_SAI_DEFAULT_DATALEN,
  .samplerate  = CONFIG_STM32H7_SAI_DEFAULT_SAMPLERATE,
  .bufsem      = SEM_INITIALIZER(0),
};
#endif

#ifdef CONFIG_STM32H7_SAI1_B
static struct stm32h7_sai_s g_sai1b_priv =
{
  .dev.ops     = &g_i2sops,
  .base        = STM32H7_SAI1_B_BASE,
  .pdmbase     = STM32_SAI1_BASE,
  .gcrbase     = STM32H7_SAI1_GCR,
  .apbclkbit   = RCC_APB2ENR_SAI1EN,
  .ccipreg     = STM32_RCC_D2CCIP1R,
  .ccipsrc     = RCC_D2CCIP1R_SAI1SEL_PLL2,
  .ccipsrcmask = RCC_D2CCIP1R_SAI1SEL_MASK,
  .irq         = STM32_IRQ_SAI1,
  .lock        = NXMUTEX_INITIALIZER,
  .frequency   = SAI_KERNEL_FREQUENCY,
  .dmamap      = DMAMAP_DMA12_SAI1B_0,
  .pdm         = false,
  .datalen     = CONFIG_STM32H7_SAI_DEFAULT_DATALEN,
  .samplerate  = CONFIG_STM32H7_SAI_DEFAULT_SAMPLERATE,
  .bufsem      = SEM_INITIALIZER(0),
};
#endif

#ifdef CONFIG_STM32H7_SAI2_A
static struct stm32h7_sai_s g_sai2a_priv =
{
  .dev.ops     = &g_i2sops,
  .base        = STM32H7_SAI2_A_BASE,
  .pdmbase     = STM32_SAI2_BASE,
  .gcrbase     = STM32H7_SAI2_GCR,
  .apbclkbit   = RCC_APB2ENR_SAI2EN,
  .ccipreg     = STM32_RCC_D2CCIP1R,
  .ccipsrc     = RCC_D2CCIP1R_SAI23SEL_PLL2,
  .ccipsrcmask = RCC_D2CCIP1R_SAI23SEL_MASK,
  .irq         = STM32_IRQ_SAI2,
  .lock        = NXMUTEX_INITIALIZER,
  .frequency   = SAI_KERNEL_FREQUENCY,
  .dmamap      = DMAMAP_DMA12_SAI2A_0,
  .pdm         = false,
  .datalen     = CONFIG_STM32H7_SAI_DEFAULT_DATALEN,
  .samplerate  = CONFIG_STM32H7_SAI_DEFAULT_SAMPLERATE,
  .bufsem      = SEM_INITIALIZER(0),
};
#endif

#ifdef CONFIG_STM32H7_SAI2_B
static struct stm32h7_sai_s g_sai2b_priv =
{
  .dev.ops     = &g_i2sops,
  .base        = STM32H7_SAI2_B_BASE,
  .pdmbase     = STM32_SAI2_BASE,
  .gcrbase     = STM32H7_SAI2_GCR,
  .apbclkbit   = RCC_APB2ENR_SAI2EN,
  .ccipreg     = STM32_RCC_D2CCIP1R,
  .ccipsrc     = RCC_D2CCIP1R_SAI23SEL_PLL2,
  .ccipsrcmask = RCC_D2CCIP1R_SAI23SEL_MASK,
  .irq         = STM32_IRQ_SAI2,
  .lock        = NXMUTEX_INITIALIZER,
  .frequency   = SAI_KERNEL_FREQUENCY,
  .dmamap      = DMAMAP_DMA12_SAI2B_0,
  .pdm         = false,
  .datalen     = CONFIG_STM32H7_SAI_DEFAULT_DATALEN,
  .samplerate  = CONFIG_STM32H7_SAI_DEFAULT_SAMPLERATE,
  .bufsem      = SEM_INITIALIZER(0),
};
#endif

#ifdef CONFIG_STM32H7_SAI4_A
static struct stm32h7_sai_s g_sai4a_priv =
{
  .dev.ops     = &g_i2sops,
  .base        = STM32H7_SAI4_A_BASE,
  .pdmbase     = STM32_SAI4_BASE,
  .gcrbase     = STM32H7_SAI4_GCR,
  .apbclkbit   = RCC_APB4ENR_SAI4EN,
  .ccipreg     = STM32_RCC_D3CCIPR,
  .ccipsrc     = RCC_D3CCIPR_SAI4ASEL_PLL2,
  .ccipsrcmask = RCC_D3CCIPR_SAI4ASEL_MASK,
  .irq         = STM32_IRQ_SAI4,
  .lock        = NXMUTEX_INITIALIZER,
  .frequency   = SAI_KERNEL_FREQUENCY,
  .dmamap      = DMAMAP_BDMA_SAI4A,
  .pdm         = true,
  .datalen     = CONFIG_STM32H7_SAI_DEFAULT_DATALEN,
  .samplerate  = CONFIG_STM32H7_SAI_DEFAULT_SAMPLERATE,
  .bufsem      = SEM_INITIALIZER(0),
};
#endif

#ifdef CONFIG_STM32H7_SAI4_B
static struct stm32h7_sai_s g_sai4b_priv =
{
  .dev.ops     = &g_i2sops,
  .base        = STM32H7_SAI4_B_BASE,
  .pdmbase     = STM32_SAI4_BASE,
  .gcrbase     = STM32H7_SAI4_GCR,
  .apbclkbit   = RCC_APB4ENR_SAI4EN,
  .ccipreg     = STM32_RCC_D3CCIPR,
  .ccipsrc     = RCC_D3CCIPR_SAI4BSEL_PLL2,
  .ccipsrcmask = RCC_D3CCIPR_SAI4BSEL_MASK,
  .irq         = STM32_IRQ_SAI4,
  .lock        = NXMUTEX_INITIALIZER,
  .frequency   = SAI_KERNEL_FREQUENCY,
  .dmamap      = DMAMAP_BDMA_SAI4B,
  .pdm         = false,
  .datalen     = CONFIG_STM32H7_SAI_DEFAULT_DATALEN,
  .samplerate  = CONFIG_STM32H7_SAI_DEFAULT_SAMPLERATE,
  .bufsem      = SEM_INITIALIZER(0),
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sai_getbitrate
 *
 * Description:
 *   Get the currently configured bitrate
 *
 * Input Parameters:
 *   priv   - private SAI device structure
 *
 * Returned Value:
 *   The current bitrate
 *
 ****************************************************************************/

static inline uint32_t sai_getbitrate(struct stm32h7_sai_s *priv)
{
  /* Calculate the bitrate in Hz */

  return priv->samplerate * priv->datalen;
}

/****************************************************************************
 * Name: sai_getreg
 ****************************************************************************/

static inline uint32_t sai_getreg(struct stm32h7_sai_s *priv, uint8_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: sai_putreg
 ****************************************************************************/

static inline void sai_putreg(struct stm32h7_sai_s *priv, uint8_t offset,
                              uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: sai_modifyreg
 *
 * Description:
 *   Clear and set bits in the SAI register at offset
 *
 ****************************************************************************/

static void sai_modifyreg(struct stm32h7_sai_s *priv, uint8_t offset,
                          uint32_t clrbits, uint32_t setbits)
{
  uint32_t regval;

  regval  = sai_getreg(priv, offset);
  regval &= ~clrbits;
  regval |= setbits;
  sai_putreg(priv, offset, regval);
}

/****************************************************************************
 * Name: sai_mckdivider
 *
 * Description:
 *   Program MCKDIV so that the interface produces 256 * FS worth of bit
 *   clock from the kernel clock.
 *
 *   With NODIV = 0, MCLK = CK / MCKDIV and
 *
 *     MCKDIV = CK / (FS * 256)
 *
 *   With NODIV = 1, the divider feeds the bit clock and the frame length
 *   enters instead:
 *
 *     MCKDIV = CK / (FS * FRL)
 *
 *   PDM picks the second form because that is what ST's reference
 *   configuration does, and it requests a frame rate of FS * 8 so that the
 *   32-bit frame yields 256 * FS.  Both forms reduce to the same numeric
 *   divider for this board: 49.143 MHz / (16000 * 256) = 12.
 *
 *   Only meaningful in master mode; hardware ignores the field in slave
 *   mode, so writing it is harmless there.
 *
 ****************************************************************************/

static void sai_mckdivider(struct stm32h7_sai_s *priv)
{
  uint32_t mckdiv;
  uint32_t tmpval;

  DEBUGASSERT(priv && priv->samplerate > 0 && priv->frequency > 0);

  /* Keep tenths of a hertz during the division to preserve significant
   * digits, then round.  PDM overrides the divisor below, so start from the
   * PCM form.
   */

  tmpval = (priv->frequency * 10) /
           (priv->samplerate * 256 * (priv->or ? 2 : 1));
  mckdiv = tmpval / 10;

  if ((tmpval % 10) > 8)
    {
      mckdiv += 1;
    }

  if (mckdiv < SAI_MCKDIV_MIN)
    {
      mckdiv = SAI_MCKDIV_MIN;
    }
  else if (mckdiv > SAI_MCKDIV_MAX)
    {
      mckdiv = SAI_MCKDIV_MAX;
    }

  /* NODIV selects which way the divider is interpreted.  PCM keeps the
   * master clock at 256 * FS for the codec; PDM takes the bit clock straight
   * from the divider, as ST's reference configuration for this board does.
   */

  sai_modifyreg(priv, STM32H7_SAI_CR1_OFFSET,
                SAI_CR1_MCKDIV_MASK | SAI_CR1_NODIV | SAI_CR1_OSR,
                SAI_CR1_MCKDIV(mckdiv) |
                (priv->pdm ? SAI_CR1_NODIV : 0) |
                (priv->or ? SAI_CR1_OSR : 0));
}

/****************************************************************************
 * Name: sai_timeout
 *
 * Description:
 *   The watchdog timeout without completion of the transfer.
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sai_timeout(wdparm_t arg)
{
  struct stm32h7_sai_s *priv = (struct stm32h7_sai_s *)arg;
  DEBUGASSERT(priv != NULL);

#ifdef CONFIG_STM32H7_SAI_DMA
  /* Cancel the DMA */

  stm32_dmastop(priv->dma);
#endif

  /* Then schedule completion of the transfer to occur on the worker
   * thread.
   */

  sai_schedule(priv, -ETIMEDOUT);
}

/****************************************************************************
 * Name: sai_dma_setup
 *
 * Description:
 *   Setup and initiate the next DMA transfer
 *
 * Input Parameters:
 *   priv - SAI state instance
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SAI_DMA
static int sai_dma_setup(struct stm32h7_sai_s *priv)
{
  struct sai_buffer_s *bfcontainer;
  struct ap_buffer_s *apb;
  stm32_dmacfg_t cfg;
  uintptr_t samp;
  apb_samp_t nbytes;
  size_t ntransfers = 0;
  size_t unit;
  int ret;

  /* If there is already an active transmission in progress, then bail
   * returning success.
   */

  if (!sq_empty(&priv->act))
    {
      return OK;
    }

  /* If there are no pending transfer, then bail returning success */

  if (sq_empty(&priv->pend))
    {
      priv->txenab = priv->rxenab = false;
      return OK;
    }

  /* Remove the pending transfer at the head of the pending queue. */

  bfcontainer = (struct sai_buffer_s *)sq_remfirst(&priv->pend);
  DEBUGASSERT(bfcontainer && bfcontainer->apb);

  apb = bfcontainer->apb;

  /* Get the transfer information, accounting for any data offset */

  samp = (uintptr_t)&apb->samp[apb->curbyte];

  /* The transfer width follows the configured data width.  For RX the driver
   * fills the buffer, so the length comes from the buffer capacity; for TX it
   * comes from the number of valid bytes.
   */

  if (priv->txenab)
    {
      nbytes = apb->nbytes - apb->curbyte;
      priv->dma_ccr = (priv->datalen > 16) ? SAI_TXDMA32_CONFIG :
                                             SAI_TXDMA16_CONFIG;
      unit = (priv->datalen > 16) ? 4 : 2;
    }
  else if (priv->rxenab)
    {
      nbytes = apb->nmaxbytes - apb->curbyte;

      if (priv->pdm)
        {
          priv->dma_ccr = SAI_RXDMA8_CONFIG;
          unit = 1;
        }
      else
        {
          priv->dma_ccr = (priv->datalen > 16) ? SAI_RXDMA32_CONFIG :
                                                 SAI_RXDMA16_CONFIG;
          unit = (priv->datalen > 16) ? 4 : 2;
        }
    }
  else
    {
      DEBUGASSERT(false);
      sai_buf_free(priv, bfcontainer);
      return -EINVAL;
    }

  /* Round down rather than assert: a buffer that is not a whole number of
   * transfer units long is trimmed by the last partial unit.
   */

  ntransfers = nbytes / unit;
  if (ntransfers == 0)
    {
      sai_buf_free(priv, bfcontainer);
      return -EINVAL;
    }

  /* The transfer count register is 16 bits wide.  A larger count is silently
   * truncated there, and a truncated count of zero leaves the stream enabled
   * with nothing to transfer: the completion callback never fires and the
   * caller waits until its timeout.  Reject it here instead.
   */

  if (ntransfers > DMA_SNDTR_NDT_MASK)
    {
      i2serr("ERROR: %zu transfers exceed the %lu limit\n",
             ntransfers, (unsigned long)DMA_SNDTR_NDT_MASK);
      sai_buf_free(priv, bfcontainer);
      return -E2BIG;
    }

  /* Reject buffers the DMA controller cannot reach.  DMA1 and DMA2 see the
   * D2 domain and the AXI SRAM, but not the tightly coupled DTCM at
   * 0x20000000: a buffer that lands there makes the transfer fail with a
   * TEIF after the block has already been enabled, which looks like a
   * peripheral fault rather than a buffer placement problem.
   *
   * The board configuration excludes DTCM from the heap for this reason, so
   * a buffer from the allocator cannot land there.  This check still matters
   * for static buffers and for any future configuration that puts DTCM back:
   * failing here names the address instead of surfacing as an unexplained
   * transfer error later.
   */

  if (samp >= STM32_DTCRAM_BASE && samp < (STM32_DTCRAM_BASE + 0x20000))
    {
      i2serr("ERROR: buffer at 0x%08" PRIxPTR " is in DTCM, "
             "unreachable by DMA\n", samp);
      sai_buf_free(priv, bfcontainer);
      return -EFAULT;
    }

  /* RX buffers must not retain cached lines that the DMA is about to
   * overwrite; the completion callback invalidates them again after the
   * transfer.  TX buffers must reach memory before the peripheral reads
   * them.
   */

  if (priv->rxenab)
    {
      up_invalidate_dcache(samp, samp + (ntransfers * unit));
    }
  else
    {
      up_clean_dcache(samp, samp + (ntransfers * unit));
    }

  cfg.paddr = priv->base + STM32H7_SAI_DR_OFFSET;
  cfg.maddr = (uint32_t)samp;
  cfg.cfg1  = priv->dma_ccr;
  cfg.cfg2  = 0;
  cfg.ndata = ntransfers;

  stm32_dmasetup(priv->dma, &cfg);

  /* Keep what was programmed so a failed transfer can be diagnosed without
   * guessing which side was misconfigured.
   */

  priv->dma_scr   = priv->dma_ccr;
  priv->dma_sndtr = (uint32_t)ntransfers;
  priv->dma_spar  = cfg.paddr;
  priv->dma_sm0ar = cfg.maddr;

  /* Add the container to the list of active DMAs */

  sq_addlast((sq_entry_t *)bfcontainer, &priv->act);

  /* Start the DMA, saving the container as the current active transfer */

  stm32_dmastart(priv->dma, sai_dma_callback, priv, false);

  /* Enable the block */

  sai_modifyreg(priv, STM32H7_SAI_CR1_OFFSET, 0, SAI_CR1_SAIEN);

  /* Start a watchdog to catch DMA timeouts */

  if (bfcontainer->timeout > 0)
    {
      ret = wd_start(&priv->dog, bfcontainer->timeout,
                     sai_timeout, (wdparm_t)priv);

      /* Check if we have successfully started the watchdog timer.  Note
       * that we do nothing in the case of failure to start the timer.  We
       * are already committed to the DMA anyway.  Let's just hope that the
       * DMA does not hang.
       */

      if (ret < 0)
        {
          i2serr("ERROR: wd_start failed: %d\n", ret);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: sai_worker
 *
 * Description:
 *   Transfer done worker
 *
 ****************************************************************************/

static void sai_worker(void *arg)
{
  struct stm32h7_sai_s *priv = (struct stm32h7_sai_s *)arg;
  struct sai_buffer_s *bfcontainer;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* Check if IDLE */

  if (sq_empty(&priv->act))
    {
      /* Then start the next transfer.  This must be done with interrupts
       * disabled.
       */

      flags = enter_critical_section();
#ifdef CONFIG_STM32H7_SAI_DMA
      sai_dma_setup(priv);
#endif
      leave_critical_section(flags);
    }

  /* Process each buffer in the done queue */

  while (sq_peek(&priv->done) != NULL)
    {
      /* Remove the buffer container from the done queue.  NOTE that
       * interrupts must be enabled to do this because the done queue is
       * also modified from the interrupt level.
       */

      flags = enter_critical_section();
      bfcontainer = (struct sai_buffer_s *)sq_remfirst(&priv->done);
      leave_critical_section(flags);

      /* Perform the transfer done callback */

      DEBUGASSERT(bfcontainer && bfcontainer->callback);
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, bfcontainer->result);

      /* Release our reference on the audio buffer.  This may very likely
       * cause the audio buffer to be freed.
       */

      apb_free(bfcontainer->apb);

      /* And release the buffer container */

      sai_buf_free(priv, bfcontainer);
    }
}

/****************************************************************************
 * Name: sai_schedule
 *
 * Description:
 *   An transfer completion or timeout has occurred.  Schedule processing on
 *   the working thread.
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - The timeout has been canceled.
 *
 ****************************************************************************/

static void sai_schedule(struct stm32h7_sai_s *priv, int result)
{
  struct sai_buffer_s *bfcontainer;
  int ret;

  /* Move all entries from the act queue to the done queue */

  while (!sq_empty(&priv->act))
    {
      /* Remove the next buffer container from the act list */

      bfcontainer = (struct sai_buffer_s *)sq_remfirst(&priv->act);

      /* Report the result of the transfer */

      bfcontainer->result = result;

      /* Add the completed buffer container to the tail of the done queue */

      sq_addlast((sq_entry_t *)bfcontainer, &priv->done);
    }

  /* If the worker has completed running, then reschedule the working thread.
   * REVISIT:  There may be a race condition here.  So we do nothing is the
   * worker is not available.
   */

  if (work_available(&priv->work))
    {
      /* Schedule the done processing to occur on the worker thread. */

      ret = work_queue(HPWORK, &priv->work, sai_worker, priv, 0);
      if (ret != 0)
        {
          i2serr("ERROR: Failed to queue work: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: sai_dma_callback
 *
 * Description:
 *   This callback function is invoked at the completion of the SAI DMA.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SAI_DMA
static void sai_dma_callback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  struct stm32h7_sai_s *priv = (struct stm32h7_sai_s *)arg;
  struct sai_buffer_s *bfcontainer;
  uintptr_t samp;
  uintptr_t end;
  size_t unit;
  bool failed;

  DEBUGASSERT(priv);

  /* Cancel the watchdog timeout */

  wd_cancel(&priv->dog);

  failed = (isr & (DMA_STATUS_TEIF | DMA_STATUS_DMEIF)) != 0;

  /* Record why the transfer ended, together with the block's status register
   * at that moment.  SAI reports overrun and wrong-clock-configuration
   * separately from the DMA error flags, and only the SAI bits distinguish a
   * misconfigured clock from a bus fault.  The value is read here rather than
   * later because CLRFR and the next transfer both overwrite it.
   */

  priv->last_isr  = isr;
  priv->last_sr   = sai_getreg(priv, STM32H7_SAI_SR_OFFSET);
  priv->last_cr1  = sai_getreg(priv, STM32H7_SAI_CR1_OFFSET);
  priv->last_frcr = sai_getreg(priv, STM32H7_SAI_FRCR_OFFSET);
  priv->last_slotr = sai_getreg(priv, STM32H7_SAI_SLOTR_OFFSET);
#ifdef CONFIG_STM32H7_SAI_PDM_MODE
  priv->last_pdmcr = priv->pdm ?
    getreg32(priv->pdmbase + STM32H7_SAI_PDMCR_OFFSET) : 0;
#else
  priv->last_pdmcr = 0;
#endif

  /* The DMA side matters as much as the SAI side: a transfer error can come
   * from a peripheral address that is not the data register, from an
   * unmapped request, or from a size mismatch.  Record what the controller
   * was actually programmed with.
   */

  priv->last_dma_scr   = priv->dma_scr;
  priv->last_dma_sndtr = priv->dma_sndtr;
  priv->last_dma_spar  = priv->dma_spar;
  priv->last_dma_sm0ar = priv->dma_sm0ar;
  priv->last_dmamux_ccr = priv->dmamux_ccr;

  /* Data landed in the RX buffers while the cache still holds the lines that
   * the CPU may have touched before the transfer started.  Invalidate now,
   * on the DMA completion path, so the client callback below observes what
   * the peripheral actually wrote.  This must happen before the containers
   * move to the done queue, because the worker calls the client.
   */

  if (priv->rxenab)
    {
      bfcontainer = (struct sai_buffer_s *)sq_peek(&priv->act);
      if (bfcontainer != NULL)
        {
          /* The transfer unit must match the one sai_dma_setup() chose, or
           * the invalidated range ends short of where the DMA wrote.
           */

          if (priv->pdm)
            {
              unit = 1;
            }
          else
            {
              unit = (priv->datalen > 16) ? 4 : 2;
            }

          samp = (uintptr_t)&bfcontainer->apb->samp[bfcontainer->apb->curbyte];
          end  = samp + ((bfcontainer->apb->nmaxbytes -
                          bfcontainer->apb->curbyte) / unit * unit);
          up_invalidate_dcache(samp, end);
        }
    }

  /* Stop driving the microphone.  Without this the block stays enabled after
   * the last buffer completes, keeps its bit clock running to the microphone
   * and keeps requesting DMA service that nothing will ever satisfy.  The
   * next transfer re-enables it in sai_dma_setup().
   */

  sai_modifyreg(priv, STM32H7_SAI_CR1_OFFSET, SAI_CR1_SAIEN, 0);

  /* Then schedule completion of the transfer to occur on the worker thread */

  sai_schedule(priv, failed ? -EIO : OK);
}
#endif

/****************************************************************************
 * Name: sai_samplerate
 *
 * Description:
 *   Set the I2S RX/TX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t sai_samplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct stm32h7_sai_s *priv = (struct stm32h7_sai_s *)dev;

  DEBUGASSERT(priv && rate > 0);

  /* Call callback to change system clock (needed for boards where the audio
   * PLL is shared with other peripherals)
   */

  if (priv->sampleratecb != NULL)
    {
      priv->frequency = priv->sampleratecb(dev, rate);
    }

  /* Save the new sample rate and update the divider */

  priv->samplerate = rate;
  sai_mckdivider(priv);

  return sai_getbitrate(priv);
}

/****************************************************************************
 * Name: sai_datawidth
 *
 * Description:
 *   Set the I2S data width.  The bitrate is determined by
 *   sample_rate * data_width.
 *
 ****************************************************************************/

static uint32_t sai_datawidth(struct i2s_dev_s *dev, int bits)
{
  struct stm32h7_sai_s *priv = (struct stm32h7_sai_s *)dev;
  uint32_t setbits;

  DEBUGASSERT(priv && bits >= 8);

  switch (bits)
    {
      case 8:
        setbits = SAI_CR1_DS_8BITS;
        break;

      case 16:
        setbits = SAI_CR1_DS_16BITS;
        break;

      case 32:
        setbits = SAI_CR1_DS_32BITS;
        break;

      default:
        i2serr("ERROR: Unsupported or invalid data width: %d\n", bits);
        return 0;
    }

  sai_modifyreg(priv, STM32H7_SAI_CR1_OFFSET, SAI_CR1_DS_MASK, setbits);

  sai_modifyreg(priv, STM32H7_SAI_FRCR_OFFSET,
                SAI_FRCR_FSALL_MASK | SAI_FRCR_FRL_MASK,
                SAI_FRCR_FSALL(bits) | SAI_FRCR_FRL(bits * 2));

  /* Save the new data width */

  priv->datalen = bits;

  return sai_getbitrate(priv);
}

/****************************************************************************
 * Name: sai_receive
 ****************************************************************************/

static int sai_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                       i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct stm32h7_sai_s *priv = (struct stm32h7_sai_s *)dev;
  struct sai_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv && apb);
  i2sinfo("apb=%p nbytes=%d arg=%p timeout=%" PRId32 "\n",
          apb, apb->nbytes - apb->curbyte, arg, timeout);

  /* Allocate a buffer container in advance */

  bfcontainer = sai_buf_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the SAI driver data */

  nxmutex_lock(&priv->lock);

  /* Verify not already TX'ing */

  if (priv->txenab)
    {
      i2serr("ERROR: SAI has no receiver\n");
      ret = -EAGAIN;
      goto errout_with_lock;
    }

  /* Receiver is always master here: the microphone's clock is generated by
   * the SAI itself, so there is no external frame reference to lock to.
   */

  sai_modifyreg(priv, STM32H7_SAI_CR1_OFFSET, SAI_CR1_MODE_MASK,
                SAI_CR1_MODE_MASTER_RX);
  priv->rxenab = true;

  /* Add a reference to the audio buffer */

  apb_reference(apb);

  /* Initialize the buffer container structure */

  bfcontainer->callback = (void *)callback;
  bfcontainer->timeout  = timeout;
  bfcontainer->arg      = arg;
  bfcontainer->apb      = apb;
  bfcontainer->result   = -EBUSY;

  /* Add the buffer container to the end of the pending queue */

  flags = enter_critical_section();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->pend);

  /* Then start the next transfer.  If there is already a transfer in
   * progress, then this will do nothing.
   */

#ifdef CONFIG_STM32H7_SAI_DMA
  ret = sai_dma_setup(priv);
#else
  ret = -ENOSYS;
#endif

  /* A setup failure has to reach the caller.  Returning OK while nothing is
   * in flight leaves the caller waiting for a completion callback that will
   * never arrive.  DEBUGASSERT would express the same intent but it is
   * compiled out of a release build, so the error would simply vanish.
   *
   * The queued container holds a reference to the audio buffer, so releasing
   * it is part of undoing the request.
   */

  if (ret < 0)
    {
      while (!sq_empty(&priv->pend))
        {
          struct sai_buffer_s *pending =
            (struct sai_buffer_s *)sq_remfirst(&priv->pend);

          apb_free(pending->apb);
          sai_buf_free(priv, pending);
        }

      priv->txenab = priv->rxenab = false;
      leave_critical_section(flags);
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  leave_critical_section(flags);
  nxmutex_unlock(&priv->lock);
  return OK;

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  sai_buf_free(priv, bfcontainer);
  return ret;
}

/****************************************************************************
 * Name: sai_send
 ****************************************************************************/

static int sai_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                    i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct stm32h7_sai_s *priv = (struct stm32h7_sai_s *)dev;
  struct sai_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv && apb);
  i2sinfo("apb=%p nbytes=%d arg=%p timeout=%" PRId32 "\n",
          apb, apb->nbytes - apb->curbyte, arg, timeout);

  /* Allocate a buffer container in advance */

  bfcontainer = sai_buf_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the SAI driver data */

  nxmutex_lock(&priv->lock);

  /* Verify not already RX'ing */

  if (priv->rxenab)
    {
      i2serr("ERROR: SAI has no transmitter\n");
      ret = -EAGAIN;
      goto errout_with_lock;
    }

  /* Transmitter is master: it drives the bit clock and frame sync towards
   * the codec, which is the only topology this driver is used in.
   */

  sai_modifyreg(priv, STM32H7_SAI_CR1_OFFSET, SAI_CR1_MODE_MASK,
                SAI_CR1_MODE_MASTER_TX);
  priv->txenab = true;

  /* Add a reference to the audio buffer */

  apb_reference(apb);

  /* Initialize the buffer container structure */

  bfcontainer->callback = (void *)callback;
  bfcontainer->timeout  = timeout;
  bfcontainer->arg      = arg;
  bfcontainer->apb      = apb;
  bfcontainer->result   = -EBUSY;

  /* Add the buffer container to the end of the pending queue */

  flags = enter_critical_section();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->pend);

  /* Then start the next transfer.  If there is already a transfer in
   * progress, then this will do nothing.
   */

#ifdef CONFIG_STM32H7_SAI_DMA
  ret = sai_dma_setup(priv);
#else
  ret = -ENOSYS;
#endif

  /* A setup failure has to reach the caller.  Returning OK while nothing is
   * in flight leaves the caller waiting for a completion callback that will
   * never arrive.  DEBUGASSERT would express the same intent but it is
   * compiled out of a release build, so the error would simply vanish.
   *
   * The queued container holds a reference to the audio buffer, so releasing
   * it is part of undoing the request.
   */

  if (ret < 0)
    {
      while (!sq_empty(&priv->pend))
        {
          struct sai_buffer_s *pending =
            (struct sai_buffer_s *)sq_remfirst(&priv->pend);

          apb_free(pending->apb);
          sai_buf_free(priv, pending);
        }

      priv->txenab = priv->rxenab = false;
      leave_critical_section(flags);
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  leave_critical_section(flags);
  nxmutex_unlock(&priv->lock);
  return OK;

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  sai_buf_free(priv, bfcontainer);
  return ret;
}

/****************************************************************************
 * Name: sai_buf_allocate
 ****************************************************************************/

static struct sai_buffer_s *sai_buf_allocate(struct stm32h7_sai_s *priv)
{
  struct sai_buffer_s *bfcontainer;
  irqstate_t flags;

  /* Set aside a buffer container.  By doing this, we guarantee that we will
   * have at least one free buffer container.
   */

  nxsem_wait_uninterruptible(&priv->bufsem);

  /* Get the buffer from the head of the free list */

  flags = enter_critical_section();
  bfcontainer = priv->freelist;
  DEBUGASSERT(bfcontainer);

  /* Unlink the buffer from the freelist */

  priv->freelist = bfcontainer->flink;
  leave_critical_section(flags);
  return bfcontainer;
}

/****************************************************************************
 * Name: sai_buf_free
 ****************************************************************************/

static void sai_buf_free(struct stm32h7_sai_s *priv,
                         struct sai_buffer_s *bfcontainer)
{
  irqstate_t flags;

  /* Put the buffer container back on the free list */

  flags = enter_critical_section();
  bfcontainer->flink  = priv->freelist;
  priv->freelist = bfcontainer;
  leave_critical_section(flags);

  /* Wake up any threads waiting for a buffer container */

  nxsem_post(&priv->bufsem);
}

/****************************************************************************
 * Name: sai_buf_initialize
 ****************************************************************************/

static void sai_buf_initialize(struct stm32h7_sai_s *priv)
{
  int i;

  /* bufsem was initialized to MAXINFLIGHT by the static initializer, which
   * is the count of free containers.  sai_buf_free() posts once per
   * container, so reset it before handing the pool out to keep the count
   * from doubling.
   */

  priv->freelist = NULL;
  nxsem_reset(&priv->bufsem, 0);

  for (i = 0; i < CONFIG_STM32H7_SAI_MAXINFLIGHT; i++)
    {
      sai_buf_free(priv, &priv->containers[i]);
    }
}

/****************************************************************************
 * Name: sai_blockenable
 *
 * Description:
 *   Power up the SAI instance and route the kernel clock.  Both blocks of an
 *   SAI share one enable bit and one clock mux, so this is idempotent.
 *
 ****************************************************************************/

static void sai_blockenable(struct stm32h7_sai_s *priv)
{
  uint32_t regval;

  /* Enable the bus clock and release the peripheral from reset */

  if (priv->apbclkbit == RCC_APB4ENR_SAI4EN)
    {
      regval = getreg32(STM32_RCC_APB4ENR);
      regval |= priv->apbclkbit;
      putreg32(regval, STM32_RCC_APB4ENR);

      regval = getreg32(STM32_RCC_APB4RSTR);
      regval |= RCC_APB4RSTR_SAI4RST;
      putreg32(regval, STM32_RCC_APB4RSTR);
      up_udelay(1);
      regval &= ~RCC_APB4RSTR_SAI4RST;
      putreg32(regval, STM32_RCC_APB4RSTR);
    }
  else
    {
      regval = getreg32(STM32_RCC_APB2ENR);
      regval |= priv->apbclkbit;
      putreg32(regval, STM32_RCC_APB2ENR);

      regval = getreg32(STM32_RCC_APB2RSTR);
      regval |= (priv->apbclkbit == RCC_APB2ENR_SAI1EN ?
                 RCC_APB2RSTR_SAI1RST : RCC_APB2RSTR_SAI2RST);
      putreg32(regval, STM32_RCC_APB2RSTR);
      up_udelay(1);
      regval &= ~(priv->apbclkbit == RCC_APB2ENR_SAI1EN ?
                  RCC_APB2RSTR_SAI1RST : RCC_APB2RSTR_SAI2RST);
      putreg32(regval, STM32_RCC_APB2RSTR);
    }

  /* Select PLL2P as the kernel clock.  PLL2 must already be running; this
   * driver does not reprogram the PLL, because on this family the audio PLL
   * is shared with other peripherals and the board owns that decision.
   */

  regval  = getreg32(priv->ccipreg);
  regval &= ~priv->ccipsrcmask;
  regval |= priv->ccipsrc;
  putreg32(regval, priv->ccipreg);
}

/****************************************************************************
 * Name: sai_pdmconfig
 *
 * Description:
 *   Program the PDM interface for a single microphone on bitstream clock 2.
 *
 *   CK2 is enabled because that is the line the board wires to the
 *   microphone's clock pin.  The sample point is selected by the block's
 *   clock strobing bit, which the caller sets through the normal CR1 path;
 *   PDM has no separate serial-interface-type field of its own.
 *
 ****************************************************************************/

static void sai_pdmconfig(struct stm32h7_sai_s *priv)
{
  uint32_t regval;

  /* PDMCR lives in the global block, which is shared by both sub-blocks */

  regval  = getreg32(priv->pdmbase + STM32H7_SAI_PDMCR_OFFSET);
  regval &= ~SAI_PDMCR_PDMEN;
  putreg32(regval, priv->pdmbase + STM32H7_SAI_PDMCR_OFFSET);

  /* The reference configuration enables the clock of bitstream 2 only, since
   * that is the clock line the board's microphone is wired to.  MICNBR counts
   * microphone pairs and is encoded one-based; the reference value of two
   * covers a stereo pair even though only one microphone is fitted.
   */

  regval &= ~(SAI_PDMCR_MICNBR_MASK | SAI_PDMCR_CKEN1 | SAI_PDMCR_CKEN2 |
              SAI_PDMCR_CKEN3 | SAI_PDMCR_CKEN4);
  regval |= SAI_PDMCR_CKEN2 | SAI_PDMCR_MICNBR(SAI_PDM_MIC_PAIRS);
  putreg32(regval, priv->pdmbase + STM32H7_SAI_PDMCR_OFFSET);

  /* No per-microphone delay: the microphone is soldered next to the SoC and
   * its data line is short.
   */

  putreg32(SAI_PDMDLY_NODELAY, priv->pdmbase + STM32H7_SAI_PDMDLY_OFFSET);

  /* The PDM bitstream is handed to memory with the least significant bit
   * first, matching the reference configuration.  A decoder that unpacks
   * the stream must assume the same order.
   */

  sai_modifyreg(priv, STM32H7_SAI_CR1_OFFSET, SAI_CR1_LSBFIRST,
                SAI_CR1_LSBFIRST);

  /* Sample on the falling edge.  ST's reference configuration for this
   * board's microphone uses SAI_CLOCKSTROBING_FALLINGEDGE, and in receive
   * mode that maps to CKSTR being set.  The microphone drives its data out
   * on one edge of the clock it is given, so sampling on the wrong edge
   * yields a bitstream whose adjacent bytes repeat rather than carrying two
   * separate half-words.
   */

  sai_modifyreg(priv, STM32H7_SAI_CR1_OFFSET, SAI_CR1_CKSTR,
                SAI_CR1_CKSTR);

  /* Frame geometry.  The PDM interface defines the slot layout itself, so
   * these values come from the reference configuration rather than from the
   * PCM paths, which set their own frame in sai_datawidth().
   *
   * SLOTEN is written explicitly: the reference enables slot 1, not slot 0.
   * That is not interchangeable with a one-slot frame, because NBSLOT stays
   * at two and the enabled slot selects which half of the 32-bit frame
   * carries the microphone data.
   */

  sai_modifyreg(priv, STM32H7_SAI_FRCR_OFFSET,
                SAI_FRCR_FRL_MASK | SAI_FRCR_FSALL_MASK,
                SAI_FRCR_FRL(SAI_PDM_FRAME_LENGTH) |
                SAI_FRCR_FSALL(SAI_PDM_ACTIVE_FRAME_LEN));

  sai_modifyreg(priv, STM32H7_SAI_SLOTR_OFFSET,
                SAI_SLOTR_NBSLOT_MASK | SAI_SLOTR_SLOTEN_MASK,
                SAI_SLOTR_NBSLOT(SAI_PDM_SLOT_COUNT) |
                SAI_SLOTR_SLOTEN(SAI_PDM_SLOT_ENABLE));

  /* Enable the PDM interface last: the frame and slot registers must be
   * stable before the interface starts driving clocks to the microphone.
   */

  regval |= SAI_PDMCR_PDMEN;
  putreg32(regval, priv->pdmbase + STM32H7_SAI_PDMCR_OFFSET);
}

/****************************************************************************
 * Name: sai_portinitialize
 *
 * Description:
 *   Initialize a SAI sub-block in its default state.
 *
 ****************************************************************************/

static void sai_portinitialize(struct stm32h7_sai_s *priv)
{
  /* Initialize buffering */

  sai_buf_initialize(priv);

  /* Configure the master clock divider and data width */

  sai_mckdivider(priv);
  sai_datawidth((struct i2s_dev_s *)priv,
                CONFIG_STM32H7_SAI_DEFAULT_DATALEN);

#ifdef CONFIG_STM32H7_SAI_DMA
  /* Get DMA channel */

  priv->dma = stm32_dmachannel(priv->dmamap);
  DEBUGASSERT(priv->dma);

  /* DMAMUX programming is not readable through the DMA driver's public
   * interface, so record what the request number should be.  The register
   * dump shows whether the controller saw the mapping.
   */

  priv->dmamux_ccr = priv->dmamap & 0xff;

  sai_modifyreg(priv, STM32H7_SAI_CR1_OFFSET, 0, SAI_CR1_DMAEN);
#endif

  /* Asynchronous operation: each sub-block runs from the kernel clock
   * directly, no internal or external frame synchronization.
   */

  sai_modifyreg(priv, STM32H7_SAI_CR1_OFFSET, SAI_CR1_SYNCEN_MASK,
                SAI_CR1_SYNCEN_ASYNCH);

  sai_modifyreg(priv, STM32H7_SAI_CR1_OFFSET, 0, SAI_CR1_OUTDRIV);

  sai_modifyreg(priv, STM32H7_SAI_CR2_OFFSET,
                SAI_CR2_FTH_MASK | SAI_CR2_COMP_MASK,
                SAI_CR2_FTH_1QF | SAI_CR2_COMP_NONE);

  /* Free protocol, frame sync as a start-of-frame pulse, active high, on the
   * first bit of slot 0.  In PDM mode the slot layout is fixed by the PDM
   * interface, so the SLOTR values below only matter for PCM blocks.
   */

  sai_modifyreg(priv, STM32H7_SAI_FRCR_OFFSET,
                SAI_FRCR_FSDEF | SAI_FRCR_FSPOL | SAI_FRCR_FSOFF,
                SAI_FRCR_FSDEF_SF | SAI_FRCR_FSPOL_HIGH |
                  SAI_FRCR_FSOFF_FB);

  sai_modifyreg(priv, STM32H7_SAI_SLOTR_OFFSET,
                SAI_SLOTR_NBSLOT_MASK | SAI_SLOTR_SLOTEN_MASK,
                SAI_SLOTR_NBSLOT(2) | SAI_SLOTR_SLOTEN_0 |
                  SAI_SLOTR_SLOTEN_1);

  /* PDM last: it overrides the frame and slot layout above, and the PDM
   * interface must be configured while the block itself is still disabled.
   */

#ifdef CONFIG_STM32H7_SAI_PDM_MODE
  if (priv->pdm)
    {
      sai_pdmconfig(priv);
    }
#endif

  /* Do not enable the block here.  Enabling happens when a transfer is
   * queued, so that an inactive interface draws no DMA requests.
   */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sai_initialize
 *
 * Description:
 *   Initialize the selected SAI block
 *
 * Input Parameters:
 *   intf - I2S interface number (identifying the "logical" SAI interface)
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *stm32_sai_initialize(int intf,
                                       stm32_sai_sampleratecb_t sampleratecb)
{
  struct stm32h7_sai_s *priv;
  irqstate_t flags;

  flags = enter_critical_section();

  switch (intf)
    {
#ifdef CONFIG_STM32H7_SAI1_A
      case SAI1_BLOCK_A:
        {
          i2sinfo("SAI1 Block A Selected\n");
          priv = &g_sai1a_priv;
          priv->sampleratecb = sampleratecb;

          stm32_configgpio(GPIO_SAI1_D2_0);
          stm32_configgpio(GPIO_SAI1_CK2_0);
          break;
        }
#endif

#ifdef CONFIG_STM32H7_SAI1_B
      case SAI1_BLOCK_B:
        {
          i2sinfo("SAI1 Block B Selected\n");
          priv = &g_sai1b_priv;
          priv->sampleratecb = sampleratecb;

          stm32_configgpio(GPIO_SAI1_SD_B_1);
          stm32_configgpio(GPIO_SAI1_FS_B_0);
          stm32_configgpio(GPIO_SAI1_SCK_B_0);
          break;
        }
#endif

#ifdef CONFIG_STM32H7_SAI2_A
      case SAI2_BLOCK_A:
        {
          i2sinfo("SAI2 Block A Selected\n");
          priv = &g_sai2a_priv;
          priv->sampleratecb = sampleratecb;

          stm32_configgpio(GPIO_SAI2_SD_A_2);
          stm32_configgpio(GPIO_SAI2_FS_A_2);
          stm32_configgpio(GPIO_SAI2_SCK_A_2);
          stm32_configgpio(GPIO_SAI2_MCK_A_2);
          break;
        }
#endif

#ifdef CONFIG_STM32H7_SAI2_B
      case SAI2_BLOCK_B:
        {
          i2sinfo("SAI2 Block B Selected\n");
          priv = &g_sai2b_priv;
          priv->sampleratecb = sampleratecb;

          stm32_configgpio(GPIO_SAI2_SD_B_4);
          stm32_configgpio(GPIO_SAI2_FS_B_2);
          stm32_configgpio(GPIO_SAI2_SCK_B_2);
          break;
        }
#endif

#ifdef CONFIG_STM32H7_SAI4_A
      case SAI4_BLOCK_A:
        {
          i2sinfo("SAI4 Block A Selected\n");
          priv = &g_sai4a_priv;
          priv->sampleratecb = sampleratecb;

          stm32_configgpio(GPIO_SAI4_D2_0);
          stm32_configgpio(GPIO_SAI4_CK2_0);
          break;
        }
#endif

#ifdef CONFIG_STM32H7_SAI4_B
      case SAI4_BLOCK_B:
        {
          i2sinfo("SAI4 Block B Selected\n");
          priv = &g_sai4b_priv;
          priv->sampleratecb = sampleratecb;

          stm32_configgpio(GPIO_SAI4_SD_B_1);
          stm32_configgpio(GPIO_SAI4_FS_B_0);
          stm32_configgpio(GPIO_SAI4_SCK_B_0);
          break;
        }
#endif

      default:
        {
          i2sinfo("No SAI interface defined\n");
          goto err;
        }
    }

  sai_blockenable(priv);
  sai_portinitialize(priv);
  leave_critical_section(flags);

  return &priv->dev;

err:
  leave_critical_section(flags);
  return NULL;
}

/****************************************************************************
 * Name: stm32_sai_staterecord
 *
 * Description:
 *   Return the register snapshot taken at the end of the last transfer.  The
 *   array order is:
 *
 *     [0] DMA interrupt status   [1] SAI SR     [2] SAI CR1
 *     [3] SAI FRCR               [4] SAI SLOTR  [5] SAI PDMCR
 *
 ****************************************************************************/

int stm32_sai_staterecord(struct i2s_dev_s *dev, uint32_t *regs, int nregs)
{
  struct stm32h7_sai_s *priv;

  if (regs == NULL || nregs < STM32H7_SAI_DIAG_NREGS)
    {
      return -EINVAL;
    }

  if (dev != NULL)
    {
      priv = (struct stm32h7_sai_s *)dev;
    }
  else
    {
      /* Fall back to the PDM-capable block, which is the only one the
       * bring-up path creates.
       */

#ifdef CONFIG_STM32H7_SAI1_A
      priv = &g_sai1a_priv;
#elif defined(CONFIG_STM32H7_SAI4_A)
      priv = &g_sai4a_priv;
#else
      return -ENODEV;
#endif
    }

  regs[0] = priv->last_isr;
  regs[1] = priv->last_sr;
  regs[2] = priv->last_cr1;
  regs[3] = priv->last_frcr;
  regs[4] = priv->last_slotr;
  regs[5] = priv->last_pdmcr;

  if (nregs > STM32H7_SAI_DIAG_NREGS)
    {
      regs[6]  = priv->last_dma_scr;
      regs[7]  = priv->last_dma_sndtr;
      regs[8]  = priv->last_dma_spar;
      regs[9]  = priv->last_dma_sm0ar;
      regs[10] = priv->last_dmamux_ccr;
    }

  return OK;
}

#endif /* CONFIG_STM32H7_SAI */
