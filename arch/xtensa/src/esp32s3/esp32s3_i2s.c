/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_i2s.c
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

/* [V644] PROBE: direct ROM console -- does NOT go through the
 * /dev/kmsg ring buffer, so it survives a panic dump.  Declared
 * here because this file uses it before the old [V644] block. */
extern int esp_rom_printf(const char *fmt, ...);
#include <syslog.h>

#ifdef CONFIG_ESP32S3_I2S

#include <debug.h>
#include <sys/param.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <math.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/clock.h>
#include <nuttx/cache.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/mqueue.h>
#include <nuttx/circbuf.h>
#include <nuttx/nuttx.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>

#include <arch/board/board.h>

#include "esp32s3_i2s.h"
#include "esp32s3_gpio.h"
#include "esp32s3_irq.h"
#include "esp32s3_dma.h"

#include "xtensa.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_i2s.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_iomux.h"
#include "hardware/esp32s3_pinmap.h"
#include "hardware/esp32s3_dma.h"

/* The TX FIFO is the first register of the block. */
#define I2S_TX_FIFO_REG(i)   (REG_I2S_BASE(i) + 0x00)

/* [V356] Compiled-out diagnostics.
 *
 * I2S_V350_REARM_PROBE: re-issuing DMA_OUTLINK_START against descriptors
 *   the FSM has already consumed is rejected (OUT_DSCR_ERR) because
 *   AUTO_WRBACK cleared their OWNER; measured dsc_err=6 == one error per
 *   loop iteration.  It also let the GDMA raise completion events while
 *   the transfer was not yet queued as active, which dropped them.
 * I2S_I2SREG_DUMP: read-only register re-sample, but it stalls the caller
 *   for 40 ms mid-stream and buries the real evidence in log noise.
 */
#define I2S_V350_REARM_PROBE 0
#define I2S_I2SREG_DUMP      0

/* Bits the NuttX register header does not name. */
#define DMA_RST_BIT       (1u << 0)
#define GDMA_INT_DONE     (1u << 0)
#define GDMA_INT_SUC_EOF  (1u << 2)
#define GDMA_INT_DSCR_ERR (1u << 3)

/* [V209/V214] I2S TX diagnostics.
 *
 * g_txirq_n is only COUNTED from the ISR (logging there is unsafe); a non-zero
 * value proves the GDMA TX completion interrupt fired at least once.
 * g_txdump_n bounds the register dump to the first few transfers.
 * g_dbg_link_after_enable is sampled inside the spinlocked start path and
 * printed later, so we can tell whether OUTLINK_START was ever really set. */
static volatile int      g_txirq_n;

/* [V342] TX GDMA interrupt forensics.
 *
 * The pre-V342 handler read DMA_OUT_INT_ST, cleared it immediately and then
 * tested it only against DMA_OUT_TOTAL_EOF (bit 3).  Any other event --
 * OUT_DONE (bit 0), OUT_EOF/SUC_EOF (bit 1) or OUT_DSCR_ERR (bit 2) -- was
 * silently swallowed, and because the status register had already been
 * cleared every later register dump read 0 and the real cause was invisible.
 *
 * g_tx_isr_status latches the first non-zero status word (never overwritten),
 * g_tx_isr_n[] counts each event separately, and g_tx_isr_err counts
 * descriptor errors.  All are written from the ISR with plain stores and read
 * by the (non-interrupt) diagnostic dump. */
static volatile uint32_t g_tx_isr_status;
static volatile unsigned int g_tx_isr_n[4];   /* idx: 0=DONE 1=EOF 2=ERR 3=TOTEOF */
static volatile unsigned int g_tx_isr_err;

/* [V345] Completion-path forensics.  These separate "the ISR never scheduled
 * a completion" from "it scheduled but the worker/callback never ran". */
static volatile unsigned int g_sched_n;      /* i2s_tx_schedule() entries     */
static volatile unsigned int g_sched_noact;  /* ... with tx.act empty         */
static volatile unsigned int g_sched_match;  /* ... descriptor matched        */
static volatile unsigned int g_worker_n;     /* i2s_tx_worker() entries       */
static volatile unsigned int g_worker_cb;    /* ... callbacks invoked         */

/* [V233] Streaming state for the FIFO writer.  audio_playback_write() splits
 * PCM into ~4 kB chunks, so the writer is called several times a second;
 * re-running the TX reset/start sequence per chunk would insert an audible
 * discontinuity at every chunk boundary.  Only the first chunk brings the TX
 * side up, and the pacing reference is carried across calls so the stream
 * stays gap-free and at the right rate. */
static int                g_tx_streaming;
static long long          g_tx_start_ns;   /* stream t0                     */
static long long          g_tx_delivered;  /* samples already delivered     */
static int               g_txdump_n;
/* [V354] Write the descriptors and the sample block back to SRAM before
 * the GDMA reads them.  The kernel is built with
 * CONFIG_ESP32S3_DATA_CACHE_32KB / 8-way / 32 B lines (see nuttx/.config)
 * and no esp32s3 driver performs any cache maintenance, so everything the
 * CPU writes into dma_link[] and into the bounce buffer can still be
 * sitting in D-cache when the GDMA engine walks it.  Set to 0 to A/B this
 * without rebuilding anything else. */
static int               g_i2s_tx_cache_clean = 1;
static volatile uint32_t g_dbg_link_after_enable;
static int               g_tx_cfg_done;   /* [V346] V264 sequence ran once    */

/* [V94] Receive-path counters (top-of-file copy so every user sees them).
 * Plain globals, no blocking calls: the DMA setup path runs with the driver
 * spinlock held and interrupts off.  The capture thread beacons them. */

volatile unsigned int g_v94_recv;      /* i2s_receive() calls            */
volatile int          g_v94_setupret;  /* i2s_rxdma_setup() last return  */
volatile unsigned int g_v94_start;     /* i2s_rxdma_start() calls        */
volatile unsigned int g_v94_isr;       /* rx interrupt entries           */
volatile unsigned int g_v94_eof;       /* rx end-of-frame interrupts     */
volatile unsigned int g_v94_pb;        /* es8311_processbegin() calls    */
volatile int          g_v94_i2sret;    /* last I2S_RECEIVE return        */
volatile unsigned int g_v94_done;      /* es8311_processdone() calls     */
volatile int          g_v94_rxonly;    /* current direction override     */

/* [V733] How many bits the RX stream is currently shifted left, measured
 * from the whole-buffer OR (see the [V733] block in the RX DMA callback).
 * 0 = aligned.  Samples are shifted right by this much before delivery. */

static int g_v733_shift;

/* [V690] Last i2s_receive() instance, so esp32s3_i2s_rx_health() (a plain
 * void entry point called from the capture thread) can report the queue
 * depths.  Single I2S port on this board, so a file-scope pointer is fine. */
static struct esp32s3_i2s_s *g_v690_priv;


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2S DMA RX/TX description number */

#define I2S_DMADESC_NUM                 (CONFIG_I2S_DMADESC_NUM)

/* I2S Clock */

#define I2S_LL_BASE_CLK                 (2 * APB_CLK_FREQ)

/* [V216] The divider maths below are derived from the selected source clock.
 * NuttX hard-codes CLK160 (2 * APB) and never checks that the source is
 * actually running.  Make it switchable so we can fall back to XTAL when the
 * I2S clock domain turns out to be dead. */
/* [V670] *** The I2S divider base on THIS board. ***
 *
 * The bring-up asked for I2S_PLL_F160M_CLK and told the divider maths that the
 * source runs at 160 MHz (2 * APB_CLK_FREQ).  Two independent measurements say
 * it actually runs at 150 MHz:
 *
 *   1. Pads, 50 ms CCOUNT-bounded windows, reproducible:
 *        BCLK = 470376 Hz / 490679 Hz  ->  MCLK = BCLK * 8 = 3.76-3.92 MHz
 *        MCLK = sclk / (39 + 1/16)     ->  sclk = 147-153 MHz
 *      Both figures read 6% below the 512 kHz / 4.096 MHz the registers claim.
 *
 *   2. Playback accounting: 31232 samples consumed over 500440024 CPU cycles
 *      = 16022 cycles per sample.  CCOUNT runs at 240 MHz and one sample is
 *      one LRCK frame, so the real frame rate is 240e6 / 16022 = 14979 Hz.
 *
 * 15000 / 16000 is exactly 15/16, i.e. exactly 150 MHz against 160 MHz.
 *
 * Effect of the old assumption: with mclk_div latched at 39 + 1/16 the real
 * MCLK came out at 3.84 MHz instead of 4.096 MHz, so BCLK was 480 kHz instead
 * of 512 kHz and the frame rate 15000 Hz instead of 16000 Hz -- playback ran
 * 6.25% slow, roughly a semitone flat. */
#define I2S_PLL_F160M_HZ   150000000u

static uint32_t g_i2s_sclk = I2S_PLL_F160M_HZ;

/* [V668] Desired TX channel mode, remembered by i2s_txchannels() so that
 * i2s_txdma_start() can re-latch it inside its TX-stop window.  Kept next to
 * the clock globals because it has exactly the same lifetime: it describes
 * how the transmitter must be programmed, independent of which of the
 * several reset sites in this file happens to run next. */
static int g_i2s_tx_mono;
static int g_i2s_rx_mono;       /* [V681] RX mirror of g_i2s_tx_mono */

/* [V682] Which RX TDM slots are armed: 0x01 = slot 0 (mono, default),
 * 0x02 = slot 1, 0x03 = both.  Refreshed from /tmp/rxslot.txt on every
 * capture start, so the candidates can be swept without a reflash. */
static uint32_t g_i2s_rx_slot_mask = I2S_RX_TDM_PDM_CHAN0_EN;

/* [V671] *** Light re-arm latch. ***
 *
 * i2s_txdma_start() runs on EVERY buffer and used to re-run the whole
 * V343/V348 configure-reset-start block each time, I2S_TX_RESET and
 * I2S_TX_FIFO_RESET included.  The FIFO reset throws away the ~64 samples
 * (about 4 ms at 16 kHz) that are still buffered in it, so every 2048-byte
 * chunk of TTS audio arrived with a hole in front of it -- roughly thirty
 * dropouts over a two second sentence.  That is the reported stutter.
 *
 * V348 made the block unconditional because, at the time, skipping it left
 * the transmitter idle (STATE=TX_IDLE, outlink parked, OWNER stuck) and the
 * transfer never completed.  That was measured with the TDM channel enable
 * bug and the wrong divider base still in place, i.e. before v669/v670.  Now
 * that CONF, TDM and the clock are right, a transfer that finds the module
 * already configured only needs its descriptor handed to the GDMA.
 *
 * g_v671_cfg_valid is 1 once the module has been fully configured and the
 * transmitter is running.  Every site that pulses I2S_TX_RESET clears it
 * again, so the next transfer reconfigures from scratch: i2s_configure(),
 * i2s_clock_try_source(), i2s_tx_channel_start() and i2s_tx_channel_stop().
 * audio_playback_open() goes through i2s_txsamplerate() ->
 * i2s_clock_try_source() on every speak, so the first transfer of each
 * utterance is always a full configure and the rest are light re-arms.
 *
 * If the light path ever turns out to be unsafe it announces itself loudly:
 * the TX never drains, so audio_playback.c reports
 * "I2S TX completion TIMEOUT after 1.5 s" and there is no sound at all. */
static int g_v671_cfg_valid;
static int g_v671_full_n;
static int g_v671_light_n;
static int      g_sclk_fallback;      /* 1 once we switched to XTAL  */
static int      g_tx_update_spins;    /* spins waiting for tx_update */
static int      g_tx_update_spins2;   /* same, after the fallback    */
static int      g_tx_start_spins;     /* [V343] spins for the post-start update */
static int      g_tx_stop_spins;      /* [V347] spins for the pre-start stop handshake */
static uint32_t g_tx_state_pre;       /* [V347] I2S_STATE before the stop/start toggle */
static uint32_t g_tx_state_post;      /* [V347] I2S_STATE after  the stop/start toggle */
static int      g_tx_channel_start_er;/* [V347] i2s_tx_channel_start() early-return count */
static int      g_tx_txdma_start_n;   /* [V347] i2s_txdma_start() prologue count  */
static int      g_tx_txdma_bail_act;  /* [V347] i2s_txdma_start() bail: tx.act non-empty */
static int      g_tx_txdma_bail_pend; /* [V347] i2s_txdma_start() bail: tx.pend empty    */

/* [V362] Rate accounting + idle-spin attribution. */
static int      g_tx_rate_calls;     /* i2s_txdma_start() total entries   */
static int      g_tx_rate_starts;    /* entries that armed the DMA        */
static int      g_tx_rate_bail_act;  /* early bail: tx.act non-empty      */
static int      g_tx_rate_bail_pend; /* early bail: tx.pend EMPTY (idle!) */
static int      g_tx_rate_skip;      /* cheap gate for the clock read     */
static int64_t  g_tx_rate_next_ms;   /* next 5 s summary deadline         */

/* [V362] Which call site is on the stack, so the 5 s log can attribute
 * every idle spin.  Set immediately before each i2s_txdma_start() call:
 *   1 = i2s_txdma_setup()   (enqueue / write path)
 *   2 = i2s_tx_schedule()   (GDMA EOF interrupt, ISR context)
 */
static int      g_tx_callsite;
static int      g_tx_rate_bail_s1;   /* idle bails attributed to site 1   */
static int      g_tx_rate_bail_s2;   /* idle bails attributed to site 2   */

/* [V364] How often the TX completion interrupt sources were gated. */
static int      g_tx_irq_mask_n;    /* masked: both queues empty        */
static int      g_tx_irq_unmask_n;  /* unmasked: a buffer was queued    */

/* [V362] Req #3: memoized TX clock.  i2s_set_clock() derives the dividers
 * with soft-float arithmetic (there is no FPU on LX7 under NuttX) and it
 * runs inside i2s_txdma_start(), which i2s_tx_schedule() calls straight
 * from the GDMA EOF ISR.  Once derived for a given parameter set we
 * simply re-apply the latched register values instead of redoing the
 * math.  The values are keyed on every input i2s_set_clock() reads.
 */
static int      g_txclk_valid;
static uint32_t g_txclk_sclk;
static uint32_t g_txclk_rate;
static uint32_t g_txclk_slot;
static uint32_t g_txclk_width;
static uint32_t g_txclk_mclk;
static uint32_t g_txclk_role;
static uint32_t g_txclk_clkm;
static uint32_t g_txclk_div;
static uint32_t g_txclk_conf1;
static int      g_rx_update_spins;    /* spins waiting for rx_update */
#define I2S_LL_MCLK_DIVIDER_BIT_WIDTH   (6)
#define I2S_LL_MCLK_DIVIDER_MAX         ((1 << I2S_LL_MCLK_DIVIDER_BIT_WIDTH) - 1)

/* I2S DMA channel number */

#define I2S_DMA_CHANNEL_MAX (2)

#ifdef CONFIG_ESP32S3_I2S0_TX
#  define I2S0_TX_ENABLED 1
#  define I2S_HAVE_TX 1
#else
#  define I2S0_TX_ENABLED 0
#endif

#ifdef CONFIG_ESP32S3_I2S0_RX
#  define I2S0_RX_ENABLED 1
#  define I2S_HAVE_RX 1
#else
#  define I2S0_RX_ENABLED 0
#endif

#ifdef CONFIG_ESP32S3_I2S1_TX
#  define I2S1_TX_ENABLED 1
#  define I2S_HAVE_TX 1
#else
#  define I2S1_TX_ENABLED 0
#endif

#ifdef CONFIG_ESP32S3_I2S1_RX
#  define I2S1_RX_ENABLED 1
#  define I2S_HAVE_RX 1
#else
#  define I2S1_RX_ENABLED 0
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_I2S_INFO
#  define CONFIG_ESP32S3_I2S_DUMPBUFFERS
#else
#  undef CONFIG_ESP32S3_I2S_DUMPBUFFERS
#endif

#define I2S_GPIO_UNUSED -1      /* For signals which are not used */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Role of the I2S port */

typedef enum
{
  I2S_ROLE_MASTER,  /* I2S controller master role, bclk and ws signal will be set to output */
  I2S_ROLE_SLAVE    /* I2S controller slave role, bclk and ws signal will be set to input */
} i2s_role_t;

/* Data width of the I2S channel */

typedef enum
{
  I2S_DATA_BIT_WIDTH_8BIT   = 8,    /* I2S channel data bit-width: 8 */
  I2S_DATA_BIT_WIDTH_16BIT  = 16,   /* I2S channel data bit-width: 16 */
  I2S_DATA_BIT_WIDTH_24BIT  = 24,   /* I2S channel data bit-width: 24 */
  I2S_DATA_BIT_WIDTH_32BIT  = 32,   /* I2S channel data bit-width: 32 */
} i2s_data_bit_width_t;

/* Multiplier of MCLK to sample rate */

typedef enum
{
  I2S_MCLK_MULTIPLE_128 = 128,  /* mclk = sample_rate * 128 */
  I2S_MCLK_MULTIPLE_256 = 256,  /* mclk = sample_rate * 256 */
  I2S_MCLK_MULTIPLE_384 = 384,  /* mclk = sample_rate * 384 */
  I2S_MCLK_MULTIPLE_512 = 512,  /* mclk = sample_rate * 512 */
} i2s_mclk_multiple_t;

/* I2S Clock Source */

typedef enum
{
  I2S_XTAL_CLK = 0,
  I2S_PLL_D2_CLK,
  I2S_PLL_F160M_CLK,
  I2S_MCLK_IN,
} i2s_clk_src_t;

/* I2S Audio Standard Mode */

typedef enum
{
  I2S_TDM_PHILIPS = 0,
  I2S_TDM_MSB,
  I2S_TDM_PCM,
  I2S_PDM,
} i2s_audio_mode_t;

/* Role of the I2S port */

typedef enum
{
  I2S_RX,   /* I2S controller receiver (RX) */
  I2S_TX    /* I2S controller transmitter (TX) */
} i2s_dir_t;

/* I2S Device hardware configuration */

struct esp32s3_i2s_config_s
{
  uint32_t port;              /* I2S port */
  uint32_t role;              /* I2S port role (master or slave) */
  uint8_t data_width;         /* I2S sample data width */
  uint32_t rate;              /* I2S sample-rate */
  uint32_t total_slot;        /* Total slot number */

  bool tx_en;                 /* Is TX enabled? */
  bool rx_en;                 /* Is RX enabled? */
  int8_t mclk_pin;            /* MCLK pin, output */

  int tx_clk_src;             /* Select the I2S TX source clock */
  int rx_clk_src;             /* Select the I2S TX source clock */

  /* BCLK pin, input in slave role, output in master role */

  int8_t bclk_pin;

  /* WS pin, input in slave role, output in master role */

  int8_t ws_pin;

  int8_t dout_pin;            /* DATA pin, output */
  int8_t din_pin;             /* DATA pin, input */

  uint32_t bclk_in_insig;     /* RX channel BCK signal (slave mode) index */
  uint32_t bclk_in_outsig;    /* RX channel BCK signal (master mode) index */
  uint32_t bclk_out_insig;    /* TX channel BCK signal (slave mode) index */
  uint32_t bclk_out_outsig;   /* TX channel BCK signal (master mode) index */
  uint32_t ws_in_insig;       /* RX channel WS signal (slave mode) index */
  uint32_t ws_in_outsig;      /* RX channel WS signal (master mode) index */
  uint32_t ws_out_insig;      /* TX channel WS signal (slave mode) index */
  uint32_t ws_out_outsig;     /* TX channel WS signal (master mode) index */
  uint32_t din_insig;         /* RX channel Data Input signal index */
  uint32_t dout_outsig;       /* TX channel Data Output signal index */
  uint32_t mclk_out_sig;      /* Master clock output index */

  uint8_t  audio_std_mode;    /* Select audio standard (i2s_audio_mode_t) */

  /* WS signal polarity, set true to enable high level first */

  bool ws_pol;
};

struct esp32s3_buffer_s
{
  struct esp32s3_buffer_s *flink; /* Supports a singly linked list */

  /* The associated DMA in/outlink */

  struct esp32s3_dmadesc_s dma_link[I2S_DMADESC_NUM];

  i2s_callback_t callback;      /* DMA completion callback */
  uint32_t timeout;             /* Timeout value of the DMA transfers */
  void *arg;                    /* Callback's argument */
  struct ap_buffer_s *apb;      /* The audio buffer */
  uint8_t *buf;                 /* The DMA's descriptor buffer */
  uint32_t nbytes;              /* The DMA's descriptor buffer size */
  int result;                   /* The result of the transfer */
};

/* Internal buffer must be aligned to the bytes_per_sample. Sometimes,
 * however, the audio buffer is not aligned and additional bytes must
 * be copied to be inserted on the next buffer. This structure keeps
 * track of the bytes that were not written to the internal buffer yet.
 */

struct esp32s3_buffer_carry_s
{
  uint32_t value;
  size_t bytes;
};

/* This structure describes the state of one receiver or transmitter
 * transport.
 */

struct esp32s3_transport_s
{
  sq_queue_t pend;              /* A queue of pending transfers */
  sq_queue_t act;               /* A queue of active transfers */
  sq_queue_t done;              /* A queue of completed transfers */
  struct work_s work;           /* Supports worker thread operations */

  /* Bytes to be written at the beginning of the next DMA buffer */

  struct esp32s3_buffer_carry_s carry;
};

/* The state of the one I2S peripheral */

struct esp32s3_i2s_s
{
  struct i2s_dev_s  dev;        /* Externally visible I2S interface */
  mutex_t           lock;       /* Ensures mutually exclusive access */
  uint8_t           cpu;        /* CPU ID */
  spinlock_t        slock;      /* Device specific lock. */

  /* Port configuration */

  const struct esp32s3_i2s_config_s *config;

  uint32_t    mclk_freq;      /* I2S actual master clock */
  uint32_t    mclk_multiple;  /* The multiple of mclk to the sample rate */
  uint32_t    channels;       /* Audio channels (1:mono or 2:stereo) */
  uint32_t    rate;           /* I2S actual configured sample-rate */
  uint32_t    data_width;     /* I2S actual configured data_width */
  uint32_t    dma_channel;    /* I2S DMA channel being used */

#ifdef I2S_HAVE_TX
  struct esp32s3_transport_s tx;  /* TX transport state */

  int  tx_irq;                    /* TX IRQ */
  bool tx_started;                /* TX channel started */
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  struct esp32s3_transport_s rx;  /* RX transport state */

  int  rx_irq;                    /* RX IRQ */
  bool rx_started;                /* RX channel started */
#endif /* I2S_HAVE_RX */

  bool streaming;                 /* Is I2S peripheral active? */

  /* Pre-allocated pool of buffer containers */

  sem_t bufsem;                         /* Buffer wait semaphore */
  struct esp32s3_buffer_s *bf_freelist; /* A list a free buffer containers */
  struct esp32s3_buffer_s containers[CONFIG_ESP32S3_I2S_MAXINFLIGHT];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register helpers */

#ifdef CONFIG_ESP32S3_I2S_DUMPBUFFERS
#  define       i2s_dump_buffer(m,b,s) lib_dumpbuffer(m,b,s)
#else
#  define       i2s_dump_buffer(m,b,s)
#endif

/* Buffer container helpers */

/* [V211] CRITICAL: I2S GDMA on the ESP32-S3 can only fetch buffers from
 * internal SRAM, but this board registers its 8 MB PSRAM as the common heap
 * (CONFIG_ESP32S3_SPIRAM_COMMON_HEAP=y), so the per-transfer calloc() in
 * i2s_txdma_setup() handed back PSRAM addresses (measured: 0x3c16a080).
 * esp32s3_dma_setup() only compensates for PSRAM on the RX side
 * ("if (!tx && buffer_in_psram)"), so on TX the descriptor simply pointed at
 * memory the DMA cannot read: the transfer made no progress at all, the GDMA
 * completion interrupt never fired (txirqN=0, OUTLINK_START cleared) and
 * i2s_send() blocked forever -- which presented as "the speaker is silent and
 * the board stops logging".
 *
 * The DMA bounce buffers therefore come from a statically allocated
 * internal-SRAM pool (.bss is always internal), which is guaranteed
 * DMA-capable.  Slots are sized to cover the maximum transfer the I2S upper
 * half may queue (ESP32S3_DMA_BUFLEN_MAX * I2S_DMADESC_NUM). */

/* Slot size matches AGENT_AUDIO_CHUNK_MAX (audio_playback.c), which caps
 * every transfer the application queues.  The pool is deliberately small:
 * internal DRAM on this board has only ~90 KB of heap headroom, so a large
 * BSS pool pushes _sheap past the ROM-reserved top of DRAM0 and the board no
 * longer starts at all.
 *
 * [V317] Raised 2 -> 6.  The old [V312] note blamed a 6-slot pool for
 * the WiFi reason=2/205 storm via ".bss address shift".  That
 * attribution is DISPROVED: V316 grew .bss by ~49 KB (IOB pool,
 * CONFIG_IOB_NBUFFERS 64->160) and WiFi then associated on the FIRST
 * try ([V103] LINKED try#0) with DHCP succeeding first attempt.  The
 * real storm cause was the authmode threshold + faced CPU starvation,
 * both fixed long ago.
 *
 * With 2 slots the capture side (audio_capture_native arms 2 RX
 * buffers) consumes the WHOLE pool, so TX can never get a bounce slot:
 *   [audio_direct] i2s_send failed: -12   (playback, ENOMEM)
 *   [AUDIO-R42] xfer ret=-12              (capture,  ENOMEM)
 * 6 slots leaves 4 free while capture is armed. */
/* [V372] RESTORED to 6.  The [V317] comment above already argued for 6 and
 * even says "raised 2 -> 6", but the literal had been put back to 2:
 * with 2 slots the capture side (audio_capture_native arms 2 RX
 * buffers) consumes the WHOLE pool, so the RX DMA never delivers and
 * audio_capture_native gives up with -EIO after six empty waits --
 * which is exactly the "capture EIO, retry 1/5..5/5" + "no speech for
 * 8000 ms" seen in the V371 log.  6 slots leaves 4 free. */
/* [V377] 5 -> 4.  Capture arms 2 buffers ("[audio_cap_nat] capture started
 * (2 buffers armed)"), so 4 still leaves 2 spare, and the 4096 bytes freed
 * here pay for the enlarged es8311 worker stacks (2 x 2048).  Raising this
 * to 6 is still not an option: it overflows dram0_0_seg by 3056 bytes. */
/* [V638] 2 -> 4.  The V317/V372/V377 notes above all describe 4 (or 6)
 * slots, but the literal had been left at 2 -- which is exactly the case
 * those notes call out as broken: audio_capture_native arms 2 RX buffers,
 * so a 2-slot pool is fully consumed the moment recording starts and the
 * TX / internal-loopback leg of the same codec gets nothing.  That is the
 * state in which the es8311 worker died with
 *   xtensa_user_panic: User Exception: EXCCAUSE=001c task: es8311
 * 4 slots leaves 2 spare.  V377 measured 6 as overflowing dram0_0_seg by
 * 3056 bytes; since then RAMLOG (-24 KB), IOB (-83 KB) and the TLS scratch
 * buffer (-4 KB) were all trimmed, so 4 is comfortably affordable. */
#define I2S_DMABUF_SLOTS     4
#define I2S_DMABUF_SLOT_SZ   4096

static uint8_t g_i2s_dmabuf[I2S_DMABUF_SLOTS][I2S_DMABUF_SLOT_SZ]
               __attribute__((aligned(16)));
static uint8_t g_i2s_dmabuf_used[I2S_DMABUF_SLOTS];

static void *i2s_dmabuf_alloc(size_t nbytes)
{
  irqstate_t flags;
  int i;

  if (nbytes > I2S_DMABUF_SLOT_SZ)
    {
      i2serr("DMA buffer request %u exceeds the internal-SRAM pool slot "
             "size %u\n", (unsigned)nbytes, I2S_DMABUF_SLOT_SZ);
      return NULL;
    }

  flags = spin_lock_irqsave(NULL);
  for (i = 0; i < I2S_DMABUF_SLOTS; i++)
    {
      if (!g_i2s_dmabuf_used[i])
        {
          g_i2s_dmabuf_used[i] = 1;
          spin_unlock_irqrestore(NULL, flags);
          return g_i2s_dmabuf[i];
        }
    }

  spin_unlock_irqrestore(NULL, flags);
  return NULL;
}

static void i2s_dmabuf_release(void *p)
{
  irqstate_t flags;
  int i;

  if (p == NULL)
    {
      return;
    }

  flags = spin_lock_irqsave(NULL);
  for (i = 0; i < I2S_DMABUF_SLOTS; i++)
    {
      if (p == (void *)g_i2s_dmabuf[i])
        {
          g_i2s_dmabuf_used[i] = 0;
          break;
        }
    }

  spin_unlock_irqrestore(NULL, flags);
}

static struct esp32s3_buffer_s *
                i2s_buf_allocate(struct esp32s3_i2s_s *priv);
static void     i2s_buf_free(struct esp32s3_i2s_s *priv,
                             struct esp32s3_buffer_s *bfcontainer);
static int      i2s_buf_initialize(struct esp32s3_i2s_s *priv);

/* DMA support */

#ifdef I2S_HAVE_TX
static IRAM_ATTR int  i2s_txdma_setup(struct esp32s3_i2s_s *priv,
                                      struct esp32s3_buffer_s *bfcontainer);
static void           i2s_tx_worker(void *arg);
static void           i2s_tx_schedule(struct esp32s3_i2s_s *priv,
                                      struct esp32s3_dmadesc_s *outlink);
/* [V338] forward decl: i2s_txdma_start() 定义在 helper 之前 */
static void           i2s_apply_tx_gdma_strategy(struct esp32s3_i2s_s *priv);
static void     i2s_tx_rate_tick(void);   /* [V362] 5 s summary logger  */
static void     i2s_tx_irq_mask(struct esp32s3_i2s_s *priv);
static void     i2s_tx_irq_unmask(struct esp32s3_i2s_s *priv);
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
static IRAM_ATTR int  i2s_rxdma_setup(struct esp32s3_i2s_s *priv,
                                      struct esp32s3_buffer_s *bfcontainer);
static void           i2s_rx_worker(void *arg);

/* [V691] Defined far below: the RX worker calls it once the stream is
 * actually running, which is the only moment the measurement means
 * anything (see patch_v691.py). */

void esp32s3_i2s_rx_health(void);

/* [V858] Defined far below (right after esp32s3_i2s_set_rxonly()) and
 * called from the three upper-half files, so they need prototypes. */

void     esp32s3_i2s_v858_capture_begin(void);
void     esp32s3_i2s_v858_after_playback(void);
void     esp32s3_i2s_v858_heal(void);
void     esp32s3_i2s_v858_relock_current(void);
uint32_t esp32s3_i2s_v858_rxconf(void);
static void           i2s_rx_schedule(struct esp32s3_i2s_s *priv,
                                      struct esp32s3_dmadesc_s *outlink);
#endif /* I2S_HAVE_RX */

/* I2S methods (and close friends) */

static int32_t  i2s_check_mclkfrequency(struct esp32s3_i2s_s *priv);
static uint32_t i2s_set_datawidth(struct esp32s3_i2s_s *priv);
static uint32_t i2s_set_clock(struct esp32s3_i2s_s *priv);
static uint32_t i2s_getmclkfrequency(struct i2s_dev_s *dev);
static uint32_t i2s_setmclkfrequency(struct i2s_dev_s *dev,
                                     uint32_t frequency);
static int      i2s_ioctl(struct i2s_dev_s *dev, int cmd, unsigned long arg);

#ifdef I2S_HAVE_TX
static void     i2s_tx_channel_start(struct esp32s3_i2s_s *priv);
static void     i2s_tx_channel_stop(struct esp32s3_i2s_s *priv);
static int      i2s_txchannels(struct i2s_dev_s *dev, uint8_t channels);
#define I2S_UPDATE_SPIN_MAX 200000

/* [V216] Does the I2S TX clock domain actually advance?
 *
 * I2S_TX_UPDATE hands the APB-domain configuration to the I2S TX clock domain
 * and clears itself when the transfer completes.  ESP-IDF spins on it before
 * setting tx_start; NuttX never waited.  If it never clears, the selected
 * source clock (CLK160) is not running and NOTHING on the TX side can work --
 * which from the GDMA side looks exactly like "the outlink parks without ever
 * fetching a descriptor". */
static int i2s_clock_domain_probe(int port)
{
  int spins = 0;

  modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_UPDATE);
  while (spins < I2S_UPDATE_SPIN_MAX &&
         (getreg32(I2S_TX_CONF_REG(port)) & I2S_TX_UPDATE))
    {
      spins++;
    }

  return spins;
}

/* [V341] *** THE SINGLE PLACE THAT SELECTS THE I2S SOURCE CLOCK ***
 *
 * Ground truth for the field encoding (esp-hal-3rdparty, esp32s3
 * hal/esp32s3/include/hal/i2s_ll.h, i2s_ll_tx_clk_set_src()):
 *     I2S_CLK_SRC_XTAL      -> tx_clk_sel = 0   (40 MHz crystal)
 *     I2S_CLK_SRC_PLL_160M  -> tx_clk_sel = 2   (PLL_F160M, fixed 160 MHz)
 * which is exactly the driver's own i2s_clk_src_t enum.  The value the driver
 * had been writing (2) is therefore PLL_F160M, not "PLL_D2".
 *
 * The measured failure (I2S_TX_CLKM_CONF = 0x34000027, CLK_SEL = 2, CLK_EN = 1,
 * CLK_ACTIVE = 1) coincides with I2S_TX_UPDATE never self-clearing
 * (spins = 200000).  tx_update is the handshake that copies the APB-domain
 * configuration into the I2S TX clock domain; it only completes while that
 * domain is actually being clocked.  So a stuck tx_update == a stalled TX
 * clock domain == the I2S never requests DMA data == PARK / DSCR=0 / silence.
 *
 * Before this patch the source was changed in three unrelated places
 * (i2s_configure(), i2s_txsamplerate() and esp32s3_i2s_set_rxonly()) and the
 * DMA start path additionally hard-coded a 160 MHz divider value
 * (0x003c0201), so the registers could disagree.  Now:
 *
 *   1. i2s_clk_src_select()      - the ONLY writer of I2S_TX/RX_CLK_SEL.
 *   2. i2s_clock_try_source()    - selects a source, programs every divider
 *                                  for it (via i2s_set_clock()) and measures
 *                                  the tx_update handshake.
 *   3. i2s_clock_source_bringup()- tries PLL_F160M first (the source the
 *                                  reference firmware used and the hardware
 *                                  default), and if that domain does not
 *                                  advance, falls back to XTAL (40 MHz), which
 *                                  is guaranteed to run because every other
 *                                  clock in the SoC is derived from it.
 *
 * The chosen source is remembered in g_i2s_clk_sel and the corresponding
 * divider base in g_i2s_sclk, so i2s_set_clock() and the DMA start path stay
 * consistent with it. */

static uint32_t g_i2s_clk_sel = I2S_PLL_F160M_CLK;

static void i2s_clk_src_select(int port, uint32_t sel)
{
  /* Open the module clock gate (bit 29 of TX_CLKM_CONF). */

  modifyreg32(I2S_TX_CLKM_CONF_REG(port), 0, I2S_CLK_EN);

  /* TX: pick the source, then mark the TX clock domain active. */

  modifyreg32(I2S_TX_CLKM_CONF_REG(port), I2S_TX_CLK_SEL_M,
              FIELD_TO_VALUE(I2S_TX_CLK_SEL, sel));
  modifyreg32(I2S_TX_CLKM_CONF_REG(port), 0, I2S_TX_CLK_ACTIVE);

  /* RX: keep the same source so a full-duplex stream cannot tear. */

#ifdef I2S_RX_CLK_SEL_M
  modifyreg32(I2S_RX_CLKM_CONF_REG(port), I2S_RX_CLK_SEL_M,
              FIELD_TO_VALUE(I2S_RX_CLK_SEL, sel));
  modifyreg32(I2S_RX_CLKM_CONF_REG(port), 0, I2S_RX_CLK_ACTIVE);
#endif
}

static int i2s_clock_try_source(struct esp32s3_i2s_s *priv, uint32_t sel,
                                uint32_t hz, uint32_t *ratep)
{
  int port = priv->config->port;

  /* Drop any pending configuration latch left over from a previous attempt
   * so the next tx_update is a fresh rising edge.  tx_reset/tx_fifo_reset do
   * not clear the APB-domain configuration registers (verified against the
   * V264 read-back, and ESP-IDF relies on the same ordering). */

  modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_RESET);
  modifyreg32(I2S_TX_CONF_REG(port), I2S_TX_RESET, 0);
  modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_FIFO_RESET);
  modifyreg32(I2S_TX_CONF_REG(port), I2S_TX_FIFO_RESET, 0);

  g_v671_cfg_valid = 0;   /* [V671] the reset above dropped the module state */

  /* TX must take its own BCK/WS, never the RX side's (see V250). */

  modifyreg32(I2S_TX_CONF_REG(port), I2S_SIG_LOOPBACK, 0);

#ifdef I2S_RX_CLK_SEL_M
  modifyreg32(I2S_RX_CONF_REG(port), 0, I2S_RX_RESET);
  modifyreg32(I2S_RX_CONF_REG(port), I2S_RX_RESET, 0);
#endif

  /* Base frequency for the divider maths, then every divider for this
   * source (integer part, fractional part and BCK divider). */

  g_i2s_sclk = hz;
  i2s_clk_src_select(port, sel);

  if (ratep != NULL)
    {
      *ratep = i2s_set_clock(priv);
    }
  else
    {
      (void)i2s_set_clock(priv);
    }

  /* Does the selected source actually advance the TX clock domain? */

  return i2s_clock_domain_probe(port);
}

static uint32_t i2s_clock_source_bringup(struct esp32s3_i2s_s *priv)
{
  int port = priv->config->port;
  int spins;
  uint32_t rate = 0;

  /* Attempt 1: PLL_F160M -- the hardware default.  The divider base is the
   * measured 150 MHz, not the 160 MHz the pin is named after (see
   * I2S_PLL_F160M_HZ above). */

  spins = i2s_clock_try_source(priv, I2S_PLL_F160M_CLK, I2S_PLL_F160M_HZ,
                               &rate);
  g_tx_update_spins = spins;

  

  if (spins < I2S_UPDATE_SPIN_MAX)
    {
      g_i2s_clk_sel = I2S_PLL_F160M_CLK;
      return rate;
    }

  /* Attempt 2: XTAL (40 MHz) -- always running, independent of the PLL. */

  spins = i2s_clock_try_source(priv, I2S_XTAL_CLK, 40000000u, &rate);
  g_tx_update_spins2 = spins;

  

  g_i2s_clk_sel = I2S_XTAL_CLK;

  if (spins >= I2S_UPDATE_SPIN_MAX)
    {
      /* Neither source advanced the domain: the stall is not caused by the
       * source clock.  Leave the (always-running) crystal selected so the
       * system is deterministic, and say so loudly. */

      
    }

  return rate;
}

static uint32_t i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t i2s_txdatawidth(struct i2s_dev_s *dev, int bits);
static int      i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                         i2s_callback_t callback, void *arg,
                         uint32_t timeout);
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
static void     i2s_rx_channel_start(struct esp32s3_i2s_s *priv);
static void     i2s_rx_channel_stop(struct esp32s3_i2s_s *priv);
static int      i2s_rx_release_inflight(struct esp32s3_i2s_s *priv);
#ifdef I2S_HAVE_TX
static int      i2s_tx_release_inflight(struct esp32s3_i2s_s *priv);
void            esp32s3_i2s_tx_flush(void);
#endif /* I2S_HAVE_TX */
static int      i2s_rxchannels(struct i2s_dev_s *dev, uint8_t channels);
static uint32_t i2s_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t i2s_rxdatawidth(struct i2s_dev_s *dev, int bits);
static int      i2s_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                            i2s_callback_t callback, void *arg,
                            uint32_t timeout);
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2s_ops_s g_i2sops =
{
#ifdef I2S_HAVE_TX
  .i2s_txchannels     = i2s_txchannels,
  .i2s_txsamplerate   = i2s_txsamplerate,
  .i2s_txdatawidth    = i2s_txdatawidth,
  .i2s_send           = i2s_send,
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  .i2s_rxchannels     = i2s_rxchannels,
  .i2s_rxsamplerate   = i2s_rxsamplerate,
  .i2s_rxdatawidth    = i2s_rxdatawidth,
  .i2s_receive        = i2s_receive,
#endif /* I2S_HAVE_RX */

  .i2s_ioctl             = i2s_ioctl,
  .i2s_getmclkfrequency  = i2s_getmclkfrequency,
  .i2s_setmclkfrequency  = i2s_setmclkfrequency,
};

#ifdef CONFIG_ESP32S3_I2S0
static const struct esp32s3_i2s_config_s esp32s3_i2s0_config =
{
  .port             = 0,
#ifdef CONFIG_ESP32S3_I2S0_ROLE_MASTER
  .role             = I2S_ROLE_MASTER,
#else
  .role             = I2S_ROLE_SLAVE,
#endif /* CONFIG_ESP32S3_I2S0_ROLE_MASTER */
  .data_width       = CONFIG_ESP32S3_I2S0_DATA_BIT_WIDTH,
  .rate             = CONFIG_ESP32S3_I2S0_SAMPLE_RATE,
  .total_slot       = 2,
  .tx_en            = I2S0_TX_ENABLED,
  .rx_en            = I2S0_RX_ENABLED,
  .tx_clk_src       = I2S_PLL_D2_CLK,
  .rx_clk_src       = I2S_PLL_D2_CLK,
#ifdef CONFIG_ESP32S3_I2S0_MCLK
  .mclk_pin         = CONFIG_ESP32S3_I2S0_MCLKPIN,
#else
  .mclk_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32S3_I2S0_MCLK */
  .bclk_pin         = CONFIG_ESP32S3_I2S0_BCLKPIN,
  .ws_pin           = CONFIG_ESP32S3_I2S0_WSPIN,
#ifdef CONFIG_ESP32S3_I2S0_DOUTPIN
  .dout_pin         = CONFIG_ESP32S3_I2S0_DOUTPIN,
#else
  .dout_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32S3_I2S0_DOUTPIN */
#ifdef CONFIG_ESP32S3_I2S0_DINPIN
  .din_pin          = CONFIG_ESP32S3_I2S0_DINPIN,
#else
  .din_pin          = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32S3_I2S0_DINPIN */
  .bclk_in_insig    = I2S0I_BCK_IN_IDX,
  .bclk_in_outsig   = I2S0I_BCK_OUT_IDX,
  .bclk_out_insig   = I2S0O_BCK_IN_IDX,
  .bclk_out_outsig  = I2S0O_BCK_OUT_IDX,
  .ws_in_insig      = I2S0I_WS_IN_IDX,
  .ws_in_outsig     = I2S0I_WS_OUT_IDX,
  .ws_out_insig     = I2S0O_WS_IN_IDX,
  .ws_out_outsig    = I2S0O_WS_OUT_IDX,
  .din_insig        = I2S0I_SD_IN_IDX,
  .dout_outsig      = I2S0O_SD_OUT_IDX,
  .mclk_out_sig     = I2S0_MCLK_OUT_IDX,
  .audio_std_mode   = I2S_TDM_PHILIPS,
};

static struct esp32s3_i2s_s esp32s3_i2s0_priv =
{
  .dev =
  {
    .ops = &g_i2sops,
  },
  .lock = NXMUTEX_INITIALIZER,
  .config = &esp32s3_i2s0_config,
  .bufsem = SEM_INITIALIZER(0),
};
#endif /* CONFIG_ESP32S3_I2S0 */

#ifdef CONFIG_ESP32S3_I2S1
static const struct esp32s3_i2s_config_s esp32s3_i2s1_config =
{
  .port             = 1,
#ifdef CONFIG_ESP32S3_I2S1_ROLE_MASTER
  .role             = I2S_ROLE_MASTER,
#else
  .role             = I2S_ROLE_SLAVE,
#endif /* CONFIG_ESP32S3_I2S1_ROLE_MASTER */
  .data_width       = CONFIG_ESP32S3_I2S1_DATA_BIT_WIDTH,
  .rate             = CONFIG_ESP32S3_I2S1_SAMPLE_RATE,
  .total_slot       = 2,
  .tx_en            = I2S1_TX_ENABLED,
  .rx_en            = I2S1_RX_ENABLED,
  .tx_clk_src       = I2S_PLL_D2_CLK,
  .tx_clk_src       = I2S_PLL_D2_CLK,
#ifdef CONFIG_ESP32S3_I2S1_MCLK
  .mclk_pin         = CONFIG_ESP32S3_I2S1_MCLKPIN,
#else
  .mclk_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32S3_I2S1_MCLK */
  .bclk_pin         = CONFIG_ESP32S3_I2S1_BCLKPIN,
  .ws_pin           = CONFIG_ESP32S3_I2S1_WSPIN,
#ifdef CONFIG_ESP32S3_I2S1_DOUTPIN
  .dout_pin         = CONFIG_ESP32S3_I2S1_DOUTPIN,
#else
  .dout_pin         = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32S3_I2S1_DOUTPIN */
#ifdef CONFIG_ESP32S3_I2S1_DINPIN
  .din_pin          = CONFIG_ESP32S3_I2S1_DINPIN,
#else
  .din_pin          = I2S_GPIO_UNUSED,
#endif /* CONFIG_ESP32S3_I2S1_DINPIN */
  .bclk_in_insig    = I2S1I_BCK_IN_IDX,
  .bclk_in_outsig   = I2S1I_BCK_OUT_IDX,
  .bclk_out_insig   = I2S1O_BCK_IN_IDX,
  .bclk_out_outsig  = I2S1O_BCK_OUT_IDX,
  .ws_in_insig      = I2S1I_WS_IN_IDX,
  .ws_in_outsig     = I2S1I_WS_OUT_IDX,
  .ws_out_insig     = I2S1O_WS_IN_IDX,
  .ws_out_outsig    = I2S1O_WS_OUT_IDX,
  .din_insig        = I2S1I_SD_IN_IDX,
  .dout_outsig      = I2S1O_SD_OUT_IDX,
  .mclk_out_sig     = I2S1_MCLK_OUT_IDX,
  .audio_std_mode   = I2S_TDM_PHILIPS,
};

static struct esp32s3_i2s_s esp32s3_i2s1_priv =
{
  .dev =
  {
    .ops = &g_i2sops,
  },
  .lock = NXMUTEX_INITIALIZER,
  .config = &esp32s3_i2s1_config,
  .bufsem = SEM_INITIALIZER(0),
};
#endif /* CONFIG_ESP32S3_I2S1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2s_buf_allocate
 *
 * Description:
 *   Allocate a buffer container by removing the one at the head of the
 *   free list
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   A non-NULL pointer to the allocate buffer container on success; NULL if
 *   there are no available buffer containers.
 *
 * Assumptions:
 *   The caller does NOT have exclusive access to the I2S state structure.
 *   That would result in a deadlock!
 *
 ****************************************************************************/

static struct esp32s3_buffer_s *i2s_buf_allocate(struct esp32s3_i2s_s *priv)
{
  struct esp32s3_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;

  /* Set aside a buffer container.  By doing this, we guarantee that we will
   * have at least one free buffer container.
   */

  ret = nxsem_wait_uninterruptible(&priv->bufsem);
  if (ret < 0)
    {
      return NULL;
    }

  /* Get the buffer from the head of the free list */

  flags = spin_lock_irqsave(&priv->slock);
  bfcontainer = priv->bf_freelist;

  /* [V723] The free list can legitimately be empty, and walking off a
   * NULL head here is exactly how the board died in the field.
   *
   * _v722_boot.log, t = 46.72:
   *
   *   [V646] UP cause=001c (LoadProhibited) vaddr=00000000
   *          pc=421f3302 -> i2s_buf_allocate
   *          a0=421f3d66 -> i2s_receive
   *          task es8311_workerthread
   *
   * Ten "[audio_direct] I2S TX completion TIMEOUT" lines came first:
   * the v711 timeout handling isolates a buffer rather than returning
   * it to the pool, internal DRAM is tight on this board ([V633]
   * intram_max has measured down to 2048), and the pool drains.
   *
   * The header above already promises "NULL if there are no available
   * buffer containers", and both callers check for NULL and report
   * -ENOMEM -- but DEBUGASSERT compiles out of release builds, so an
   * empty list went straight into bfcontainer->flink.
   *
   * Repairing the TX drain is a separate job.  This makes exhaustion
   * survivable: return NULL and give the semaphore count back, so the
   * pool can recover instead of deadlocking the next caller. */

  if (bfcontainer == NULL)
    {
      spin_unlock_irqrestore(&priv->slock, flags);
      nxsem_post(&priv->bufsem);
      return NULL;
    }

  DEBUGASSERT(bfcontainer);

  /* Unlink the buffer from the freelist */

  priv->bf_freelist = bfcontainer->flink;
  spin_unlock_irqrestore(&priv->slock, flags);
  
  return bfcontainer;
}

/****************************************************************************
 * Name: i2s_buf_free
 *
 * Description:
 *   Free buffer container by adding it to the head of the free list
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *   bfcontainer - The buffer container to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has exclusive access to the I2S state structure
 *
 ****************************************************************************/

static void i2s_buf_free(struct esp32s3_i2s_s *priv,
                         struct esp32s3_buffer_s *bfcontainer)
{
  irqstate_t flags;

  /* Put the buffer container back on the free list (circbuf) */

  flags = spin_lock_irqsave(&priv->slock);

  /* [V307] If this container still holds a DMA bounce slot (RX path
   * aborted before the worker ran), return it to the pool so the slot
   * cannot leak.  i2s_dmabuf_release() tolerates NULL. */

  i2s_dmabuf_release(bfcontainer->buf);

  bfcontainer->apb = NULL;
  bfcontainer->buf = NULL;
  bfcontainer->nbytes = 0;
  bfcontainer->flink  = priv->bf_freelist;
  priv->bf_freelist = bfcontainer;

  spin_unlock_irqrestore(&priv->slock, flags);

  /* Wake up any threads waiting for a buffer container */

  nxsem_post(&priv->bufsem);
}

/****************************************************************************
 * Name: i2s_buf_initialize
 *
 * Description:
 *   Initialize the buffer container allocator by adding all of the
 *   pre-allocated buffer containers to the free list
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 * Assumptions:
 *   Called early in I2S initialization so that there are no issues with
 *   concurrency.
 *
 ****************************************************************************/

static int i2s_buf_initialize(struct esp32s3_i2s_s *priv)
{
#ifdef I2S_HAVE_TX
  priv->tx.carry.bytes = 0;
  priv->tx.carry.value = 0;
#endif /* I2S_HAVE_TX */

  priv->bf_freelist = NULL;
  for (int i = 0; i < CONFIG_ESP32S3_I2S_MAXINFLIGHT; i++)
    {
      i2s_buf_free(priv, &priv->containers[i]);
    }

  return OK;
}

/****************************************************************************
 * Name: i2s_txdma_start
 *
 * Description:
 *   Initiate the next TX DMA transfer. The DMA outlink was previously bound
 *   so it is safe to start the next DMA transfer at interrupt level.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static int IRAM_ATTR i2s_txdma_start(struct esp32s3_i2s_s *priv)
{

  /* [V362] *** IDLE-SPIN FIX (team-lead V362 requirement #1). ***
   *
   * Up to V361 the two "nothing to do" bail-outs lived AFTER
   * i2s_apply_tx_gdma_strategy() and after two getreg32() reads, so an
   * empty call still
   *   - re-wrote DMA_OUT_CONF0 / CONF1 / PRI (6 register RMWs),
   *   - read I2S_STATE + I2S_TX_CONF,
   *   - emitted two syslog records,
   * before returning.  i2s_tx_schedule() reaches this function from the
   * GDMA EOF interrupt handler, and it does so exactly when tx.act is
   * empty -- i.e. every empty call was re-programming a live DMA channel
   * from interrupt context.  The measured consequence is 412
   * STA_DISCONNECT events in 240 s: the CPU never gets back to the WiFi
   * driver, LwIP misses its auth/assoc deadlines and the whole network
   * stack dies, which is why ASR and the LLM never even get to run.
   *
   * From here on, if there is nothing to transmit we return before
   * touching ANY register and before calling the GDMA helper.
   */

  g_tx_rate_calls++;

  if (!sq_empty(&priv->tx.act))
    {
      /* A transmission is already in flight -- nothing to arm. */
      g_tx_rate_bail_act++;
      i2s_tx_rate_tick();
      return OK;
    }

  if (sq_empty(&priv->tx.pend))
    {
      /* *** THE IDLE SPIN ***: no active and no queued buffer. */
      g_tx_rate_bail_pend++;

      if (g_tx_callsite == 1)
        {
          g_tx_rate_bail_s1++;
        }
      else if (g_tx_callsite == 2)
        {
          g_tx_rate_bail_s2++;
        }

      /* [V362] Requirement #2: name the caller.  The first few idle
       * entries are printed with their call-site id so the log itself
       * answers "who is calling with an empty buffer?".
       *   1 = i2s_txdma_setup()  (enqueue path)
       *   2 = i2s_tx_schedule()  (GDMA EOF ISR)
       */
      if (g_tx_rate_bail_pend <= 5)
        {
          
        }

      /* [V364] Both queues are empty: nothing is in flight and nothing is
       * queued, so any further TX completion interrupt is spurious.  Gate
       * the source before returning -- this is the actual fix for the
       * interrupt storm; V362 only made each arrival cheap.
       * i2s_tx_irq_unmask() re-enables it the moment a buffer is queued.
       */

      i2s_tx_irq_mask(priv);

      i2s_tx_rate_tick();
      return OK;
    }

  g_tx_rate_starts++;
  i2s_tx_rate_tick();
  /* [V338] 传输前的最后一步：再应用一次 GDMA strategy。
   *
   * V337 证明策略能写进去（CONF0=0x34/CONF1=0x1000/PRI=1），但
   * i2s_tx_channel_start() 里的 DMA_OUT_RST 会把它冲回 0x08。
   * 这里是设 OUTLINK_START 之前，之后不会再有复位，
   * 因此在这里补一次才能保证传输时策略在位。 */

  i2s_apply_tx_gdma_strategy(priv);

  struct esp32s3_buffer_s *bfcontainer;
  int light;                       /* [V671] 1 = DMA re-arm only */

  /* [V347] Prologue probe: answer team-lead's question "does the 2nd transfer
   * reach the I2S_TX_START write at all, or is it swallowed by an early
   * return?".  Log once per transfer what state we enter with. */

  g_tx_txdma_start_n++;
  

  /* If there is already an active transmission in progress, then bail
   * returning success.
   */

  if (!sq_empty(&priv->tx.act))
    {
      g_tx_txdma_bail_act++;
      
      return OK;
    }

  /* If there are no pending transfer, then bail returning success */

  if (sq_empty(&priv->tx.pend))
    {
      g_tx_txdma_bail_pend++;
      
      return OK;
    }

  /* [V671] *** Which path does this transfer take? ***
   *
   * light == 1 means the module is configured and the transmitter is already
   * running, so all this transfer needs is a descriptor for the GDMA.  That
   * is what upstream NuttX does, and it is the only way to keep the TX FIFO
   * from being flushed between two 2048-byte chunks (see g_v671_cfg_valid).
   *
   * light == 0 means a reset path ran since the last transfer, so re-run the
   * full configure/reset/start sequence exactly as before. */

  light = (g_v671_cfg_valid != 0 &&
           (getreg32(I2S_TX_CONF_REG(priv->config->port)) & I2S_TX_START) != 0);

  if (light)
    {
      g_v671_light_n++;
    }
  else
    {
      g_v671_full_n++;
    }

  /* Start transmission if no data is already being transmitted */

  bfcontainer = (struct esp32s3_buffer_s *)sq_remfirst(&priv->tx.pend);

  /* [V349] *** Stop the transmitter BEFORE the configure/reset block. ***
   *
   * The v347/v348 register trace isolates the one thing that differs between
   * the transfer that works and the transfers that do not:
   *
   *   first transfer: I2S_TX_CONF.START = 0 (TXCONF=0x00081200) when the
   *                   configure/reset block runs -> the reset really takes
   *                   effect -> after the start edge STATE=0x00000000 (TX
   *                   running), the DMA request is raised, transfer completes.
   *
   *   later transfers: I2S_TX_CONF.START = 1 (TXCONF=0x00081204, the bit the
   *                   previous transfer left behind) when the configure/reset
   *                   block runs -> the module is reset while tx_start is
   *                   already asserted, so the later start edge is swallowed
   *                   and STATE stays 0x00000001 (TX idle): no DMA request,
   *                   GDMA outlink parked, OWNER stuck, "completion TIMEOUT".
   *
   * (v345 ran this block every transfer and still failed; v346 ran it once and
   * failed; v347 added a stop->start toggle but placed the stop AFTER the
   * reset, so the reset still saw START=1; v348 restored the per-transfer
   * reset but likewise left START set.  The fix is ordering: stop first.)
   *
   * Do exactly what ESP-IDF's i2s_ll_tx_stop() does -- drive tx_start 0 and
   * latch it -- but do it here, ahead of the reset, and only when it is
   * actually asserted so the very first transfer is untouched. */

  if (!light &&
      (getreg32(I2S_TX_CONF_REG(priv->config->port)) & I2S_TX_START))
    {
      int pspins = 0;

      modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_START, 0);
      modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_TX_UPDATE);
      while (pspins < 200000 &&
             (getreg32(I2S_TX_CONF_REG(priv->config->port)) & I2S_TX_UPDATE))
        {
          pspins++;
        }

      
    }

  /* [V264] *** One block, in ESP-IDF's order: configure, LATCH, then start ***
   *
   * This replaces the V258 and V262 blocks, which were ordered wrongly: V262
   * wrote the official register values AFTER V258 had already run its
   * tx_update + tx_start, so those values only ever existed in the APB domain.
   * I2S_TX_UPDATE is what hands tx_conf to the TX clock domain; anything
   * written after it -- and in particular after tx_start -- is invisible to the
   * transmitter.  The clock domain therefore kept the old, wrong settings
   * (tx_stop_en 0 from V217, tx_left_align 1 from V236, tx_mono/chan_equal 1)
   * and the register read-backs looked "aligned" while the hardware was not.
   *
   * ESP-IDF's order, which this now follows literally:
   *
   *     configure -> tx_reset -> tx_fifo_reset -> tx_update (wait) -> tx_start
   *
   * Values are the ones measured on a WORKING image on this exact board:
   *   TX_CONF 0x00083204 (mono 0, chan_equal 0, left_align 0, stop_en 1)
   *   TX_TDM  0x00010003, TX_P2P 0x004aa004
   *   GDMA OUT CONF0 0x08, CONF1 0x00, PRI 0x00, PD_CONF 0x20
   *
   * Nothing below this block touches tx_conf, TX_TDM_CTRL, TX_PCM2PDM_CONF,
   * I2S_TX_UPDATE or the GDMA channel registers again. */

  /* [V346] Run the heavy configure/reset/start sequence once per channel
   * activation, not on every transfer.
   *
   * Upstream NuttX starts the I2S transmitter once -- at channel start -- and
   * every later transfer only re-arms the GDMA.  This block instead reset the
   * whole TX module (I2S_TX_RESET + I2S_TX_FIFO_RESET), re-derived the clock
   * dividers and re-ran tx_update/tx_start for every buffer, and because
   * i2s_tx_schedule() calls i2s_txdma_start() it did so from interrupt context
   * too.  The measurement shows the consequence: the first transfer starts and
   * completes (txirqN 0->1 and the completion callback fires) while later
   * transfers are left with the transmitter idle (I2S_STATE TX_IDLE=1, DSCR=0,
   * descriptor OWNER still 1) and never complete -- which is why the second
   * and later beeps time out.  Configure the module once; afterwards only
   * re-arm the DMA (the code below, outside this guard, still runs every
   * transfer). */

  /* [V348] *** Run the full configure/reset/start sequence on EVERY transfer.
   *
   * V346 made this block run only once (on the first transfer) to match the
   * upstream "start once, re-arm the DMA per transfer" model.  The V347 trace
   * shows that is wrong for THIS hardware path: every transfer that SKIPS the
   * block ends with the transmitter idle -- STATE=0x00000001, GDMA outlink
   * parked, descriptor OWNER stuck at 1, "I2S TX completion TIMEOUT" -- while
   * every transfer that RUNS it starts cleanly (STATE=0x00000000) and
   * completes:
   *
   *   txdma_start #1 (cfg_done=0): tx stop -> start -> STATE=0x00000000, OK
   *   txdma_start #3 (cfg_done=1): tx stop -> start -> STATE=0x00000001, stuck
   *
   * The load-bearing part is the module reset (I2S_TX_RESET + I2S_TX_FIFO_RESET)
   * plus the re-derived clock; without them the stop->start edge does not make
   * the TX core actually run.  So make every transfer look exactly like the
   * first, known-good one. */

  if (!light)
  {
    uint32_t port = priv->config->port;
    uint32_t ch   = priv->dma_channel;
    int sp = 0;

    g_tx_cfg_done = 1;   /* [V346] kept for the prologue diagnostic log only */

    /* ---- 1. configuration ---- */

    /* [V343] tx_stop_en is cleared, NOT set.  The V264 block used to write it
     * as 1 ("measured on a working image"), but that value is the direct
     * cause of the deadlock the V217 note already predicted: with tx_stop_en
     * set the bit clock is gated off the moment the TX FIFO is empty, and the
     * I2S never asserts its DMA request again, so the GDMA outlink has nothing
     * to move and parks (DSCR=0, I2S TX_IDLE=1, DOUT stuck at 0).  Clearing it
     * keeps BCK running so the FIFO can drain and the DMA request is raised.
     * The V217 clear inside i2s_tx_channel_start() was being undone here. */

    modifyreg32(I2S_TX_CONF_REG(port),
                I2S_TX_MONO_M | I2S_TX_CHAN_EQUAL_M |
                I2S_TX_LEFT_ALIGN_M | I2S_TX_STOP_EN_M,
                0);

    /* [V669] *** THE 2x-FAST BUG. ***
     *
     * This used to be a hard-coded
     *
     *     putreg32(0x00010003u, I2S_TX_TDM_CTRL_REG(port));
     *
     * i.e. TOT_CHAN_NUM = 1 (two slots per frame) with BOTH CHAN0_EN (bit 0)
     * and CHAN1_EN (bit 1) set.  Two enabled channels means the transmitter
     * pulls TWO samples out of the FIFO per LRCK period.  The mono PCM MiMo
     * synthesises is then played at exactly double rate -- pitch up an
     * octave, unintelligibly fast speech, the reported symptom.
     *
     * i2s_txchannels(1) already programs the right value for this board
     * (TOT_CHAN_NUM = 1 + CHAN0_EN only, measured as tdm = 0x00010001) but
     * this write, which runs on EVERY buffer, overrode it again.  That is
     * also why chasing MONO / CHAN_EQUAL went nowhere: those two bits only
     * control what is *transmitted* (duplicating the left sample onto the
     * right slot), never how many samples a frame consumes.  Latched into the
     * TX domain with the transmitter stopped (v668), the read-back was a
     * clean "mono_bit=1 eq_bit=1" and the rate did not move: 28 ksample/s
     * against an LRCK of 15109 Hz, i.e. still two samples per frame.
     *
     * Keep the two-slot frame structure (TOT_CHAN_NUM = 1) so BCLK stays at
     * 32 bits per frame and the ES8311 needs no change; just stop feeding the
     * right-hand slot out of the FIFO unless stereo is actually wanted.
     *
     * Same provenance as the I2S_TX_PCM2PDM_CONF_REG write below: 0x004aa004
     * is a value measured on the struggling image, kept as-is. */

    {
      uint32_t tdm = FIELD_TO_VALUE(I2S_TX_TDM_TOT_CHAN_NUM, 1) |
                     I2S_TX_TDM_CHAN0_EN;

      if (!g_i2s_tx_mono)
        {
          tdm |= I2S_TX_TDM_CHAN1_EN;
        }

      putreg32(tdm, I2S_TX_TDM_CTRL_REG(port));

    }
    putreg32(0x004aa004u, I2S_TX_PCM2PDM_CONF_REG(port));

    /* [V341] The clock divider is no longer hard-coded.  The old value
     * 0x003c0201 was measured on a 160 MHz-sourced image and is simply wrong
     * for any other source.  i2s_set_clock() re-derives TX/RX_CLKM_DIV_CONF
     * and the BCK divider from g_i2s_sclk (the source the common bring-up
     * actually selected) a few lines below, after the module reset. */

    /* [V339] 原来这里硬编码的是工作镜像上量到的复位值
     *   CONF0=0x08 / CONF1=0x00 / PRI=0x00
     * 而这三样恰恰是 V251 指出的致命项：
     *   out_auto_wrback(bit2)=0  ⇒ 发完不清 OWNER 位、不存传输长度
     *   out_dscr_burst(bit4)=out_data_burst(bit5)=0
     *   out_check_owner(conf1 bit12)=0
     *   out_pri=0
     * ⇒ outlink FSM 无法越过第一个描述符 ⇒ DSCR 恒 0、FIFO 永不排空。
     * 而且它写在 i2s_apply_tx_gdma_strategy() 之后，会把策略整个覆盖掉
     * —— 这就是之前怎么补 CONF0 都会回到 0x08 的原因。
     *
     * 改为 ESP-IDF 的配置值（owner_check=TRUE / desc_write_back=TRUE /
     * eof_till_popped=FALSE / burst 全开 / PRI=1）：
     *   CONF0 = 0x34 (AUTO_WRBACK|DSCR_BURST|DATA_BURST, EOF_MODE=0)
     *   CONF1 = 0x1000 (CHECK_OWNER)
     *   PRI   = 0x1 */
    putreg32(0x00000034u, DMA_OUT_CONF0_CH0_REG + ch * GDMA_REG_OFFSET);
    putreg32(0x00001000u, DMA_OUT_CONF1_CH0_REG + ch * GDMA_REG_OFFSET);
    putreg32(0x00000001u, DMA_OUT_PRI_CH0_REG + ch * GDMA_REG_OFFSET);

    modifyreg32(DMA_PD_CONF_REG,
                DMA_DMA_RAM_FORCE_PD_M | DMA_DMA_RAM_CLK_FO_M,
                DMA_DMA_RAM_FORCE_PU_M);

    /* ---- 2. reset the module and its FIFO ---- */

    modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_RESET);
    modifyreg32(I2S_TX_CONF_REG(port), I2S_TX_RESET, 0);
    modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_FIFO_RESET);
    modifyreg32(I2S_TX_CONF_REG(port), I2S_TX_FIFO_RESET, 0);

    /* [V341] Re-derive every clock divider for the source clock that the
     * common bring-up selected (g_i2s_sclk / g_i2s_clk_sel).  This replaces
     * the old hard-coded 160 MHz value and keeps DIV_CONF, CLKM_DIV_NUM and
     * the BCK divider consistent with the actual source. */

    /* [V362] Requirement #3: do not re-derive the TX clock on every
     * buffer.
     *
     * i2s_set_clock() divides with float/double (no FPU on LX7 under
     * NuttX) and then walks a/b up to I2S_LL_MCLK_DIVIDER_MAX looking
     * for the best fraction -- several hundred soft-float operations
     * executed from the GDMA EOF ISR, once per buffer.  Its result
     * depends only on (sclk, role, rate, total_slot, data_width,
     * mclk_freq), so derive it once and afterwards just re-apply the
     * latched register values.  We RE-APPLY rather than skip, because
     * the I2S_TX_RESET that runs a few lines above may have reverted
     * the divider registers; the values written are bit-identical to
     * what i2s_set_clock() would have produced.
     */

    if (!g_txclk_valid ||
        g_txclk_sclk  != (uint32_t)g_i2s_sclk ||
        g_txclk_role  != (uint32_t)priv->config->role ||
        g_txclk_rate  != (uint32_t)priv->rate ||
        g_txclk_slot  != (uint32_t)priv->config->total_slot ||
        g_txclk_width != (uint32_t)priv->data_width ||
        g_txclk_mclk  != (uint32_t)priv->mclk_freq)
      {
        (void)i2s_set_clock(priv);

        g_txclk_sclk  = (uint32_t)g_i2s_sclk;
        g_txclk_role  = (uint32_t)priv->config->role;
        g_txclk_rate  = (uint32_t)priv->rate;
        g_txclk_slot  = (uint32_t)priv->config->total_slot;
        g_txclk_width = (uint32_t)priv->data_width;
        g_txclk_mclk  = (uint32_t)priv->mclk_freq;

        g_txclk_clkm  = getreg32(I2S_TX_CLKM_CONF_REG(port));
        g_txclk_div   = getreg32(I2S_TX_CLKM_DIV_CONF_REG(port));
        g_txclk_conf1 = getreg32(I2S_TX_CONF1_REG(port));
        g_txclk_valid = 1;

        
      }
    else
      {
        putreg32(g_txclk_clkm,  I2S_TX_CLKM_CONF_REG(port));
        putreg32(g_txclk_div,   I2S_TX_CLKM_DIV_CONF_REG(port));
        putreg32(g_txclk_conf1, I2S_TX_CONF1_REG(port));
      }

    /* ---- 3. hand the configuration to the TX clock domain ---- */

    modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_UPDATE);
    while (sp < 200000 &&
           (getreg32(I2S_TX_CONF_REG(port)) & I2S_TX_UPDATE))
      {
        sp++;
      }

    

    

    /* [V265] tx_start deliberately NOT written here.  The original driver
     * armed the GDMA first and started the I2S last, and that ordering has
     * never been tested with the registers at their official values.  Starting
     * the transmitter while the FIFO is still empty is the one thing that can
     * make it go idle immediately; arming the DMA first means data is already
     * waiting when tx_start lands. */
  }

  /* [V352] Step-0 diagnostic: dump the whole outlink chain BEFORE the GDMA
   * is armed.
   *
   * DMA_OUT_AUTO_WRBACK (OUT_CONF0 bit 2) is set, so when the GDMA finishes a
   * descriptor it writes that descriptor back to SRAM: it clears OWNER and
   * rewrites the length field.  If anything downstream reuses the chain
   * without rebuilding it, a later transfer hands the GDMA a descriptor whose
   * OWNER is already 0 -- the outlink then parks immediately (DSCR=0x0,
   * PARK=1), the TX FIFO never receives data and the transmitter falls back
   * to TX_IDLE.  That is word for word what v349/v350 measured.
   *
   * Printing this once per i2s_txdma_start() and comparing transfer 1 against
   * transfers 2 and 3 decides it in a single run:
   *   identical  -> the outlink is rebuilt every transfer, not the cause;
   *   different  -> especially OWNER cleared or length zeroed, the chain is
   *                 being consumed and never restored. */

  if (g_tx_txdma_start_n <= 6)
    {
      for (int di = 0; di < I2S_DMADESC_NUM; di++)
        {
          volatile struct esp32s3_dmadesc_s *d =
            (volatile struct esp32s3_dmadesc_s *)&bfcontainer->dma_link[di];

          
        }
    }

  /* [V354] *** Hand the freshly written outlink and samples to physical SRAM. ***
   *
   * Everything above wrote bfcontainer->dma_link[] (size / length / EOF /
   * OWNER / next) and bfcontainer->buf[] (the PCM bytes) through the data
   * cache.  With CONFIG_ESP32S3_DATA_CACHE_32KB enabled and no cache
   * maintenance anywhere in the esp32s3 drivers, none of that is guaranteed to
   * be visible to the GDMA engine, which reads physical SRAM directly.
   *
   * A stale descriptor would look exactly like the reported failure: the FSM
   * has nothing it may legally fetch, so DMA_OUT_DSCR stays 0, the outlink
   * parks (link.PARK=1) and, with no data reaching the FIFO, the transmitter
   * falls back to TX_IDLE.
   *
   * Only CLEAN, never INVALIDATE:  dma_link[] shares cache lines with
   * neighbouring members of this same container (callback, apb, nbytes, ...),
   * so discarding those lines would throw away live data.  Cleaning writes
   * dirty lines back and leaves the lines valid, which is sufficient for a
   * TX path where the CPU is the only writer. */

  if (g_i2s_tx_cache_clean)
    {
      uintptr_t dstart = (uintptr_t)bfcontainer->dma_link;
      uintptr_t dend   = dstart + sizeof(bfcontainer->dma_link);
      uintptr_t bstart = (uintptr_t)bfcontainer->buf;
      uintptr_t bend   = bstart + bfcontainer->nbytes;

      up_clean_dcache(dstart, dend);

      if (bfcontainer->buf != NULL && bfcontainer->nbytes > 0)
        {
          up_clean_dcache(bstart, bend);
        }

      
    }

  /* [V356] *** Queue the transfer as ACTIVE before it is armed. ***
   *
   * This used to happen only at the very end, after the GDMA had already
   * been armed and started.  Any event the GDMA raised in between found
   * priv->tx.act empty, so i2s_tx_schedule() counted it as g_sched_noact
   * and threw the completion away -- the caller then blocked for the full
   * 1.5 s timeout even though the samples had really gone out.
   * Registering the buffer ahead of esp32s3_dma_load()/enable() removes
   * that window entirely; nothing between here and the end of this
   * function can fail, so no container can leak into tx.act. */

  sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.act);

  esp32s3_dma_load(bfcontainer->dma_link, priv->dma_channel, I2S_TX);
  esp32s3_dma_enable(priv->dma_channel, I2S_TX);

  /* [V214] Snapshot OUTLINK immediately after the START bit is written: if the
   * hardware clears it before i2s_send() can look, the outlink never ran. */
  if (g_txdump_n < 3)
    {
      g_dbg_link_after_enable =
        GET_GDMA_CH_REG(DMA_OUT_LINK_CH0_REG, priv->dma_channel);
    }

  /* [V353] *** Start in ESP-IDF's order: update -> wait -> START LAST. ***
   *
   * The V343 build wrote tx_start and only THEN tx_update, which is the exact
   * reverse of Espressif's i2s_ll_tx_start():
   *
   *     hw->tx_conf.tx_update = 1;
   *     while (hw->tx_conf.tx_update);    // wait for the copy to complete
   *     hw->tx_conf.tx_start = 1;         // start is the LAST step
   *
   * ("Have to update registers before start".)  tx_conf lives in the APB
   * domain until a tx_update copies it into the I2S TX clock domain, and that
   * copy is only defined while the transmitter is stopped.  Writing tx_update
   * *after* tx_start re-latches the whole tx_conf while the TX FSM is already
   * supposed to be running -- which is precisely the measured symptom: STATE
   * reads 0 on the instruction after the start write and has fallen back to
   * TX_IDLE on the next one, the TX FIFO stays empty and the GDMA outlink
   * parks.  V350's re-arm loop repeated the same ordering, so nothing further
   * downstream could recover it.
   *
   * A CPU-polling fallback is NOT available on this part.  Espressif's own
   * soc/esp32s3/include/soc/i2s_struct.h opens the I2S block with
   *     uint32_t reserved_0;  // 0x00
   *     uint32_t reserved_4;  // 0x04
   *     uint32_t reserved_8;  // 0x08
   *     union { ... } int_raw;// 0x0C
   * i.e. ESP32-S3 has no CPU-accessible TX FIFO port; the only "fifo" members
   * of tx_conf are the tx_fifo_reset / rx_fifo_reset control bits.  The TX
   * FIFO can only be filled by the GDMA, so the outlink has to be made to
   * work.  The I2S_TX_FIFO_REG(i) = base + 0x00 macro above therefore does not
   * map to anything real and must not be used. */

  if (!light)
  {
    uint32_t port = priv->config->port;
    int      stop_spins = 0;

    g_tx_state_pre = getreg32(I2S_STATE_REG(port));

    /* 1. i2s_ll_tx_stop(): drive tx_start low and hand that to the clock
     *    domain.  Without this the write in step 2 is not an edge at all,
     *    which is the failure mode V347 already documented. */

    modifyreg32(I2S_TX_CONF_REG(port), I2S_TX_START, 0);
    modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_UPDATE);
    while (stop_spins < I2S_UPDATE_SPIN_MAX &&
           (getreg32(I2S_TX_CONF_REG(port)) & I2S_TX_UPDATE))
      {
        stop_spins++;
      }

    /* [V668] *** Latch the channel mode while the transmitter is STOPPED. ***
     *
     * (Lines immediately above wrote TX_START = 0 and pushed a TX_UPDATE
     * through; TX_START is only driven high again a few lines below, and the
     * comment there says outright that no further update is issued after
     * that write.)
     *
     * Why this is the right moment.  TX_CONF is shadowed into the TX clock
     * domain and only takes effect when an update is handed over while
     * tx_start is low.  Everything the caller-side workaround (v664/v665/v666)
     * did wrote MONO / CHAN_EQUAL while the transmitter was already running,
     * so the APB copy read back correct -- "[V666] forced mono:
     * conf=0x00081264 mono=1 eq=1" -- while the datapath kept clocking at
     * 26104 samples/s instead of 16000 (measured, v667).
     *
     * What is being restored.  The V343 block earlier in this function clears
     * I2S_TX_MONO | I2S_TX_CHAN_EQUAL | I2S_TX_LEFT_ALIGN on every single
     * buffer, and the I2S_TX_RESET after it clears them again.  That block
     * exists to make a transmitter that used to deadlock actually run, and it
     * works -- but it leaves the transmitter in stereo mode permanently.
     *
     * Consequence of stereo mode: I2S_TX_TDM_TOT_CHAN_NUM is 1 here (bit field
     * [19:16]; measured tdm = 0x00010001, i.e. two slots with CHAN0 enabled),
     * so a frame carries two 16-bit slots and, with MONO / CHAN_EQUAL clear,
     * the transmitter lifts TWO samples out of the FIFO per LRCK period.  The
     * mono PCM that MiMo synthesises is then played at double rate: pitch up
     * an octave, unintelligibly fast speech -- the reported symptom.  Every
     * measurement agrees: registers all correct (MCLK 4.096 MHz, BCK_DIV 7 ->
     * BCLK 512 kHz, BITS_MOD 15 -> 16 bits/slot, BCLK/LRCK = 32) yet the clip
     * finishes early.
     *
     * I2S_TX_LEFT_ALIGN rides along because i2s_configure() sets it for
     * I2S_TDM_PHILIPS (V236) and this reset drops it too. */

    {
      uint32_t mode = 0;
      int      latch_spins = 0;

      if (g_i2s_tx_mono)
        {
          mode |= I2S_TX_MONO | I2S_TX_CHAN_EQUAL;
        }

      if (priv->config->audio_std_mode == I2S_TDM_PHILIPS)
        {
          mode |= I2S_TX_LEFT_ALIGN;
        }

      modifyreg32(I2S_TX_CONF_REG(port),
                  I2S_TX_MONO_M | I2S_TX_CHAN_EQUAL_M | I2S_TX_LEFT_ALIGN_M,
                  mode);

      modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_UPDATE);
      while (latch_spins < I2S_UPDATE_SPIN_MAX &&
             (getreg32(I2S_TX_CONF_REG(port)) & I2S_TX_UPDATE))
        {
          latch_spins++;
        }

    }

    g_tx_stop_spins = stop_spins;
    g_tx_state_post = getreg32(I2S_STATE_REG(port));

    /* 2. i2s_ll_tx_start(): tx_start is the LAST thing written, and no
     *    tx_update is issued afterwards. */

    modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_START);
  }

  /* [V671] The module is configured and the transmitter is running, so from
   * the next transfer on the light re-arm path is allowed.  Reached from both
   * paths (the light one skips the block above and lands here unchanged). */

  g_v671_cfg_valid = 1;

  /* [V353] Read I2S_STATE three times straight after the start write.  The
   * previous builds could only infer the "STATE 0 -> 1 one instruction later"
   * fall-back from separate syslog lines; three back-to-back reads make it
   * visible in one record. */

  {
    uint32_t s0 = getreg32(I2S_STATE_REG(priv->config->port));
    uint32_t s1 = getreg32(I2S_STATE_REG(priv->config->port));
    uint32_t s2 = getreg32(I2S_STATE_REG(priv->config->port));

    
  }

  /* [V347] Post-start verify: the whole point of this version is that this
   * reads STATE=0x00000000 (TX running) on EVERY transfer, not only the first,
   * and that I2S_TX_CONF.START is 1. */

  {
    uint32_t st = getreg32(I2S_STATE_REG(priv->config->port));
    uint32_t cf = getreg32(I2S_TX_CONF_REG(priv->config->port));

    g_tx_state_post = st;
    
  }


#if I2S_V350_REARM_PROBE
  /* [V350] *** Bounded re-arm / restart retry. ***
   *
   * The v349 trace pins the failure down to the GDMA side, not the I2S
   * configuration: on the 2nd+ transfer every register write is correct
   * (START=0 during the module reset, a clean stop->start edge, STATE reads
   * 0x00000000 immediately after the start handshake) yet STATE falls back to
   * TX_IDLE one instruction later, the TX FIFO stays EMPTY (OUTFIFO_STATUS
   * OUTFIFO_EMPTY=1) and the GDMA outlink parks (OUTLINK_PARK=1) with the
   * descriptor OWNER never cleared.  In other words the transmitter starts,
   * sees no data, and idles because the GDMA never serviced its request.
   *
   * Rather than guess at which single ordering the GDMA wants, re-arm the
   * outlink and re-assert tx_start a few times until the transmitter actually
   * stays running (or we run out of tries, which the log will show).  Each
   * iteration re-loads the descriptor address, re-enables the channel (which
   * pulses DMA_OUT_RST and sets OUTLINK_START), and drives tx_start 0 -> 1 with
   * a tx_update in between. */

  {
    uint32_t port = priv->config->port;
    uint32_t ch   = priv->dma_channel;
    int      tries;

    for (tries = 0; tries < 6; tries++)
      {
        uint32_t st   = getreg32(I2S_STATE_REG(port));
        uint32_t fifo = GET_GDMA_CH_REG(DMA_OUTFIFO_STATUS_CH0_REG, ch);
        uint32_t link = GET_GDMA_CH_REG(DMA_OUT_LINK_CH0_REG, ch);

        if ((st & 1u) == 0)
          {
            
            break;
          }

        esp32s3_dma_load(bfcontainer->dma_link, ch, I2S_TX);
        esp32s3_dma_enable(ch, I2S_TX);

        modifyreg32(I2S_TX_CONF_REG(port), I2S_TX_START, 0);
        modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_START);
        modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_UPDATE);
        {
          int u = 0;
          while (u < 200000 && (getreg32(I2S_TX_CONF_REG(port)) & I2S_TX_UPDATE))
            {
              u++;
            }
        }

        
      }
  }

#endif


  return OK;
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_rxdma_start
 *
 * Description:
 *   Initiate the next RX DMA transfer. Assuming the DMA inlink is already
 *   bound, it's safe to start the next DMA transfer in an interrupt context.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

/* [V388] rx_update spin counts around rx_start, and how many times the
 * snapshot has been logged.  Declared here (not next to the other V380
 * globals, which sit below i2s_rxdma_start()) so they are in scope.
 */

/* [V390] A/B switch: 1 = run the receiver as I2S master (own divider),
 * 0 = keep it as a slave of the TX generator (the current behaviour).
 */
volatile int g_v390_rxmaster = 1;

/* [V858] *** The receiver's clock role, and the counters behind the
 * post-playback relock. ***
 *
 * g_v858_rxslave is the role the relock programs:
 *   1 = the receiver is a slave and takes BCK/WS from the pads, which
 *       carry the transmitter generator's frame clock
 *   0 = the receiver runs from its own divider
 * The self-heal in voice_channel.c flips it, because which of the two
 * lands in phase with the ES8311 depends on where the two dividers
 * happened to start on this boot -- see the measurements in the [V858]
 * comment on i2s_v858_relock() below. */

volatile int g_v858_rxslave  = 1;
volatile int g_v858_relocks;
volatile int g_v858_heals;
volatile int g_v388_pre_spins;
volatile int g_v388_post_spins;
volatile int g_v388_logs;

/* [V691] *** the microphone root cause ***
 *
 * Every pin sample on record falls into exactly two states:
 *
 *   txstart=1, rxstart=0 ->  BCLK(10)=995..1004/2000,
 *                            WS(8)=1002..1014/2000,
 *                            DIN(9)=404..739/2000     <- all three ALIVE
 *   txstart=0, rxstart=1 ->  BCLK(10)=0/2000,
 *                            WS(8)=2000/2000 (stuck),
 *                            DIN(9)=0/2000             <- all three DEAD
 *
 * DIN only moves while I2S_TX_START is set.  The pads carry the TX
 * generator's BCK/WS (see the [V390] note in i2s_configure), so with
 * TX_START cleared the ES8311 has no bit clock, never drives DIN, and the
 * self-clocked receiver shifts in a static pad level -- exactly the
 * constant 0xFFFA (peak=6) every chunk has shown since v658.
 *
 * esp32s3_i2s_set_rxonly() clears TX_START and i2s_configure() pulses
 * I2S_TX_RESET; the [V382] clock-only start in i2s_rx_channel_start() is
 * undone a few lines later by i2s_configure() and nothing restores it.
 *
 * This helper puts it back.  It only acts while a capture is armed, so the
 * playback path is not affected.  verbose=0 must be used where it can run
 * in interrupt context (see the [V393] IRQ-stack note).
 */

static unsigned long g_v691_rearm;

/* [V724] *************************************************************
 * Read-only forensic probe for the "RX collapses after playback" defect.
 *
 * Nothing here changes hardware state on its own.  The register writes
 * are gated behind /tmp/i2sw.txt, which exists only when a person puts
 * it there -- the v721 lesson, where a self-driving sweep table quietly
 * reprogrammed RX_MSB_SHIFT and sent the diagnosis down a blind alley
 * ("clipping, lower the gain"), is exactly what this must not repeat.
 */

#define V724_REGLIST(X, p)                                     \
  X("CONF_SIGLE", I2S_CONF_SIGLE_DATA_REG,   p)                \
  X("INT_RAW",    I2S_INT_RAW_REG,           p)                \
  X("INT_ST",     I2S_INT_ST_REG,            p)                \
  X("INT_ENA",    I2S_INT_ENA_REG,           p)                \
  X("LC_HUNG",    I2S_LC_HUNG_CONF_REG,      p)                \
  X("STATE",      I2S_STATE_REG,             p)                \
  X("RX_CONF",    I2S_RX_CONF_REG,           p)                \
  X("RX_CONF1",   I2S_RX_CONF1_REG,          p)                \
  X("RX_TDM",     I2S_RX_TDM_CTRL_REG,       p)                \
  X("RX_TIMING",  I2S_RX_TIMING_REG,         p)                \
  X("RX_EOFNUM",  I2S_RXEOF_NUM_REG,         p)                \
  X("RX_CLKM",    I2S_RX_CLKM_CONF_REG,      p)                \
  X("RX_CLKDIV",  I2S_RX_CLKM_DIV_CONF_REG,  p)                \
  X("TX_CONF",    I2S_TX_CONF_REG,           p)                \
  X("TX_CONF1",   I2S_TX_CONF1_REG,          p)                \
  X("TX_TDM",     I2S_TX_TDM_CTRL_REG,       p)                \
  X("TX_TIMING",  I2S_TX_TIMING_REG,         p)                \
  X("TX_CLKM",    I2S_TX_CLKM_CONF_REG,      p)                \
  X("TX_CLKDIV",  I2S_TX_CLKM_DIV_CONF_REG,  p)

static uint32_t g_v724_prev[24];
static int      g_v724_init;

static int v724_streq(const char *a, const char *b)
{
  while (*a != '\0' && *a == *b)
    {
      a++;
      b++;
    }

  return (*a == '\0' && *b == '\0');
}

/* A bare block, not do/while(0): the list expands to twenty of these in
 * a row and only the last one would get the semicolon.
 */

#define V724_CAP(nm, reg, p)                                        \
  {                                                                 \
    uint32_t v724v = getreg32(reg(p));                              \
                                                                    \
    if (!g_v724_init || v724v != g_v724_prev[v724i])                \
      {                                                             \
        if (g_v724_init && v724n < 10)                              \
          {                                                         \
            syslog(LOG_INFO, "[V724] %s %s 0x%08x -> 0x%08x\n",      \
                   tag, nm, (unsigned)g_v724_prev[v724i],           \
                   (unsigned)v724v);                                \
            v724n++;                                                \
          }                                                         \
                                                                    \
        g_v724_prev[v724i] = v724v;                                 \
      }                                                             \
                                                                    \
    v724i++;                                                        \
  }

/* Print only what moved.  The first call seeds the shadow copy. */

static void i2s_v724_diff(int port, const char *tag)
{
  int v724i = 0;
  int v724n = 0;

  V724_REGLIST(V724_CAP, port)
  g_v724_init = 1;
}

#define V724_WR(nm, reg, p)                                         \
  {                                                                 \
    if (v724_streq(v724name, nm))                                   \
      {                                                             \
        putreg32(v724val, reg(p));                                  \
        return 1;                                                   \
      }                                                             \
  }

static int i2s_v724_write(int port, const char *v724name,
                          uint32_t v724val)
{
  V724_REGLIST(V724_WR, port)
  return 0;
}

/* [V724-C] /tmp/i2sw.txt, one "NAME 0xVALUE" pair per line.  Applied
 * straight from the rx completion path, so a candidate fix is live in
 * about a hundred milliseconds and no reflash is needed to test it.
 */

static void i2s_v724_hook(int port)
{
  static unsigned long s_v724_seen;
  FILE   *v724f;
  char    v724buf[256];
  size_t  v724n;
  size_t  v724i;
  size_t  v724j;
  unsigned long v724sum = 0u;
  int     v724applied = 0;

  v724f = fopen("/tmp/i2sw.txt", "r");
  if (v724f == NULL)
    {
      s_v724_seen = 0u;
      return;
    }

  v724n = fread(v724buf, 1, sizeof(v724buf) - 1u, v724f);
  fclose(v724f);
  v724buf[v724n] = '\0';

  v724i = 0u;
  while (v724i < v724n)
    {
      char     v724nm[32];
      unsigned v724k = 0u;
      uint32_t v724val = 0u;
      int      v724ok = 0;

      while (v724i < v724n &&
             (v724buf[v724i] == ' ' || v724buf[v724i] == '\t' ||
              v724buf[v724i] == '\n' || v724buf[v724i] == '\r'))
        {
          v724i++;
        }

      if (v724i >= v724n)
        {
          break;
        }

      while (v724i < v724n && v724buf[v724i] != ' ' &&
             v724buf[v724i] != '\t' && v724buf[v724i] != '\n' &&
             v724buf[v724i] != '\r' && v724k < (sizeof(v724nm) - 1u))
        {
          v724nm[v724k++] = v724buf[v724i++];
        }

      v724nm[v724k] = '\0';

      while (v724i < v724n &&
             (v724buf[v724i] == ' ' || v724buf[v724i] == '\t'))
        {
          v724i++;
        }

      if ((v724i + 1u) < v724n && v724buf[v724i] == '0' &&
          (v724buf[v724i + 1u] == 'x' || v724buf[v724i + 1u] == 'X'))
        {
          v724i += 2u;
        }

      while (v724i < v724n)
        {
          char     c = v724buf[v724i];
          unsigned d;

          if (c >= '0' && c <= '9')
            {
              d = (unsigned)(c - '0');
            }
          else if (c >= 'a' && c <= 'f')
            {
              d = (unsigned)(c - 'a') + 10u;
            }
          else if (c >= 'A' && c <= 'F')
            {
              d = (unsigned)(c - 'A') + 10u;
            }
          else
            {
              break;
            }

          v724val = (v724val << 4) | d;
          v724i++;
          v724ok = 1;
        }

      while (v724i < v724n && v724buf[v724i] != '\n')
        {
          v724i++;
        }

      v724i++;

      if (v724ok && v724nm[0] != '\0')
        {
          v724sum = v724sum * 131u + (unsigned long)v724val +
                    (unsigned long)v724nm[0];

          if (i2s_v724_write(port, v724nm, v724val))
            {
              v724applied++;
            }
        }
    }

  if (v724sum != s_v724_seen)
    {
      s_v724_seen = v724sum;
      syslog(LOG_INFO, "[V724] hook: %d pair(s) written\n", v724applied);
    }
}

/* [V724-D] /tmp/pins.txt -- is the codec actually driving DIN?
 *
 * The [V383] block inside i2s_rx_channel_start() already knows how to
 * do this but its syslog is compiled behind "if (0)".  Same 4000 reads,
 * now on demand:
 *
 *   BCLK(10) hi ~= 2000/4000  the frame clock is running
 *   WS(8)    hi ~= 2000/4000  frames are being delimited
 *   DIN(9)   hi ~= 2000/4000  the ES8311 is shifting data at us
 *   either extreme            that line is stuck
 */

static void i2s_v724_pins(void)
{
  FILE        *v724f;
  unsigned int v724b = 0u;
  unsigned int v724w = 0u;
  unsigned int v724d = 0u;
  unsigned int v724k;

  v724f = fopen("/tmp/pins.txt", "r");
  if (v724f == NULL)
    {
      return;
    }

  fclose(v724f);

  modifyreg32(IO_MUX_GPIO8_REG,  0, (1u << 9));   /* WS   */
  modifyreg32(IO_MUX_GPIO9_REG,  0, (1u << 9));   /* DIN  */
  modifyreg32(IO_MUX_GPIO10_REG, 0, (1u << 9));   /* BCLK */

  for (v724k = 0u; v724k < 4000u; v724k++)
    {
      if (esp32s3_gpioread(10))
        {
          v724b++;
        }

      if (esp32s3_gpioread(8))
        {
          v724w++;
        }

      if (esp32s3_gpioread(9))
        {
          v724d++;
        }
    }

  syslog(LOG_INFO,
         "[V724] pins BCLK(10) hi=%u WS(8) hi=%u DIN(9) hi=%u /4000\n",
         v724b, v724w, v724d);
}

/* [V714] *** Rate limiter for the [V692] probe below. ***
 *
 * g_v692_stopfix only increments when I2S_TX_STOP_EN was actually found
 * set, and that happens once per capture -- so it parks at 1 for the whole
 * run.  The old guard
 *
 *     if (verbose && (g_v692_stopfix <= 6 || (g_v692_stopfix % 64) == 0))
 *
 * therefore stayed true on every call and the probe printed one line per RX
 * buffer completion: ~17 lines/s, which drowned the NSH console and crowded
 * the 32 KB kmsg ring.  Count the calls instead of the stopfixes. */

static unsigned long g_v714_calls;
static unsigned long g_v714_lastfix = ~0ul;

static unsigned long g_v692_stopfix;

static void i2s_v691_clock_ensure(struct esp32s3_i2s_s *priv, int verbose)
{
  uint32_t port = priv->config->port;
  int spins;

  /* Nothing armed -> not a capture -> leave the port alone. */

  if (sq_empty(&priv->rx.act) && sq_empty(&priv->rx.pend))
    {
      return;
    }

  /* [V692] *** I2S_TX_STOP_EN (bit13) is what actually kills the frame
   * clock. ***
   *
   * ESP-IDF's i2s_ll_tx_set_stop_when_empty() maps auto_clear to this bit:
   * with it set, the transmitter halts as soon as its FIFO runs dry.  The
   * capture uses the transmitter purely as a clock generator, so its FIFO
   * is ALWAYS dry -- it stops on the first frame and BCK/WS drop to their
   * idle levels.
   *
   * Measured, and perfectly consistent with that bit:
   *   txconf=0x00089264  STOP_EN clear -> BCLK 995..1004/2000,
   *                                        WS  1002..1014/2000,
   *                                        DIN 404..739/2000      ALIVE
   *   txconf=0x0008b2xx  STOP_EN set   -> BCLK 0/2000,
   *                                        WS  2000/2000 (stuck),
   *                                        DIN 0/2000             DEAD
   *
   * Cleared only while a capture is armed; i2s_configure() puts it back
   * for the playback path.
   */

  if (getreg32(I2S_TX_CONF_REG(port)) & I2S_TX_STOP_EN)
    {
      modifyreg32(I2S_TX_CONF_REG(port), I2S_TX_STOP_EN, 0);
      g_v692_stopfix++;
    }

  if (!(getreg32(I2S_TX_CONF_REG(port)) & I2S_TX_START))
    {
      modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_START);
      g_v691_rearm++;
    }

  modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_UPDATE);

  for (spins = 0;
       spins < I2S_UPDATE_SPIN_MAX &&
       (getreg32(I2S_TX_CONF_REG(port)) & I2S_TX_UPDATE);
       spins++)
    {
    }

  g_v691_rearm++;

  g_v714_calls++;

  if (verbose && (g_v714_calls <= 6 || (g_v714_calls % 2048) == 0 ||
                  g_v692_stopfix != g_v714_lastfix))
    {
      g_v714_lastfix = g_v692_stopfix;
      syslog(LOG_INFO,
             "[V692] tx clock: stopfix=%lu rearm=%lu spins=%d txconf=0x%08x "
             "rxconf=0x%08x\n",
             g_v692_stopfix, g_v691_rearm, spins,
             (unsigned)getreg32(I2S_TX_CONF_REG(port)),
             (unsigned)getreg32(I2S_RX_CONF_REG(port)));
    }
}

#ifdef I2S_HAVE_RX
static int i2s_rxdma_start(struct esp32s3_i2s_s *priv)
{
  g_v94_start++;
  struct esp32s3_buffer_s *bfcontainer;
  size_t eof_nbytes;
  irqstate_t flags;            /* [V395] arm+enqueue critical section */

  /* If there is already an active transmission in progress, then bail
   * returning success.
   */

  if (!sq_empty(&priv->rx.act))
    {
      return OK;
    }

  /* If there are no pending transfer, then bail returning success */

  if (sq_empty(&priv->rx.pend))
    {
      return OK;
    }

  bfcontainer = (struct esp32s3_buffer_s *)sq_remfirst(&priv->rx.pend);

  /* If there isn't already an active transmission in progress,
   * then start it.
   */

  /* [V385] The register counts in BITS, not bytes:
   *   trigger = (I2S_RX_BITS_MOD + 1) * (REG_RX_EOF_NUM + 1)
   * so the old MIN(nbytes, DMA_BUFLEN_MAX) = 4095 asked for 8192 bytes
   * while the inlink only covers 4096 - the link ran off its NULL tail
   * before EOF, no SUC_EOF, no interrupt, capture EIO.  Start with a
   * deliberately small threshold so the interrupt has to fire if it is
   * merely mis-programmed.  See patch_v385.py.
   */

  /* [V687] Size the EOF threshold to the WHOLE buffer.
   *
   * I2S_RX_EOF_NUM counts BITS (hardware/esp32s3_i2s.h:1841):
   *   "The receive data bit length is (I2S_RX_BITS_MOD[4:0] + 1) *
   *    (REG_RX_EOF_NUM[11:0] + 1).  It will trigger in_suc_eof
   *    interrupt in the configured DMA RX channel."
   * so for 16-bit words the trigger in bytes is 2 * (N + 1).
   *
   * v385 programmed 512, i.e. a 1026-byte trigger, while the buffer the
   * app hands us is 4096 bytes, which CONFIG_I2S_DMADESC_NUM=2 splits
   * into 4095 + 1 descriptors.  Both consumers of the EOF bit only look
   * at the FIRST EOF-marked descriptor: i2s_rx_schedule() completes the
   * transfer only when DMA_IN_SUC_EOF_DES_ADDR names the last EOF-marked
   * descriptor, and i2s_rx_worker() sums DATALEN only while the EOF bit
   * is set.  With the EOF in the middle of descriptor 0 every arm
   * delivered ~1 KB of 4 KB and the capture starved: v686
   * (_v686_btnfix.log, one 8 s press) logged 6 chunks / 9208 bytes where
   * ~256000 were expected, a single SUC_EOF for the whole session, and
   * three "recording thread exit" lines.
   *
   * Keep the trigger just INSIDE the first descriptor.  2044 -> 16 * 2045
   * bits = 4090 bytes, which fits either possible first-descriptor size
   * (4092 with burst, 4095 without); the six tail bytes are 0.15% of the
   * stream.  A buffer small enough to be a single descriptor takes the
   * exact whole-buffer value, which is lossless.
   *
   * v385's "descriptor chain runs off its NULL tail" failure is what
   * happens when the trigger EXCEEDS the chain - MIN(nbytes, 4095) asked
   * for 8192 bytes over a 4096-byte inlink, the chain ended before EOF and
   * no SUC_EOF was ever raised.  Never program more than the chain holds.
   */

  if (bfcontainer->nbytes >= 4u && bfcontainer->nbytes <= 4092u)
    {
      eof_nbytes = (size_t)(bfcontainer->nbytes / 2u) - 1u;
    }
  else
    {
      eof_nbytes = 2044u;
    }

  /* [V394] IRQ-stack: silenced */ if (0) syslog(LOG_INFO, "[V385] rx eof_num=%u (nbytes=%u)\n",
         (unsigned)eof_nbytes, (unsigned)bfcontainer->nbytes);

  modifyreg32(I2S_RXEOF_NUM_REG(priv->config->port), I2S_RX_EOF_NUM_M,
              FIELD_TO_VALUE(I2S_RX_EOF_NUM, eof_nbytes));

  /* [V392] Enqueue BEFORE arming, not after.
   *
   * This is the RX twin of the completion-loss bug V356 fixed on the TX
   * side.  The GDMA is armed by esp32s3_dma_load() below; any completion
   * raised before sq_addlast(&priv->rx.act) runs finds rx.act empty and
   * is dropped, so the reader waits forever on a buffer that has in fact
   * already been filled.  With the enqueue first there is no window.
   */

  /* [V395] Arm and enqueue atomically - see the note above. */

  flags = enter_critical_section();

  esp32s3_dma_load(bfcontainer->dma_link, priv->dma_channel, I2S_RX);
  esp32s3_dma_enable(priv->dma_channel, I2S_RX);

  sq_addlast((sq_entry_t *)bfcontainer, &priv->rx.act);

  leave_critical_section(flags);

  /* [V388] ESP-IDF i2s_ll_rx_start() hands the APB configuration to the
   * RX clock domain with rx_update *immediately before* raising rx_start,
   * after every configuration write that has to reach the receiver.
   * i2s_rx_channel_start() issues its rx_update before this function
   * programs I2S_RXEOF_NUM_REG and arms the GDMA, so the threshold - and
   * rx_start itself - go out with no handshake behind them.  Do it here,
   * on both sides of rx_start, and time how long it takes to self-clear.
   * If it never clears the RX module is not accepting configuration at
   * all, which is exactly what "clocks on the pads, data on DIN, and
   * still zero bytes" looks like.
   */

  {
    int spins;

    modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0, I2S_RX_UPDATE);

    for (spins = 0;
         spins < I2S_UPDATE_SPIN_MAX &&
         (getreg32(I2S_RX_CONF_REG(priv->config->port)) & I2S_RX_UPDATE);
         spins++)
      {
      }

    g_v388_pre_spins = spins;
  }

  /* [V681] Mono capture takes ONE sample per frame.
   *
   * i2s_configure()'s RX branch enables CHAN0_EN and CHAN1_EN at the same
   * bit positions as the TX 2x-speed bug fixed in v669, so the RX FIFO
   * handed the DMA two samples per frame.  The board captures mono, and
   * with no_dac_ref the mic sits in slot 0, so only slot 0 must be armed.
   * Done on every arm so no later configure can undo it.
   */
  /* [V682] *** the real root cause of the silent microphone ***
   *
   * The chan_en half of rx_tdm_ctrl must be ASSIGNED, not modified.
   * esp32s3_i2s.h:1359 says I2S_RX_TDM_PDM_CHAN0_EN has "default: 1'h1",
   * so bits 2..15 come out of reset ENABLED.  modifyreg32() with only the
   * CHAN0_EN|CHAN1_EN mask never cleared them, and the register read back
   * as 0x0001ffff: sixteen channels armed for a frame that carries two
   * slots.  The receiver de-serialises against a 16-channel map, so what
   * it stores is not what the codec sent -- every chunk was silence.
   *
   * ESP-IDF does it this way (hal/esp32s3/include/hal/i2s_ll.h):
   *
   *     tdm_ctrl  = hw->rx_tdm_ctrl.val;
   *     tdm_ctrl &= 0xFFFF0000;      <-- clears all sixteen chanN_en
   *     tdm_ctrl |= chan_mask;       <-- then the wanted slots only
   *     hw->rx_tdm_ctrl.val = tdm_ctrl;
   *
   * i2s_ll_rx_select_std_slot() passes 0x01 (left) / 0x02 (right) /
   * 0x03 (both) for the 2-slot frame it just set up.  The transmitter
   * never had this bug because i2s_txchannels() assigns the whole word --
   * which is exactly why playback has always worked.
   */
  {
    uint32_t v682_tdm = getreg32(I2S_RX_TDM_CTRL_REG(priv->config->port));

    v682_tdm &= 0xffff0000u;
    v682_tdm |= FIELD_TO_VALUE(I2S_RX_TDM_TOT_CHAN_NUM, 1);  /* 2 slots */
    v682_tdm |= g_i2s_rx_slot_mask;
    putreg32(v682_tdm, I2S_RX_TDM_CTRL_REG(priv->config->port));
  }
  {
    static int s_v681_logged;
    if (s_v681_logged < 3)
      {
        s_v681_logged++;
        syslog(LOG_INFO,
               "[V683] rx tdm=0x%08x slot=0x%02x conf1=0x%08x "
               "clkm=0x%08x clkdiv=0x%08x eofnum=0x%08x dw=%lu rate=%lu "
               "mclk=%lu mult=%lu ch=%lu\n",
               (unsigned)getreg32(I2S_RX_TDM_CTRL_REG(priv->config->port)),
               (unsigned)g_i2s_rx_slot_mask,
               (unsigned)getreg32(I2S_RX_CONF1_REG(priv->config->port)),
               (unsigned)getreg32(I2S_RX_CLKM_CONF_REG(priv->config->port)),
               (unsigned)getreg32(I2S_RX_CLKM_DIV_CONF_REG(
                                      priv->config->port)),
               (unsigned)getreg32(I2S_RXEOF_NUM_REG(priv->config->port)),
               (unsigned long)priv->data_width,
               (unsigned long)priv->rate,
               (unsigned long)priv->mclk_freq,
               (unsigned long)priv->mclk_multiple,
               (unsigned long)priv->channels);
      }
  }

  modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0, I2S_RX_START);

  {
    int spins;

    modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0, I2S_RX_UPDATE);

    for (spins = 0;
         spins < I2S_UPDATE_SPIN_MAX &&
         (getreg32(I2S_RX_CONF_REG(priv->config->port)) & I2S_RX_UPDATE);
         spins++)
      {
      }

    g_v388_post_spins = spins;
  }

  /* [V691] Restore the TX clock generator the moment the RX is armed.
   * No syslog here: this function also runs in IRQ context. */

  i2s_v691_clock_ensure(priv, 0);

  if (g_v388_logs < 4)
    {
      g_v388_logs++;
      /* [V394] IRQ-stack: silenced */ if (0) syslog(LOG_INFO,
             "[V388] rx_update spins pre=%d post=%d rxconf=0x%08x "
             "eofnum=0x%08x fifo=0x%08x intena=0x%08x\n",
             g_v388_pre_spins, g_v388_post_spins,
             (unsigned)getreg32(I2S_RX_CONF_REG(priv->config->port)),
             (unsigned)getreg32(I2S_RXEOF_NUM_REG(priv->config->port)),
             (unsigned)getreg32(I2S_RX_CLKM_CONF_REG(priv->config->port)),
             (unsigned)getreg32(I2S_INT_ENA_REG(priv->config->port)));

      /* [V389] I2S_INT_ST_REG is gated by INT_ENA, which reads 0, so it
       * can never show anything.  I2S_INT_RAW_REG is not gated:
       *   bit0 RX_DONE, bit1 TX_DONE, bit2 RX_HUNG, bit3 TX_HUNG
       * and rx_tdm_ctrl is the register V389 now programs.
       */
      /* [V394] IRQ-stack: silenced */ if (0) syslog(LOG_INFO,
             "[V389] intraw=0x%08x rxtdmctrl=0x%08x rxconf1=0x%08x\n",
             (unsigned)getreg32(I2S_INT_RAW_REG(priv->config->port)),
             (unsigned)getreg32(I2S_RX_TDM_CTRL_REG(priv->config->port)),
             (unsigned)getreg32(I2S_RX_CONF1_REG(priv->config->port)));
    }

  /* [V393] V387's post-arm dump is disabled: i2s_rxdma_start() runs in
   * interrupt context when it re-arms the next buffer, and its 200000
   * spin loop plus syslog() overflowed the 2 KB IRQ stack
   * (v392run.log: sp 0x3fccbeb0, base 0x3fccb800, size 2048).
   * It has served its purpose - it is what proved the receiver was
   * never requesting a transfer. */
#if 0
  /* [V387] Give the channel a moment, then read what it actually did. */

  {
    static int s_v387_n;

    if (s_v387_n < 4)
      {
        volatile uint32_t sink;
        uint32_t k;

        s_v387_n++;

        for (k = 0; k < 200000u; k++)
          {
            sink = GET_GDMA_CH_REG(DMA_IN_INT_RAW_CH0_REG,
                                   priv->dma_channel);
            if (sink & (DMA_IN_SUC_EOF_CH0_INT_RAW |
                        DMA_IN_DONE_CH0_INT_RAW |
                        DMA_IN_ERR_EOF_CH0_INT_RAW |
                        DMA_IN_DSCR_ERR_CH0_INT_RAW |
                        DMA_IN_DSCR_EMPTY_CH0_INT_RAW |
                        DMA_INFIFO_OVF_L3_CH0_INT_RAW |
                        DMA_INFIFO_UDF_L1_CH0_INT_RAW))
              {
                break;
              }
          }

        

        
      }
  }

#endif

  /* [RX-DIAG] Dump RX state on every DMA arm to diagnose 0-data capture */
  {
    static unsigned int arm_cnt = 0;
    arm_cnt++;
    /* [V394] IRQ-stack: silenced */ if (0) syslog(LOG_INFO,
           "[RXDUMP] arm#%u rxconf=0x%08x eof_nbytes=0x%08x i2sintst=0x%08x"
           " gdmainst=0x%08x isr=%u eof=%u\n",
           arm_cnt,
           (unsigned)getreg32(I2S_RX_CONF_REG(priv->config->port)),
           (unsigned)getreg32(I2S_RXEOF_NUM_REG(priv->config->port)),
           (unsigned)getreg32(I2S_INT_ST_REG(priv->config->port)),
           (unsigned)GET_GDMA_CH_REG(DMA_IN_INT_ST_CH0_REG,
                                      priv->dma_channel),
           g_v94_isr, g_v94_eof);
  }

  return OK;
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: i2s_txdma_setup
 *
 * Description:
 *   Setup the next TX DMA transfer
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *   bfcontainer - The buffer container to be set up
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static IRAM_ATTR int i2s_txdma_setup(struct esp32s3_i2s_s *priv,
                                     struct esp32s3_buffer_s *bfcontainer)
{
  int ret = OK;
  size_t carry_size;
  uint32_t bytes_queued;
  uint32_t data_copied;
  struct ap_buffer_s *apb;
  struct esp32s3_dmadesc_s *outlink;
  apb_samp_t samp_size;
  irqstate_t flags;
  uint8_t *buf;
  uint8_t padding;
  uint8_t *samp;

  DEBUGASSERT(bfcontainer && bfcontainer->apb);

  apb = bfcontainer->apb;
  outlink = bfcontainer->dma_link;

  /* Get the transfer information, accounting for any data offset */

  const apb_samp_t bytes_per_sample = priv->data_width / 8;
  samp = &apb->samp[apb->curbyte];
  samp_size = (apb->nbytes - apb->curbyte) + priv->tx.carry.bytes;
  carry_size = samp_size % bytes_per_sample;

  /* Allocate the current audio buffer considering the remaining bytes
   * carried from the last upper half audio buffer.
   */

  /* [V211] Must be internal SRAM: see i2s_dmabuf_alloc(). */
  bfcontainer->buf = i2s_dmabuf_alloc(bfcontainer->nbytes);
  if (bfcontainer->buf == NULL)
    {
      i2serr("Failed to allocate the DMA internal buffer "
             "[%" PRIu32 " bytes]", bfcontainer->nbytes);
      return -ENOMEM;
    }

  data_copied = 0;
  buf = bfcontainer->buf;

  /* Copy the remaining bytes from the last audio buffer to the current
   * audio buffer. The remaining bytes are part of a sample that was split
   * between the last and the current audio buffer. Also, copy the bytes
   * from that split sample that are on the current buffer to the internal
   * buffer.
   */

  if (priv->tx.carry.bytes)
    {
      memcpy(buf, &priv->tx.carry.value, priv->tx.carry.bytes);
      buf += priv->tx.carry.bytes;
      data_copied += priv->tx.carry.bytes;
      memcpy(buf, samp, (bytes_per_sample - priv->tx.carry.bytes));
      buf += (bytes_per_sample - priv->tx.carry.bytes);
      samp += (bytes_per_sample - priv->tx.carry.bytes);
      data_copied += (bytes_per_sample - priv->tx.carry.bytes);
    }

  /* Copy the upper half buffer to the internal buffer considering that
   * the current upper half buffer may not contain a complete sample at
   * the end of the buffer (and those bytes needs to be carried to the
   * next audio buffer).
   */

  memcpy(buf, samp, samp_size - (data_copied + carry_size));
  buf += samp_size - (data_copied + carry_size);
  samp += samp_size - (data_copied + carry_size);
  data_copied += samp_size - (data_copied + carry_size);

  /* If the audio buffer's size is not a multiple of the sample size,
   * it's necessary to carry the remaining bytes that are part of what
   * would be the last sample on this buffer. These bytes will then be
   * saved and inserted at the beginning of the next DMA buffer to
   * rebuild the sample correctly.
   */

  priv->tx.carry.bytes = carry_size;

  if (priv->tx.carry.bytes)
    {
      memcpy((uint8_t *)&priv->tx.carry.value, samp, priv->tx.carry.bytes);
    }

  /* Release our reference on the audio buffer. This may very likely
   * cause the audio buffer to be freed.
   */

  apb_free(bfcontainer->apb);

  /* Configure DMA stream */

  bytes_queued = esp32s3_dma_setup(outlink, I2S_DMADESC_NUM,
                                   bfcontainer->buf,
                                   bfcontainer->nbytes, I2S_TX,
                                   priv->dma_channel);

  if (bytes_queued != bfcontainer->nbytes)
    {
      i2serr("Failed to enqueue I2S buffer "
             "(%" PRIu32 " bytes of %" PRIu32 ")\n",
             bytes_queued, bfcontainer->nbytes);
      i2s_dmabuf_release(bfcontainer->buf);   /* [V211] return pool slot */
      bfcontainer->buf = NULL;
      return -bytes_queued;
    }

  flags = spin_lock_irqsave(&priv->slock);

  /* Add the buffer container to the end of the TX pending queue */

  sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.pend);

  /* Trigger DMA transfer if no transmission is in progress */

  /* [V362] attribute any bail from this call to site 1. */
  g_tx_callsite = 1;

  /* [V364] A buffer is about to be armed: re-enable the TX completion
   * interrupt sources that the idle bail may have gated.  This is the
   * only place a buffer enters priv->tx.pend, so it is the only place the
   * unmask can be needed -- and doing it here guarantees we can never
   * leave the channel permanently muted.
   */

  i2s_tx_irq_unmask(priv);

  ret = i2s_txdma_start(priv);

  spin_unlock_irqrestore(&priv->slock, flags);

  return ret;
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_rxdma_setup
 *
 * Description:
 *   Setup the next RX DMA transfer
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *   bfcontainer - The buffer container to be set up
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static int i2s_rxdma_setup(struct esp32s3_i2s_s *priv,
                           struct esp32s3_buffer_s *bfcontainer)
{
  int ret = OK;
  struct esp32s3_dmadesc_s *inlink;
  uint32_t bytes_queued;
  irqstate_t flags;

  DEBUGASSERT(bfcontainer && bfcontainer->apb);

  inlink = bfcontainer->dma_link;

  /* [V307][RXBOUNCE] The capture apb payload (apb->samp) comes from the
   * common heap, which on this board is PSRAM
   * (CONFIG_ESP32S3_SPIRAM_COMMON_HEAP=y).  The I2S GDMA IN channel never
   * completes a single descriptor when the target buffer is in PSRAM:
   * i2s_rxdma_start() arms fine ([RXDUMP] xfer ret=0) yet neither the I2S
   * nor the GDMA interrupt status ever moves (i2sintst=0 gdmainst=0
   * isr=0), the SUC_EOF interrupt never fires and the capture starves --
   * reported by audio_capture_native as "capture EIO".  Mirror the TX V211
   * strategy: DMA into an internal-SRAM bounce slot, then i2s_rx_worker()
   * copies the received bytes back into the apb and returns the slot. */

  
  bfcontainer->buf = NULL;
  uint8_t *rx_target = bfcontainer->apb->samp;
  if (esp32s3_ptr_extram(rx_target))
    {
      bfcontainer->buf = i2s_dmabuf_alloc(bfcontainer->nbytes);
      if (bfcontainer->buf == NULL)
        {
          i2serr("[RXBOUNCE] no internal-SRAM slot for %u bytes\n",
                 (unsigned)bfcontainer->nbytes);

          /* [V755] *** The bounce pool is shared with the TX path. ***
           *
           * A playback whose GDMA never raised its completion interrupt
           * leaves its containers parked in priv->tx.act for ever:
           * i2s_txdma_start() bails out while tx.act is non-empty, and
           * until v755 there was no TX-side counterpart of
           * i2s_rx_release_inflight() to reclaim them.  With
           * CONFIG_ES8311_INFLIGHT=2 two stuck playbacks occupy all
           * I2S_DMABUF_SLOTS=4 slots and every later capture fails with
           * -ENOMEM here, which the upper half surfaces as
           * "capture EIO" and an endless "listen: no speech for 8000
           * ms, restarting session" loop -- the UI frozen on
           * "Start listening".
           *
           * Reclaim the stalled TX buffers and try once more.  This is
           * self-healing on the exact path that used to die, and it
           * costs nothing on the healthy path (the pool is never full
           * there).  esp32s3_i2s_tx_flush() itself returns immediately
           * when all three TX queues are empty, so a live playback is
           * never torn down by this. */

          esp32s3_i2s_tx_flush();

          bfcontainer->buf = i2s_dmabuf_alloc(bfcontainer->nbytes);
          if (bfcontainer->buf == NULL)
            {
              return -ENOMEM;
            }
        }

      /* [V659] Rate-limited: this is one line per received buffer, so a
       * capture session alone produced ~190 of them and helped push the real
       * diagnostics out of the 32 KB kmsg ring. */

      {
        static unsigned s_v659_bounce;

        if (s_v659_bounce < 4u || (s_v659_bounce % 256u) == 0u)
          {
            syslog(LOG_INFO, "[RXBOUNCE] apb samp %p in PSRAM -> bounce %p\n",
                   (void *)bfcontainer->apb->samp, (void *)bfcontainer->buf);
          }

        s_v659_bounce++;
      }
      rx_target = bfcontainer->buf;
    }

  

  /* Configure DMA stream */

  bytes_queued = esp32s3_dma_setup(inlink, I2S_DMADESC_NUM,
                                   rx_target,
                                   bfcontainer->nbytes, I2S_RX,
                                   priv->dma_channel);

  

  

  if (bytes_queued != bfcontainer->nbytes)
    {
      i2serr("Failed to enqueue I2S buffer "
             "(%" PRIu32 " bytes of %" PRIu32 ")\n",
             bytes_queued, bfcontainer->nbytes);
      i2s_dmabuf_release(bfcontainer->buf);   /* [V307] no-op if NULL */
      bfcontainer->buf = NULL;
      return -bytes_queued;
    }

  flags = spin_lock_irqsave(&priv->slock);

  

  /* Add the buffer container to the end of the RX pending queue */

  sq_addlast((sq_entry_t *)bfcontainer, &priv->rx.pend);

  /* Trigger DMA transfer if no transmission is in progress */

  

  ret = i2s_rxdma_start(priv);

  

  spin_unlock_irqrestore(&priv->slock, flags);

  

  return ret;
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: i2s_tx_schedule
 *
 * Description:
 *   An TX DMA completion has occurred.  Schedule processing on
 *   the working thread.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *   outlink - DMA outlink descriptor that triggered the interrupt.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static void IRAM_ATTR i2s_tx_schedule(struct esp32s3_i2s_s *priv,
                                      struct esp32s3_dmadesc_s *outlink)
{
  struct esp32s3_buffer_s *bfcontainer;
  struct esp32s3_dmadesc_s *bfdesc;
  int ret;

  /* [V345] Record every entry and whether there was an active buffer. */

  g_sched_n++;

  if (sq_empty(&priv->tx.act))
    {
      g_sched_noact++;
    }

  /* Upon entry, the transfer(s) that just completed are the ones in the
   * priv->tx.act queue.
   */

  /* Move all entries from the tx.act queue to the tx.done queue */

  if (!sq_empty(&priv->tx.act))
    {
      /* Remove the next buffer container from the tx.act list */

      bfcontainer = (struct esp32s3_buffer_s *)sq_peek(&priv->tx.act);

      /* Check if the DMA descriptor that generated an EOF interrupt is the
       * last descriptor of the current buffer container's DMA outlink.
       * REVISIT: what to do if we miss syncronization and the descriptor
       * that generated the interrupt is different from the expected (the
       * oldest of the list containing active transmissions)?
       */

      /* Find the last descriptor of the current buffer container */

      bfdesc = bfcontainer->dma_link;
      while (!(bfdesc->ctrl & ESP32S3_DMA_CTRL_EOF))
        {
          DEBUGASSERT(bfdesc->next);
          bfdesc = bfdesc->next;
        }

      if (bfdesc == outlink)
        {
          g_sched_match++;   /* [V345] */

          sq_remfirst(&priv->tx.act);

          /* Report the result of the transfer */

          bfcontainer->result = OK;

          /* Add the completed buffer container to the tail of the tx.done
           * queue
           */

          sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.done);

          /* Check if the DMA is IDLE */

          if (sq_empty(&priv->tx.act))
            {
              /* Then start the next DMA. */

              /* [V362] attribute any bail from this call to site 2. */
              g_tx_callsite = 2;
              i2s_txdma_start(priv);
            }
        }

      /* If the worker has completed running, then reschedule the working
       * thread.
       */

      if (work_available(&priv->tx.work))
        {
          /* Schedule the TX DMA done processing to occur on the worker
           * thread.
           */

          ret = work_queue(HPWORK, &priv->tx.work, i2s_tx_worker, priv, 0);
          if (ret != 0)
            {
              i2serr("ERROR: Failed to queue TX work: %d\n", ret);
            }
        }
    }
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_rx_schedule
 *
 * Description:
 *   An RX DMA completion has occurred.  Schedule processing on
 *   the working thread.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *   inlink - DMA inlink descriptor that triggered the interrupt.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static void i2s_rx_schedule(struct esp32s3_i2s_s *priv,
                            struct esp32s3_dmadesc_s *inlink)
{
  struct esp32s3_buffer_s *bfcontainer;
  struct esp32s3_dmadesc_s *bfdesc;
  int ret;

  /* Upon entry, the transfer(s) that just completed are the ones in the
   * priv->rx.act queue.
   */

  /* Move all entries from the rx.act queue to the rx.done queue */

  if (!sq_empty(&priv->rx.act))
    {
      /* Remove the next buffer container from the rx.act list */

      bfcontainer = (struct esp32s3_buffer_s *)sq_peek(&priv->rx.act);

      /* Find the last descriptor of the current buffer container */

      bfdesc = bfcontainer->dma_link;

      while (bfdesc->next != NULL &&
             (bfdesc->next->ctrl & ESP32S3_DMA_CTRL_EOF))
        {
          bfdesc = bfdesc->next;
        }

      if (bfdesc == inlink)
        {
          sq_remfirst(&priv->rx.act);

          /* Report the result of the transfer */

          bfcontainer->result = OK;

          /* Add the completed buffer container to the tail of the rx.done
           * queue
           */

          sq_addlast((sq_entry_t *)bfcontainer, &priv->rx.done);

          /* Check if the DMA is IDLE */

          if (sq_empty(&priv->rx.act))
            {
              /* Then start the next DMA. */

              i2s_rxdma_start(priv);
            }
        }

      /* If the worker has completed running, then reschedule the working
       * thread.
       */

      if (work_available(&priv->rx.work))
        {
          /* Schedule the RX DMA done processing to occur on the worker
           * thread.
           */

          ret = work_queue(HPWORK, &priv->rx.work, i2s_rx_worker, priv, 0);
          if (ret != 0)
            {
              i2serr("ERROR: Failed to queue RX work: %d\n", ret);
            }
        }
    }
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: i2s_tx_worker
 *
 * Description:
 *   TX transfer done worker
 *
 * Input Parameters:
 *   arg - the I2S device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static void i2s_tx_worker(void *arg)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)arg;
  struct esp32s3_buffer_s *bfcontainer;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* When the transfer was started, the active buffer containers were removed
   * from the tx.pend queue and saved in the tx.act queue.  We get here when
   * the DMA is finished.
   *
   * In any case, the buffer containers in tx.act will be moved to the end
   * of the tx.done queue and tx.act will be emptied before this worker is
   * started.
   *
   */

  g_worker_n++;   /* [V345] */

  

  /* Process each buffer in the tx.done queue */

  while (sq_peek(&priv->tx.done) != NULL)
    {
      /* Remove the buffer container from the tx.done queue.  NOTE that
       * interrupts must be disabled to do this because the tx.done queue is
       * also modified from the interrupt level.
       */

      flags = spin_lock_irqsave(&priv->slock);
      bfcontainer = (struct esp32s3_buffer_s *)sq_remfirst(&priv->tx.done);
      spin_unlock_irqrestore(&priv->slock, flags);

      /* Perform the TX transfer done callback */

      g_worker_cb++;   /* [V345] */
      DEBUGASSERT(bfcontainer && bfcontainer->callback);
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, bfcontainer->result);

      /* Release the internal buffer used by the DMA outlink */

      i2s_dmabuf_release(bfcontainer->buf);   /* [V211] internal-SRAM pool */
      bfcontainer->buf = NULL;                /* [V307] avoid double free */

      /* And release the buffer container */

      i2s_buf_free(priv, bfcontainer);
    }
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_rx_worker
 *
 * Description:
 *   RX transfer done worker
 *
 * Input Parameters:
 *   arg - the I2S device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX

/* [V717] Configure generation.  Bumped once per i2s_configure() with the
 * receiver active.  Two consumers: the RX override block uses it to index
 * the profile table (one profile per always-on listening session), and the
 * [V709] forensic dump re-arms itself whenever it changes, so every profile
 * gets its own raw frame without any file or NSH involvement.
 *
 * Why: the v716 file hooks never fired.  NSH itself was proven healthy
 * (`echo hello123` and `touch /tmp/v709dump.txt` both echoed back, so the
 * files exist), yet `[V716] rxregs` and `[V709] raw` produced zero lines in
 * steady state while `[V716] rate`, from the same DMA callback, produced
 * dozens.  The RX block of i2s_configure() simply is not re-entered the way
 * the boot path is, so a hook that needs an operator to install it can end
 * up never applying -- which is exactly what the p1..p4 sweep showed: every
 * profile delivered the same ~33,900 B/s. */

static volatile unsigned g_v717_gen;

static void i2s_rx_worker(void *arg)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)arg;
  struct esp32s3_buffer_s *bfcontainer;
  struct esp32s3_dmadesc_s *dmadesc;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* [V691] Keep the frame clock on the pads for the whole capture, and take
   * a late pin snapshot once the stream has settled.  Every previous
   * [V660] pins line came from the capture-start probe, i.e. before the
   * engines were up, which is why they all read rxstart=0 and BCLK=0. */

  i2s_v691_clock_ensure(priv, 1);

  {
    static unsigned int s_v691_n;

    s_v691_n++;

    if (s_v691_n == 4 || s_v691_n == 24 || s_v691_n == 64)
      {
        esp32s3_i2s_rx_health();
      }
  }

  /* When the transfer was started, the active buffer containers were removed
   * from the rx.pend queue and saved in the rx.act queue. We get here when
   * the DMA is finished.
   *
   * In any case, the buffer containers in rx.act will be moved to the end
   * of the rx.done queue and rx.act will be emptied before this worker is
   * started.
   *
   */

  i2sinfo("rx.act.head=%p rx.done.head=%p\n",
          priv->rx.act.head, priv->rx.done.head);

  /* Process each buffer in the rx.done queue */

  while (sq_peek(&priv->rx.done) != NULL)
    {
      /* Remove the buffer container from the rx.done queue.  NOTE that
       * interrupts must be disabled to do this because the rx.done queue is
       * also modified from the interrupt level.
       */

      flags = spin_lock_irqsave(&priv->slock);
      bfcontainer = (struct esp32s3_buffer_s *)sq_remfirst(&priv->rx.done);
      spin_unlock_irqrestore(&priv->slock, flags);

      dmadesc = bfcontainer->dma_link;

      

      bfcontainer->apb->nbytes = 0;

      /* [V689] Sum SIZE (ctrl[11:0]), not DATALEN (ctrl[23:12]).
       *
       * The V688 one-shot dump captured a completed RX descriptor as
       *     d0 = 0x407feffc
       *   OWN    (bit31)   = 0            released by the DMA
       *   EOF    (bit30)   = 1            suc_eof landed on this one
       *   LENGTH ([23:12]) = 0x7fe = 2046
       *   SIZE   ([11:0])  = 0xffc = 4092 <- bytes the hardware received
       *
       * So the DMA had already stored the WHOLE 4092-byte buffer (the V688
       * single-descriptor / EOF-threshold change worked), but
       * ESP32S3_DMA_CTRL_DATALEN_S is 12, i.e. the old walk summed LENGTH
       * and reported 2046, after which i2s_rx_worker() memcpy'd only 2046
       * of the 4092 bytes: half of every capture buffer was discarded.
       *
       * The RX received-byte count lives in SIZE, i.e.
       * ESP32S3_DMA_CTRL_BUFLEN_S (0).  This walk is the only one, so TX
       * is untouched. */

      while (dmadesc != NULL && (dmadesc->ctrl & ESP32S3_DMA_CTRL_EOF))
        {
          /* [V690] DATA LENGTH, not the programmed buffer SIZE.
           *
           * arch/xtensa/src/esp32s3/esp32s3_dma.c:315 --
           *   "Only set Data Length if it's a TX buffer. Otherwise, it
           *    will be written automatically by hardware."
           * So on RX the hardware owns ctrl[23:12] (DATALEN) and software
           * owns ctrl[11:0] (BUFLEN/SIZE).  v689 summed SIZE, which is the
           * constant 4092 we programmed, so every completion looked full and
           * the worker memcpy'd 4092 bytes out of a bounce slot of which
           * only DATALEN were ever written by the DMA -- the app then read
           * stale bytes from the previous arm as if they were audio.
           * Measured with the v689 d0 dump: 0x407feffc -> SIZE 4092,
           * LENGTH 2046. */

          bfcontainer->apb->nbytes +=
              (dmadesc->ctrl >> ESP32S3_DMA_CTRL_DATALEN_S) &
              ESP32S3_DMA_CTRL_DATALEN_V;
          dmadesc = dmadesc->next;
        }

      /* [V688] Descriptor forensics, first 8 RX buffers only.
       *
       * v687 reported "chunk#1: 2044 bytes" from a threshold of 2044,
       * whose "counts bits" reading would be 2 * (2044 + 1) = 4090 bytes.
       * The walk therefore yields exactly the threshold value, which fits
       * "I2S_RX_EOF_NUM counts BYTES" (trigger 2045) but not "counts
       * bits".  Reading the raw control words back settles it:
       *   ctrl[11:0]  SIZE   (RX: written by hardware = bytes received)
       *   ctrl[23:12] LENGTH
       *   ctrl[30]    EOF (suc_eof)
       *   ctrl[31]    OWN
       * A second line with the re-armed threshold follows on the next
       * arm, so one press is enough.
       */

      {
        static unsigned s_v688_dump;

        if (s_v688_dump < 8u)
          {
#if I2S_DMADESC_NUM >= 2
            uint32_t d1 = (uint32_t)bfcontainer->dma_link[1].ctrl;
#else
            uint32_t d1 = 0u;
#endif
            s_v688_dump++;

            syslog(LOG_INFO,
                   "[V688] nb=%u walk=%u d0=0x%08x d1=0x%08x\n",
                   (unsigned)bfcontainer->nbytes,
                   (unsigned)bfcontainer->apb->nbytes,
                   (unsigned)bfcontainer->dma_link[0].ctrl,
                   (unsigned)d1);
          }
      }

      /* [V307][RXBOUNCE] DMA landed in the internal-SRAM bounce slot;
       * deliver the received bytes to the real (PSRAM) apb payload and
       * return the slot to the pool. */

      if (bfcontainer->buf != NULL)
        {
          uint32_t ncopy = bfcontainer->apb->nbytes;

          if (ncopy > bfcontainer->apb->nmaxbytes)
            {
              ncopy = bfcontainer->apb->nmaxbytes;
            }

          
          /* [V707] De-interleave the two-slot RX frame.
           *
           * The RX engine runs with TOT_CHAN_NUM = 1, i.e. a TWO-slot
           * frame, because that is what keeps BCLK at 32 x fs for the
           * ES8311.  ES8311 is mono, so only one slot carries audio and
           * the other is all zeros.  The v706 field capture shows 79 zero
           * samples for every 7 real ones, with a live sequence of
           *
           *   fffa 7fff 0000 fbaa 0000 fd8a 0000 006c 0000 01a4 0000 0306
           *        ^samp      ^0        ^0        ^0        ^0
           *
           * Handed to a 16 kHz consumer that sounds half speed and
           * hollow, which no ASR can read.
           *
           * DATALEN is 2046, not a multiple of 4, so which slot is live
           * flips parity from buffer to buffer -- measure the energy of
           * each parity and keep the louder half rather than assuming
           * one.  Zero padding has no energy, so this is unambiguous.
           *
           * Note the memcpy that used to follow is gone: the loop below
           * writes the compacted samples straight into apb->samp, and
           * re-copying from buf would undo the work.
           */
          {
            const uint8_t *v707s = (const uint8_t *)bfcontainer->buf;
            uint32_t v707n = ncopy / 2u;
            uint32_t v707i;
            uint32_t v707w = 0u;
            uint32_t v707e0 = 0u;
            uint32_t v707e1 = 0u;
            int      v707deint;

            /* [V709] Runtime de-interleave switch.
             *
             * Default is the v707 behaviour (OFF is not the default), so
             * this build is a drop-in for v707.  /tmp/deint.txt whose
             * first character is '0' turns the de-interleave off for the
             * next delivered buffer; the loop below then walks every word
             * instead of every other one, which is byte-for-byte the
             * pre-v707 memcpy(result, buf, ncopy).  Any other content, or
             * no file at all, leaves it on.  Re-read every 32 buffers, so
             * an A/B test needs no reflash.
             *
             * Why this matters: v707 pinned the MIC PGA at step 0 -- the
             * [V660] reg10 dump reads back "1f 7f 00 10 1a 40 04 bf",
             * i.e. 0x16 = 0x04 -- yet the samples are still the full-scale
             * square wave that the v706 "16 04" experiment had cured.
             * The v706 logs disagree in exactly one way:
             *
             *   _v706_slotL  fffa 7fff | 0000 fbaa | 0000 fd8a ...
             *                even words zero, odd words audio -> clean
             *   _v706_slotb  fffa 0000 | 6000 7fff | 8000 8000 ...
             *                both words full scale            -> square
             *   v707         output 8000 0000 | 7fff 8000 | 7fff 6000
             *                the slotb shape
             *
             * _v706_slotb ran rxslot = 0x03; v707 prints rxslot = 0x01
             * and still yields the slotb shape, so either that print shows
             * a software variable rather than the register, or something
             * rewrites I2S_RX_TDM_CTRL after i2s_rxdma_start() programmed
             * it.  The B3 dump reads the register at delivery time.
             */
            {
              static unsigned v709tick;

              /* [V717] Default is now OFF.  Measured on v716: with
               * /tmp/deint.txt = 0 the application received 417384 bytes
               * per session instead of 197440 -- 2.11x -- and each chunk
               * went from 1022 to 2046 bytes.  The receiver hands over
               * about one 16-bit word per audio frame (33,900 B/s raw =
               * 16,950 words/s, tdm=0x00010001 enables only CHAN0), so the
               * stream is already mono; taking every other word is plain
               * 2:1 decimation of ~17 kHz down to 8.5 kHz.  e0=8478720
               * against e1=8192000 confirms both parities carry the same
               * energy, i.e. it is one continuous stream and not the
               * "one slot audio, one slot zero" case the parity picker was
               * written for.
               *
               * This fixes the rate only.  The content is still not audio
               * (peak stays 6 with the de-interleave off), so the alignment
               * question is still open -- do not read one as the other. */

              static int      v709on = 0;

              if ((v709tick++ & 31u) == 0u)
                {
                  FILE *v709f = fopen("/tmp/deint.txt", "r");

                  if (v709f != NULL)
                    {
                      v709on = (fgetc(v709f) == '0') ? 0 : 1;
                      fclose(v709f);
                    }

                  /* [V718] No file: KEEP the current value.  The old
                   * `else { v709on = 1; }` re-asserted 1 every 32 buffers,
                   * which is why v717's default-0 initialiser had no effect
                   * and v717b still logged deint=1 in every dump.
                   *
                   * OFF is right here.  One 16-bit word arrives per frame
                   * (33,900 B/s raw over 16.86 buffers/s = 17.25 k words/s,
                   * and tdm=0x00010001 enables CHAN0 only), so the stream is
                   * already mono at ~16 kHz; stepping by two is a plain 2:1
                   * decimation to 8 kHz.  e0 = 8478720 against e1 = 8192000
                   * says both parities carry the same energy, i.e. one
                   * continuous stream, not the one-slot-audio case the
                   * parity picker was written for. */
                }

              v707deint = v709on;
            }

            for (v707i = 0u; v707i < v707n; v707i++)
              {
                uint16_t v707u = (uint16_t)((uint16_t)v707s[v707i * 2u] |
                                  ((uint16_t)v707s[v707i * 2u + 1u] << 8));
                uint16_t v707a = (v707u & 0x8000u) ?
                                 (uint16_t)(~v707u + 1u) : v707u;

                if (v707i & 1u) { v707e1 += v707a; }
                else            { v707e0 += v707a; }
              }

            /* [V709] deint on  -> start on the louder parity, step 2.
             *        deint off -> start at 0, step 1, i.e. every word.
             */

            for (v707i = v707deint ? ((v707e1 >= v707e0) ? 1u : 0u) : 0u;
                 v707i < v707n;
                 v707i += v707deint ? 2u : 1u)
              {
                memcpy((uint8_t *)bfcontainer->apb->samp + v707w,
                       v707s + v707i * 2u, 2u);
                v707w += 2u;
              }

            ncopy = v707w;

            /* [V709] Forensics for the first three buffers after boot: the
             * raw frame as the DMA delivered it, the frame handed to the
             * upper half, the parity energies and the live TDM/clock
             * registers.  Reading the register here -- not in the health
             * helper -- is what tells whether arm-time programming
             * survives to data time.
             */

            {
              static unsigned      v709dump;
              static int           v709arm;
              static unsigned      v709gen;
              static unsigned long v716bufs;
              static unsigned long v716bytes;
              uint32_t v716or = 0u;
              uint16_t v716pk = 0u;
              uint16_t v716mn = 0xffffu;
              uint32_t v716k;

              /* [V716] Re-armable: the one-shot dump used to stop after the
               * first three buffers of boot, which is useless for a sweep.
               * Touching /tmp/v709dump.txt arms it for the next 24. */

              {
                FILE *v716a = fopen("/tmp/v709dump.txt", "r");

                if (v716a != NULL)
                  {
                    fclose(v716a);
                    v709dump = 0u;
                    v709arm  = 1;
                  }

                /* [V717] Also re-arm on every configure generation, so each
                 * profile in the sweep table gets its own raw frame without
                 * anything having to be installed by hand. */

                if (v709gen != g_v717_gen)
                  {
                    v709gen  = g_v717_gen;
                    v709dump = 0u;
                    v709arm  = 1;
                  }
              }

              v716bufs++;
              v716bytes += (unsigned long)v707n * 2u;

              for (v716k = 0u; v716k < v707n; v716k++)
                {
                  uint16_t v716u = (uint16_t)((uint16_t)v707s[v716k * 2u] |
                                    ((uint16_t)v707s[v716k * 2u + 1u] << 8));

                  v716or |= (uint32_t)v716u;

                  if (v716u > v716pk) { v716pk = v716u; }
                  if (v716u < v716mn) { v716mn = v716u; }
                }

              /* [V724-A] Effective bit depth of THIS buffer, from the OR
               * [V716] already computed: count the low zero bits of
               * v716or and the residue is how many bits of the sample
               * actually carry anything.  Real audio sits at 2..3; the
               * post-playback break shows 12..15.
               *
               * Printing only on a class change keeps this off the kmsg
               * budget, and the class change is also when V724-B takes
               * its register snapshot -- so the good->junk boundary
               * produces a before/after pair instead of a guess.
               */

              {
                static int s_v724_cls = -2;
                int        v724b;
                int        v724cls;
                uint32_t   v724o = v716or;

                if (v724o == 0u)
                  {
                    v724b = -1;                     /* dead silent */
                  }
                else
                  {
                    v724b = 0;
                    while ((v724o & 1u) == 0u)
                      {
                        v724o >>= 1;
                        v724b++;
                      }
                  }

                v724cls = (v724b < 0) ? -1 : ((v724b >= 8) ? 1 : 0);

                if (v724cls != s_v724_cls)
                  {
                    syslog(LOG_INFO,
                           "[V724] bits %d -> %d (lowzero=%d) bufs=%lu "
                           "or=0x%08x\n",
                           s_v724_cls, v724cls, v724b, v716bufs,
                           (unsigned)v716or);
                    s_v724_cls = v724cls;
                    i2s_v724_diff(priv->config->port, "BIT");
                  }

                if ((v716bufs % 256u) == 0u)
                  {
                    i2s_v724_diff(priv->config->port, "PER");
                  }

                /* [V724-T1] DIAGNOSTIC: 0 disables the file-polling pair.
                 * The hpwork panic at t=24.5 s (pc=421f9382, getumask,
                 * vaddr=0x1a, a2=2 read out of a8=0x3fc92000 which sits
                 * 0x50 BELOW the hpwork stack base 0x3fc92050) is 100%
                 * reproducible, so this isolates whether the two fopen()
                 * calls -- and the 256-byte buffer they sit on -- are what
                 * tips the work queue over.  Restore to 1 once it is known.
                 */

                if (0 && (v716bufs % 8u) == 0u)
                  {
                    i2s_v724_hook(priv->config->port);
                    i2s_v724_pins();
                  }
              }

              /* One line per 32 buffers, ~1.9 s -- never per buffer.  or is
               * the OR of every raw word in the window: while its low
               * twelve bits stay zero, "only the top nibble carries
               * information" is a measurement rather than an impression,
               * and the moment they light up under one of the rxcfg
               * profiles, that profile is the answer. */

              if ((v716bufs & 31u) == 0u)
                {
                  syslog(LOG_INFO,
                         "[V716] rate bufs=%lu rawbytes=%lu or=0x%08x "
                         "pk=%u mn=%u\n",
                         v716bufs, v716bytes, (unsigned)v716or,
                         (unsigned)v716pk, (unsigned)v716mn);
                }

              /* [V733] *** Self-healing bit alignment. ***
               *
               * The whole-buffer OR that [V716] already computes is the
               * measurement: real audio lights the low bits of the OR
               * (healthy boot: or=0x0000ffff), while a stream the receiver
               * latched k bits early leaves k low zeros (or=0x0000fffc ->
               * k=2, "0000 7200 0200 5200" -> k=9).  Every sample is then
               * 2^k times its true value, which is what turns a quiet room
               * into adev 16465 / peak 32768 and keeps the VAD open.
               *
               * Shift it back.  Only ever to the right, so the worst case
               * is a loud stream reduced to what it really was.
               *
               * The amplitude gate matters: a genuinely quiet buffer has
               * empty low bits too, and dividing it by 4 would be wrong.
               * Such a buffer needs no correction anyway. */

              if (((unsigned)v716pk - (unsigned)v716mn) > 20000u)
                {
                  uint32_t v733o = v716or;
                  int      v733lz;

                  if (v733o == 0u)
                    {
                      v733lz = -1;
                    }
                  else
                    {
                      v733lz = 0;

                      while ((v733o & 1u) == 0u)
                        {
                          v733o >>= 1;
                          v733lz++;
                        }
                    }

                  if (v733lz >= 2 && v733lz <= 10)
                    {
                      if (g_v733_shift != v733lz)
                        {
                          syslog(LOG_INFO,
                                 "[V733] rxalign shift %d -> %d "
                                 "(or=0x%08x pk=%u mn=%u)\n",
                                 g_v733_shift, v733lz, (unsigned)v716or,
                                 (unsigned)v716pk, (unsigned)v716mn);
                        }

                      g_v733_shift = v733lz;
                    }
                  else if (v733lz >= 0 && v733lz <= 1 && g_v733_shift != 0)
                    {
                      syslog(LOG_INFO,
                             "[V733] rxalign aligned again (or=0x%08x)\n",
                             (unsigned)v716or);
                      g_v733_shift = 0;
                    }
                }

              if (g_v733_shift > 0)
                {
                  int16_t *v733p = (int16_t *)bfcontainer->apb->samp;
                  uint32_t v733c = ncopy / 2u;
                  uint32_t v733q;

                  for (v733q = 0u; v733q < v733c; v733q++)
                    {
                      v733p[v733q] =
                        (int16_t)(((int32_t)v733p[v733q]) >> g_v733_shift);
                    }
                }

              if (v709dump < (v709arm ? 24u : 3u))
                {
                  static const char v709h[] = "0123456789abcdef";
                  char     v709r[16 * 5 + 1];
                  char     v709d[16 * 5 + 1];
                  unsigned v709k;
                  uint16_t v709u;

                  v709dump++;

                  for (v709k = 0u; v709k < 16u && v709k < v707n; v709k++)
                    {
                      v709u = (uint16_t)((uint16_t)v707s[v709k * 2u] |
                               ((uint16_t)v707s[v709k * 2u + 1u] << 8));

                      v709r[v709k * 5u + 0u] = v709h[(v709u >> 12) & 0xfu];
                      v709r[v709k * 5u + 1u] = v709h[(v709u >> 8) & 0xfu];
                      v709r[v709k * 5u + 2u] = v709h[(v709u >> 4) & 0xfu];
                      v709r[v709k * 5u + 3u] = v709h[v709u & 0xfu];
                      v709r[v709k * 5u + 4u] = ' ';
                    }

                  v709r[v709k * 5u] = '\0';

                  for (v709k = 0u;
                       v709k < 16u && (v709k * 2u + 1u) < v707w;
                       v709k++)
                    {
                      v709u = (uint16_t)(
                                (uint16_t)bfcontainer->apb->samp[v709k * 2u] |
                                ((uint16_t)bfcontainer->apb->samp
                                   [v709k * 2u + 1u] << 8));

                      v709d[v709k * 5u + 0u] = v709h[(v709u >> 12) & 0xfu];
                      v709d[v709k * 5u + 1u] = v709h[(v709u >> 8) & 0xfu];
                      v709d[v709k * 5u + 2u] = v709h[(v709u >> 4) & 0xfu];
                      v709d[v709k * 5u + 3u] = v709h[v709u & 0xfu];
                      v709d[v709k * 5u + 4u] = ' ';
                    }

                  v709d[v709k * 5u] = '\0';

                  syslog(LOG_INFO,
                         "[V709] #%u words=%u e0=%u e1=%u keep=%u deint=%d\n",
                         v709dump, (unsigned)v707n, (unsigned)v707e0,
                         (unsigned)v707e1,
                         (unsigned)((v707e1 >= v707e0) ? 1u : 0u), v707deint);
                  syslog(LOG_INFO, "[V709] raw : %s\n", v709r);
                  syslog(LOG_INFO, "[V709] post: %u bytes: %s\n",
                         (unsigned)v707w, v709d);
                  syslog(LOG_INFO,
                         "[V709] tdm=0x%08x conf1=0x%08x rxconf=0x%08x\n",
                         (unsigned)getreg32(I2S_RX_TDM_CTRL_REG(
                                     priv->config->port)),
                         (unsigned)getreg32(I2S_RX_CONF1_REG(
                                     priv->config->port)),
                         (unsigned)getreg32(I2S_RX_CONF_REG(
                                     priv->config->port)));
                }
            }
          }

          bfcontainer->apb->nbytes = ncopy;
          i2s_dmabuf_release(bfcontainer->buf);
          bfcontainer->buf = NULL;
        }

      /* Perform the RX transfer done callback */

      DEBUGASSERT(bfcontainer && bfcontainer->callback);

      /* [V698] *** ROOT CAUSE OF "the second PTT hangs on listening" ***
       *
       * This used to be:
       *
       *   if (priv->streaming == false)
       *     {
       *       bfcontainer->apb->flags |= AUDIO_APB_FINAL;
       *     }
       *
       * priv->streaming is written only by i2s_ioctl(AUDIOIOC_START /
       * AUDIOIOC_STOP) below, and the only caller of I2S_IOCTL() in this tree
       * is audio_i2s.c -- the generic audio<->I2S lower half.  Our lower half
       * is es8311, which drives this port through I2S_RECEIVE()/I2S_SEND()
       * and never issues AUDIOIOC_START.  priv->streaming was therefore
       * permanently false and EVERY captured buffer was flagged
       * AUDIO_APB_FINAL.
       *
       * es8311_returnbuffers() reads that flag as "end of stream", latches
       * priv->terminating and lets the es8311 worker exit as soon as inflight
       * reaches zero, so the capture stream died after the first buffer while
       * the UI still said "listening".  The messy teardown freed apbs the
       * driver still owned; the double apb_free() pushed one heap block onto
       * the free list twice, and the next mm_malloc() walked into it
       * (EXCCAUSE=001d, vaddr=0x8, task xiaozhi_ui).
       *
       * The lower half has no business ending the stream.  The upper half
       * learns that from AUDIOIOC_STOP: es8311_stop() sends AUDIO_MSG_STOP,
       * which latches priv->terminating for real.
       */

      if ((bfcontainer->apb->flags & AUDIO_APB_FINAL) != 0)
        {
          syslog(LOG_ERR,
                 "[V698] *** stray FINAL on live RX apb=%p\n",
                 bfcontainer->apb);
        }

      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, bfcontainer->result);

      /* Release our reference on the audio buffer. This may very likely
       * cause the audio buffer to be freed.
       */

      apb_free(bfcontainer->apb);

      /* And release the buffer container */

      i2s_buf_free(priv, bfcontainer);
    }
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: i2s_configure
 *
 * Description:
 *   Configure I2S
 *
 * Input Parameters:
 *   priv - Partially initialized I2S device structure.  This function
 *          will complete the I2S specific portions of the initialization
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

/* [V93] Time-division direction override.
 *
 * See the file header: with both engines enabled the receiver is a slave to
 * the transmitter's clock, and the transmitter is silent during capture, so
 * capture never sees a bit.  While a capture is in flight we make
 * i2s_configure() believe tx_en = 0, which selects the "rx + master" branch.
 */

static int g_i2s_rxonly_port = -1;

/* [V380] Diagnostic direction overrides.  g_v380_loopback is switched by
 * the capture layer between phase 1 (internal loopback) and phase 2
 * (TX clocks the real codec); g_v380_lb_applied remembers the value that
 * i2s_configure() has already written to the hardware so flipping the
 * flag actually takes effect on the next call.
 */

volatile int g_v380_loopback = 0;

volatile int g_v380_txmaster = 1;
static int   g_v380_lb_applied = -1;

static bool i2s_tx_active(FAR struct esp32s3_i2s_s *priv)
{
  /* [V380] Both phases need the transmitter to own the port: the
   * receiver stays a slave to the TX BCK/WS generator and the pins keep
   * carrying the TX clock output signals.  The rxonly override is what
   * used to flip this back and forth between the two es8311 workers.
   */

  if (g_v380_txmaster || g_v380_loopback)
    {
      return true;
    }

  if (g_i2s_rxonly_port >= 0 &&
      (int)priv->config->port == g_i2s_rxonly_port)
    {
      return false;
    }

  return priv->config->tx_en;
}

static bool i2s_rx_active(FAR struct esp32s3_i2s_s *priv)
{
  return priv->config->rx_en;
}



static void i2s_configure(struct esp32s3_i2s_s *priv)
{
  uint32_t tx_conf  = 0;
  uint32_t tx_conf1 = 0;
  uint32_t rx_conf  = 0;
  uint32_t rx_conf1 = 0;

  /* Set peripheral clock and clear reset */

  if (priv->config->port == ESP32S3_I2S0)
    {
      modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, SYSTEM_I2S0_CLK_EN);
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, 0, SYSTEM_I2S0_RST);
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_I2S0_RST, 0);
    }
  else
    {
      modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, SYSTEM_I2S1_CLK_EN);
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, 0, SYSTEM_I2S1_RST);
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_I2S1_RST, 0);
    }

  /* [V340] GDMA 模块初始化：这个调用原先根本不存在。
   *
   * esp32s3_dma_init() 使能 GDMA 外设时钟、解除其复位，
   * 并给 GDMA 的『描述符 RAM』上电 + 打开时钟门控 (DMA_PD_CONF)。
   * 不做这三件事，outlink FSM 一起来就停车：START 写了被清、PARK=1、
   * OWNER 位永不被取走、中断永不来 —— 也就是我们喇叭的症状。
   * 该函数在全树里只有定义、没有任何调用者（V226 写了修复却没接线）。 */

  esp32s3_dma_init();

  if (!(getreg32(I2S_TX_CLKM_CONF_REG(priv->config->port)) & I2S_CLK_EN))
    {
      i2sinfo("Enabling I2S clock...\n");
      modifyreg32(I2S_TX_CLKM_CONF_REG(priv->config->port), 0, I2S_CLK_EN);
    }

  /* Configure multiplexed pins as connected on the board */

  /* TODO: check for loopback mode */

  /* Enable TX channel */

  if (priv->config->dout_pin != I2S_GPIO_UNUSED)
    {
      /* If TX channel is used, enable the clock source */

      if (!(getreg32(I2S_TX_CLKM_CONF_REG(priv->config->port)) &
            I2S_TX_CLK_ACTIVE))
        {
          i2sinfo("Activating I2S TX clock...\n");

          /* [V341] The source clock is selected in exactly ONE place now:
           * i2s_clk_src_select(), driven by i2s_clock_source_bringup() at the
           * end of this function.  Only the clock gate is opened here. */

          modifyreg32(I2S_TX_CLKM_CONF_REG(priv->config->port), 0,
                      I2S_TX_CLK_ACTIVE);
        }

      esp32s3_gpiowrite(priv->config->dout_pin, 1);
      esp32s3_configgpio(priv->config->dout_pin, OUTPUT_FUNCTION_2);
      esp32s3_gpio_matrix_out(priv->config->dout_pin,
                              priv->config->dout_outsig, 0, 0);
    }

  /* Enable RX channel */

  if (priv->config->din_pin != I2S_GPIO_UNUSED)
    {
      /* If RX channel is used, enable the clock source */

      if (!(getreg32(I2S_RX_CLKM_CONF_REG(priv->config->port)) &
            I2S_RX_CLK_ACTIVE))
        {
          i2sinfo("Activating I2S RX clock...\n");

          /* [V341] Source selected centrally (see i2s_clk_src_select()). */

          modifyreg32(I2S_RX_CLKM_CONF_REG(priv->config->port), 0,
                      I2S_RX_CLK_ACTIVE);
        }

      esp32s3_configgpio(priv->config->din_pin, INPUT_FUNCTION_2);
      esp32s3_gpio_matrix_in(priv->config->din_pin,
                             priv->config->din_insig, 0);
    }

  if (priv->config->role == I2S_ROLE_SLAVE)
    {
      if (i2s_tx_active(priv) && !i2s_rx_active(priv))
        {
          /* For "tx + slave" mode, select TX signal index for ws and bck */

          esp32s3_gpiowrite(priv->config->ws_pin, 1);
          esp32s3_configgpio(priv->config->ws_pin, INPUT_FUNCTION_2);
          esp32s3_gpio_matrix_in(priv->config->ws_pin,
                                 priv->config->ws_out_insig, 0);

          esp32s3_gpiowrite(priv->config->bclk_pin, 1);
          esp32s3_configgpio(priv->config->bclk_pin, INPUT_FUNCTION_2);
          esp32s3_gpio_matrix_in(priv->config->bclk_pin,
                                 priv->config->bclk_out_insig, 0);
        }
      else
        {
          /* For "tx + rx + slave" or "rx + slave" mode, select RX signal
           * index for ws and bck.
           */

          esp32s3_gpiowrite(priv->config->ws_pin, 1);
          esp32s3_configgpio(priv->config->ws_pin, INPUT_FUNCTION_2);
          esp32s3_gpio_matrix_in(priv->config->ws_pin,
                                 priv->config->ws_in_insig, 0);

          esp32s3_gpiowrite(priv->config->bclk_pin, 1);
          esp32s3_configgpio(priv->config->bclk_pin, INPUT_FUNCTION_2);
          esp32s3_gpio_matrix_in(priv->config->bclk_pin,
                                 priv->config->bclk_in_insig, 0);
        }
    }
  else
    {
      /* Considering master role for the I2S port */

      /* Set MCLK pin */

      if (priv->config->mclk_pin != I2S_GPIO_UNUSED)
        {
          i2sinfo("Configuring GPIO%" PRIu8 " to output master clock\n",
                  priv->config->mclk_pin);

          esp32s3_gpiowrite(priv->config->mclk_pin, 1);
          esp32s3_configgpio(priv->config->mclk_pin, OUTPUT_FUNCTION_2);
          esp32s3_gpio_matrix_out(priv->config->mclk_pin,
                                  priv->config->mclk_out_sig, 0, 0);
        }

      if (i2s_rx_active(priv) && !i2s_tx_active(priv))
        {
          /* For "rx + master" mode, select RX signal index for ws and bck */

          esp32s3_gpiowrite(priv->config->ws_pin, 1);
          esp32s3_configgpio(priv->config->ws_pin, OUTPUT_FUNCTION_2);
          esp32s3_gpio_matrix_out(priv->config->ws_pin,
                                  priv->config->ws_in_outsig, 0, 0);

          esp32s3_gpiowrite(priv->config->bclk_pin, 1);
          esp32s3_configgpio(priv->config->bclk_pin, OUTPUT_FUNCTION_2);
          esp32s3_gpio_matrix_out(priv->config->bclk_pin,
                                  priv->config->bclk_in_outsig, 0, 0);
        }
      else
        {
          /* For "tx + rx + master" or "tx + master" mode, select TX signal
           * index for ws and bck.
           */

          esp32s3_gpiowrite(priv->config->ws_pin, 1);
          esp32s3_configgpio(priv->config->ws_pin, OUTPUT_FUNCTION_2);
          esp32s3_gpio_matrix_out(priv->config->ws_pin,
                                  priv->config->ws_out_outsig, 0, 0);

          esp32s3_gpiowrite(priv->config->bclk_pin, 1);
          esp32s3_configgpio(priv->config->bclk_pin, OUTPUT_FUNCTION_2);
          esp32s3_gpio_matrix_out(priv->config->bclk_pin,
                                  priv->config->bclk_out_outsig, 0, 0);
        }
    }

  /* [V386] Feed the pads back into the receiver's clock inputs.
   *
   * Both branches above leave BCK/WS configured as outputs only.  When
   * the receiver is a slave (i2s_configure() sets I2S_RX_SLAVE_MOD
   * whenever the transmitter owns the port) it expects its clock on the
   * I2S0I_BCK_IN / I2S0I_WS_IN peripheral inputs, and those
   * gpio_matrix_in() connections are only made in the SLAVE branch of
   * this function.  In this master configuration nothing is attached to
   * them, so the receiver never sees a bit clock: it shifts nothing,
   * never reaches I2S_RXEOF_NUM, never raises a DMA request, and the
   * capture starves (v384/v385: i2sintst=0 gdmainst=0 isr=0 eof=0 with
   * the pads measured toggling).  A pad can drive an output and feed a
   * peripheral input at the same time, so this is free.
   */

  if (priv->config->rx_en)
    {
      esp32s3_gpio_matrix_in(priv->config->ws_pin,
                             priv->config->ws_in_insig, 0);
      esp32s3_gpio_matrix_in(priv->config->bclk_pin,
                             priv->config->bclk_in_insig, 0);

      /* [V394] IRQ-stack: silenced */ if (0) syslog(LOG_INFO,
             "[V386] rx clock input routed: ws(gpio%d)<-sig%u "
             "bclk(gpio%d)<-sig%u\n",
             (int)priv->config->ws_pin,
             (unsigned)priv->config->ws_in_insig,
             (int)priv->config->bclk_pin,
             (unsigned)priv->config->bclk_in_insig);
    }

  /* Share BCLK and WS if in full-duplex mode */

  /* [V250] Keep the TX on its OWN BCK/WS -- never on the RX's.
   *
   * NuttX sets I2S_SIG_LOOPBACK whenever both directions are active, i.e. in
   * exactly the full-duplex configuration this board runs (ES8311 with mic
   * and speaker).  With that bit set the TX module takes its BCK/WS from the
   * RX side instead of its own generator, and the RX frame generator only runs
   * once the RX channel is actually started.  A TX without a shift clock never
   * drains its FIFO, and a TX that never drains its FIFO never asserts the
   * peripheral DMA request -- the GDMA outlink therefore reports PARK=0 (its
   * FSM has been kicked) yet OWNER in the first descriptor stays set forever
   * and no SUC_EOF interrupt is ever raised, which is precisely what the
   * register dump shows:
   *
   *   LINK=0x000b9db8 (START=0 PARK=0) OWNER=1 INTST=0x00000000 txirqN=0
   *
   * The BCLK/WS pins on this board are routed to the TX generator outputs
   * (bclk_out_outsig / ws_out_outsig), so the TX's own divider is the one the
   * codec actually sees.  The reference firmware that does produce sound on
   * this board (ESP-IDF i2s_std + esp_codec_dev) never sets this bit either.
   * Leave it cleared and let the TX clock itself. */

  if (g_v380_loopback)
    {
      modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_SIG_LOOPBACK);
    }
  else
    {
      modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_SIG_LOOPBACK, 0);
    }

  /* [V394] IRQ-stack: silenced */ if (0) syslog(LOG_INFO, "[RAW] sig_loopback %s (tx_active=%d rx_active=%d)\n",
         g_v380_loopback ? "SET" : "cleared",
         (int)i2s_tx_active(priv), (int)i2s_rx_active(priv));

  /* Configure the TX module */

  if (i2s_tx_active(priv))
    {
      /* Reset I2S TX module */

      modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_TX_RESET);
      modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_RESET, 0);

      g_v671_cfg_valid = 0;   /* [V671] TX module reset */

      /* Enable/disable I2S TX slave mode */

      if (priv->config->role == I2S_ROLE_SLAVE)
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      0, I2S_TX_SLAVE_MOD);
        }
      else
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_SLAVE_MOD, 0);
        }

      /* Configure TX chan bit, audio data bit and mono mode.
       * On ESP32-S3, sample_bit should equals to data_bit.
       */

      /* Set TX data width */

      priv->data_width = priv->config->data_width;
      i2s_set_datawidth(priv);

      /* Set channel mode control */

      if (priv->config->audio_std_mode <= I2S_TDM_PCM)
        {
          /* [V228] TDM mode must stay ENABLED.
           *
           * Clearing tx_tdm_en (to match ESP-IDF's i2s_std, which never sets
           * it) was tried and measured: WS(8) stopped toggling entirely
           * (hi=2000/2000, stuck high, versus 1002/2000 with TDM enabled),
           * i.e. this driver only generates a frame clock in TDM mode.
           * The setting is load-bearing for the way this driver programs
           * tx_conf1 / tx_tdm_ctrl, so it is left alone. */
          modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_TDM_EN_M,
                      I2S_TX_TDM_EN);
          modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_PDM_EN, 0);

          /* [V229] *** THE MISSING PIECE ***
           *
           * ESP-IDF's i2s_ll_tx_enable_tdm() -- which is exactly what
           * i2s_ll_tx_enable_std() calls for standard I2S -- is:
           *     tx_conf.tx_pdm_en        = false;
           *     tx_conf.tx_tdm_en        = true;
           *     tx_pcm2pdm_conf.pcm2pdm_conv_en = false;   <-- this one
           * This driver set the first two but NEVER wrote
           * I2S_TX_PCM2PDM_CONF_REG at all (verified: zero writes in the whole
           * file).  With the PCM->PDM converter enabled the TX data path is
           * routed into the PDM filter instead of onto the DAC/DMA path, which
           * matches every measurement: the bit clock and WS keep running, the
           * configuration registers all read back correct, and yet the I2S
           * never requests data from the GDMA so the outlink parks without
           * fetching a single descriptor.
           *
           * Standard I2S uses none of the PDM/PCM2PDM block, so clear the
           * whole register -- this mirrors ESP-IDF setting conv_en = false. */
          putreg32(0, I2S_TX_PCM2PDM_CONF_REG(priv->config->port));

          if (priv->config->audio_std_mode == I2S_TDM_PHILIPS)
            {
              modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
                          I2S_TX_MSB_SHIFT_M, I2S_TX_MSB_SHIFT);

          /* [V236] Match ESP-IDF: I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG sets
           * left_align = true and i2s_hal_std_set_tx_slot() programs
           * i2s_ll_tx_enable_left_align(); this driver never touched
           * tx_left_align. */
          modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0,
                      I2S_TX_LEFT_ALIGN);
            }
          else
            {
              modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
                          I2S_TX_MSB_SHIFT, 0);
            }
        }
      else
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_PDM_EN_M,
                      I2S_TX_PDM_EN);
        }

      /* Set the default level of WS */

      if (priv->config->ws_pol == 1)
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0,
                      I2S_TX_WS_IDLE_POL);
        }
      else
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_WS_IDLE_POL, 0);
        }

      /* The default value for the master clock frequency (MCLK frequency)
       * can be set from the sample rate multiplied by a fixed value, known
       * as MCLK multiplier. This multiplier, however, should be divisible
       * by the number of bytes from a sample, i.e, for 24 bits, the
       * multiplier should be divisible by 3. NOTE: the MCLK frequency can
       * be adjusted on runtime, so this value remains valid only if the
       * upper half does not implement the `i2s_setmclkfrequency` method.
       */

      if (priv->config->data_width == I2S_DATA_BIT_WIDTH_24BIT)
        {
          priv->mclk_multiple = I2S_MCLK_MULTIPLE_384;
        }
      else
        {
          priv->mclk_multiple = I2S_MCLK_MULTIPLE_256;
        }

      i2s_setmclkfrequency((struct i2s_dev_s *)priv, (priv->config->rate *
                           priv->mclk_multiple));

      priv->rate = priv->config->rate;

      /* [V341] Dividers + source clock are brought up once, at the end of
       * this function (i2s_clock_source_bringup()). */
    }

  /* Configure the RX module */

  if (i2s_rx_active(priv))
    {
      /* Reset I2S RX module */

      modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0, I2S_RX_RESET);
      modifyreg32(I2S_RX_CONF_REG(priv->config->port), I2S_RX_RESET, 0);

      /* Enable/disable I2S RX slave mode */

      if (priv->config->role == I2S_ROLE_SLAVE)
        {
          modifyreg32(I2S_RX_CONF_REG(priv->config->port),
                      0, I2S_RX_SLAVE_MOD);
        }
      else
        {
          /* Since BCLK and WS are shared, only TX or RX can be master. In
           * this case, force RX as slave to avoid conflict of clock signal.
           */

          /* [V391] V390 showed this decision is what actually sticks:
           * i2s_configure() runs after i2s_rx_channel_start() and
           * re-asserts slave mode, so flipping the bit there was
           * overwritten before the DMA was ever armed.  With
           * g_v390_rxmaster set, run the receiver from its own divider
           * instead of from I2S0I_BCK_IN / I2S0I_WS_IN.
           */

          /* [V691] No-reflash A/B for the receiver's clock source:
           *
           *     nsh> echo m > /tmp/rxrole.txt   RX master (own divider)
           *     nsh> echo s > /tmp/rxrole.txt   RX slave (TX generator)
           *
           * Re-read on every i2s_configure(), so the next capture picks the
           * new value up without touching the firmware. */

          {
            FILE *v691f = fopen("/tmp/rxrole.txt", "r");

            if (v691f != NULL)
              {
                int v691c = fgetc(v691f);

                fclose(v691f);

                if (v691c == 'm' || v691c == 'M')
                  {
                    g_v390_rxmaster = 1;
                  }
                else if (v691c == 's' || v691c == 'S')
                  {
                    g_v390_rxmaster = 0;
                  }
              }
          }

          if (i2s_tx_active(priv) && !g_v390_rxmaster)
            {
              modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0,
                          I2S_RX_SLAVE_MOD);
            }
          else
            {
              modifyreg32(I2S_RX_CONF_REG(priv->config->port),
                          I2S_RX_SLAVE_MOD, 0);
            }

          /* [V394] IRQ-stack: silenced */ if (0) syslog(LOG_INFO,
                 "[V391] rx role=%s rxconf=0x%08x\n",
                 g_v390_rxmaster ? "MASTER" : "SLAVE",
                 (unsigned)getreg32(
                   I2S_RX_CONF_REG(priv->config->port)));
        }

      /* Congfigure RX chan bit, audio data bit and mono mode.
       * On ESP32, sample_bit should equals to data_bit.
       */

      /* Set RX data width */

      priv->data_width = priv->config->data_width;
      i2s_set_datawidth(priv);

      /* Set channel mode control */

      if (priv->config->audio_std_mode <= I2S_TDM_PCM)
        {
          /* [V228] See the TX comment: rx_tdm_en must stay enabled. */
          modifyreg32(I2S_RX_CONF_REG(priv->config->port), I2S_RX_TDM_EN_M,
                      I2S_RX_TDM_EN);

          /* [V389] *** the register this driver has never written ***
           *
           * hal/esp32s3/include/hal/i2s_ll.h:602, in
           * i2s_ll_rx_select_std_slot() -- which every standard-I2S
           * receive path runs:
           *
           *     hw->rx_tdm_ctrl.rx_tdm_tot_chan_num = 1;  (2 slots - 1)
           *     hw->rx_tdm_ctrl.val = chan_mask;          (slot enable)
           *
           * I2S_RX_TDM_CTRL_REG (0x50) is absent from this file entirely,
           * so rx_tdm_tot_chan_num has been sitting at its reset default
           * of 0 while RX_TDM_EN is set and BCLK/WS/DIN are all measured
           * toggling.  The receiver then never produces a DMA request:
           * descriptor OWNER stays 1, no GDMA interrupt, zero bytes --
           * including with I2S_SIG_LOOPBACK set, which proves the fault
           * is inside the RX channel, not on the pins or in the codec.
           *
           * Program it exactly the way ESP-IDF does: 2 slots, both
           * enabled (the reset value already has every chanN_en bit set,
           * but write them so the intent survives a reset default
           * change).
           */
          /* [V682] Same defect as i2s_rxdma_start(): the chan_en half
           * has to be assigned, because bits 2..15 reset to 1.  The old
           * modifyreg32() pair here left the register at 0x0001ffff. */
          {
            uint32_t v682c =
              getreg32(I2S_RX_TDM_CTRL_REG(priv->config->port));

            v682c &= 0xffff0000u;
            v682c |= FIELD_TO_VALUE(I2S_RX_TDM_TOT_CHAN_NUM, 1);
            v682c |= g_i2s_rx_slot_mask;
            putreg32(v682c, I2S_RX_TDM_CTRL_REG(priv->config->port));
          }

          /* [V229] See the TX comment: ESP-IDF's i2s_ll_rx_enable_tdm() also
           * clears rx_pdm2pcm_en, and this driver never wrote the register. */
          modifyreg32(I2S_RX_CONF_REG(priv->config->port),
                      I2S_RX_PDM2PCM_EN, 0);

          if (priv->config->audio_std_mode == I2S_TDM_PHILIPS)
            {
              modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                          I2S_RX_MSB_SHIFT_M, I2S_RX_MSB_SHIFT);
            }
          else
            {
              modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                          I2S_RX_MSB_SHIFT, 0);
            }
        }
      else
        {
          modifyreg32(I2S_RX_CONF_REG(priv->config->port), I2S_RX_PDM_EN_M,
                      I2S_RX_PDM_EN);
        }

      /* Set I2S RX right channel first */

      if (priv->config->ws_pol == 1)
        {
          modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0,
                      I2S_RX_BIG_ENDIAN);
        }
      else
        {
          modifyreg32(I2S_RX_CONF_REG(priv->config->port),
                      I2S_RX_BIG_ENDIAN, 0);
        }

      /* The default value for the master clock frequency (MCLK frequency)
       * can be set from the sample rate multiplied by a fixed value, known
       * as MCLK multiplier. This multiplier, however, should be divisible
       * by the number of bytes from a sample, i.e, for 24 bits, the
       * multiplier should be divisible by 3. NOTE: the MCLK frequency can
       * be adjusted on runtime, so this value remains valid only if the
       * upper half does not implement the `i2s_setmclkfrequency` method.
       */

      if (priv->config->data_width == I2S_DATA_BIT_WIDTH_24BIT)
        {
          priv->mclk_multiple = I2S_MCLK_MULTIPLE_384;
        }
      else
        {
          priv->mclk_multiple = I2S_MCLK_MULTIPLE_256;
        }

      i2s_setmclkfrequency((struct i2s_dev_s *)priv, (priv->config->rate *
                           priv->mclk_multiple));

      priv->rate = priv->config->rate;
    }

  /* [V341] Bring up the source clock exactly once, after the TX and/or RX
   * blocks above have programmed priv->rate / priv->mclk_freq / data_width.
   * This both selects the source and verifies that it actually advances the
   * TX clock domain (the tx_update handshake), falling back to XTAL if the
   * 160 MHz source is not running. */

  if (i2s_tx_active(priv) || i2s_rx_active(priv))
    {
      (void)i2s_clock_source_bringup(priv);
    }

  /* [V716] *** RX register sweep, no reflash between runs. ***
   *
   * Straight off the DMA the captured words look like this (v714):
   *
   *   [V709] raw : ffff 0000 f000 2000 e000 3000 a000 4000 c000 4000 ...
   *   [V709] tdm=0x00010001 conf1=0x2f3de381 rxconf=0x00089604
   *
   * conf1 decodes, at the real SOC field positions, to RX_BITS_MOD =
   * RX_TDM_CHAN_BITS = RX_HALF_SAMPLE_BITS = 15 and RX_TDM_WS_WIDTH = 1
   * -- a two-bit-clock frame sync, since the header defines the ws_out
   * width as (RX_TDM_WS_WIDTH + 1) * T_bck.  The delivered rate is wrong
   * too: 2046 valid bytes per 4092-byte descriptor, ~36 kB/s where 16 kHz
   * stereo 16-bit wants 64 kB/s, so roughly 9.1 kframe/s instead of 16.
   *
   * i2s_set_datawidth() mirrors its TX and RX conditions.  For this port
   * (audio_std_mode is I2S_TDM_PHILIPS, not I2S_TDM_PCM) TX is given
   * TX_TDM_WS_WIDTH = data_width - 1 = 15 and plays cleanly, while RX is
   * given RX_TDM_WS_WIDTH = 1 -- and both get HALF_SAMPLE_BITS =
   * data_width - 1, where ESP-IDF's HAL uses half the slot width.
   *
   * That is a strong suspicion, not a proof, so the fields are made
   * sweepable instead of guessed at.  /tmp/rxcfg.txt holds a list of
   * single-character actions, applied in order, so combinations are just
   * concatenation:
   *
   *   nsh> echo 1  > /tmp/rxcfg.txt   RX_HALF_SAMPLE_BITS = dw/2 - 1
   *   nsh> echo 4  > /tmp/rxcfg.txt   RX_TDM_WS_WIDTH     = dw - 1
   *   nsh> echo 14 > /tmp/rxcfg.txt   both
   *   nsh> echo 1234 > /tmp/rxcfg.txt all of bits/chan/half/ws
   *   nsh> rm      /tmp/rxcfg.txt     driver defaults
   *
   * (1 = HALF_SAMPLE_BITS, 2 = TDM_CHAN_BITS, 3 = BITS_MOD, 4 =
   * TDM_WS_WIDTH, 5 = MSB_SHIFT -> 0, 6 = LEFT_ALIGN -> 0.)
   *
   * Read here, at the end of i2s_configure().  The capture path reaches
   * this function through audio_capture_open(), so the always-on listener
   * re-reads it once per session (~14 s).  Change the file, wait one
   * session, read the log.
   */

  if (i2s_rx_active(priv))
    {
      int   v716port = priv->config->port;
      FILE *v716f;

      /* RX_LEFT_ALIGN is the one field i2s_configure() never writes, so an
       * override would survive into every later session.  Restart from its
       * reset default on each pass and let the file decide.  Everything
       * else is re-programmed by i2s_set_datawidth() and the TDM_CTRL
       * block earlier in this same call, so no other field needs undoing. */

      /* [V721] Was I2S_RX_LEFT_ALIGN (= 1).  This diagnostic block runs at
       * the end of i2s_configure(), i.e. after i2s_set_datawidth(), so it
       * used to overwrite the V721B-1 fix on every configure and put the
       * line back to the dead setting.  v721 slot 7 measured it: 1 -> all
       * words in {0,1,3}, 0 -> real audio.  Write 0 so both writers
       * agree. */

      modifyreg32(I2S_RX_CONF_REG(v716port), I2S_RX_LEFT_ALIGN_M,
                  0);

      {
        /* [V717] Self-driving profile table.  One entry per configure call,
         * in this order:
         *
         * [V718] Retargeted.  V718-1 repaired the default, so entry 0 is
         * now the fixed baseline and "4" would be a no-op.  Every remaining
         * entry turns ONE alignment knob, hunting the residual 3-bit left
         * shift: under WS=15 all 51 raw dumps have low3_or = 0x0, i.e. every
         * sample is a multiple of 8 and only 13 of the 16 bits carry signal.
         *
         *   0 ""      repaired default (WS_WIDTH = dw - 1)
         *   1 "6"     RX_LEFT_ALIGN       = 0   (i2s_configure forces 1)
         *   2 "5"     RX_MSB_SHIFT        = 0   (currently 1, PHILIPS)
         *   3 "56"    left-align 0 + msb-shift 0
         *   4 "3"     RX_BITS_MOD         = dw/2 - 1
         *   5 "2"     RX_TDM_CHAN_BITS    = dw/2 - 1
         *   6 "1"     RX_HALF_SAMPLE_BITS = dw/2 - 1
         *   7 "561"   left-align 0 + msb-shift 0 + half
         *   8 "56"    repeat of entry 3, as a stability check
         *
         * One entry per configure, ~2 minutes for the sweep, no NSH and no
         * reflash.  /tmp/rxcfg.txt still overrides the table when readable. */

        /* [V722] The sweep table is what broke the microphone -- there
         * was no missing fix left to find.
         *
         * v721b landed correct RX framing (WS_WIDTH = dw - 1,
         * LEFT_ALIGN = 0) and this block's entry 0 leaves it alone, so
         * _v721b_boot.log starts out healthy: 64 raw dumps under
         * conf1 = 0x2f3de38f, medDx 96, maxabs 624, real audio.  Then
         * at t = 38.36 this table reached entry 2 ("5"), wrote
         * I2S_RX_MSB_SHIFT = 0, and the remaining 69 dumps are words
         * drawn from {0000,4000,8000,c000} -- medDx 16384, maxabs
         * 32768.  The split by conf1 is exact: 64/64 good versus
         * 69/69 junk, no overlap.
         *
         * Those junk dumps are also what the handoff filed as
         * "possible clipping".  They are not: the good dumps peak at
         * 624, about -36 dBFS, so the line is far too quiet rather
         * than overdriven, and cutting ADC gain would have made it
         * worse.
         *
         * i2s_configure() does re-assert MSB_SHIFT in its PHILIPS
         * branch, but this block runs after it, so the scanner just
         * re-broke the line on each configure -- and it walks further
         * into the table the longer the board is up, entries 3..8
         * turning two or three knobs at once.  A device left running
         * degrades permanently, one knob per listening session.
         * Entries 0 and 1 are harmless, which is why v721b looked
         * fine for its first 38 seconds.
         *
         * So: one entry, the known-good baseline.  /tmp/rxcfg.txt
         * still overrides it, which is the whole point of the block,
         * so nothing is lost -- it simply stops inventing
         * configurations on its own. */

        static const char *v717tab[] =
          { "" };
        static unsigned    v717n;
        char   v716b[32];
        size_t v716n;
        size_t v716k;

        v716f = fopen("/tmp/rxcfg.txt", "r");

        if (v716f != NULL)
          {
            v716n = fread(v716b, 1, sizeof(v716b) - 1u, v716f);
            fclose(v716f);
          }
        else
          {
            const char *v717t = v717tab[0];

            v716n = strlen(v717t);
            memcpy(v716b, v717t, v716n);
          }

        syslog(LOG_INFO, "[V717] profile=%u tok='%s' dw=%d rate=%d\n",
               (unsigned)v717n, v716b, (int)priv->data_width,
               (int)priv->rate);

        v717n++;
        g_v717_gen++;
        v716b[v716n] = '\0';

          for (v716k = 0u; v716k < v716n; v716k++)
            {
              uint32_t v716v;

              switch (v716b[v716k])
                {
                  case '1':
                    v716v = (uint32_t)(priv->data_width >> 1) - 1u;
                    modifyreg32(I2S_RX_CONF1_REG(v716port),
                                I2S_RX_HALF_SAMPLE_BITS_M,
                                v716v << I2S_RX_HALF_SAMPLE_BITS_S);
                    break;

                  case '2':
                    v716v = (uint32_t)(priv->data_width >> 1) - 1u;
                    modifyreg32(I2S_RX_CONF1_REG(v716port),
                                I2S_RX_TDM_CHAN_BITS_M,
                                v716v << I2S_RX_TDM_CHAN_BITS_S);
                    break;

                  case '3':
                    v716v = (uint32_t)(priv->data_width >> 1) - 1u;
                    modifyreg32(I2S_RX_CONF1_REG(v716port),
                                I2S_RX_BITS_MOD_M,
                                v716v << I2S_RX_BITS_MOD_S);
                    break;

                  case '4':
                    v716v = (uint32_t)priv->data_width - 1u;
                    modifyreg32(I2S_RX_CONF1_REG(v716port),
                                I2S_RX_TDM_WS_WIDTH_M,
                                v716v << I2S_RX_TDM_WS_WIDTH_S);
                    break;

                  case '5':
                    modifyreg32(I2S_RX_CONF1_REG(v716port),
                                I2S_RX_MSB_SHIFT_M, 0);
                    break;

                  case '6':
                    modifyreg32(I2S_RX_CONF_REG(v716port),
                                I2S_RX_LEFT_ALIGN_M, 0);
                    break;

                  default:
                    break;
                }
            }

      }

      syslog(LOG_INFO,
             "[V717] regs conf1=0x%08x rxconf=0x%08x tdm=0x%08x\n",
             (unsigned)getreg32(I2S_RX_CONF1_REG(v716port)),
             (unsigned)getreg32(I2S_RX_CONF_REG(v716port)),
             (unsigned)getreg32(I2S_RX_TDM_CTRL_REG(v716port)));
    }
}

/* [V93] Give the active direction sole ownership of the port.
 *
 * Called from the ES8311 worker immediately before an I2S_SEND or an
 * I2S_RECEIVE.  Idempotent: reconfiguration happens only when the direction
 * actually changes, so a stream of capture buffers does not restart the DMA.
 */

/* [V660] One-shot capture-path health dump.
 *
 * Called from the application's capture thread immediately after
 * AUDIOIOC_START returns, i.e. from ordinary task context.  The same
 * measurement used to live in i2s_rx_channel_start() and had to be silenced
 * by [V394] because syslog() on the interrupt stack overflowed it (all of
 * [V382]/[V383]/[V390]/[RXCLK] are wrapped in `if (0)` for that reason).
 *
 * Why it matters: every PTT chunk in the v658 field run reported peak=6, and
 * the [V380] calibration says a healthy loopback shows peak == 8 while real
 * microphone audio shows "a much larger peak".  A constant 6 is an idle
 * serial stream, so the first question is whether the clocks are moving:
 *
 *   MCLK(11)/BCLK(10) hi ~= 1000/2000 -> clock running
 *   WS(8)             hi ~=  500/2000 -> 50% duty frame sync
 *   DIN(9)            hi ~= 1000/2000 -> the codec is shifting data
 *   any line stuck at 0 or 2000       -> that line is dead
 */

/* [V690] Tiny queue walker: sq_count() is not in every NuttX revision and
 * the RX queues are short, so guard the walk explicitly. */

static int i2s_v690_qlen(FAR sq_queue_t *q)
{
  FAR sq_entry_t *e;
  int n = 0;

  for (e = sq_peek(q); e != NULL; e = sq_next(e))
    {
      if (++n > 64)
        {
          break;
        }
    }

  return n;
}

void esp32s3_i2s_rx_health(void)
{
  static int s_v660_n;
  unsigned int mhi = 0;
  unsigned int bhi = 0;
  unsigned int whi = 0;
  unsigned int dhi = 0;
  unsigned int k;

  /* [V682] Slot-mask override, read on EVERY capture start (before the
   * one-shot guard below) so the candidates can be swept from NSH:
   *
   *     echo l > /tmp/rxslot.txt    slot 0 only  (mono, default)
   *     echo r > /tmp/rxslot.txt    slot 1 only
   *     echo b > /tmp/rxslot.txt    both slots
   *
   * The value is picked up by the next i2s_rxdma_start() re-arm, i.e.
   * within a few milliseconds, without touching the codec.
   */
  {
    FILE *v682f = fopen("/tmp/rxslot.txt", "r");

    if (v682f != NULL)
      {
        int v682c = fgetc(v682f);

        fclose(v682f);

        if (v682c == 'r' || v682c == 'R')
          {
            g_i2s_rx_slot_mask = I2S_RX_TDM_PDM_CHAN1_EN;
          }
        else if (v682c == 'b' || v682c == 'B')
          {
            g_i2s_rx_slot_mask = I2S_RX_TDM_PDM_CHAN0_EN |
                                 I2S_RX_TDM_PDM_CHAN1_EN;
          }
        else if (v682c == 'l' || v682c == 'L')
          {
            g_i2s_rx_slot_mask = I2S_RX_TDM_PDM_CHAN0_EN;
          }
      }
  }

  /* [V690] RX pipeline counters.
   *
   * Every capture session so far delivered exactly six 4092-byte buffers and
   * then went quiet, and the 4 s kmsg pump makes counting log lines useless
   * (lines replay 2-3x and multi-line records merge).  These counters settle
   * it: if recv grows but eof does not, the transfer is being armed and
   * never completing; if eof grows but act stays at 1, i2s_rx_schedule()
   * never matched SUC_EOF_DES_ADDR and i2s_rxdma_start() is bailing out on
   * the "rx.act not empty" early return.
   *
   * Written to a file as well, because the kmsg ring is only 32 KB. */
  {
    char v690b[192];

    snprintf(v690b, sizeof(v690b),
             "recv=%u start=%u eof=%u isr=%u pb=%u done=%u pend=%d act=%d "
             "done_q=%d slot=0x%02x mono=%d bounce=%d txa=%d txp=%d txd=%d\n",
             (unsigned)g_v94_recv, (unsigned)g_v94_start,
             (unsigned)g_v94_eof, (unsigned)g_v94_isr,
             (unsigned)g_v94_pb, (unsigned)g_v94_done,
             g_v690_priv ? i2s_v690_qlen(&g_v690_priv->rx.pend) : -1,
             g_v690_priv ? i2s_v690_qlen(&g_v690_priv->rx.act)  : -1,
             g_v690_priv ? i2s_v690_qlen(&g_v690_priv->rx.done) : -1,
             (unsigned)g_i2s_rx_slot_mask, (int)g_i2s_rx_mono,
             (int)(g_i2s_dmabuf_used[0] + g_i2s_dmabuf_used[1] +
                   g_i2s_dmabuf_used[2] + g_i2s_dmabuf_used[3]),
             g_v690_priv ? i2s_v690_qlen(&g_v690_priv->tx.act)  : -1,
             g_v690_priv ? i2s_v690_qlen(&g_v690_priv->tx.pend) : -1,
             g_v690_priv ? i2s_v690_qlen(&g_v690_priv->tx.done) : -1);

    syslog(LOG_INFO, "[V690] rx %s", v690b);

    {
      FILE *v690f = fopen("/tmp/rxstat.txt", "w");

      if (v690f != NULL)
        {
          fputs(v690b, v690f);
          fclose(v690f);
        }

      v690f = fopen("/tmp/rxlog.txt", "a");
      if (v690f != NULL)
        {
          fputs(v690b, v690f);
          fclose(v690f);
        }
    }
  }

  /* [V694] *** Arm both engines here -- this is the only call site that is
   * guaranteed to run AFTER every i2s_configure(). ***
   *
   * native_capture_start() calls this function immediately after
   * AUDIOIOC_START returns, whereas i2s_configure() pulses I2S_TX_RESET and
   * clears I2S_RX_START / I2S_TX_START *after* i2s_rx_channel_start().  The
   * v691 keep-alive lives in i2s_rxdma_start(), in the RX worker and in
   * set_rxonly(), none of which can run when the transfer never starts, so
   * whether the clocks came up at all depended on the interleaving.  With
   * the START bits clear the ES8311 has no bit clock, never drives DIN, and
   * the session dies with five "capture EIO" retries and zero bytes:
   *
   *   good: rxconf=0x0008960c txconf=0x00089204  (BIT(2) set in both)
   *   bad : rxconf=0x00089600 txconf=0x00089260  (BIT(2) clear in both)
   *
   * This block is deliberately ABOVE the s_v660_n one-shot guard below, so
   * it runs on every call, and it is idempotent.
   */
  if (g_v690_priv != NULL)
    {
      static unsigned long s_v694_arms;
      int      v694port = (int)g_v690_priv->config->port;
      uint32_t v694tx   = getreg32(I2S_TX_CONF_REG(v694port));
      uint32_t v694rx   = getreg32(I2S_RX_CONF_REG(v694port));

      if ((v694tx & I2S_TX_STOP_EN) != 0)
        {
          modifyreg32(I2S_TX_CONF_REG(v694port), I2S_TX_STOP_EN, 0);
          modifyreg32(I2S_TX_CONF_REG(v694port), 0, I2S_TX_UPDATE);
        }

      if ((v694tx & I2S_TX_START) == 0)
        {
          modifyreg32(I2S_TX_CONF_REG(v694port), 0, I2S_TX_START);
          modifyreg32(I2S_TX_CONF_REG(v694port), 0, I2S_TX_UPDATE);
        }

      if ((v694rx & I2S_RX_START) == 0)
        {
          modifyreg32(I2S_RX_CONF_REG(v694port), 0, I2S_RX_START);
          modifyreg32(I2S_RX_CONF_REG(v694port), 0, I2S_RX_UPDATE);
        }

      if (s_v694_arms < 8u)
        {
          s_v694_arms++;
          syslog(LOG_INFO,
                 "[V694] arm port=%d tx 0x%08x->0x%08x rx 0x%08x->0x%08x\n",
                 v694port, (unsigned)v694tx,
                 (unsigned)getreg32(I2S_TX_CONF_REG(v694port)),
                 (unsigned)v694rx,
                 (unsigned)getreg32(I2S_RX_CONF_REG(v694port)));
        }
    }

  if (s_v660_n >= 12)
    {
      return;
    }

  s_v660_n++;

  modifyreg32(IO_MUX_GPIO8_REG,  0, (1u << 9));   /* WS   */
  modifyreg32(IO_MUX_GPIO9_REG,  0, (1u << 9));   /* DIN  */
  modifyreg32(IO_MUX_GPIO10_REG, 0, (1u << 9));   /* BCLK */
  modifyreg32(IO_MUX_GPIO11_REG, 0, (1u << 9));   /* MCLK */

  for (k = 0; k < 2000; k++)
    {
      if (esp32s3_gpioread(11)) { mhi++; }
      if (esp32s3_gpioread(10)) { bhi++; }
      if (esp32s3_gpioread(8))  { whi++; }
      if (esp32s3_gpioread(9))  { dhi++; }
    }

  syslog(LOG_INFO,
         "[V660] pins MCLK(11)=%u BCLK(10)=%u WS(8)=%u DIN(9)=%u /2000 | "
         "rxconf=0x%08x txconf=0x%08x rxstart=%d txstart=%d rxonly=%d "
         "loopback=%d txmaster=%d rxslot=0x%02x rxtdm=0x%08x\n",
         mhi, bhi, whi, dhi,
         (unsigned)getreg32(I2S_RX_CONF_REG(0)),
         (unsigned)getreg32(I2S_TX_CONF_REG(0)),
         (getreg32(I2S_RX_CONF_REG(0)) & I2S_RX_START) ? 1 : 0,
         (getreg32(I2S_TX_CONF_REG(0)) & I2S_TX_START) ? 1 : 0,
         g_i2s_rxonly_port, g_v380_loopback, g_v380_txmaster,
         (unsigned)g_i2s_rx_slot_mask,
         (unsigned)getreg32(I2S_RX_TDM_CTRL_REG(0)));
}

void esp32s3_i2s_set_rxonly(FAR struct i2s_dev_s *dev, bool on)
{
  FAR struct esp32s3_i2s_s *priv = (FAR struct esp32s3_i2s_s *)dev;
  int port = (int)priv->config->port;
  /* [V380] While the transmitter owns the port the capture worker must
   * not be able to take it away -- handing it back and forth is what
   * restarted the DMA underneath the other worker.  The loopback flag is
   * still honoured: a change there has to reach i2s_configure().
   */

  int want = (on && !g_v380_txmaster) ? port : -1;

  if (want == g_i2s_rxonly_port && g_v380_loopback == g_v380_lb_applied)
    {
      return;
    }

  /* [V644] PROBE: mid-stream port reconfiguration (the [V89] hazard). */
  

  /* Stop both engines before rewiring the clock. */

      /* [V341] The clock source and its dividers are now handled in exactly
       * one place: i2s_configure() -> i2s_clock_source_bringup().  The old
       * V241 block that re-wrote g_i2s_sclk / I2S_TX_CLK_SEL here (and thus
       * fought i2s_configure()) has been removed.  i2s_configure() is invoked
       * at the bottom of this function and will (re)select the source and
       * verify that it actually advances the TX clock domain. */

  modifyreg32(I2S_TX_CONF_REG(port), I2S_TX_START, 0);
  modifyreg32(I2S_RX_CONF_REG(port), I2S_RX_START, 0);

  g_i2s_rxonly_port = want;
  g_v380_lb_applied = g_v380_loopback;

  i2s_configure(priv);

  /* [V691] i2s_configure() pulses I2S_TX_RESET, which drops TX_START, and
   * the pads carry the TX generator's BCK/WS.  Put the frame clock back
   * before the caller arms the DMA. */

  if (on)
    {
      i2s_v691_clock_ensure(priv, 1);
    }
}

/* [V858] *** Re-lock the receiver so it can never be left behind a
 * playback. ***
 *
 * The symptom: after the first TTS playback the capture returns all
 * zeros (or a frozen DC such as 0x0012, peak 18) for the rest of the
 * boot, while the ES8311 readback stays perfectly healthy
 * (r00=0x80 r32=0xbf i2c_errs=0) and the pads are still clocked.  QA
 * hit it in 3 of 4 cold boots.
 *
 * It is NOT the clocks.  Every [V660] pins probe in the dead window
 * still reads
 *     MCLK(11)=1513 BCLK(10)=991 WS(8)=1008 DIN(9)=148 /2000
 * i.e. the codec is being clocked and is shifting data at us.  What is
 * wrong is where the receiver's bit-aligned frame sits RELATIVE to that
 * data, and the register file cannot tell us: v857 qa2 and qa4 both
 * carry stretches of
 *     rxconf=0x00081604 tdm=0x00010001 conf1=0x2f3de38f
 * with one of them delivering real audio and the other a constant.
 * The two roles behave the same way -- each has been measured both
 * working and dead, which is the signature of a free-running phase,
 * not of a wrong setting.
 *
 * So this function does the one thing that IS deterministic: it walks
 * the port through exactly the sequence it walks through at boot, in
 * the same order, every time, instead of inheriting whatever state the
 * transmitter was left in by the playback that just finished.  The
 * [V382] clock-only start inside i2s_rx_channel_start() is guarded by
 * "if the transmitter is not already started" and therefore does
 * nothing at all once a playback has left TX_START set with the
 * playback frame (MONO / CHAN_EQUAL / LEFT_ALIGN / STOP_EN all changed
 * by i2s_txdma_start()) latched in -- which is the state the board is
 * in when the microphone dies.
 *
 * The relock is idempotent and cheap (~ninety register writes, no
 * waiting on the codec), and it never touches the ES8311's I2C
 * registers, so the verified codec configuration survives it. */

static int i2s_v858_tx_busy(struct esp32s3_i2s_s *priv)
{
  return !(sq_empty(&priv->tx.act) && sq_empty(&priv->tx.pend) &&
           sq_empty(&priv->tx.done));
}

static int i2s_v858_rx_busy(struct esp32s3_i2s_s *priv)
{
  return !(sq_empty(&priv->rx.act) && sq_empty(&priv->rx.pend));
}

static void i2s_v858_relock(int slave, int verbose)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)g_v690_priv;
  int      port;
  uint32_t rx0;
  uint32_t tx0;
  uint32_t tdm;
  int      spins;

  if (priv == NULL || !priv->config->rx_en)
    {
      return;
    }

  port = (int)priv->config->port;

  rx0 = getreg32(I2S_RX_CONF_REG(port));
  tx0 = getreg32(I2S_TX_CONF_REG(port));

  g_v858_relocks++;

  /* 1. both engines off.  Everything below has to be latched into the
   *    clock domains while they are stopped, which is exactly what the
   *    [V382] / [V721] / [V722] fixes established. */

  modifyreg32(I2S_RX_CONF_REG(port), I2S_RX_START, 0);
  modifyreg32(I2S_TX_CONF_REG(port), I2S_TX_START, 0);

  /* 2. reset both data paths, the way the boot path does. */

  modifyreg32(I2S_RX_CONF_REG(port), 0, I2S_RX_RESET);
  modifyreg32(I2S_RX_CONF_REG(port), I2S_RX_RESET, 0);
  modifyreg32(I2S_RX_CONF_REG(port), 0, I2S_RX_FIFO_RESET);
  modifyreg32(I2S_RX_CONF_REG(port), I2S_RX_FIFO_RESET, 0);

  modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_RESET);
  modifyreg32(I2S_TX_CONF_REG(port), I2S_TX_RESET, 0);
  modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_FIFO_RESET);
  modifyreg32(I2S_TX_CONF_REG(port), I2S_TX_FIFO_RESET, 0);

  /* 3. the divider the common bring-up selected and validated.  The
   *    [V670] 150 MHz source plus the [V341] bring-up is the only
   *    place the clock is derived; reuse the values it latched instead
   *    of re-deriving them (i2s_set_clock() is far too heavy to run
   *    on a capture start). */

  if (g_txclk_valid)
    {
      putreg32(g_txclk_clkm,  I2S_TX_CLKM_CONF_REG(port));
      putreg32(g_txclk_div,   I2S_TX_CLKM_DIV_CONF_REG(port));
      putreg32(g_txclk_conf1, I2S_TX_CONF1_REG(port));
    }

  /* 4. the pads have to be readable by the receiver's clock inputs.
   *
   * i2s_configure()'s master branch leaves BCK and WS configured as
   * outputs only, and esp32s3_gpio_matrix_in() cannot deliver a level
   * the pad's input buffer never sees.  The [V383] / [V660] / [V724]
   * probes all enable IO_MUX FUN_IE (bit 9) before they read the pad,
   * which is what makes them read anything at all; the capture path
   * used to depend on one of those probes having run.  Do it here so
   * the slave clock path has a real input whether or not any probe
   * has fired yet. */

  modifyreg32(IO_MUX_GPIO8_REG,  0, (1u << 9));   /* WS   */
  modifyreg32(IO_MUX_GPIO9_REG,  0, (1u << 9));   /* DIN  */
  modifyreg32(IO_MUX_GPIO10_REG, 0, (1u << 9));   /* BCLK */

  esp32s3_gpio_matrix_in(priv->config->ws_pin,
                         priv->config->ws_in_insig, 0);
  esp32s3_gpio_matrix_in(priv->config->bclk_pin,
                         priv->config->bclk_in_insig, 0);

  /* 5. receiver framing: 16-bit Philips, two slots, slot 0 only.  The
   *    three fields the [V718] / [V721] / [V722] sweeps isolated,
   *    written together so no later i2s_configure() can split them. */

  modifyreg32(I2S_RX_CONF_REG(port), I2S_RX_LEFT_ALIGN_M, 0);

  modifyreg32(I2S_RX_CONF1_REG(port), I2S_RX_TDM_WS_WIDTH_M,
              FIELD_TO_VALUE(I2S_RX_TDM_WS_WIDTH, 15));

  modifyreg32(I2S_RX_CONF1_REG(port), I2S_RX_MSB_SHIFT_M, I2S_RX_MSB_SHIFT);

  tdm  = getreg32(I2S_RX_TDM_CTRL_REG(port)) & 0xffff0000u;
  tdm |= FIELD_TO_VALUE(I2S_RX_TDM_TOT_CHAN_NUM, 1);
  tdm |= g_i2s_rx_slot_mask;
  putreg32(tdm, I2S_RX_TDM_CTRL_REG(port));

  /* 6. the role.  The self-heal alternates this. */

  if (slave)
    {
      modifyreg32(I2S_RX_CONF_REG(port), 0, I2S_RX_SLAVE_MOD);
    }
  else
    {
      modifyreg32(I2S_RX_CONF_REG(port), I2S_RX_SLAVE_MOD, 0);
    }

  /* 7. transmitter = the frame clock on the pads, in the clock-only
   *    shape [V382] / [V725] established: STOP_EN, MONO, CHAN_EQUAL
   *    and LEFT_ALIGN cleared, the playback TDM frame and PCM2PDM
   *    values re-latched, then started.  This is the step that a
   *    finished playback used to skip, leaving the transmitter in the
   *    playback shape with its FIFO empty. */

  modifyreg32(I2S_TX_CONF_REG(port),
              I2S_TX_MONO_M | I2S_TX_CHAN_EQUAL_M |
              I2S_TX_LEFT_ALIGN_M | I2S_TX_STOP_EN_M, 0);

  tdm = FIELD_TO_VALUE(I2S_TX_TDM_TOT_CHAN_NUM, 1) | I2S_TX_TDM_CHAN0_EN;

  if (!g_i2s_tx_mono)
    {
      tdm |= I2S_TX_TDM_CHAN1_EN;
    }

  putreg32(tdm, I2S_TX_TDM_CTRL_REG(port));
  putreg32(0x004aa004u, I2S_TX_PCM2PDM_CONF_REG(port));

  modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_START);
  modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_UPDATE);

  for (spins = 0;
       spins < I2S_UPDATE_SPIN_MAX &&
       (getreg32(I2S_TX_CONF_REG(port)) & I2S_TX_UPDATE);
       spins++)
    {
    }

  modifyreg32(I2S_RX_CONF_REG(port), 0, I2S_RX_UPDATE);

  for (spins = 0;
       spins < I2S_UPDATE_SPIN_MAX &&
       (getreg32(I2S_RX_CONF_REG(port)) & I2S_RX_UPDATE);
       spins++)
    {
    }

  /* 8. receiver on, with the rx_update handshake on both sides of
   *    rx_start the way [V388] requires. */

  modifyreg32(I2S_RX_CONF_REG(port), 0, I2S_RX_START);
  modifyreg32(I2S_RX_CONF_REG(port), 0, I2S_RX_UPDATE);

  for (spins = 0;
       spins < I2S_UPDATE_SPIN_MAX &&
       (getreg32(I2S_RX_CONF_REG(port)) & I2S_RX_UPDATE);
       spins++)
    {
    }

  if (verbose || g_v858_relocks <= 8 || (g_v858_relocks % 32) == 0)
    {
      syslog(LOG_ERR,
             "[V858] relock #%u role=%s rx 0x%08x->0x%08x "
             "tx 0x%08x->0x%08x tdm=0x%08x conf1=0x%08x\n",
             (unsigned)g_v858_relocks, slave ? "SLAVE" : "MASTER",
             (unsigned)rx0,
             (unsigned)getreg32(I2S_RX_CONF_REG(port)),
             (unsigned)tx0,
             (unsigned)getreg32(I2S_TX_CONF_REG(port)),
             (unsigned)getreg32(I2S_RX_TDM_CTRL_REG(port)),
             (unsigned)getreg32(I2S_RX_CONF1_REG(port)));
    }
}

/* [V858] Called at the top of native_capture_start(), BEFORE the empty
 * buffers are primed, so the receiver is re-locked while nothing of
 * ours is in flight and the GDMA inlink cannot be disturbed. */

void esp32s3_i2s_v858_capture_begin(void)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)g_v690_priv;

  if (priv == NULL)
    {
      return;
    }

  if (i2s_v858_tx_busy(priv) || i2s_v858_rx_busy(priv))
    {
      return;
    }

  i2s_v858_relock(g_v858_rxslave, 0);
}

/* [V858] Called from audio_playback_close(), which until now only
 * drained the queue and never touched the port.  Anything the playback
 * abandoned (a quarantined container after an "I2S TX completion
 * TIMEOUT", whose DMA descriptor is still linked) is released first,
 * then the port is put back into the shape it had at boot.
 *
 * Skipped while a capture is streaming: that capture re-locks itself on
 * its next session, and tearing the receiver down under it would cost
 * it a buffer. */

void esp32s3_i2s_v858_after_playback(void)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)g_v690_priv;

  if (priv == NULL)
    {
      return;
    }

  if (i2s_v858_rx_busy(priv))
    {
      return;
    }

  esp32s3_i2s_tx_flush();

  i2s_v858_relock(g_v858_rxslave, 0);
}

/* [V858] Self-heal: flip the receiver's clock role and re-lock.
 *
 * Neither role is right on every boot -- the phase between two
 * free-running dividers decides, and it is set by where each happened
 * to start.  The only way out of a frozen capture is to move, so each
 * heal alternates and the [V858] log says which one was in force when
 * the microphone came back. */

void esp32s3_i2s_v858_heal(void)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)g_v690_priv;

  if (priv == NULL)
    {
      return;
    }

  g_v858_heals++;
  g_v858_rxslave = g_v858_rxslave ? 0 : 1;

  i2s_v858_relock(g_v858_rxslave, 1);

  syslog(LOG_ERR,
         "[V858] heal #%u role=%s (heals=%u relocks=%u) rxconf=0x%08x\n",
         (unsigned)g_v858_heals,
         g_v858_rxslave ? "SLAVE" : "MASTER",
         (unsigned)g_v858_heals, (unsigned)g_v858_relocks,
         (unsigned)getreg32(I2S_RX_CONF_REG(priv->config->port)));
}

/* [V858b] Re-lock with the role that is in force right now.  This is
 * the move that cleared the post-playback window in the v858 field
 * run: the registers were already right, the bit phase was not, and a
 * fresh start edge draws a new one. */

void esp32s3_i2s_v858_relock_current(void)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)g_v690_priv;

  if (priv == NULL)
    {
      return;
    }

  i2s_v858_relock(g_v858_rxslave, 0);
}

uint32_t esp32s3_i2s_v858_rxconf(void)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)g_v690_priv;

  if (priv == NULL)
    {
      return 0;
    }

  return (uint32_t)getreg32(I2S_RX_CONF_REG(priv->config->port));
}

/****************************************************************************
 * Name: i2s_check_mclkfrequency
 *
 * Description:
 *   Check if MCLK frequency is compatible with the current data width and
 *   bits/sample set. Master clock should be multiple of the sample rate and
 *   bclk at the same time.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   Returns the current master clock or a negated errno value on failure.
 *
 ****************************************************************************/

static int32_t i2s_check_mclkfrequency(struct esp32s3_i2s_s *priv)
{
  uint32_t mclk_freq;
  uint32_t mclk_multiple = priv->mclk_multiple;
  uint32_t bclk = priv->rate * priv->config->total_slot * priv->data_width;
  int i;

  /* If the master clock is divisible by both the sample rate and the bit
   * clock, everything is as expected and we can return the current master
   * clock frequency.
   */

  if (priv->mclk_freq % priv->rate == 0 && priv->mclk_freq % bclk == 0)
    {
      priv->mclk_multiple = priv->mclk_freq / priv->rate;
      return priv->mclk_freq;
    }

  /* Select the lowest multiplier for setting the master clock */

  for (mclk_multiple = I2S_MCLK_MULTIPLE_128;
       mclk_multiple <= I2S_MCLK_MULTIPLE_512;
       mclk_multiple += I2S_MCLK_MULTIPLE_128)
    {
      mclk_freq = priv->rate * mclk_multiple;
      if (mclk_freq % priv->rate == 0 && mclk_freq % bclk == 0)
        {
          priv->mclk_multiple = mclk_multiple;
          i2s_setmclkfrequency((struct i2s_dev_s *)priv, mclk_freq);
          return priv->mclk_freq;
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: i2s_set_datawidth
 *
 * Description:
 *   Set the I2S TX/RX data width.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   Returns the resulting data width
 *
 ****************************************************************************/

static uint32_t i2s_set_datawidth(struct esp32s3_i2s_s *priv)
{
#ifdef I2S_HAVE_TX
  if (priv->config->tx_en)
    {
      modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
                  I2S_TX_BITS_MOD_M, FIELD_TO_VALUE(I2S_TX_BITS_MOD,
                  priv->data_width - 1));

      modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
                  I2S_TX_TDM_CHAN_BITS_M,
                  FIELD_TO_VALUE(I2S_TX_TDM_CHAN_BITS,
                  priv->data_width - 1));

      modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
                  I2S_TX_HALF_SAMPLE_BITS_M,
                  FIELD_TO_VALUE(I2S_TX_HALF_SAMPLE_BITS,
                  priv->data_width - 1));

      if (priv->config->audio_std_mode == I2S_TDM_PCM)
        {
          modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
                      I2S_TX_TDM_WS_WIDTH_M,
                      FIELD_TO_VALUE(I2S_TX_TDM_WS_WIDTH, 1));
        }
      else
        {
          modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
                      I2S_TX_TDM_WS_WIDTH_M,
                      FIELD_TO_VALUE(I2S_TX_TDM_WS_WIDTH,
                      priv->data_width - 1));
        }

      /* Set I2S_TX_UPDATE bit to update configs. Cleared automatically */

      modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                  0, I2S_TX_UPDATE);
    }
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  if (priv->config->rx_en)
    {
      modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                  I2S_RX_BITS_MOD_M, FIELD_TO_VALUE(I2S_RX_BITS_MOD,
                  priv->data_width - 1));

      modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                  I2S_RX_TDM_CHAN_BITS_M,
                  FIELD_TO_VALUE(I2S_RX_TDM_CHAN_BITS,
                  priv->data_width - 1));

      modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                  I2S_RX_HALF_SAMPLE_BITS_M,
                  FIELD_TO_VALUE(I2S_RX_HALF_SAMPLE_BITS,
                  priv->data_width - 1));

      /* [V718] Mirror the transmitter -- this condition was inverted.
       *
       * For this port audio_std_mode is not I2S_TDM_PCM, so the TX branch
       * (which has always worked, the speaker is audible) programs
       * TX_TDM_WS_WIDTH = data_width - 1 = 15, while the RX branch here
       * programmed 1, i.e. a WS high time of two BCK instead of sixteen.
       *
       * The receiver owns WS during capture, so that is the frame the
       * ES8311 saw, and it drove DIN against the wrong boundary.  Pairing
       * every [V709] raw dump with its conf1 shows the two outcomes:
       *
       *   WS_WIDTH = 15  -> audio (0030 0000 ... 1aa0 4620 e3e8 c2b0 19b0,
       *                            1bc8 1c28 1c00 1bd8 ..., 24 dumps)
       *   WS_WIDTH =  1  -> every word in {ffff, 0000, 8000}, 24 dumps
       *
       * The second shape is the "fixed digital pattern" chased since v698. */

      if (priv->config->audio_std_mode == I2S_TDM_PCM)
        {
          modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                      I2S_RX_TDM_WS_WIDTH_M,
                      FIELD_TO_VALUE(I2S_RX_TDM_WS_WIDTH, 1));
        }
      else
        {
          modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                      I2S_RX_TDM_WS_WIDTH_M,
                      FIELD_TO_VALUE(I2S_RX_TDM_WS_WIDTH,
                      priv->data_width - 1));

          /* [V721] RX_LEFT_ALIGN has to be 0 for 16-bit Philips.
           *
           * soc/i2s_reg.h:166 -- I2S_RX_LEFT_ALIGN is RX_CONF bit 15,
           * reset default 1'b1, and i2s_configure() never writes it, so
           * this port has been running with it set since bring-up.
           *
           * The v721 sweep (every slot restored first, one knob per slot,
           * two independent runs) isolated it: this field is the ONLY
           * difference between a dead line and real audio.
           *
           *   LEFT_ALIGN = 1  rxconf 0x00089604  70 dumps, medDx 0,
           *                   corr 0.46, std 1, every word in {0,1,3}
           *   LEFT_ALIGN = 0  rxconf 0x00081604  64 dumps, medDx 20,
           *                   corr 0.634, std 42, smooth audio:
           *                   0000 06f2 0724 0702 0714 071e 073c 073c ...
           *                   03a8 0388 038a 0382 0388 0382 0398 03c0 ...
           *
           * Both runs agree, and the run-2 numbers (medDx 25, std 40)
           * track run 1, so this is not a one-off.
           *
           * TDM keeps the old behaviour: that branch is unused on this
           * port and left-align is meaningful there. */

          modifyreg32(I2S_RX_CONF_REG(priv->config->port),
                      I2S_RX_LEFT_ALIGN_M, 0);

          /* [V722] RX_MSB_SHIFT has to stay 1 (Philips).
           *
           * soc/i2s_reg.h -- I2S_RX_MSB_SHIFT is RX_CONF1 bit 29,
           * reset default 1'b1: the one-BCK delay between the WS edge
           * and the first data bit that the Philips standard requires.
           * i2s_configure() re-asserts it in its PHILIPS branch, but
           * the v717 diagnostic block used to clear it at the end of
           * the same call, and _v721b_boot.log prices the damage:
           *
           *   MSB_SHIFT = 1  conf1 0x2f3de38f  64 dumps  medDx 96
           *                  maxabs 624        real audio
           *   MSB_SHIFT = 0  conf1 0x0f3de38f  69 dumps  medDx 16384
           *                  maxabs 32768      {0000,4000,8000,c000}
           *
           * Writing it here puts the three framing fields this RX path
           * depends on -- WS_WIDTH, LEFT_ALIGN, MSB_SHIFT -- in one
           * place, so the correct setting survives even if the
           * diagnostic block is removed later. */

          modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
                      I2S_RX_MSB_SHIFT_M, I2S_RX_MSB_SHIFT);
        }
    }
#endif /* I2S_HAVE_RX */

  return priv->data_width;
}

/****************************************************************************
 * Name: i2s_set_clock
 *
 * Description:
 *   Set the I2S TX sample rate by adjusting I2S clock.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t i2s_set_clock(struct esp32s3_i2s_s *priv)
{
  uint32_t rate;
  uint32_t bclk;
  uint32_t mclk;
  uint32_t sclk;
  uint32_t mclk_div;
  int denominator;
  int numerator;
  uint32_t regval;
  uint32_t freq_diff;
  uint16_t bclk_div;

  /* TODO: provide other clock support modes */

  /* I2S module will using PLL_D2_CLK(160M) as source clock. */

  sclk = g_i2s_sclk;

  /* fmclk = bck_div * fbclk = fsclk / (mclk_div + b / a)
   * mclk_div is the I2S clock divider's integral value
   * b is the fraction clock divider's numerator value
   * a is the fraction clock divider's denominator value
   */

  if (priv->config->role == I2S_ROLE_MASTER)
    {
      bclk = priv->rate * priv->config->total_slot * priv->data_width;
      mclk = priv->mclk_freq;
      bclk_div = mclk / bclk;
    }
  else
    {
      /* For slave mode, mclk >= bclk * 8, so fix bclk_div to 2 first */

      bclk_div = 8;
      bclk = priv->rate * priv->config->total_slot * priv->data_width;
      mclk = bclk * bclk_div;
    }

  /* Calculate the nearest integer value of the I2S clock divider */

  mclk_div = sclk / mclk;

  i2sinfo("Clock division info: [sclk]%" PRIu32 " Hz [mdiv] %d "
          "[mclk] %" PRIu32 " Hz [bdiv] %d [bclk] %" PRIu32 " Hz\n",
          sclk, mclk_div, mclk, bclk_div, bclk);

  freq_diff = abs((int)sclk - (int)(mclk * mclk_div));

  denominator = 1;
  numerator = 0;

  if (freq_diff)
    {
      float decimal = freq_diff / (float)mclk;

      /* Carry bit if the decimal is greater than
       * 1.0 - 1.0 / (63.0 * 2) = 125.0 / 126.0
       */

      if (decimal > 125.0f / 126.0f)
        {
          mclk_div++;
        }
      else
        {
          uint32_t min = UINT32_MAX;

          for (int a = 2; a <= I2S_LL_MCLK_DIVIDER_MAX; a++)
            {
              int b = (int)(a * (freq_diff / (double)mclk) + 0.5);
              int ma = freq_diff * a;
              int mb = mclk * b;
              if (ma == mb)
                {
                  denominator = a;
                  numerator = b;
                  break;
                }

              if (abs((mb - ma)) < min)
                {
                  denominator = a;
                  numerator = b;
                  min = abs(mb - ma);
                }
            }
        }
    }

  i2sinfo("Clock register: [mclk] %" PRIu32 " Hz [numerator] %d "
          "[denominator] %d\n", mclk, numerator, denominator);

  /* Setup TX Clock */

  regval = getreg32(I2S_TX_CLKM_CONF_REG(priv->config->port));
  regval &= ~I2S_TX_CLKM_DIV_NUM_M;
  regval |= FIELD_TO_VALUE(I2S_TX_CLKM_DIV_NUM, mclk_div);
  putreg32(regval, I2S_TX_CLKM_CONF_REG(priv->config->port));

  regval = getreg32(I2S_TX_CLKM_DIV_CONF_REG(priv->config->port));
  regval &= ~(I2S_TX_CLKM_DIV_Z_M | I2S_TX_CLKM_DIV_Y_M |
              I2S_TX_CLKM_DIV_X_M | I2S_TX_CLKM_DIV_YN1_M);
  regval |= FIELD_TO_VALUE(I2S_TX_CLKM_DIV_Z,
                           numerator <= (denominator / 2) ?
                           numerator : (denominator - numerator));

  regval |= FIELD_TO_VALUE(I2S_TX_CLKM_DIV_Y,
                           numerator <= (denominator / 2) ?
                           (denominator % numerator) :
                           (denominator % (denominator - numerator)));

  regval |= FIELD_TO_VALUE(I2S_TX_CLKM_DIV_X,
                           numerator <= (denominator / 2) ?
                           (int)(floor(denominator / numerator) - 1) :
                           (int)(floor(denominator /
                                       (denominator - numerator))-1));

  regval |= FIELD_TO_VALUE(I2S_TX_CLKM_DIV_YN1,
                           numerator <= (denominator / 2) ?
                           0 : 1);
  putreg32(regval, I2S_TX_CLKM_DIV_CONF_REG(priv->config->port));

  /* Setup RX Clock */

  regval = getreg32(I2S_RX_CLKM_CONF_REG(priv->config->port));
  regval &= ~I2S_RX_CLKM_DIV_NUM_M;
  regval |= FIELD_TO_VALUE(I2S_RX_CLKM_DIV_NUM, mclk_div);
  putreg32(regval, I2S_RX_CLKM_CONF_REG(priv->config->port));

  regval = getreg32(I2S_RX_CLKM_DIV_CONF_REG(priv->config->port));
  regval &= ~(I2S_RX_CLKM_DIV_Z_M | I2S_RX_CLKM_DIV_Y_M |
              I2S_RX_CLKM_DIV_X_M | I2S_RX_CLKM_DIV_YN1_M);
  regval |= FIELD_TO_VALUE(I2S_RX_CLKM_DIV_Z,
                           numerator <= (denominator / 2) ?
                           numerator : (denominator - numerator));

  regval |= FIELD_TO_VALUE(I2S_RX_CLKM_DIV_Y,
                           numerator <= (denominator / 2) ?
                           (denominator % numerator) :
                           (denominator % (denominator - numerator)));

  regval |= FIELD_TO_VALUE(I2S_RX_CLKM_DIV_X,
                           numerator <= (denominator / 2) ?
                           (int)(floor(denominator / numerator) - 1) :
                           (int)(floor(denominator /
                                       (denominator - numerator))-1));

  regval |= FIELD_TO_VALUE(I2S_RX_CLKM_DIV_YN1,
                           numerator <= (denominator / 2) ?
                           0 : 1);
  putreg32(regval, I2S_RX_CLKM_DIV_CONF_REG(priv->config->port));

  /* Set I2S TX bck div num
   * Note: I2S_TX/RX_BCK_DIV_NUM cannot be 1
   */

#ifdef I2S_HAVE_TX
  modifyreg32(I2S_TX_CONF1_REG(priv->config->port),
              I2S_TX_BCK_DIV_NUM_M,
              FIELD_TO_VALUE(I2S_TX_BCK_DIV_NUM, bclk_div - 1));

  /* Set I2S_TX_UPDATE bit to update the configs.
   * This bit is automatically cleared.
   */

  modifyreg32(I2S_TX_CONF_REG(priv->config->port),
              0, I2S_TX_UPDATE);
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  modifyreg32(I2S_RX_CONF1_REG(priv->config->port),
              I2S_RX_BCK_DIV_NUM_M,
              FIELD_TO_VALUE(I2S_RX_BCK_DIV_NUM, bclk_div - 1));

  /* Set I2S_RX_UPDATE bit to update the configs.
   * This bit is automatically cleared.
   */

  modifyreg32(I2S_RX_CONF_REG(priv->config->port),
              0, I2S_RX_UPDATE);
#endif /* I2S_HAVE_RX */

  /* Returns the actual sample rate */

  bclk = sclk / (float)((mclk_div + numerator / (float)denominator) *
                        bclk_div);
  rate = bclk / (float)(priv->config->total_slot * priv->data_width);

  return rate;
}

/****************************************************************************
 * Name: i2s_tx_channel_start
 *
 * Description:
 *   Start TX channel for the I2S port
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
/* [V337] 无论 tx_started 与否都先应用 GDMA strategy。
 *
 * 实测 `[I2SREG] #1 ch=1 txirq=77 tx_started=1` —— 本函数一进来 tx_started
 * 就已是 1，所以下面的早退会把它后面所有的 GDMA 配置（含 V251 的 strategy
 * 与 V336 追加的那次）全部跳过，DMA_OUT_CONF0 因此恒为复位值 0x00000008
 * （无 auto_wrback / 无 burst / 无 owner_check / PRI=0）。
 *
 * 而 V251 注释已写明：out_auto_wrback=0 时控制器发完不清 OWNER 位、
 * outlink FSM 无法前进 —— 正是 DSCR 恒 0、FIFO 不排空的成因。
 *
 * 因此把 strategy 提到最前面：先保证它一定生效，再走原有的早退逻辑。 */
/**
 * Name: i2s_tx_rate_tick
 *
 * Description:
 *   [V362] Emit one aggregate summary line every 5 seconds describing
 *   how often i2s_txdma_start() was entered and how many of those
 *   entries were the *idle* kind (nothing active, nothing queued).
 *
 *   A healthy stream shows starts >= 1 and bail_pend == 0.
 *   The failure this instrumentation was written to prove shows
 *   bail_pend in the thousands with starts == 0, and the s1/s2 split
 *   then names the caller that is spinning.
 *
 *   Cost control: clock_gettime() is only reached once every 64 calls;
 *   every other call returns after a counter increment, so a kHz-rate
 *   idle spin cannot turn this into a second CPU hog.
 */

/**
 * Name: i2s_tx_irq_mask / i2s_tx_irq_unmask
 *
 * Description:
 *   [V364] Gate the three GDMA TX completion interrupt sources
 *   (OUT_DONE, OUT_EOF, OUT_TOTAL_EOF) instead of letting them fire into
 *   an idle driver.
 *
 *   Rationale: V362 proved from the code structure that every idle entry
 *   into i2s_txdma_start() comes from i2s_tx_schedule(), which the GDMA
 *   interrupt handler calls as soon as the outlink raises an EOF.  When
 *   the last buffer has already been reported the queues are empty and
 *   the interrupt has nothing left to report, yet nothing stopped it
 *   from being raised again.  V362 made each entry cheap; this stops the
 *   entries.
 *
 *   OUT_DSCR_ERR is deliberately left enabled in both directions -- it is
 *   an error source, not a completion source, and we want to see it.
 *
 *   Both helpers drop the corresponding INT_ST/INT_RAW bits first.  The
 *   descriptor pool is recycled, so a stale RAW bit that survived the
 *   gate could otherwise fire the instant we re-enable and, because the
 *   address matches, be accepted by i2s_tx_schedule() as the completion
 *   of the buffer that was just queued.
 */

static void i2s_tx_irq_mask(struct esp32s3_i2s_s *priv)
{
  SET_GDMA_CH_REG(DMA_OUT_INT_CLR_CH0_REG, priv->dma_channel,
                  DMA_OUT_TOTAL_EOF_CH0_INT_ST |
                  DMA_OUT_EOF_CH0_INT_ST |
                  DMA_OUT_DONE_CH0_INT_ST);

  CLR_GDMA_CH_BITS(DMA_OUT_INT_ENA_CH0_REG, priv->dma_channel,
                   DMA_OUT_TOTAL_EOF_CH0_INT_ENA |
                   DMA_OUT_EOF_CH0_INT_ENA |
                   DMA_OUT_DONE_CH0_INT_ENA);

  g_tx_irq_mask_n++;

  if (g_tx_irq_mask_n <= 3)
    {
      
    }
}

static void i2s_tx_irq_unmask(struct esp32s3_i2s_s *priv)
{
  SET_GDMA_CH_REG(DMA_OUT_INT_CLR_CH0_REG, priv->dma_channel,
                  DMA_OUT_TOTAL_EOF_CH0_INT_ST |
                  DMA_OUT_EOF_CH0_INT_ST |
                  DMA_OUT_DONE_CH0_INT_ST);

  SET_GDMA_CH_BITS(DMA_OUT_INT_ENA_CH0_REG, priv->dma_channel,
                   DMA_OUT_TOTAL_EOF_CH0_INT_ENA |
                   DMA_OUT_EOF_CH0_INT_ENA |
                   DMA_OUT_DONE_CH0_INT_ENA);

  g_tx_irq_unmask_n++;

  if (g_tx_irq_unmask_n <= 3)
    {
      
    }
}

static void i2s_tx_rate_tick(void)
{
  struct timespec ts;
  int64_t now_ms;

  if ((++g_tx_rate_skip & 0x3f) != 0)
    {
      return;
    }

  ts.tv_sec = 0;
  ts.tv_nsec = 0;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
      return;
    }

  now_ms = ((int64_t)ts.tv_sec * 1000) + (ts.tv_nsec / 1000000);

  if (g_tx_rate_next_ms == 0)
    {
      g_tx_rate_next_ms = now_ms + 5000;
      return;
    }

  if (now_ms < g_tx_rate_next_ms)
    {
      return;
    }

  g_tx_rate_next_ms = now_ms + 5000;

  
}

static void i2s_apply_tx_gdma_strategy(struct esp32s3_i2s_s *priv)
{
  SET_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, priv->dma_channel,
                   DMA_OUT_DATA_BURST_EN_CH0_M |
                   DMA_OUTDSCR_BURST_EN_CH0_M |
                   DMA_OUT_AUTO_WRBACK_CH0_M);
  CLR_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, priv->dma_channel,
                   DMA_OUT_EOF_MODE_CH0);
  SET_GDMA_CH_BITS(DMA_OUT_CONF1_CH0_REG, priv->dma_channel,
                   DMA_OUT_CHECK_OWNER_CH0_M);
  SET_GDMA_CH_REG(DMA_OUT_PRI_CH0_REG, priv->dma_channel, 1);

  
}

static void i2s_tx_channel_start(struct esp32s3_i2s_s *priv)
{
  if (priv->config->tx_en)
    {
      /* [V337] 早退之前先落 strategy（原来它在早退之后，永远执行不到） */

      i2s_apply_tx_gdma_strategy(priv);

      if (priv->tx_started)
        {
          /* [V347] Team-lead hypothesis probe.  If this ever fires *per
           * transfer* it would explain a missing I2S_TX_START; keep a counter
           * and a log so the trace says so unambiguously.  (Note: this
           * function is only entered from i2s_start()/i2s_stop()/ioctl and the
           * RAW one-shot path -- the DMA start path that actually writes
           * I2S_TX_START is i2s_txdma_start(), instrumented separately.) */

          g_tx_channel_start_er++;
          

          i2swarn("TX channel of port %d was previously started\n",
                  priv->config->port);
          return;
        }

      /* Reset the DMA operation */

      SET_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, priv->dma_channel,
                       DMA_OUT_RST_CH0_M);
      CLR_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, priv->dma_channel,
                       DMA_OUT_RST_CH0_M);
      CLR_GDMA_CH_BITS(DMA_OUT_CONF1_CH0_REG, priv->dma_channel,
                       DMA_OUT_CHECK_OWNER_CH0);

      /* [V251] Mirror ESP-IDF's AHB-GDMA TX channel configuration exactly.
       *
       * ESP-IDF builds every AHB channel through gdma_hal_ahb_v1.c, which
       * calls gdma_ll_tx_enable_data_burst(), gdma_ll_tx_enable_descriptor_burst(),
       * gdma_ll_tx_enable_owner_check() and gdma_ll_tx_set_priority() before
       * it ever sets the descriptor address.  NuttX leaves all four at their
       * reset values:
       *
       *   DMA_OUT_CONF0 (0x60) bit4 OUTDSCR_BURST_EN  default 0
       *   DMA_OUT_CONF0 (0x60) bit5 OUT_DATA_BURST_EN default 0
       *   DMA_OUT_CONF1 (0x64) bit12 OUT_CHECK_OWNER  default 0
       *   DMA_OUT_PRI   (0xa4) OUT_PRI                default 0
       *
       * Measured before this change: DMA_OUT_CONF0 = 0x00000008, i.e. only
       * out_eof_mode (bit3, a reset default) -- no burst of either kind and
       * the lowest possible priority.  With the outlink FSM confirmed running
       * (PARK=0) the channel still will not pull the first descriptor, which
       * is what an unburstable, zero-priority AHB master looks like.
       *
       * out_check_owner makes the GDMA verify descriptor ownership before use;
       * desc0.ctrl reads 0xc0fe0fe0, whose bit31 OWNER is 1, so the check
       * passes.
       *
       * [V252] *** OUT_AUTO_WRBACK and OUT_EOF_MODE -- the last two ***
       *
       * ESP-IDF's gdma_ahb_hal_set_strategy() writes THREE things on the TX
       * path:
       *
       *   gdma_ll_tx_enable_owner_check(dev, chan, en_owner_check);
       *   gdma_ll_tx_enable_auto_write_back(dev, chan, en_desc_write_back);
       *   gdma_ll_tx_set_eof_mode(dev, chan, eof_till_popped);
       *
       * and the I2S driver reaches it with GDMA_STRATEGY_FORWARD_DSCR and
       * no_auto_update = false, i.e. owner_check = TRUE, desc_write_back =
       * TRUE, eof_till_popped = FALSE.  NuttX, which drives the channel by
       * hand, sets none of the three.  out_auto_wrback (conf0 bit 2) and
       * out_eof_mode (conf0 bit 3) therefore sit at their reset values 0 and 1.
       *
       * out_auto_wrback is the one that matters: with it clear the controller
       * never writes the descriptor back after transmission.  It does not
       * clear the OWNER bit, does not store the transferred length, and the
       * outlink FSM consequently cannot advance past the first entry.  That is
       * exactly what the hardware reports:
       *
       *   FSM: DSCR=0x00000000 OUTFIFO=0x00408415 STATE=0x002f9db8
       *        CONF0=0x00000038 CONF1=0x00001000 PRI=0x00000001
       *
       * STATE holds the outlink descriptor address (low 18 bits 0x39db8 ==
       * 0x3fcb9db8) but DMA_OUT_DSCR, the descriptor the FSM is actually
       * working on, is still zero; the first descriptor's OWNER bit stays 1
       * across the whole write; no SUC_EOF is ever raised and txirqN stays 0.
       *
       * Set both to ESP-IDF's values. */

      SET_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, priv->dma_channel,
                       DMA_OUT_DATA_BURST_EN_CH0_M |
                       DMA_OUTDSCR_BURST_EN_CH0_M |
                       DMA_OUT_AUTO_WRBACK_CH0_M);
      CLR_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, priv->dma_channel,
                       DMA_OUT_EOF_MODE_CH0);
      SET_GDMA_CH_BITS(DMA_OUT_CONF1_CH0_REG, priv->dma_channel,
                       DMA_OUT_CHECK_OWNER_CH0_M);
      SET_GDMA_CH_REG(DMA_OUT_PRI_CH0_REG, priv->dma_channel, 1);

      /* Reset the TX channel */

      modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_TX_RESET);
      modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_RESET, 0);

      /* Reset TX FIFO */

      modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_TX_FIFO_RESET);
      modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_FIFO_RESET, 0);

      g_v671_cfg_valid = 0;   /* [V671] TX channel (re)started */

      /* Set I2S_RX_UPDATE bit to update the configs.
       * This bit is automatically cleared.
       */

      modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_TX_UPDATE);

      /* [V336] ★ 在 I2S 复位之后**重新应用** GDMA strategy。
       *
       * 实测 CONF0 一直是 0x00000008（只有 out_eof_mode），说明上面那次
       * V251 写入被紧随其后的 I2S_TX_RESET/FIFO_RESET 冲掉了 —— 而
       * out_auto_wrback 为 0 会让控制器发完不清 OWNER 位、outlink FSM
       * 无法前进（DSCR 恒为 0、FIFO 不排空、完成中断不来），
       * 这就是喇叭"TX DMA 从不排空"的根因。
       *
       * 顺序修正：复位完成后重新写一遍，确保它留得住。 */

      SET_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, priv->dma_channel,
                       DMA_OUT_DATA_BURST_EN_CH0_M |
                       DMA_OUTDSCR_BURST_EN_CH0_M |
                       DMA_OUT_AUTO_WRBACK_CH0_M);
      CLR_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, priv->dma_channel,
                       DMA_OUT_EOF_MODE_CH0);
      SET_GDMA_CH_BITS(DMA_OUT_CONF1_CH0_REG, priv->dma_channel,
                       DMA_OUT_CHECK_OWNER_CH0_M);
      SET_GDMA_CH_REG(DMA_OUT_PRI_CH0_REG, priv->dma_channel, 1);

      


      /* [V221] Wait for tx_update to self-clear, exactly as ESP-IDF's
       * i2s_ll_tx_start() does.  tx_update hands the APB-domain configuration
       * to the I2S TX clock domain and clears itself on completion.  If it
       * never clears the configuration never reaches the clock domain, so
       * tx_start has no effect there, the I2S never raises its DMA request and
       * the GDMA outlink parks without fetching a single descriptor.  The
       * measurement was inconsistent across boots (0 spins vs a full timeout),
       * which points at the source clock rather than the configuration. */
      {
        int spins = 0;

        while (spins < 100000 &&
               (getreg32(I2S_TX_CONF_REG(priv->config->port)) & I2S_TX_UPDATE))
          {
            spins++;
          }

        g_tx_update_spins = spins;
      }


      /* [V217] Keep BCK/WS running when the TX FIFO is empty.
       *
       * tx_stop_en (tx_conf bit 13) gates the bit clock off as soon as the TX
       * FIFO drains.  With the FIFO empty that also stops the I2S from
       * asserting its DMA request, so the GDMA outlink sits parked with the
       * very first descriptor untouched -- which is precisely what we
       * measured (START written then cleared, PARK toggling, OWNER never
       * cleared, INT_ST and the interrupt count permanently zero).  Clearing
       * it lets the clock run continuously so the FIFO can drain and the DMA
       * request can be raised. */
#ifdef I2S_TX_STOP_EN
      modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_STOP_EN, 0);
#endif
      modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_TX_UPDATE);



      /* Enable DMA interrupt */

      up_enable_irq(priv->tx_irq);

      /* [V342] Enable every TX completion event, not just OUT_TOTAL_EOF.
       * OUT_EOF (SUC_EOF) is the event that actually populates
       * DMA_OUT_EOF_DES_ADDR and is what the RX side of this driver uses;
       * OUT_DONE covers the per-descriptor case.  Leaving them disabled meant
       * the only enabled events were OUT_TOTAL_EOF and OUT_DSCR_ERR, so a
       * transfer that ended in SUC_EOF never produced the completion
       * callback -- exactly the "[audio_direct] I2S TX completion TIMEOUT". */

      SET_GDMA_CH_BITS(DMA_OUT_INT_ENA_CH0_REG, priv->dma_channel,
                       DMA_OUT_TOTAL_EOF_CH0_INT_ENA |
                       DMA_OUT_EOF_CH0_INT_ENA |
                       DMA_OUT_DONE_CH0_INT_ENA |
                       DMA_OUT_DSCR_ERR_CH0_INT_ENA);

      /* REVISIT: enable I2S interrupt and set the FIFO hung counter togit
       * enable the `I2S_TX_HUNG_INT` and check for data timeout.
       */

      /* [V346] A fresh channel activation must re-run the one-time
       * configure/reset/start sequence on its first transfer. */

      g_tx_cfg_done = 0;

      priv->tx_started = true;

      i2sinfo("Started TX channel of port %d\n", priv->config->port);
    }
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_rx_channel_start
 *
 * Description:
 *   Start RX channel for the I2S port
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static void i2s_rx_channel_start(struct esp32s3_i2s_s *priv)
{
  if (priv->config->rx_en)
    {
      if (priv->rx_started)
        {
          i2swarn("RX channel of port %d was previously started\n",
                  priv->config->port);
          return;
        }

      /* Reset the DMA operation */

      SET_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, priv->dma_channel,
                       DMA_IN_RST_CH0_M);
      CLR_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, priv->dma_channel,
                       DMA_IN_RST_CH0_M);

      /* [RX-FIX-2] Program the GDMA IN channel the way the TX side already
       * is (see the V251/V252 strategy sequence in i2s_tx_channel_start).
       * NuttX left it at its reset value: no burst of either kind, no owner
       * check, priority 0.  The symptom here is the same one that sequence
       * fixed for TX -- the RX DMA reports "armed" ([AUDIO-R42] xfer ret=0)
       * yet never raises SUC_EOF, so the native capture path starves and
       * audio_capture_native gives up with -EIO after six empty waits. */
      SET_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, priv->dma_channel,
                       DMA_IN_DATA_BURST_EN_CH0_M |
                       DMA_INDSCR_BURST_EN_CH0_M);
      SET_GDMA_CH_BITS(DMA_IN_CONF1_CH0_REG, priv->dma_channel,
                       DMA_IN_CHECK_OWNER_CH0_M);
      SET_GDMA_CH_REG(DMA_IN_PRI_CH0_REG, priv->dma_channel, 1);

      /* Reset the RX channel */

      modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0, I2S_RX_RESET);
      modifyreg32(I2S_RX_CONF_REG(priv->config->port), I2S_RX_RESET, 0);

      /* [V390] A/B: choose where the receiver takes its bit clock from.
       * i2s_configure() forces I2S_RX_SLAVE_MOD whenever the transmitter
       * owns the port, which is the case here, so the RX module runs off
       * I2S0I_BCK_IN / I2S0I_WS_IN.  Flip it to master to test whether
       * that input path is the reason the receiver never asks the GDMA
       * for anything.  The pads keep the TX generator's clock either way,
       * so the codec stays clocked in both arms of the test.
       */

      if (g_v390_rxmaster)
        {
          modifyreg32(I2S_RX_CONF_REG(priv->config->port),
                      I2S_RX_SLAVE_MOD, 0);
        }
      else
        {
          modifyreg32(I2S_RX_CONF_REG(priv->config->port),
                      0, I2S_RX_SLAVE_MOD);
        }

      /* [V727] *** The receiver has to be a slave.  Insist on it. ***
       *
       * I2S_RX_SLAVE_MOD is RX_CONF bit 3 (soc/i2s_reg.h:220, default
       * 1'b0) and it measured 0 in the steady state (rxconf =
       * 0x00081604) even though the transmitter owns the pads here:
       * [V382] runs it as a clock generator, [V386] loops BCK and WS
       * back into I2S0I_BCK_IN / I2S0I_WS_IN, and the [V390] block
       * above asks for slave mode whenever g_v390_rxmaster is 0.
       * Something clears the bit again afterwards.
       *
       * With the bit clear the receiver generates its own frame.  The
       * FREQUENCY is right -- 178 buffers in 11.52 s at 1023 frames per
       * buffer is 15857 frames/s, and BCK measures 512 kHz -- but the
       * PHASE is free with respect to the ES8311, so every capture is
       * the real signal shifted by an arbitrary number of bits and the
       * shift wanders:
       *
       *   {0000,8000}               or=0x0000000f, app peak 15
       *   {0000,4000,8000,c000}     (v721b)
       *
       * Rewriting RX_CONF re-locks the receiver onto a new arbitrary
       * offset each time (peak 1112..2188 once, 10176..16383 the next),
       * which is what made this look like a race for so long.
       *
       * Measured with the v726 live hook, one write, no reflash
       * (_work\\_v726_sweep3.log):
       *
       *   40..65 s  SLAVE_MOD = 0   or=0x0000000f   peak 15         dead
       *   70..150 s SLAVE_MOD = 1   or=0x0000ffff   peak 115..1845  real,
       *                             full 16 bits, stable the whole
       *                             80-second window
       *   160..190  SLAVE_MOD = 0   or=0x0000ffc0 -> 0x000003ff
       *                             peak 7360..32768 then 5..1023   broken
       *
       * The [V390] A/B above is untouched, so flipping
       * g_v390_rxmaster still reaches this line; this just makes the
       * outcome unconditional, which is what the port needs. */

      modifyreg32(I2S_RX_CONF_REG(priv->config->port),
                  0, I2S_RX_SLAVE_MOD);

      /* Reset RX FIFO */

      modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0, I2S_RX_FIFO_RESET);
      modifyreg32(I2S_RX_CONF_REG(priv->config->port), I2S_RX_FIFO_RESET, 0);

      /* Set I2S_RX_UPDATE bit to update the configs.
       * This bit is automatically cleared.
       */

      modifyreg32(I2S_RX_CONF_REG(priv->config->port), 0, I2S_RX_UPDATE);

      /* [RX-FIX-1] Wait for rx_update to self-clear, exactly as the TX path
       * (V221) and ESP-IDF's i2s_ll_rx_start() do.  rx_update hands the
       * APB-domain RX configuration to the I2S RX clock domain and clears
       * itself on completion.  If it never clears, the configuration never
       * reaches the clock domain, the RX never starts, the I2S never raises
       * its DMA request and the capture path starves -- exactly what
       * audio_capture_native reports as "capture EIO" after 6 s of empty
       * waits.  The RX path previously had no such handshake at all. */
      {
        int spins = 0;

        while (spins < I2S_UPDATE_SPIN_MAX &&
               (getreg32(I2S_RX_CONF_REG(priv->config->port)) & I2S_RX_UPDATE))
          {
            spins++;
          }

        g_rx_update_spins = spins;
        /* [V394] IRQ-stack: silenced */ if (0) syslog(LOG_INFO,
               "[V390] rx role=%s conf=0x%08x\n",
               g_v390_rxmaster ? "MASTER" : "SLAVE",
               (unsigned)getreg32(I2S_RX_CONF_REG(priv->config->port)));
        /* [V394] IRQ-stack: silenced */ if (0) syslog(LOG_INFO,
               "[RXCLK] rx_update spins=%d conf=0x%08x rate=%u\n",
               spins,
               (unsigned)getreg32(I2S_RX_CONF_REG(priv->config->port)),
               (unsigned)priv->rate);
      }

      /* Enable DMA interrupt */

      up_enable_irq(priv->rx_irq);

      SET_GDMA_CH_BITS(DMA_IN_INT_ENA_CH0_REG, priv->dma_channel,
                       DMA_IN_SUC_EOF_CH0_INT_ENA);

      /* REVISIT: enable I2S interrupt and set the FIFO hung counter to
       * enable the `I2S_RX_HUNG_INT` and check for data timeout.
       */

      /* [V382] Run the TX clock generator while a capture streams.
       *
       * The receiver on this port is a slave to the transmitter's
       * BCK/WS, and the transmitter only shifts while it has something
       * to shift.  With TX stopped there is no frame clock on the pins,
       * the ES8311 never puts a bit on DIN and the RX GDMA waits for
       * ever -- logged as [RXDUMP] ... isr=0 eof=0 and reported by
       * audio_capture_native as "capture EIO".
       *
       * Feeding the transmitter from a second es8311 instance
       * (the V373/V378/V381 clock keeper) does produce a clock but
       * panics the board the moment its TX DMA starts next to a running
       * capture.  So start the generator on its own instead: with the
       * TX DMA idle the transmitter underruns and shifts zeros, which
       * is inaudible but keeps BCLK and WS alive.  One instance, no
       * second thread, no DMA restart, nothing to deadlock.
       */

      if (!(getreg32(I2S_TX_CONF_REG(priv->config->port)) & I2S_TX_START))
        {
          int spins;

          /* [V725] *** The clock generator has to use the frame playback
           * uses, not the reset default. ***
           *
           * The receiver here is a slave to this transmitter's BCK/WS
           * ([V382] below), so whatever the TX side does to the frame is
           * what the ES8311's ADC and the RX shifter both see.  Until
           * the first transmission nothing has configured that frame,
           * and the hardware reset default is
           *
           *     TX_CONF     bit19 TX_TDM_EN = 1   (TDM mode is on)
           *     TX_TDM_CTRL 0x0000ffff            (all 16 CHANn_EN)
           *
           * The [V724] register diff caught exactly that: across the
           * first playback window only two registers moved --
           *
           *     TX_CONF 0x00089204 -> 0x00089264   (MONO + CHAN_EQUAL)
           *     TX_TDM  0x0000ffff -> 0x00010001
           *
           * -- and the rx stream's effective bit depth changed class at
           * the same instant.  The values on the right are the ones
           * i2s_txdma_start() writes on every tx buffer (~1496-1548);
           * the values on the left are what the clock-only window was
           * left with.
           *
           * So replay that same configuration here, while the
           * transmitter is still stopped, and the clock-only window
           * produces the identical frame.  Nothing on the playback path
           * changes.
           */

          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_MONO_M | I2S_TX_CHAN_EQUAL_M |
                      I2S_TX_LEFT_ALIGN_M | I2S_TX_STOP_EN_M,
                      0);

          {
            uint32_t v725tdm = FIELD_TO_VALUE(I2S_TX_TDM_TOT_CHAN_NUM, 1) |
                               I2S_TX_TDM_CHAN0_EN;

            if (!g_i2s_tx_mono)
              {
                v725tdm |= I2S_TX_TDM_CHAN1_EN;
              }

            putreg32(v725tdm, I2S_TX_TDM_CTRL_REG(priv->config->port));
          }

          putreg32(0x004aa004u,
                   I2S_TX_PCM2PDM_CONF_REG(priv->config->port));

          modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0,
                      I2S_TX_START);
          modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0,
                      I2S_TX_UPDATE);

          for (spins = 0;
               spins < I2S_UPDATE_SPIN_MAX &&
               (getreg32(I2S_TX_CONF_REG(priv->config->port)) &
                I2S_TX_UPDATE);
               spins++)
            {
            }

          /* [V394] IRQ-stack: silenced */ if (0) syslog(LOG_INFO, "[V382] tx clock-only start spins=%d "
                 "txconf=0x%08x\n",
                 spins,
                 (unsigned)getreg32(I2S_TX_CONF_REG(priv->config->port)));
        }

      /* [V383] Are the clocks and the codec's data actually on the pins?
       *
       * The RX register dump keeps showing i2sintst=0 / gdmainst=0, i.e.
       * not a single frame reached the receiver.  That is equally
       * consistent with "no BCLK" and with "BCLK is fine but the ES8311
       * is silent".  The pins are driven through the GPIO matrix, so
       * enabling only the pad input buffer makes the level readable
       * (same trick the [V244] block in esp32s3_i2s_tx_write_raw uses).
       *
       *   BCLK(10) hi ~= 2000/4000  -> the frame clock is running
       *   DIN(9)   hi ~= 2000/4000  -> the codec is shifting data at us
       *   either extreme (0 or 4000) -> that line is stuck
       */

      {
        static int s_v383_n;

        if (s_v383_n < 8)
          {
            unsigned int bhi = 0;
            unsigned int whi = 0;
            unsigned int dhi = 0;
            unsigned int k;

            s_v383_n++;

            modifyreg32(IO_MUX_GPIO8_REG,  0, (1u << 9));   /* WS   */
            modifyreg32(IO_MUX_GPIO9_REG,  0, (1u << 9));   /* DIN  */
            modifyreg32(IO_MUX_GPIO10_REG, 0, (1u << 9));   /* BCLK */

            for (k = 0; k < 4000; k++)
              {
                if (esp32s3_gpioread(10))
                  {
                    bhi++;
                  }

                if (esp32s3_gpioread(8))
                  {
                    whi++;
                  }

                if (esp32s3_gpioread(9))
                  {
                    dhi++;
                  }
              }

            /* [V394] IRQ-stack: silenced */ if (0) syslog(LOG_INFO, "[V383] pins BCLK(10) hi=%u WS(8) hi=%u "
                   "DIN(9) hi=%u /4000\n", bhi, whi, dhi);
          }
      }

      priv->rx_started = true;

      i2sinfo("Started RX channel of port %d\n", priv->config->port);
    }
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: i2s_tx_channel_stop
 *
 * Description:
 *   Stop TX channel for the I2S port
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static void i2s_tx_channel_stop(struct esp32s3_i2s_s *priv)
{
  if (priv->config->tx_en)
    {
      if (!priv->tx_started)
        {
          i2swarn("TX channel of port %d was previously stopped\n",
                  priv->config->port);
          return;
        }

      /* Stop TX channel */

      modifyreg32(I2S_TX_CONF_REG(priv->config->port), I2S_TX_START, 0);

      g_v671_cfg_valid = 0;   /* [V671] transmitter stopped */

      /* Stop outlink */

      SET_GDMA_CH_BITS(DMA_OUT_LINK_CH0_REG, priv->dma_channel,
                       DMA_OUTLINK_STOP_CH0_M);

      /* Disable DMA interrupt */

      /* [V342] Disable the same set that i2s_tx_channel_start() enables. */

      CLR_GDMA_CH_BITS(DMA_OUT_INT_ENA_CH0_REG, priv->dma_channel,
                       DMA_OUT_TOTAL_EOF_CH0_INT_ENA |
                       DMA_OUT_EOF_CH0_INT_ENA |
                       DMA_OUT_DONE_CH0_INT_ENA |
                       DMA_OUT_DSCR_ERR_CH0_INT_ENA);

      /* Disable DMA operation mode */

      up_disable_irq(priv->tx_irq);

      priv->tx_started = false;
      g_tx_cfg_done    = 0;   /* [V346] force reconfig on next activation */

      
    }
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_rx_channel_stop
 *
 * Description:
 *   Stop RX channel for the I2S port
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
/* [V697] *** Return every RX buffer that is still in flight. ***
 *
 * Called from the capture client's teardown, right after AUDIOIOC_STOP and
 * BEFORE it frees the ap_buffers, because the alternative is what killed the
 * board: the abandoned container keeps a pointer to an apb the client is
 * about to release, and the next DMA or worker write lands on the heap
 * header that sits directly above the apb's payload.
 *
 * Deliberately silent towards the client: es8311_processdone() asserts
 * priv->running, which is false during a teardown, so handing these buffers
 * to the worker would turn a leak into a panic.  Dropping the driver's own
 * reference here is enough -- the client's reference is what
 * AUDIOIOC_FREEBUFFER / native_free_buffer() releases afterwards.
 */

static int i2s_rx_release_inflight(struct esp32s3_i2s_s *priv)
{
  struct esp32s3_buffer_s *bfcontainer;
  struct esp32s3_dmadesc_s *bfdesc;
  irqstate_t flags;
  int n = 0;
  int k;

  for (;;)
    {
      flags = spin_lock_irqsave(&priv->slock);

      bfcontainer = (struct esp32s3_buffer_s *)sq_remfirst(&priv->rx.act);

      if (bfcontainer == NULL)
        {
          bfcontainer = (struct esp32s3_buffer_s *)sq_remfirst(&priv->rx.pend);
        }

      if (bfcontainer == NULL)
        {
          bfcontainer = (struct esp32s3_buffer_s *)sq_remfirst(&priv->rx.done);
        }

      spin_unlock_irqrestore(&priv->slock, flags);

      if (bfcontainer == NULL)
        {
          break;
        }

      /* Nothing was received into it. */

      bfcontainer->apb->nbytes  = 0;
      bfcontainer->apb->curbyte = 0;
      bfcontainer->result       = -EAGAIN;

      /* Clear the EOF marks.  i2s_rx_worker() sums DATALEN over the
       * descriptors that carry EOF, and a stale mark would make it count
       * (and copy) bytes the DMA never wrote. */

      bfdesc = bfcontainer->dma_link;

      for (k = 0; k < I2S_DMADESC_NUM && bfdesc != NULL; k++)
        {
          bfdesc->ctrl &= ~ESP32S3_DMA_CTRL_EOF;
          bfdesc = bfdesc->next;
        }

      /* Drop our reference on the audio buffer, then hand the container
       * (and the internal-SRAM bounce slot it holds -- i2s_buf_free()
       * releases that itself) back to their pools. */

      apb_free(bfcontainer->apb);
      i2s_buf_free(priv, bfcontainer);

      n++;
    }

  return n;
}

void esp32s3_i2s_rx_flush(void)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)g_v690_priv;
  int n;

  if (priv == NULL || !priv->config->rx_en)
    {
      return;
    }

  /* Stop the receiver FIRST.  Releasing a bounce slot the GDMA is still
   * writing into would hand it to the next arm while the old transfer is
   * still landing there. */

  i2s_rx_channel_stop(priv);

  n = i2s_rx_release_inflight(priv);

  syslog(LOG_INFO, "[V697] rx flush: %d in-flight buffer(s) released\n", n);
}

/****************************************************************************
 * Name: i2s_tx_release_inflight
 *
 * Description:
 *   [V755] TX twin of i2s_rx_release_inflight().  Give back every buffer
 *   container stranded in the three TX queues, together with the
 *   internal-SRAM bounce slot each one holds.
 *
 *   *** Why this is NOT a copy of the RX version ***
 *
 *   On the RX side i2s_receive() takes its own apb_reference() and
 *   i2s_rx_release_inflight() therefore drops it with apb_free().
 *   On the TX side i2s_txdma_setup() already called apb_free() at the
 *   end of the setup, so by the time a container sits in tx.act the
 *   apb is kept alive ONLY by the es8311 upper half's own reference.
 *   Ownership must go back through es8311_processdone(), which moves
 *   it to priv->doneq and decrements inflight; es8311_returnbuffers()
 *   then drops the last reference.
 *
 *   Calling apb_free() here as well would double-free -- the very same
 *   class of bug v753 created by zeroing priv->inflight on the RX side
 *   (it panicked the board on its first real execution, EXCCAUSE=001d).
 *   Skipping the callback instead would leave inflight stuck above
 *   zero, so the es8311 worker could never exit and would be
 *   "abandoned" again -- the loop this patch exists to break.
 *
 * Assumptions:
 *   The caller has already stopped the GDMA TX channel.  Handing a
 *   bounce slot to the next transfer while the old one is still
 *   landing in it corrupts both.
 *
 * Returned Value:
 *   Number of containers reclaimed.
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static int i2s_tx_release_inflight(struct esp32s3_i2s_s *priv)
{
  struct esp32s3_buffer_s *bfcontainer;
  irqstate_t flags;
  volatile int n = 0;

  for (;;)
    {
      flags = spin_lock_irqsave(&priv->slock);

      bfcontainer = (struct esp32s3_buffer_s *)sq_remfirst(&priv->tx.act);

      if (bfcontainer == NULL)
        {
          bfcontainer = (struct esp32s3_buffer_s *)sq_remfirst(&priv->tx.pend);
        }

      if (bfcontainer == NULL)
        {
          bfcontainer = (struct esp32s3_buffer_s *)sq_remfirst(&priv->tx.done);
        }

      spin_unlock_irqrestore(&priv->slock, flags);

      if (bfcontainer == NULL)
        {
          break;
        }

      /* Report the transfer as aborted and hand the apb back through the
       * upper half's own bookkeeping (doneq + inflight--).
       * See the header comment: NO apb_free() here. */

      bfcontainer->result = -EAGAIN;

      if (bfcontainer->callback != NULL)
        {
          bfcontainer->callback(&priv->dev, bfcontainer->apb,
                                bfcontainer->arg, bfcontainer->result);
        }

      /* Release the internal-SRAM bounce slot, then the container
       * itself.  i2s_buf_free() would release a non-NULL buf anyway;
       * doing it explicitly keeps the two resources visibly paired. */

      i2s_dmabuf_release(bfcontainer->buf);
      bfcontainer->buf = NULL;

      i2s_buf_free(priv, bfcontainer);

      n++;
    }

  return n;
}

/****************************************************************************
 * Name: esp32s3_i2s_tx_flush
 *
 * Description:
 *   [V755] Reclaim every stalled TX buffer container so the shared
 *   internal-SRAM bounce pool can be reused by the capture path.
 *
 *   Safe to call at any time: it returns without touching a single
 *   register when all three TX queues are empty, so a playback that is
 *   genuinely in flight is never torn down.
 ****************************************************************************/

void esp32s3_i2s_tx_flush(void)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)g_v690_priv;
  volatile int n;

  if (priv == NULL || !priv->config->tx_en)
    {
      return;
    }

  /* Nothing to reclaim => leave the transmitter alone. */

  if (sq_empty(&priv->tx.act) &&
      sq_empty(&priv->tx.pend) &&
      sq_empty(&priv->tx.done))
    {
      return;
    }

  /* Stop the transmitter FIRST, for the same reason
   * esp32s3_i2s_rx_flush() stops the receiver: releasing a bounce slot
   * the GDMA is still reading from would hand it to the next transfer
   * while the old one is still in progress. */

  i2s_tx_channel_stop(priv);

  n = i2s_tx_release_inflight(priv);

  syslog(LOG_INFO,
         "[V755] tx flush: %d stalled buffer(s) released "
         "(bounce_now=%d)\n",
         (int)n,
         (int)(g_i2s_dmabuf_used[0] + g_i2s_dmabuf_used[1] +
               g_i2s_dmabuf_used[2] + g_i2s_dmabuf_used[3]));
}
#endif /* I2S_HAVE_TX */

static void i2s_rx_channel_stop(struct esp32s3_i2s_s *priv)
{
  if (priv->config->rx_en)
    {
      if (!priv->rx_started)
        {
          i2swarn("RX channel of port %d was previously stopped\n",
                  priv->config->port);
          return;
        }

      /* Stop RX channel */

      modifyreg32(I2S_RX_CONF_REG(priv->config->port), I2S_RX_START, 0);

      /* Stop outlink */

      SET_GDMA_CH_BITS(DMA_IN_LINK_CH0_REG, priv->dma_channel,
                       DMA_INLINK_STOP_CH0_M);

      /* Disable DMA interrupt */

      CLR_GDMA_CH_BITS(DMA_IN_INT_ENA_CH0_REG, priv->dma_channel,
                       DMA_IN_SUC_EOF_CH0_INT_ENA);

      /* Disable DMA operation mode */

      up_disable_irq(priv->rx_irq);

      priv->rx_started = false;

      i2sinfo("Stopped RX channel of port %d\n", priv->config->port);
    }
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: i2s_tx_interrupt
 *
 * Description:
 *   Common I2S DMA interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info
 *   arg     - I2S controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
/* [V209] The TX interrupt counter lives with the other diagnostics at the top
 * of this file, because i2s_txdma_start() (below) also needs g_txdump_n. */

static int IRAM_ATTR i2s_tx_interrupt(int irq, void *context, void *arg)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)arg;
  struct esp32s3_dmadesc_s *cur = NULL;
  uint32_t status;

  /* [V342] Latch the RAW status word BEFORE clearing it.  The previous code
   * cleared the status first and then looked at the same register, so by the
   * time any diagnostic ran the cause was gone.  This is the authoritative
   * record of which GDMA event actually woke the CPU. */

  status = GET_GDMA_CH_REG(DMA_OUT_INT_ST_CH0_REG, priv->dma_channel);

  g_txirq_n++;

  if (status != 0)
    {
      if (g_tx_isr_status == 0)
        {
          g_tx_isr_status = status;
        }

      if (status & DMA_OUT_DONE_CH0_INT_ST)
        {
          g_tx_isr_n[0]++;
        }

      if (status & DMA_OUT_EOF_CH0_INT_ST)
        {
          g_tx_isr_n[1]++;
        }

      if (status & DMA_OUT_DSCR_ERR_CH0_INT_ST)
        {
          g_tx_isr_n[2]++;
          g_tx_isr_err++;
        }

      if (status & DMA_OUT_TOTAL_EOF_CH0_INT_ST)
        {
          g_tx_isr_n[3]++;
        }
    }

  SET_GDMA_CH_REG(DMA_OUT_INT_CLR_CH0_REG, priv->dma_channel, status);

  if (priv->config->tx_en)
    {
      /* [V342] Complete on OUT_DONE, OUT_EOF (SUC_EOF) *or* OUT_TOTAL_EOF.
       *
       * Which of these the controller raises depends on out_eof_mode and on
       * how the outlink terminates, so acting on OUT_TOTAL_EOF alone made the
       * completion callback order-dependent: the RX path in this very driver
       * already completes on SUC_EOF, and here DMA_OUT_EOF_DES_ADDR is the
       * register the hardware fills on the SUC_EOF event.  i2s_tx_schedule()
       * itself de-duplicates: it only reports a buffer whose last descriptor
       * address matches the one it is handed, so a spurious extra call is a
       * harmless no-op. */

      if (status & (DMA_OUT_TOTAL_EOF_CH0_INT_ST |
                    DMA_OUT_EOF_CH0_INT_ST |
                    DMA_OUT_DONE_CH0_INT_ST))
        {
          cur = (struct esp32s3_dmadesc_s *)
                GET_GDMA_CH_REG(DMA_OUT_EOF_DES_ADDR_CH0_REG,
                                priv->dma_channel);

          /* Schedule completion of the transfer on the worker thread */

          i2s_tx_schedule(priv, cur);
        }
    }

  return 0;
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_rx_interrupt
 *
 * Description:
 *   Common I2S DMA interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info
 *   arg     - I2S controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static int i2s_rx_interrupt(int irq, void *context, void *arg)
{
  g_v94_isr++;
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)arg;
  struct esp32s3_dmadesc_s *cur = NULL;

  uint32_t status = GET_GDMA_CH_REG(DMA_IN_INT_ST_CH0_REG,
                                    priv->dma_channel);

  SET_GDMA_CH_REG(DMA_IN_INT_CLR_CH0_REG, priv->dma_channel,
                  UINT32_MAX);

  if (priv->config->rx_en)
    {
      if (status & DMA_IN_SUC_EOF_CH0_INT_ST)
        {
          /* [RX-DIAG] SUC_EOF seen -- proof RX data is flowing */
          {
            static int first_eof = 1;
            if (first_eof) {
              first_eof = 0;
              syslog(LOG_INFO,
                     "[RXEOF] first SUC_EOF! isr=%u status=0x%08x\n",
                     g_v94_isr, (unsigned)status);
            }
          }
          g_v94_eof++;
          cur = (struct esp32s3_dmadesc_s *)
                GET_GDMA_CH_REG(DMA_IN_SUC_EOF_DES_ADDR_CH0_REG,
                                priv->dma_channel);

          /* Schedule completion of the transfer on the worker thread */

          i2s_rx_schedule(priv, cur);
        }
    }

  return 0;
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: i2s_getmclkfrequency
 *
 * Description:
 *   Get the current master clock frequency.
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *
 * Returned Value:
 *   Returns the current master clock.
 *
 ****************************************************************************/

static uint32_t i2s_getmclkfrequency(struct i2s_dev_s *dev)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

  return priv->mclk_freq;
}

/****************************************************************************
 * Name: i2s_setmclkfrequency
 *
 * Description:
 *   Set the master clock frequency. Usually, the MCLK is a multiple of the
 *   sample rate. Most of the audio codecs require setting specific MCLK
 *   frequency according to the sample rate.
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *   frequency  - The I2S master clock's frequency
 *
 * Returned Value:
 *   Returns the resulting master clock or a negated errno value on failure.
 *
 ****************************************************************************/

static uint32_t i2s_setmclkfrequency(struct i2s_dev_s *dev,
                                     uint32_t frequency)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

  /* Check if the master clock frequency is beyond the highest possible
   * value and return an error.
   */

  if (frequency >= (g_i2s_sclk / 2))
    {
      return -EINVAL;
    }

  priv->mclk_freq = frequency;

  return frequency;
}

/****************************************************************************
 * Name: i2s_txchannels
 *
 * Description:
 *   Set the I2S TX number of channels.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   channels - The I2S numbers of channels
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static int i2s_txchannels(struct i2s_dev_s *dev, uint8_t channels)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;
  uint32_t channels_mask;

  if (priv->config->tx_en)
    {
      if (channels != 1 && channels != 2)
        {
          return -EINVAL;
        }

      i2s_tx_channel_stop(priv);

      priv->channels = channels;

      /* [V668] Remember it.  priv->channels itself is shared with the RX half
       * (i2s_rxchannels() writes the same field), so it cannot be used later
       * as the authority on how the transmitter should be configured. */
      g_i2s_tx_mono = (channels == 1) ? 1 : 0;

      /* Always consider two channels. For mono (1-channel), we set the
       * I2S_TX_TDM_CHAN1_EN to 0 and I2S_TX_CHAN_EQUAL to 1 to send out
       * the data of the previous channel.
       */

      /* I2S_TX_TDM_TOT_CHAN_NUM = channels - 1 */

      modifyreg32(I2S_TX_TDM_CTRL_REG(priv->config->port),
                  I2S_TX_TDM_TOT_CHAN_NUM_M,
                  FIELD_TO_VALUE(I2S_TX_TDM_TOT_CHAN_NUM, 1));

      channels_mask = getreg32(I2S_TX_TDM_CTRL_REG(priv->config->port));
      channels_mask &= 0xffff0000;

      if (priv->channels == 1)
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_MONO_M, I2S_TX_MONO);
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_CHAN_EQUAL_M, I2S_TX_CHAN_EQUAL);
          channels_mask |= I2S_TX_TDM_CHAN0_EN;
        }
      else
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_MONO_M, 0);
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_CHAN_EQUAL_M, 0);
          channels_mask |= I2S_TX_TDM_CHAN0_EN | I2S_TX_TDM_CHAN1_EN;
        }

      putreg32(channels_mask, I2S_TX_TDM_CTRL_REG(priv->config->port));

      /* Set I2S_TX_UPDATE bit to update the configs.
       * This bit is automatically cleared.
       */

      modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                  0, I2S_TX_UPDATE);

      i2s_tx_channel_start(priv);

      return OK;
    }

  return -ENOTTY;
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_rxchannels
 *
 * Description:
 *   Set the I2S RX number of channels.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   channels - The I2S numbers of channels
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static int i2s_rxchannels(struct i2s_dev_s *dev, uint8_t channels)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

  if (priv->config->rx_en)
    {
      if (channels != 1 && channels != 2)
        {
          return -EINVAL;
        }

      priv->channels = channels;
      g_i2s_rx_mono = (channels == 1) ? 1 : 0;   /* [V681] */
      return OK;
    }

  return -ENOTTY;
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: i2s_txsamplerate
 *
 * Description:
 *   Set the I2S TX sample rate.  NOTE:  This will have no effect if (1) the
 *   driver does not support an I2S transmitter or if (2) the sample rate is
 *   driven by the I2S frame clock.  This may also have unexpected side-
 *   effects of the TX sample is coupled with the RX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static uint32_t i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

  if (priv->config->tx_en)
    {
      i2s_tx_channel_stop(priv);

      priv->rate = rate;

      if (i2s_check_mclkfrequency(priv) < OK)
        {
          return 0;
        }

      /* [V341] The source clock and every divider are handled centrally now.
       *
       * This used to re-write I2S_TX_CLK_SEL here (the V221 / V249 blocks),
       * duplicating -- and often contradicting -- the choice i2s_configure()
       * makes.  i2s_clock_source_bringup() selects the source (PLL_F160M
       * first, XTAL as a guaranteed-running fallback), programs all dividers
       * for it via i2s_set_clock(), and verifies the tx_update handshake so a
       * stalled clock domain can never again be selected silently.
       * g_i2s_sclk and g_i2s_clk_sel are updated by that function. */

      rate = i2s_clock_source_bringup(priv);

      i2s_tx_channel_start(priv);

      /* [V664] *** Restore the channel mode that the reset just dropped. ***
       *
       * i2s_clock_source_bringup() -> i2s_clock_try_source() pulses
       * I2S_TX_RESET, and that clears I2S_TX_MONO / I2S_TX_CHAN_EQUAL out
       * of I2S_TX_CONF as well.  The comment inside i2s_clock_try_source()
       * claims tx_reset leaves the configuration registers alone -- it does
       * not.  Measured [V663] with TX already running:
       *
       *     I2S_TXCHANNELS(1) then I2S_TXSAMPLERATE(16000)
       *     TX_CONF = 0x00081204   -> bit5 (MONO) = 0, bit6 (CHAN_EQUAL) = 0
       *     TX_TDM_CTRL = 0x00010001 -> TOT_CHAN_NUM = 1, CHAN0_EN = 1
       *
       * The TDM half proves i2s_txchannels() did run, so the two CONF bits
       * were written and then wiped here.
       *
       * Consequences of running stereo with mono PCM: the hardware takes
       * TWO 16-bit samples out of the FIFO per LRCK frame (one per slot),
       * so 30720 samples are consumed in 15360 frames instead of 30720 and
       * the clip plays at exactly 2x -- pitch up an octave, speech at double
       * speed, which is the field report.  (30720/2/16000 = 0.960 s,
       * matching the 1.005 s v663 measured for the whole 61440 B.)
       *
       * i2s_txchannels() has already stored priv->channels, so put the two
       * bits back here -- after every reset, before anything can be queued.
       * channels == 0 means nobody ever asked, so leave the default alone. */

      if (priv->channels == 1)
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_MONO_M, I2S_TX_MONO);
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_CHAN_EQUAL_M, I2S_TX_CHAN_EQUAL);
          modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_TX_UPDATE);

        }
      else if (priv->channels == 2)
        {
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_MONO_M, 0);
          modifyreg32(I2S_TX_CONF_REG(priv->config->port),
                      I2S_TX_CHAN_EQUAL_M, 0);
          modifyreg32(I2S_TX_CONF_REG(priv->config->port), 0, I2S_TX_UPDATE);
        }

      /* [V662] *** Playback-rate forensics: what is LRCK really? ***
       *
       * The field report is "TTS speaks much too fast, unintelligible".
       * Every candidate cause collapses onto one quantity: the real LRCK
       * frequency of I2S0 TX.  What i2s_txsamplerate() returns is only
       * the nominal value i2s_set_clock() recomputes from its own
       * assumptions (sclk, total_slot, data_width, bclk_div) -- if any of
       * those disagrees with the hardware, it still cheerfully reports
       * 16000.  So print three mutually independent groups of facts:
       *
       *   1) driver state        rate / mclk_freq / data_width / channels /
       *                          total_slot / role / g_i2s_sclk / clk_sel
       *   2) TX hardware regs    conf (BITS_MOD) / conf1 (BCK_DIV_NUM) /
       *                          clkm_conf / clkm_div_conf / tdm_ctrl
       *   3) measured pin edges  WS(GPIO8) and BCLK(GPIO10), CCOUNT timebase
       *
       * The TX clock keeps running with no data queued, so this can be
       * sampled right here, before the first ap_buffer is ever sent.
       *
       * Interpretation: 16000 -> I2S is innocent, look at the data path.
       * ~32000 -> one level forgot a factor of two (bits_mod / bck_div).
       * ~24000 -> sclk is really the 240 MHz PLL, not 160 MHz.
       */
      return rate;
    }

  return 0;
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_rxsamplerate
 *
 * Description:
 *   Set the I2S RX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static uint32_t i2s_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

  if (priv->config->rx_en)
    {
      /* [V697] See i2s_rxdatawidth(): a configure that asks for the rate we
       * are already running at must not stop the receiver.  The capture and
       * the playback instance share one I2S port, so every TTS session
       * reconfigures it, and each of those reconfigures used to kill the
       * capture engine and strand a buffer. */

      if (priv->rx_started && priv->rate == rate)
        {
          return rate;
        }

      i2s_rx_channel_stop(priv);

      priv->rate = rate;

      if (i2s_check_mclkfrequency(priv) < OK)
        {
          return 0;
        }

      rate = i2s_set_clock(priv);

      i2s_rx_channel_start(priv);

      return rate;
    }

  return 0;
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: i2s_txdatawidth
 *
 * Description:
 *   Set the I2S TX data width.  The TX bitrate is determined by
 *   sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The I2S data with in bits.
 *
 * Returned Value:
 *   Returns the resulting data width
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static uint32_t i2s_txdatawidth(struct i2s_dev_s *dev, int bits)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

  if (priv->config->tx_en)
    {
      i2s_tx_channel_stop(priv);

      priv->data_width = bits;

      i2s_set_datawidth(priv);

      i2s_tx_channel_start(priv);

      return bits;
    }

  return 0;
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_rxdatawidth
 *
 * Description:
 *   Set the I2S RX data width.  The RX bitrate is determined by
 *   sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The I2S data with in bits.
 *
 * Returned Value:
 *   Returns the resulting data width
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static uint32_t i2s_rxdatawidth(struct i2s_dev_s *dev, int bits)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

  if (priv->config->rx_en)
    {
      /* [V726] *** Live register hook, read in TASK context. ***
   *
   * es8311_configure() calls this function on every configure, and the
   * log shows one configure per listening cycle (~20 s), so this is the
   * first place that is both live and safe.
   *
   * It is deliberately NOT in the rx completion path.  v724 tried that
   * and the board panicked on every single boot at t = 24.5 s,
   * pc=421f9382 (getumask, `l32i.n a2, a2, 24`, vaddr=0x1a because a2
   * had been loaded from a8=0x3fc92000, 0x50 below the hpwork stack
   * base 0x3fc92050), task hpwork, chain
   * work_thread -> fopen -> open -> nx_vopen.  Compiling the two fopen
   * calls out made the panic disappear for the whole 120 s run.
   * File I/O does not belong on the work queue on this port.
   *
   * Cost: a candidate written here takes effect at the NEXT configure,
   * about twenty seconds later.  Worth it -- the alternative is a
   * rebuild and a reflash per hypothesis.
   *
   *   echo "RX_CONF1 0x0f3de38f" > /tmp/i2sw.txt    # MSB_SHIFT = 0
   *   echo 1 > /tmp/pins.txt                        # probe the pins
   */

  i2s_v724_hook(priv->config->port);
  i2s_v724_pins();

  /* [V697] *** Nothing to do is not a reason to tear the engine down. ***
       *
       * The stop/start below aborts whatever transfer is in flight, and the
       * container it was given is then abandoned in priv->rx.act for ever
       * (i2s_rxdma_start() bails out with OK while rx.act is non-empty, so
       * no buffer is ever armed again).  es8311_configure() calls this on
       * EVERY configure -- playback included -- and the value is almost
       * always the same one, so the abort used to happen several times per
       * PTT session and the second capture never started:
       *
       *   [R36]   I2S RX rate 16000 -> 16000
       *   [V690]  rx recv=6 start=10 eof=4 isr=4 pb=6 done=0 pend=1
       *           act=1 done_q=0 slot=0x01 bounce=2
       *
       * Leaving the running engine alone when the width has not changed is
       * also what keeps priv->rx_started true, which is the state this
       * function is supposed to maintain. */

      if (priv->rx_started && priv->data_width == bits)
        {
          return (uint32_t)bits;
        }

      i2s_rx_channel_stop(priv);

      priv->data_width = bits;

      i2s_set_datawidth(priv);

      i2s_rx_channel_start(priv);

      return bits;
    }

  return 0;
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: i2s_send
 *
 * Description:
 *   Send a block of data on I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer from which to send data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be cancelled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef I2S_HAVE_TX
static int i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                    i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

  if (priv->config->tx_en)
    {
      struct esp32s3_buffer_s *bfcontainer;
      int ret = OK;
      uint32_t nbytes;
      uint32_t nsamp;

      /* Check audio buffer data size from the upper half. If the buffer
       * size is not a multiple of the data width, the remaining bytes
       * must be sent along with the next audio buffer.
       */

      nbytes = (apb->nbytes - apb->curbyte) + priv->tx.carry.bytes;

      nbytes -= (nbytes % (priv->data_width / 8));

      /* [V810] Transmit-side sample dump.
       *
       * The speaker is audible with the right words and a hiss underneath, and
       * everything measured so far describes the microphone: [V716]'s window OR
       * and [V718]'s peak both sit on the RX path, and the RX register file reads
       * 0x2f3de38f on every one of 146 captures, which is the healthy setting.
       * So the input side is clean and the hiss is being generated downstream of
       * it -- but nothing in the build looks at what is actually handed to the
       * transmitter.
       *
       * That is the gap this closes.  The first sixteen 16-bit words show
       * whether the samples are plausible audio at all, and the low-zero count
       * is the same test the RX side already uses (a receiver that latched the
       * frame k bits early leaves k low zeros; the transmitter being fed a
       * stream that was shifted the same way would show the same signature).
       *
       * Diagnosis only: nothing here is written back, and the count is bounded so
       * the log stays readable.
       *
       * [V810b] Field names were wrong on the first attempt: ap_buffer_s carries
       * the data in `samp` (not `sample`) and the driver spells the channel
       * count `channels` (not `txchannels`).  Guessing member names costs a
       * build cycle; reading the struct costs one grep. */
      {
        static uint32_t v810_bufs;

        if ((v810_bufs++ % 32u) == 1u)
          {
            const int16_t *w   = (const int16_t *)apb->samp;
            uint32_t       orv = 0;
            int            loz = -1;
            int            i;

            for (i = 0; i < 16 && (uint32_t)(i * 2 + 1) < apb->nbytes; i++)
              {
                orv |= (uint32_t)(uint16_t)w[i];
              }

            if (orv != 0)
              {
                loz = 0;

                while ((orv & 1u) == 0u && loz < 16)
                  {
                    orv >>= 1;
                    loz++;
                  }
              }

            syslog(LOG_INFO,
                   "[V810] TX nbytes=%u width=%u ch=%u or=0x%08x lowzero=%d "
                   "samples:", (unsigned)nbytes,
                   (unsigned)priv->data_width,
                   (unsigned)(priv->channels ? priv->channels : 1),
                   (unsigned)orv, loz);

            for (i = 0; i < 16 && (uint32_t)(i * 2 + 1) < apb->nbytes; i++)
              {
                syslog(LOG_INFO, "%04x ", (unsigned)(uint16_t)w[i]);
              }

            syslog(LOG_INFO, "\n");
          }
      }

      if (nbytes > (ESP32S3_DMA_BUFLEN_MAX * I2S_DMADESC_NUM))
        {
          i2serr("Required buffer size can't fit into DMA outlink "
                 "(exceeds in %" PRIu32 " bytes). Try to increase the "
                 "number of the DMA descriptors (CONFIG_I2S_DMADESC_NUM).",
                 nbytes - (ESP32S3_DMA_BUFLEN_MAX * I2S_DMADESC_NUM));
          return -EFBIG;
        }

      /* Allocate a buffer container in advance */

      bfcontainer = i2s_buf_allocate(priv);
      if (bfcontainer == NULL)
        {
          i2serr("Failed to allocate the buffer container");

          /* [V755] *** This is where the capture actually dies. ***
           *
           * There are only CONFIG_ESP32S3_I2S_MAXINFLIGHT=4 buffer
           * containers in total and they are shared with the TX path.
           * A playback that was abandoned (es8311 worker did not exit)
           * leaves its containers parked in priv->tx.act / priv->tx.pend
           * for ever; measured steady state on this board is
           * txa=1 txp=3, i.e. all four.  Every capture then fails right
           * here with -ENOMEM before it ever reaches
           * i2s_rxdma_setup(), which the upper half reports as
           * "capture EIO" and turns into an endless
           * "listen: no speech for 8000 ms, restarting session" loop.
           *
           * v755 hung the reclaim on the bounce-slot failure further
           * down and it therefore never ran (0 flushes in 150 s while
           * xfer ret=-12 fired 7 times).  Reclaim here, on the line
           * that actually fails, and try once more. */

          esp32s3_i2s_tx_flush();

          bfcontainer = i2s_buf_allocate(priv);
          if (bfcontainer == NULL)
            {
              return -ENOMEM;
            }
        }

      /* Get exclusive access to the I2S driver data */

      ret = nxmutex_lock(&priv->lock);
      if (ret < 0)
        {
          goto errout_with_buf;
        }

      /* Add a reference to the audio buffer */

      apb_reference(apb);

      /* Initialize the buffer container structure */

      bfcontainer->callback = callback;
      bfcontainer->timeout  = timeout;
      bfcontainer->arg      = arg;
      bfcontainer->apb      = apb;
      bfcontainer->nbytes   = nbytes;
      bfcontainer->result   = -EBUSY;

      ret = i2s_txdma_setup(priv, bfcontainer);

      if (ret != OK)
        {
          goto errout_with_buf;
        }

      i2sinfo("Queued %d bytes into DMA buffers\n", apb->nbytes);
      i2s_dump_buffer("Audio pipeline buffer:", &apb->samp[apb->curbyte],
                      apb->nbytes - apb->curbyte);

      nxmutex_unlock(&priv->lock);

      /* [V209] One-shot TX register dump.  A dead I2S TX clock and a dead
       * GDMA outlink both present as "the completion callback never fires",
       * which is indistinguishable from twenty other faults; these four
       * registers separate them in a single boot. */
      if (I2S_I2SREG_DUMP && g_txdump_n < 3)
        {
          uint32_t conf  = getreg32(I2S_TX_CONF_REG(priv->config->port));
          uint32_t clkm  = getreg32(I2S_TX_CLKM_CONF_REG(priv->config->port));
          uint32_t conf1 = getreg32(I2S_TX_CONF1_REG(priv->config->port));
          uint32_t lnk   = GET_GDMA_CH_REG(DMA_OUT_LINK_CH0_REG,
                                           priv->dma_channel);
          uint32_t c0    = GET_GDMA_CH_REG(DMA_OUT_CONF0_CH0_REG,
                                           priv->dma_channel);

          g_txdump_n++;
          syslog(LOG_INFO,
                 "[I2SREG] #%d ch=%d txirq=%d tx_started=%d txirqN=%d\n",
                 g_txdump_n, priv->dma_channel, priv->tx_irq,
                 (int)priv->tx_started, g_txirq_n);
          syslog(LOG_INFO,
                 "[I2SREG] TXCONF=0x%08x START=%d RESET=%d FIFORST=%d "
                 "SIGLOOP=%d\n", (unsigned)conf,
                 !!(conf & I2S_TX_START), !!(conf & I2S_TX_RESET),
                 !!(conf & I2S_TX_FIFO_RESET), !!(conf & I2S_SIG_LOOPBACK));
          syslog(LOG_INFO,
                 "[I2SREG] CLKM=0x%08x CLK_EN=%d CLK_SEL=%u CLK_ACTIVE=%d\n",
                 (unsigned)clkm, !!(clkm & I2S_CLK_EN),
                 (unsigned)((clkm >> I2S_TX_CLK_SEL_S) & 0x3),
                 !!(clkm & I2S_TX_CLK_ACTIVE));
          uint32_t ist  = GET_GDMA_CH_REG(DMA_OUT_INT_ST_CH0_REG,
                                          priv->dma_channel);
          uint32_t ien  = GET_GDMA_CH_REG(DMA_OUT_INT_ENA_CH0_REG,
                                          priv->dma_channel);
          uint32_t eofd = GET_GDMA_CH_REG(DMA_OUT_EOF_DES_ADDR_CH0_REG,
                                          priv->dma_channel);
          uint32_t d0   = *(volatile uint32_t *)bfcontainer->dma_link;

          syslog(LOG_INFO,
                 "[I2SREG] TXCONF1=0x%08x DMA_OUTLINK=0x%08x "
                 "DMA_OUT_CONF0=0x%08x\n", (unsigned)conf1,
                 (unsigned)lnk, (unsigned)c0);
          syslog(LOG_INFO,
                 "[I2SREG] INT_ST=0x%08x INT_ENA=0x%08x EOF_DES=0x%08x "
                 "LINKSTART=%d\n", (unsigned)ist, (unsigned)ien,
                 (unsigned)eofd, !!(lnk & DMA_OUTLINK_START_CH0_M));

          /* [V342] Authoritative record of which GDMA TX event actually
           * fired (latched by the ISR before it clears the status), plus the
           * per-event counters.  g_tx_isr_status == 0 with txirqN > 0 would
           * mean the interrupt came from somewhere other than OUT_INT_ST. */

          

          /* [V345] Completion-path accounting: how many ISR schedules ran,
           * how many found an empty tx.act, how many matched the descriptor,
           * and how many actually reached the user callback. */

          
          syslog(LOG_INFO,
                 "[I2SREG] desc0.ctrl=0x%08x OWNER(bit31)=%d EOF(bit30)=%d "
                 "len=%u dma_link=%p buf=%p\n", (unsigned)d0,
                 !!(d0 & 0x80000000u), !!(d0 & 0x40000000u),
                 (unsigned)(d0 & 0xfff),
                 (void *)bfcontainer->dma_link, (void *)bfcontainer->buf);
          {
            /* [V214] The three remaining suspects: is the GDMA actually bound
             * to I2S0, has the I2S TX FIFO reported a hang (i.e. the FIFO
             * filled and BCLK never advanced), and was OUTLINK_START ever
             * really set? */
            uint32_t psel = GET_GDMA_CH_REG(DMA_OUT_PERI_SEL_CH0_REG,
                                            priv->dma_channel);
            uint32_t ists = getreg32(I2S_INT_ST_REG(priv->config->port));
            uint32_t hung = getreg32(I2S_LC_HUNG_CONF_REG(priv->config->port));

            syslog(LOG_INFO,
                   "[I2SREG] PERI_SEL=0x%08x (want 3=I2S0)  I2S_INT_ST=0x%08x "
                   "TX_HUNG=%d\n", (unsigned)psel, (unsigned)ists,
                   !!(ists & I2S_TX_HUNG_INT_ST));
            syslog(LOG_INFO,
                   "[I2SREG] PCM2PDM_CONF=0x%08x  TXCONF=0x%08x "
                   "(TXPDM=%d TXTDM=%d)\n",
                   (unsigned)getreg32(I2S_TX_PCM2PDM_CONF_REG(
                                      priv->config->port)),
                   (unsigned)getreg32(I2S_TX_CONF_REG(priv->config->port)),
                   !!(getreg32(I2S_TX_CONF_REG(priv->config->port)) &
                      I2S_TX_PDM_EN),
                   !!(getreg32(I2S_TX_CONF_REG(priv->config->port)) &
                      I2S_TX_TDM_EN));
            syslog(LOG_INFO,
                   "[I2SREG] tx_update_spins=%d (max %d) fallback=%d "
                   "after_fallback=%d\n", g_tx_update_spins,
                   I2S_UPDATE_SPIN_MAX, g_sclk_fallback, g_tx_update_spins2);
            syslog(LOG_INFO,
                   "[I2SREG] LC_HUNG=0x%08x  LINK_after_enable=0x%08x "
                   "START_was=%d\n", (unsigned)hung,
                   (unsigned)g_dbg_link_after_enable,
                   !!(g_dbg_link_after_enable & DMA_OUTLINK_START_CH0_M));

            /* [V215] Is the GDMA outlink FSM alive at all?  Sample link +
             * INT_ST now, again 20 ms later, then try an explicit retry start
             * and sample a third time.  If the retry finally consumes the
             * descriptor (OWNER clears) the first start is being lost; if
             * OWNER never clears and INT_ST stays 0, the engine is not
             * fetching the descriptor at all. */
            uint32_t l1 = GET_GDMA_CH_REG(DMA_OUT_LINK_CH0_REG,
                                          priv->dma_channel);
            uint32_t i1 = GET_GDMA_CH_REG(DMA_OUT_INT_ST_CH0_REG,
                                          priv->dma_channel);

            up_mdelay(20);

            uint32_t l2 = GET_GDMA_CH_REG(DMA_OUT_LINK_CH0_REG,
                                          priv->dma_channel);
            uint32_t i2 = GET_GDMA_CH_REG(DMA_OUT_INT_ST_CH0_REG,
                                          priv->dma_channel);
            uint32_t dd = *(volatile uint32_t *)bfcontainer->dma_link;

            syslog(LOG_INFO,
                   "[I2SREG] t0: LINK=0x%08x INTST=0x%08x | "
                   "+20ms: LINK=0x%08x OWNER=%d INTST=0x%08x\n",
                   (unsigned)l1, (unsigned)i1, (unsigned)l2,
                   !!(dd & 0x80000000u), (unsigned)i2);

            /* [V344] The explicit re-arm that used to live here (a second
             * OUTLINK_START) has been removed.  It re-issued the start against
             * a descriptor the FSM had already consumed -- its OWNER was
             * already written back to 0 -- which the GDMA with
             * out_check_owner = 1 rejects with OUT_DSCR_ERR.  It also ran while
             * the real transfer was still in flight and stole its completion.
             * The read-only re-sample below is kept. */

            up_mdelay(20);

            {
              uint32_t l3 = GET_GDMA_CH_REG(DMA_OUT_LINK_CH0_REG,
                                            priv->dma_channel);
              uint32_t i3 = GET_GDMA_CH_REG(DMA_OUT_INT_ST_CH0_REG,
                                            priv->dma_channel);
              uint32_t d3 = *(volatile uint32_t *)bfcontainer->dma_link;

              syslog(LOG_INFO,
                     "[I2SREG] after RETRY+20ms: LINK=0x%08x "
                     "(START=%d PARK=%d) OWNER=%d INTST=0x%08x txirqN=%d\n",
                     (unsigned)l3, !!(l3 & DMA_OUTLINK_START_CH0_M),
                     !!(l3 & 0x00800000u), !!(d3 & 0x80000000u),
                     (unsigned)i3, g_txirq_n);

  /* [V251] The three registers that say WHY the outlink will not move:
   *   DMA_OUT_DSCR_CH0           current descriptor address held by the FSM
   *   DMA_OUTFIFO_STATUS_CH0     level/full/empty of the GDMA's own OUT FIFO
   *   DMA_OUT_STATE_CH0          the FSM state codes + outlink dscr address
   * If DSCR still reads 0 the FSM never loaded a descriptor; if OUTFIFO is
   * full the peripheral is not accepting data; if OUTFIFO is empty the
   * peripheral never asked for any. */

  

  /* [V344] The V260 TDM-slot / mono sweep that used to run here has been
   * removed.  It was a one-shot experiment, but it did lasting damage:
   *
   *   - case 7 set I2S_SIG_LOOPBACK and the restore block never cleared it,
   *     so every later transfer ran with the TX output looped back internally
   *     instead of driving DOUT (the register dump confirms SIGLOOP=1);
   *   - its closing restore wrote tx_mono=1 / tx_chan_equal=1, contradicting
   *     the V343 mono=0 / chan_equal=0 configuration;
   *   - its eight re-arms issued OUTLINK_START against descriptors the FSM had
   *     already consumed (OWNER written back to 0), which is what produced the
   *     OUT_DSCR_ERR storm seen in the ISR counters;
   *   - it reconfigured TX_TDM_CTRL and restarted the module while the real
   *     transfer was in flight.
   *
   * All of the information it collected is already captured by the read-only
   * [I2SREG] dump above and by the [V342] ISR forensics, so nothing is lost. */


            }
          }
        }

      return OK;

errout_with_buf:
      nxmutex_unlock(&priv->lock);
      i2s_buf_free(priv, bfcontainer);
      return ret;
    }

  return -ENOTTY;
}
#endif /* I2S_HAVE_TX */

/****************************************************************************
 * Name: i2s_receive
 *
 * Description:
 *   Receive a block of data on I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer in which to receive data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be cancelled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef I2S_HAVE_RX
static int i2s_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                       i2s_callback_t callback, void *arg, uint32_t timeout)
{
  g_v94_recv++;
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;

  g_v690_priv = priv;   /* [V690] for the rx health beacon */

  if (priv->config->rx_en)
    {
      struct esp32s3_buffer_s *bfcontainer;
      int ret = OK;
      uint32_t nbytes;
      uint32_t nsamp;

      /* Check max audio buffer data size from the upper half and align the
       * receiving buffer according to the data width.
       */

      nbytes = apb->nmaxbytes;

      nbytes -= (nbytes % (priv->data_width / 8));

      nbytes = MIN(nbytes, ESP32S3_DMA_BUFLEN_MAX);

      /* Allocate a buffer container in advance */

      bfcontainer = i2s_buf_allocate(priv);
      if (bfcontainer == NULL)
        {
          i2serr("Failed to allocate the buffer container");
          return -ENOMEM;
        }

      /* Get exclusive access to the I2S driver data */

      ret = nxmutex_lock(&priv->lock);
      if (ret < 0)
        {
          goto errout_with_buf;
        }

      /* Add a reference to the audio buffer */

      apb_reference(apb);

      /* Initialize the buffer container structure */

      bfcontainer->callback = callback;
      bfcontainer->timeout  = timeout;
      bfcontainer->arg      = arg;
      bfcontainer->apb      = apb;
      bfcontainer->nbytes   = nbytes;
      bfcontainer->result   = -EBUSY;

      ret = i2s_rxdma_setup(priv, bfcontainer);

      if (ret != OK)
        {
          goto errout_with_buf;
        }

      i2sinfo("Prepared %d bytes to receive DMA buffers\n", apb->nmaxbytes);
      i2s_dump_buffer("Audio pipeline buffer:", &apb->samp[apb->curbyte],
                      apb->nbytes - apb->curbyte);

      nxmutex_unlock(&priv->lock);

      return OK;

errout_with_buf:
      nxmutex_unlock(&priv->lock);
      i2s_buf_free(priv, bfcontainer);
      return ret;
    }

  return -ENOTTY;
}
#endif /* I2S_HAVE_RX */

/****************************************************************************
 * Name: i2s_ioctl
 *
 * Description:
 *   Implement the lower-half logic ioctl commands
 *
 * Input parameters:
 *   dev - A reference to the lower-half I2S driver device
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

/* [V671] Export the transfer-path counters so the playback layer can print
 * one line per utterance.  "full" counts transfers that had to reconfigure and
 * reset the transmitter; "light" counts the ones that only re-armed the DMA.
 * A healthy session is full == 1 and light == (transfers - 1).
 */

void esp32s3_i2s_v671_stats(uint32_t *full, uint32_t *light)
{
  if (full != NULL)
    {
      *full = (uint32_t)g_v671_full_n;
    }

  if (light != NULL)
    {
      *light = (uint32_t)g_v671_light_n;
    }
}

static int i2s_ioctl(struct i2s_dev_s *dev, int cmd, unsigned long arg)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;
  struct audio_buf_desc_s  *bufdesc;
  int ret = -ENOTTY;

  switch (cmd)
    {
      /* AUDIOIOC_START - Start the audio stream.
       *
       *   ioctl argument:  Audio session
       */

      case AUDIOIOC_START:
        {
          i2sinfo("AUDIOIOC_START\n");

          priv->streaming = true;

          ret = OK;
        }
        break;

      /* AUDIOIOC_STOP - Stop the audio stream.
       *
       *   ioctl argument:  Audio session
       */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
      case AUDIOIOC_STOP:
        {
          i2sinfo("AUDIOIOC_STOP\n");

          priv->streaming = false;

          ret = OK;
        }
        break;
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

      /* AUDIOIOC_ALLOCBUFFER - Allocate an audio buffer
       *
       *   ioctl argument:  pointer to an audio_buf_desc_s structure
       */

      case AUDIOIOC_ALLOCBUFFER:
        {
          i2sinfo("AUDIOIOC_ALLOCBUFFER\n");

          bufdesc = (struct audio_buf_desc_s *) arg;
          ret = apb_alloc(bufdesc);
        }
        break;

      /* AUDIOIOC_FREEBUFFER - Free an audio buffer
       *
       *   ioctl argument:  pointer to an audio_buf_desc_s structure
       */

      case AUDIOIOC_FREEBUFFER:
        {
          i2sinfo("AUDIOIOC_FREEBUFFER\n");

          bufdesc = (struct audio_buf_desc_s *) arg;
          DEBUGASSERT(bufdesc->u.buffer != NULL);
          apb_free(bufdesc->u.buffer);
          ret = sizeof(struct audio_buf_desc_s);
        }
        break;

      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: i2s_dma_setup
 *
 * Description:
 *   Configure the DMA for the I2S peripheral
 *
 * Input Parameters:
 *   priv - Partially initialized I2S device structure. This function
 *          will complete the I2S specific portions of the initialization
 *          regarding the DMA operation.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_dma_setup(struct esp32s3_i2s_s *priv)
{
  int ret;
  int i2s_dma_dev;

  if (priv->config->port == 0)
    {
      i2s_dma_dev = ESP32S3_DMA_PERIPH_I2S0;
    }
  else
    {
      i2s_dma_dev = ESP32S3_DMA_PERIPH_I2S1;
    }

  /* Request a GDMA channel for the I2S peripheral */

  priv->dma_channel = esp32s3_dma_request(i2s_dma_dev, 1, 1, false);
  if (priv->dma_channel < 0)
    {
      i2serr("Failed to allocate GDMA channel\n");
      return ERROR;
    }

  /* [V268] *** Only ONE GDMA channel may serve I2S0 ***
   *
   * Measured: a second I2S instance runs during bring-up and takes channel 0,
   * binding DMA_PERI_OUT_SEL = 3 on both directions, i.e. to I2S0 -- the same
   * peripheral this instance serves from channel 1.  Two channels with the
   * same peripheral id means the I2S's DMA request fans out to both and the
   * data path is no longer deterministic, which fits every measurement so
   * far: the engine pulls the descriptor into its own FIFO (FIFO reads FULL)
   * yet nothing is ever handed to the I2S, so the transmitter drains nothing,
   * goes idle and the transfer times out.
   *
   * If the other channel is bound to THIS peripheral, unbind it and let one
   * channel own I2S0.  A channel bound to a different peripheral is left
   * strictly alone. */

  {
    uint32_t c0o = getreg32(DMA_OUT_PERI_SEL_CH0_REG);
    uint32_t c0i = getreg32(DMA_IN_PERI_SEL_CH0_REG);

    

    if (priv->dma_channel != 0 && c0o == 3u && c0i == 3u)
      {
        SET_GDMA_CH_REG(DMA_OUT_PERI_SEL_CH0_REG, 0, 0x3fu);
        SET_GDMA_CH_REG(DMA_IN_PERI_SEL_CH0_REG, 0, 0x3fu);
        
      }
    else if (priv->dma_channel != 0 && c0o == 0x3fu && c0i == 0x3fu)
      {
        priv->dma_channel = 0;
        SET_GDMA_CH_REG(DMA_OUT_PERI_SEL_CH0_REG, 0, 3u);
        SET_GDMA_CH_REG(DMA_IN_PERI_SEL_CH0_REG, 0, 0x3fu);
        
      }
  }

  /* Set up to receive GDMA interrupts on the current CPU. Each TX/RX channel
   * will be assigned to a different CPU interrupt.
   */

  priv->cpu = this_cpu();

#ifdef I2S_HAVE_TX
  if (priv->config->tx_en)
    {
      int periph = ESP32S3_PERIPH_DMA_OUT_CH0 + priv->dma_channel;
      int cpuint = esp32s3_setup_irq(priv->cpu, periph, 1,
                                     ESP32S3_CPUINT_LEVEL);
      if (cpuint < 0)
        {
          i2serr("Failed to allocate a CPU interrupt.\n");
          return ERROR;
        }

      priv->tx_irq = ESP32S3_PERIPH2IRQ(periph);
      ret = irq_attach(priv->tx_irq, i2s_tx_interrupt, priv);
      if (ret != OK)
        {
          i2serr("Couldn't attach IRQ to handler.\n");
          esp32s3_teardown_irq(priv->cpu, periph, cpuint);
          return ret;
        }
    }
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  if (priv->config->rx_en)
    {
      int periph = ESP32S3_PERIPH_DMA_IN_CH0 + priv->dma_channel;
      int cpuint = esp32s3_setup_irq(priv->cpu, periph, 1,
                                     ESP32S3_CPUINT_LEVEL);
      if (cpuint < 0)
        {
          i2serr("Failed to allocate a CPU interrupt.\n");
          return ERROR;
        }

      priv->rx_irq = ESP32S3_PERIPH2IRQ(periph);
      ret = irq_attach(priv->rx_irq, i2s_rx_interrupt, priv);
      if (ret != OK)
        {
          i2serr("Couldn't attach IRQ to handler.\n");
          esp32s3_teardown_irq(priv->cpu, periph, cpuint);
          return ret;
        }
    }
#endif /* I2S_HAVE_RX */

  return OK;
}

/****************************************************************************
 * Name: esp32s3_i2sbus_initialize
 *
 * Description:
 *   Initialize the selected I2S port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple I2S interfaces)
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *esp32s3_i2sbus_initialize(int port)
{
  int ret;
  struct esp32s3_i2s_s *priv = NULL;
  irqstate_t flags;

  i2sinfo("port: %d\n", port);

  /* Statically allocated I2S' device strucuture */

  switch (port)
    {
#ifdef CONFIG_ESP32S3_I2S0
      case ESP32S3_I2S0:
        priv = &esp32s3_i2s0_priv;
        break;
#endif
#ifdef CONFIG_ESP32S3_I2S1
      case ESP32S3_I2S1:
        priv = &esp32s3_i2s1_priv;
        break;
#endif
      default:
        return NULL;
    }

  /* Allocate buffer containers */

  ret = i2s_buf_initialize(priv);
  if (ret < 0)
    {
      goto err;
    }

  flags = spin_lock_irqsave(&priv->slock);

  i2s_configure(priv);

  ret = i2s_dma_setup(priv);
  if (ret < 0)
    {
      goto err;
    }

#ifdef I2S_HAVE_TX
  /* Start TX channel */

  if (priv->config->tx_en)
    {
      priv->tx_started = false;
      g_tx_cfg_done    = 0;   /* [V346] force reconfig on next activation */
      i2s_tx_channel_start(priv);
    }
#endif /* I2S_HAVE_TX */

#ifdef I2S_HAVE_RX
  /* Start RX channel */

  if (priv->config->rx_en)
    {
      priv->rx_started = false;
      i2s_rx_channel_start(priv);
    }
#endif /* I2S_HAVE_RX */

  spin_unlock_irqrestore(&priv->slock, flags);

  /* Success exit */

  i2sinfo("I2S%d was successfully initialized\n", priv->config->port);

  return &priv->dev;

  /* Failure exit */

err:
  spin_unlock_irqrestore(&priv->slock, flags);
  return NULL;
}

#endif /* CONFIG_ESP32S3_I2S */


/****************************************************************************
 * [V222] Self-contained I2S0 TX one-shot write.
 *
 * Written directly against the authoritative ESP32-S3 register definitions
 * (soc/gdma_struct.h, soc/i2s_struct.h) and following ESP-IDF's ordering,
 * instead of the driver's hand-rolled txdma helpers.  Every call re-derives
 * the whole chain from scratch so no stale state can survive, and completion
 * is POLLED rather than taken from the GDMA interrupt.
 *
 * Returns the number of bytes written, or a negated errno.
 ****************************************************************************/

static struct esp32s3_dmadesc_s g_raw_desc __attribute__((aligned(4)));
static int g_raw_log = 4;

int esp32s3_i2s_tx_write_raw(FAR struct i2s_dev_s *dev,
                             FAR const uint8_t *buf, size_t len)
{
  struct esp32s3_i2s_s *priv = (struct esp32s3_i2s_s *)dev;
  const int16_t *samp;
  size_t total;                 /* number of 16-bit mono samples */
  size_t i = 0;
  int port;
  struct timespec t0;
  struct timespec now;
  uint32_t spin;
  uint32_t k;

  if (dev == NULL || buf == NULL || len < 2 || (len & 1) != 0)
    {
      return -EINVAL;
    }

  port = priv->config->port;
  samp  = (const int16_t *)buf;
  total = len / 2;

  /* 1. Bring the TX side up exactly as before: reset, push the configuration
   *    into the I2S clock domain, then start.  The DMA engine is deliberately
   *    not touched -- it cannot fetch descriptors on this board (measured at
   *    register level: outlink parks, no fetch, no error flag, with every
   *    clock and power domain verified enabled). */

  if (!g_tx_streaming)
      /* [V246] *** THE ROOT CAUSE ***
       *
       * es8311.c:1465 calls
       *     esp32s3_i2s_set_rxonly(priv->i2s,
       *                             priv->audio_mode != ES_MODULE_DAC);
       * This board runs the codec as ES_MODULE_ADC_DAC, so the argument is
       * true and g_i2s_rxonly_port becomes 0.  i2s_tx_active() then returns
       * false for port 0, so the "configure the TX module" block inside
       * i2s_configure() (TX reset, slave mode, data width, TDM/WS setup,
       * clock) is SKIPPED ENTIRELY.  The TX module is therefore never
       * brought up: it neither shifts data onto DOUT (measured stuck at 0
       * while BCLK/WS/MCLK ran) nor raises a DMA request, so the GDMA
       * outlink stays parked.  Nothing ever calls it with on=false.
       *
       * Take the port back for TX.  set_rxonly(false) clears the override
       * and re-runs i2s_configure(), which restores the whole TX block. */

      syslog(LOG_INFO, "[RAW] rxonly_port=%d -> taking TX ownership\n",
             g_i2s_rxonly_port);

      esp32s3_i2s_set_rxonly(dev, false);

      syslog(LOG_INFO, "[RAW] rxonly_port=%d after TX bring-up\n",
             g_i2s_rxonly_port);

      /* [V240] The XTAL source experiment (V238) is REVERTED.
       *
       * Forcing I2S_TX_CLK_SEL=0 and recomputing the dividers from 40 MHz made
       * the speaker output disappear entirely (user report), while the previous
       * CLK160 selection produced audible output.  Whatever the MCLK path does
       * differently in the XTAL case, it is not worth chasing: the clock source
       * is back to i2s_configure()'s choice. */

    {
    modifyreg32(I2S_TX_CONF_REG(port), I2S_TX_START, 0);
    modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_RESET);
    modifyreg32(I2S_TX_CONF_REG(port), I2S_TX_RESET, 0);
    modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_FIFO_RESET);
    modifyreg32(I2S_TX_CONF_REG(port), I2S_TX_FIFO_RESET, 0);

    modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_UPDATE);
    for (spin = 0;
         spin < I2S_UPDATE_SPIN_MAX &&
         (getreg32(I2S_TX_CONF_REG(port)) & I2S_TX_UPDATE);
         spin++)
      {
      }

    modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_UPDATE);
    for (spin = 0;
         spin < I2S_UPDATE_SPIN_MAX &&
         (getreg32(I2S_TX_CONF_REG(port)) & I2S_TX_UPDATE);
         spin++)
      {
      }
    modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_START);

      /* [V236] tx_conf lives in the APB domain; only tx_update pushes it
       * into the I2S TX clock domain.  Set start first, then update once
       * more so the clock domain actually sees tx_start -- otherwise the
       * serialiser never reads the FIFO and every write is dropped. */
      modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_UPDATE);
      for (spin = 0;
           spin < I2S_UPDATE_SPIN_MAX &&
           (getreg32(I2S_TX_CONF_REG(port)) & I2S_TX_UPDATE);
           spin++)
        {
        }

      clock_gettime(CLOCK_MONOTONIC, &t0);
      g_tx_start_ns = (long long)t0.tv_sec * 1000000000LL + t0.tv_nsec;
      g_tx_delivered = 0;
      g_tx_streaming = 1;
    }

  /* 2. Push PCM straight into the 32-bit TX FIFO, paced in real time.
   *
   *    One FIFO word is one frame (left+right slot), so a mono sample is
   *    duplicated into both halves.  That is correct whether the hardware
   *    reads one 32-bit frame or two 16-bit slots per FIFO entry.
   *
   *    Pacing: 16 kHz mono => one frame every 62.5 us.  The FIFO holds 64
   *    frames (4 ms); pushing 48 frames (3 ms) per burst keeps it between
   *    16 and 64 entries, so neither an underrun nor an overflow can occur
   *    even with the +-1 ms granularity of usleep().  Long waits yield the
   *    CPU so Wi-Fi and the rest of the system keep running; only the last
   *    millisecond is busy-waited for accuracy. */

  /* [V244] Prove the DATA pin is actually carrying bits.
   *
   * BCLK/WS/MCLK were verified toggling earlier, but the serial data output
   * was never checked.  If DOUT (GPIO7) is stuck the codec receives clocks and
   * no data -- exactly the "amplifier hiss, no tone" symptom.  The pins are
   * driven through the GPIO matrix, so enabling only the pad input buffer makes
   * the level readable. */
  if (g_raw_log > 0)
    {
      unsigned int dhi = 0;
      unsigned int chi = 0;
      unsigned int k;

      modifyreg32(IO_MUX_GPIO7_REG, 0, (1u << 9));    /* FUN_IE */
      modifyreg32(IO_MUX_GPIO10_REG, 0, (1u << 9));

      for (k = 0; k < 4000; k++)
        {
          if (esp32s3_gpioread(7))
            {
              dhi++;
            }

          if (esp32s3_gpioread(10))
            {
              chi++;
            }
        }

      syslog(LOG_INFO,
             "[RAW] pins: DOUT(7) hi=%u/4000  BCLK(10) hi=%u/4000\n",
             dhi, chi);
    }

  /* [V245] Feed the I2S through the GDMA using ESP-IDF's LL sequence.
   *
   * ESP32-S3 has no CPU-accessible I2S FIFO port -- 0x00/0x04/0x08 are
   * reserved in the SoC definition (the first real I2S register is int_raw at
   * 0x0C) -- so the earlier polled-FIFO writer was pushing samples into
   * reserved space.  That is why DOUT (GPIO7) measured stuck at 0 while BCLK
   * ran, and why the codec produced only its noise floor.  The I2S FIFO can
   * only be fed by the GDMA, so the outlink has to work.
   *
   * The sequence mirrors ESP-IDF's gdma_ll_tx_* helpers one for one, and
   * samples out.dscr (the descriptor the FSM has pre-fetched) plus link.park
   * (FSM idle) so the outlink state is observable instead of inferred. */

  {
    uint32_t base = DMA_OUT_CONF0_CH0_REG + priv->dma_channel *
                                             GDMA_REG_OFFSET;
    uint32_t link = DMA_OUT_LINK_CH0_REG + priv->dma_channel *
                                             GDMA_REG_OFFSET;

    g_raw_desc.ctrl = (uint32_t)(len & ESP32S3_DMA_CTRL_BUFLEN_V) |
                      ((uint32_t)(len & ESP32S3_DMA_CTRL_DATALEN_V)
                       << ESP32S3_DMA_CTRL_DATALEN_S) |
                      ESP32S3_DMA_CTRL_EOF | ESP32S3_DMA_CTRL_OWN;
    g_raw_desc.pbuf = buf;
    g_raw_desc.next = NULL;

    modifyreg32(base, 0, 1u << 0);          /* out_rst = 1            */
    modifyreg32(base, 1u << 0, 0);          /* out_rst = 0            */

    SET_GDMA_CH_REG(DMA_OUT_PERI_SEL_CH0_REG, priv->dma_channel, 3);

    CLR_GDMA_CH_BITS(DMA_OUT_CONF1_CH0_REG, priv->dma_channel,
                     DMA_OUT_CHECK_OWNER_CH0);

    modifyreg32(base, 0, (1u << 4));        /* outdscr_burst_en       */
    modifyreg32(base, 1u << 5, 0);          /* out_data_burst_en off  */

    SET_GDMA_CH_REG(DMA_OUT_INT_CLR_CH0_REG, priv->dma_channel, UINT32_MAX);
    SET_GDMA_CH_REG(DMA_OUT_INT_ENA_CH0_REG, priv->dma_channel, 0);

    putreg32((uint32_t)&g_raw_desc, link);

    syslog(LOG_INFO, "[RAW] gdma: link=0x%08x base=0x%08x desc=0x%08x\n",
           (unsigned)getreg32(link), (unsigned)getreg32(base),
           (unsigned)((uint32_t)&g_raw_desc));

    SET_GDMA_CH_BITS(DMA_OUT_LINK_CH0_REG, priv->dma_channel,
                     DMA_OUTLINK_START_CH0_M);

    modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_UPDATE);
    for (spin = 0;
         spin < I2S_UPDATE_SPIN_MAX &&
         (getreg32(I2S_TX_CONF_REG(port)) & I2S_TX_UPDATE);
         spin++)
      {
      }
    modifyreg32(I2S_TX_CONF_REG(port), 0, I2S_TX_START);

    for (k = 0; k < 200000; k++)
      {
        uint32_t lnk = getreg32(link);
        uint32_t dsc = GET_GDMA_CH_REG(DMA_OUT_DSCR_CH0_REG,
                                       priv->dma_channel);

        if (k == 0 || k == 50000)
          {
            syslog(LOG_INFO,
                   "[RAW] fsm k=%u park=%d start=%d dscr=0x%08x owner=%d\n",
                   (unsigned)k, !!(lnk & 0x00800000u), !!(lnk & 0x00200000u),
                   (unsigned)dsc,
                   !!(g_raw_desc.ctrl & ESP32S3_DMA_CTRL_OWN));
          }

        if ((g_raw_desc.ctrl & ESP32S3_DMA_CTRL_OWN) == 0)
          {
            break;
          }

        up_udelay(10);
      }

    syslog(LOG_INFO, "[RAW] gdma done: owner=%d waited=%u dscr=0x%08x\n",
           !!(g_raw_desc.ctrl & ESP32S3_DMA_CTRL_OWN), (unsigned)k,
           (unsigned)GET_GDMA_CH_REG(DMA_OUT_DSCR_CH0_REG,
                                     priv->dma_channel));
  }


  if (g_raw_log > 0)
    {
      g_raw_log--;
      clock_gettime(CLOCK_MONOTONIC, &now);
      syslog(LOG_INFO,
             "[RAW] FIFO wrote %u samples (%u bytes) in %lld us; "
             "txconf=0x%08x\\n",
             (unsigned)total, (unsigned)len,
             (long long)((now.tv_sec - t0.tv_sec) * 1000000LL +
                         (now.tv_nsec - t0.tv_nsec) / 1000LL),
             (unsigned)getreg32(I2S_TX_CONF_REG(port)));
    }

  return (int)len;
}
