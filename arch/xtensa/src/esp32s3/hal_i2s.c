/****************************************************************************
 * arch/xtensa/src/esp32s3/hal_i2s.c
 *
 * Refactored on 2026-08-25: migrated from "hand-written register direct
 * writes" to
 * Wrapper layer for the NuttX official esp32s3_i2s driver (keeping the
 * hal_i2s.h API unchanged).
 *
 * Background (why migrate):
 * the handwritten register version mishandles/omits several things
 * automatically processed by the driver framework:
 * 1) RX_RESET clears clock domain frame configuration → RX 48k misaligned
 * noise ("sounds the same whether talking or not")
 * 2) Wrong DMA completion signal used (I2S TX_DONE=FIFO empty false
 * completion) → playback truncation "audible click"
 * 3) GDMA OUT/IN share CPU interrupt line → interrupt storm deadlock
 * 4) neural network-level noise reduction is missing → manual filtering
 * cannot handle ES7210 wideband noise
 * Complete implementation from the official driver (esp32s3_i2s.c):
 * automatic clock division, DMA/EOF interrupts,
 * Full-duplex SIG_LOOPBACK, configured playback, same architecture as the
 * IDF example.
 *
 * ⚠️ 2026-09-02 Design finalized (most memory efficient, do not revert
 * to pipeline/double-buffer/ring queue):
 * We once tried an "RX pipeline (2 APB in-flight to simulate continuous TX)"
 * -- introduced complexity and was reverted.
 * new bugs (pre/post-commit interval overlap, busy-waiting, callback race
 * conditions), and actual recording tests
 * Early exit. Root cause analysis: the diagnostic speech segment's "constant
 * non-zero rate of 69%" was not RX data loss,
 * But ai_voice.c's s_out[1600] out-of-bounds (every 4 blocks write
 * [1422..1896) exceeds 1600)
 * write-through to adjacent s_chunk48 + callback out-of-bounds read
 * **statistical corruption**. After fixing the out-of-bounds, reverted to
 * The official driver's native "single APB serial" is sufficient—zero
 * extra memory, zero concurrency complexity.
 * Prerequisite: hal_i2s_read_slot fills its internal loop on a single call
 * (inter-block processing < I2S RX
 * FIFO depth (64B≈0.67ms) no overflow; verification see diag non-zero rate
 * ≥95%).
 *
 * ⚠️ 2026-09-02 TX/RX switching (three-strategy fusion finalized, fix
 * plant voice loop failure):
 * Phenomenon: After recording (pre-read chain running) and playback: the
 * first hal_i2s_write fails,
 * Print hal_i2s_write=-5. Old fix only sets s_rx_prefetch_paused flag—
 * in-flight RX APB (≤4092B≈43ms@24k stereo) still holds RX DMA, TX and
 * RX
 * Concurrent startup fails (measured behavior under official driver's single
 * DMA semantics).
 * Fusion: 1. Strict channel lifecycle: stop RX before sending TX
 * (Equivalent to i2s_channel_disable(rx) → write(tx)); ②OpenVela
 * strategy:
 * use official driver primitives + actual errno (hal_i2s_write no longer
 * folds errors into -EIO,
 * Failure with I2S register context); ③ XiaoZhi strategy: retain circular
 * FIFO prefetch chain to ensure
 * Continuous recording (chain syncs stop before TX, next read_slot
 * auto-restarts).
 * Implementation: hal_i2s_rx_prefetch_stop() sets flag then【sync wait for
 * RX DMA idle】
 * (poll in-flight count s_rx_prefetch_inflight to zero, ≤300ms fallback),
 * then send TX.
 *
 * This wrapper keeps hal_i2s.h API unchanged (voice_agent/ai_voice does not
 * need to change calls):
 * - TX: official driver 2-slot stereo (ES8311 DAC natively matched)
 * - RX: Official driver 2-slot stereo (ES7210 2-mic standard mode, LRCK
 * low=left/MIC1),
 * hal_i2s_read/read_slot takes mono (slot 0=left MIC1 / 1=right MIC2)
 * - Sample rate 24kHz / 16bit (Kconfig:
 * CONFIG_ESP32S3_I2S0_SAMPLE_RATE=24000)
 * - MCLK=GPIO2 BCLK=GPIO17 WS=GPIO45 DOUT=GPIO15 DIN=GPIO16 PA=GPIO46
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>
#include <sys/param.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>

#include "xtensa.h"
#include "esp32s3_gpio.h"
#include "hardware/esp32s3_i2s.h"
#include "hal_i2s.h"

/* Official driver bus initialization (resolved at link time) */

extern FAR struct i2s_dev_s *esp32s3_i2sbus_initialize(int port);

/* Sampling parameters (consistent with hal_i2s.h; Kconfig must configure
 * 24000/16bit/2ch)
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HAL_RATE        24000
#define HAL_BITS        16
#define HAL_FRAME_BYTES 4 /* stereo 2 slots × 16bit */

static FAR struct i2s_dev_s *g_dev;
static sem_t g_tx_done_sem;   /* TX post once per block completion (windowed pipeline counting) */
static int g_tx_result;
static int g_rx_result;

/* ⚠️ 2026-09-02 TX pipeline: official driver single i2s_send ≤ 2044B,
 * if per-block "Queue→wait complete→queue again", DMA idle between
 * blocks (HPWORK worker round-trip > FIFO headroom) → playback glitch
 * (pure tone "squeak-beep"). Changed to continuous queueing within window,
 * after EOF Official driver ISR directly chains pending queue → seamless
 * DMA.
 */

#define HAL_TX_MAX_INFLIGHT   3   /* Maximum in-flight i2s_send blocks (driver container pool=4, leaving 1 margin) */

static int s_tx_inflight = 0;     /* Number of queued but incomplete blocks (single TX user, application thread exclusive) */

/* RX ring FIFO + prefetch auto-renew (2026-09-01 data integrity fix)
 *
 * Background: NuttX official esp32s3_i2s.c RX is "single transfer"
 * (descriptors non-circular, Each apb stops when full). Original scheme:
 * 'single pre-read buffer (1023 samples/42.6ms) + read_slot' Manual renewal
 * "during read_slot pause (speech block callback/HTTP upload/WiFi jitter)
 * DMA stop → FIFO(64B≈0.67ms) overflow data loss → voice
 * glitch/missing (user-reported "can't hear voice").
 *
 * Fix (aligning with XiaoZhi IDF circular DMA + auto_clear_after_cb approach
 * NuttX equivalent):
 * 1. Ring FIFO 2048 samples (4KB static): RX data continuously enqueued,
 * read_slot on-demand
 * Fetch; FIFO full overwrites oldest (equivalent to auto_clear, no residual
 * aliasing)."
 * 2. Prefetch auto-renew: in prefetch APB completion callback [immediately
 * submit next round] → DMA continuous receive,
 * During read_slot/callback/upload no stop → no FIFO overflow data loss.
 * 3. Callback (worker) writes to FIFO, read_slot (calling thread) reads,
 * critical section protects pointers.
 *
 * Memory cost: +4KB static (original 2KB prefetch buffer deleted, net +2KB
 * .bss).
 */

#define HAL_RX_FIFO_SAMPLES 2048 /* ring FIFO sample count (4KB) */
#define HAL_RX_FIFO_MASK     (HAL_RX_FIFO_SAMPLES - 1)

/* ⚠️ 2026-09-02 RX dual-deep prefetch ("recording stuttering" root cause
 * fix): Hardware RX FIFO only 64B≈0.67ms. Single-depth pre-read = after
 * receiving 1 apb (21ms) only then by Worker submits next → APB inter-DMA
 * idle window (worker round-trip >0.67ms) → periodic sample loss
 * (recording wall clock ≈2× nominal, playback stutters). Depth 2:
 * prefetch chain always maintains 2 APBs in transit (1 active receiving + 1
 * pending queued) → APB EOF interrupt (ISR, microsecond-level) directly
 * load next from pend → seamless. Hardware GDMA single-channel serial +
 * driver rx.act non-null protection → Two APBs never overlap receiving
 * data. Memory increment only +1 APB (~4KB instantaneous heap, segment
 * length/ring FIFO unchanged).
 */

/* ⚠️ 2026-09-16 Finalized: depth must be 2, cannot increase further!
 * Tried 6 → board directly freezes (plant voice raw/rec all no output).
 * Reason in official driver:
 * 1) Driver's built-in container pool only has
 * CONFIG_ESP32S3_I2S_MAXINFLIGHT=4 entries.
 * (esp32s3_i2s.c i2s_buf_initialize), and i2s_buf_allocate() is
 * nxsem_wait_uninterruptible —— permanent block if pool empty;
 * 2) esp32s3_i2s.c RX completion check also designed as '1 receiving + 1
 * queued'.
 * For deeper pipeline, must simultaneously modify driver completion logic
 * and enlarge container pool, high risk. Conclusion: depth 2, rely on
 * "application side not putting slow I/O in capture callback" to ensure no
 * sample loss.
 */
#define HAL_RX_PREFETCH_DEPTH  2    /* Prefetch in-transit APB depth (A/B ping-pong) */

static int16_t s_rx_fifo[HAL_RX_FIFO_SAMPLES];    /* ring buffer (static 4KB) */
static volatile uint32_t s_rx_fifo_rd;            /* Read pointer */
static volatile uint32_t s_rx_fifo_cnt;           /* valid sample count */

/* ⚠️ 2026-09-16 Dropped Sample Count (Recording Quality Regression
 * Metric): Covers overwrite when RX ring FIFO is full Oldest sample,
 * equivalent to dropping the sample → timeline compressed (speech becomes
 * faster/incomplete/unclear). This counter increments rapidly when the
 * collection thread is preempted from CPU by slow I/O. 0 = none dropped.
 */

static volatile uint32_t s_rx_fifo_drop;
static sem_t   s_rx_prefetch_sem;                /* Data arrival notification */
static struct hal_i2s_rx_ctx s_rx_prefetch_ctx;  /* Prefetch slot context (shared slot) */
static volatile bool s_rx_prefetch_paused;       /* Pause prefetch chain during TX playback */

/* ⚠️ 2026-09-02: In-flight RX apb count (depth 2, replacing single
 * boolean 'active')— Target <= HAL_RX_PREFETCH_DEPTH; TX synchronization
 * wait is determined by count reaching zero. Increment/decrement occurs in
 * critical section (concurrent between read_slot application thread and
 * worker callback).
 */

static volatile uint32_t s_rx_prefetch_inflight;  /* In-flight RX APB count */

/* ⚠️ 2026-09-16 Static RX apb pool (root cause fix for "sporadic heap
 * corruption -> board hang")
 *
 * Measured root cause: The official apb_alloc() allocates the apb structure
 * and sampling buffer **separately** from the UMM heap. Allocation, while
 * this board's UMM heap includes PSRAM region (CONFIG_MM_REGIONS=2). WiFi
 * association etc. During heap jitter, both may land in external memory
 * (measured apb=0x3c1c2018), at which point the DMA target No longer in
 * internal SRAM; and during recording generates one alloc/free pair every
 * ~21ms (~90 heap operations/sec) causes the heap to become long-term
 * fragmented. On-site consequences (both observed):
 * ① hpwork: apb_free → free → mm_malloc_size.c:78 assertion
 * ② Callback reads apb->samp (hal_i2s.c:217) triggering LoadProhibited,
 * VADDR is garbage
 * Fix: Initialize pool once at startup, use static buffers for sample
 * buffers (guaranteed in internal SRAM, 32-byte alignment = cache line), and
 * reserves a "permanent reference" for each apb to The reference count of
 * apb_free never drops to 1 → zero runtime allocation, zero deallocation,
 * DMA always reachable.
 *
 * Reference count deduction (each round): this function +1, i2s_receive +1,
 * completion callback -1, Official driver: -1 -> net 0 per cycle, count
 * remains >=2.
 */

#define HAL_RX_POOL_N       4      /* Pool size must be > prefetch depth (2) */
#define HAL_RX_SAMP_N       1024   /* 2044B limit = 1022 samples, using 1024 */
#define HAL_RX_SAMP_BYTES   (HAL_RX_SAMP_N * 2)

static int16_t s_rx_pool_samp[HAL_RX_POOL_N][HAL_RX_SAMP_N]
              __attribute__((aligned(32)));
static FAR struct ap_buffer_s *s_rx_pool_apb[HAL_RX_POOL_N];
static bool  s_rx_pool_ready;      /* Pool ready */
static int   s_rx_pool_next;       /* rotation index */

static int hal_i2s_rx_pool_init(void)
{
  struct audio_buf_desc_s desc;
  int i;

  if (s_rx_pool_ready)
    {
      return 0;
    }

  for (i = 0; i < HAL_RX_POOL_N; i++)
    {
      s_rx_pool_apb[i] = NULL;
      memset(&desc, 0, sizeof(desc));
      desc.numbytes  = 32;              /* Placeholder, actual buffer uses s_rx_pool_samp */
      desc.u.pbuffer = &s_rx_pool_apb[i];

      if (apb_alloc(&desc) < 0 || s_rx_pool_apb[i] == NULL)
        {
          printf("[I2S] RX apb pool allocation failed (slot %d)\n", i);
          return -ENOMEM;
        }

      apb_reference(s_rx_pool_apb[i]);  /* Permanent reference (see above deduction) */
      s_rx_pool_apb[i]->samp = (FAR uint8_t *)s_rx_pool_samp[i];
    }

  s_rx_pool_ready = true;
  return 0;
}

static int hal_i2s_rx_prefetch_submit(int slot);
static void hal_i2s_rx_prefetch_start(int slot);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Internal: DMA completion callback (synchronous wrapper for official
 * driver's async API)
 *
 * ⚠️ 2026-08-25 apb reference race condition fix (UAF → heap
 * corruption/leak):
 * Official driver completion path = callback() → apb_free(apb) (releases
 * driver reference).
 * If the caller only calls apb_free after sem wakeup, it may preempt the
 * official driver and take crefs
 * Drop to 0 frees memory -> official driver then accesses freed memory in
 * apb_free (UAF).
 * Fix: apb_free within callback (our reference 2→1), official driver
 * subsequently 1→0.
 * Final release—all release order is within worker thread, no race
 * condition; data extraction also moved to
 * callback (callback does not access apb after returning).
 ****************************************************************************/

struct hal_i2s_rx_ctx
{
  FAR int16_t *dst;   /* destination mono buffer */
  int           slot; /* 0=left MIC1 / 1=right MIC2 */
  uint32_t      got;  /* Number of samples extracted */
};

static void hal_i2s_tx_cb(FAR struct i2s_dev_s *dev,
                          FAR struct ap_buffer_s *apb,
                          FAR void *arg, int result)
{
  g_tx_result = result;

  /* Release our reference; official driver then releases (1→0) completing
   * final release
   */

  apb_free(apb);
  sem_post(&g_tx_done_sem);
}

/* Pre-reading apb completion callback (worker thread): ring-buffer FIFO
 * write + post + auto-replenish (continuous DMA)
 */

static void hal_i2s_rx_prefetch_cb(FAR struct i2s_dev_s *dev,
                                   FAR struct ap_buffer_s *apb,
                                   FAR void *arg, int result)
{
  FAR struct hal_i2s_rx_ctx *ctx = (FAR struct hal_i2s_rx_ctx *)arg;
  uint32_t nframe = (uint32_t)(apb->nbytes / 2);
  FAR const int16_t *src = (FAR const int16_t *)apb->samp;
  uint32_t i;
  irqstate_t flags;

  (void)dev;
  g_rx_result = result;

  /* In-flight count -1 (critical section: concurrent with read_slot's submit
   * reservation). DMA for this apb is complete; pre-synchronization wait for
   * TX before transmission uses this to check idle.
   */

  flags = enter_critical_section();
  if (s_rx_prefetch_inflight > 0)
    {
      s_rx_prefetch_inflight--;
    }

  leave_critical_section(flags);

  apb_free(apb);                  /* Release first, APB pool freed up */

  /* Ring-buffer FIFO write (overwrite oldest on full = equivalent to
   * auto_clear, no residual aliasing) ⚠️ 2026-09-08 Fix: write position
   * must be ring tail rd+cnt; old method was wr+cnt. (wr is never updated,
   * always equals writing at absolute index cnt; when rd>0, it periodically
   * Reading unwritten bucket slot (static initialized 0) → every ~61ms a
   * segment of zero data, Audible "hissing" noise floor during recording.
   */

  flags = enter_critical_section();
  for (i = 0; i < nframe / 2; i++)
    {
      uint32_t w = (s_rx_fifo_rd + s_rx_fifo_cnt) & HAL_RX_FIFO_MASK;

      s_rx_fifo[w] = src[2 * i + ctx->slot];
      if (s_rx_fifo_cnt < HAL_RX_FIFO_SAMPLES)
        {
          s_rx_fifo_cnt++;
        }
      else
        {
          s_rx_fifo_rd = (s_rx_fifo_rd + 1) & HAL_RX_FIFO_MASK;
          s_rx_fifo_drop++;      /* FIFO full → overwrite oldest = sample drop (counted) */
        }
    }

  leave_critical_section(flags);
  sem_post(&s_rx_prefetch_sem);   /* Notify read_slot that data is available */

  /* ⚠️ Auto-renew to depth 2: this APB completes → add one to pend, to
   * keep queue always ≥1 While waiting -> next apb's EOF interrupt (ISR)
   * directly loads, no worker idle gap. Only resumes if unpaused
   * (playing)—otherwise stop is ineffective, TX remains concurrent with
   * RX.
   */

  if (!s_rx_prefetch_paused)
    {
      hal_i2s_rx_prefetch_submit(ctx->slot);
    }
}

/* Submit a prefetched RX apb (non-blocking). The window (≤
 * HAL_RX_PREFETCH_DEPTH) uses Critical section "check+reserve"
 * guarantee—read_slot (application thread) and completion callback
 * (worker) May be called concurrently, prevents deep/duplicate submissions.
 */

static int hal_i2s_rx_prefetch_submit(int slot)
{
  FAR struct ap_buffer_s *apb;
  uint32_t want;
  int idx;
  int ret = -ENOMEM;
  irqstate_t flags;

  /* Window check + reservation (critical section) */

  flags = enter_critical_section();
  if (s_rx_prefetch_paused ||
      s_rx_prefetch_inflight >= HAL_RX_PREFETCH_DEPTH)
    {
      leave_critical_section(flags);
      return 0;   /* Paused / window full */
    }

  s_rx_prefetch_inflight++;       /* Reserved: this apb will be enqueued */
  leave_critical_section(flags);

  want = HAL_RX_FIFO_SAMPLES * 2;   /* Stereo 2 slots × 16bit = mono samples × 4 */
  if (want > HAL_RX_SAMP_BYTES)
    {
      want = HAL_RX_SAMP_BYTES;     /* must not exceed static buffer */
    }

  if (want > 2044)
    {
      /* ⚠️ 2026-09-02 Memory adaptation: original 4092B apb when system
       * heap is low (free≈3.2KB/largest≈3.2KB, LVGL 80KB + Camera 37.5KB
       * static squeeze) apb_alloc fails → prefetch chain not started →
       * RX_START=0 → 0 data. Reduce 2044B (511 frames ≈21ms/round) to
       * ensure apb heap allocation always succeeds. after double depth, two
       * apb in-flight ≈ +4KB instant heap (heap free 22KB, no pressure).
       */

      want = 2044;
    }

  if (hal_i2s_rx_pool_init() < 0)
    {
      goto errout_with_resv;
    }

  idx = s_rx_pool_next;
  s_rx_pool_next = (s_rx_pool_next + 1) % HAL_RX_POOL_N;
  apb = s_rx_pool_apb[idx];

  s_rx_prefetch_ctx.slot = slot;
  s_rx_prefetch_ctx.got = 0;

  apb_reference(apb);          /* +1 for this cycle (apb in pool is never truly released) */
  apb->samp      = (FAR uint8_t *)s_rx_pool_samp[idx];
  apb->nmaxbytes = want;
  apb->nbytes    = want;
  apb->curbyte   = 0;

  ret = g_dev->ops->i2s_receive(g_dev, apb, hal_i2s_rx_prefetch_cb,
                                &s_rx_prefetch_ctx, SEC2TICK(3));
  if (ret < 0)
    {
      apb_free(apb);           /* Release the reference added this cycle */
      goto errout_with_resv;
    }

  return 0;

errout_with_resv:

  /* Enqueue failure: rollback reservation (retry next read_slot/callback) */

  flags = enter_critical_section();
  if (s_rx_prefetch_inflight > 0)
    {
      s_rx_prefetch_inflight--;
    }

  leave_critical_section(flags);
  return ret;
}

/* Start / top-up prefetch chain to depth 2 (idempotent: if already running,
 * submit automatically skips due to window full). read_slot calls every time
 * to ensure the chain is running and full.
 */

static void hal_i2s_rx_prefetch_start(int slot)
{
  s_rx_prefetch_paused = false;
  s_rx_prefetch_ctx.slot = slot;   /* Fix slot at chain start (unchanged throughout recording) */

  while (s_rx_prefetch_inflight < HAL_RX_PREFETCH_DEPTH)
    {
      if (hal_i2s_rx_prefetch_submit(slot) != 0)
        {
          break;   /* Allocation failure etc.: read_slot retry on next entry */
        }
    }
}

/* Pause prefetch chain (called during TX playback): after in-flight apb
 * completes, callback will not continue chain -> RX DMA stops, apb pool
 * releases to TX. read_slot will auto-restart on next call.
 *
 * ⚠️ 2026-09-02 Three-strategy merged fix (plant voice loop playback -5
 * root cause): Setting paused only is insufficient - [in-transit RX apb
 * still occupies RX DMA], starting TX here causes practical issues failure.
 * Must synchronously wait for RX DMA to be truly idle (in-flight count
 * returns to zero) before returning:
 * ① Xiaozhi strategy: strict channel lifecycle, stop RX before
 * transmitting TX (equivalent
 * i2s_channel_disable(rx) → i2s_channel_write(tx)）；
 * ② OpenVela strategy: Official driver uses single DMA, callback signals
 * transfer complete;
 * ③ Implement small companion strategy: pre-read chain maintains recording
 * continuity, only synchronous stop before TX here.
 * Race condition boundary: under extreme conditions, two prefetch apb may be
 * in-flight simultaneously (active flag will to clearing it as soon as the
 * first one completes), so use s_rx_prefetch_inflight count to
 * determine——each apb enqueue +1, callback complete -1, zero = all RX
 * DMA done. 300ms fallback to prevent RX dead chain causes infinite waiting.
 */

static void hal_i2s_rx_prefetch_stop(void)
{
  int wait_ms = 0;

  /* ⚠️ set paused: callback will see this and not continue chain; after
   * in-flight apb completes, chain naturally stops. Note: cannot clear
   * active/inflight—they are the ground truth for "in-flight RX apbs,"
   * Clearing it invalidates the wait below; callback updates it
   * automatically.
   */

  s_rx_prefetch_paused = true;

  /* Sync wait for RX DMA idle: in-transit count zeroed (all RX apb DMA
   * completed) Do not send TX before
   */

  while (s_rx_prefetch_inflight > 0 && wait_ms < 300)
    {
      nxsig_usleep(1000);
      wait_ms++;
    }
}

/****************************************************************************
 * Public API
 ****************************************************************************/

int hal_i2s_init(void)
{
  int ret;

  if (g_dev != NULL)
    {
      return 0;
    }

  sem_init(&g_tx_done_sem, 0, 0);
  sem_init(&s_rx_prefetch_sem, 0, 0);
  s_rx_prefetch_paused = false;
  s_rx_prefetch_inflight = 0;

  /* PA enable (GPIO46, high = speaker amplifier on) */

  esp32s3_configgpio(HAL_PA_PIN, OUTPUT);
  esp32s3_gpiowrite(HAL_PA_PIN, true);

  /* Official driver init (per Kconfig: master, 24k, 16bit, 2-slot full
   * duplex; Auto-configures clock/DMA/interrupt internally; TX+RX channels
   * already started)
   */

  g_dev = esp32s3_i2sbus_initialize(0);
  if (g_dev == NULL)
    {
      return -ENODEV;
    }

  /* Explicitly confirm runtime parameters (consistent with Kconfig, prevents
   * default drift)
   */

  if (g_dev->ops->i2s_txsamplerate != NULL)
    {
      g_dev->ops->i2s_txsamplerate(g_dev, HAL_RATE);
    }

  if (g_dev->ops->i2s_rxsamplerate != NULL)
    {
      g_dev->ops->i2s_rxsamplerate(g_dev, HAL_RATE);
    }

  if (g_dev->ops->i2s_txdatawidth != NULL)
    {
      g_dev->ops->i2s_txdatawidth(g_dev, HAL_BITS);
    }

  if (g_dev->ops->i2s_rxdatawidth != NULL)
    {
      g_dev->ops->i2s_rxdatawidth(g_dev, HAL_BITS);
    }

  if (g_dev->ops->i2s_txchannels != NULL)
    {
      g_dev->ops->i2s_txchannels(g_dev, 2);
    }

  if (g_dev->ops->i2s_rxchannels != NULL)
    {
      g_dev->ops->i2s_rxchannels(g_dev, 2);
    }

  /* ⚠️ 2026-08-25: Maintain BCK/WS output even when TX FIFO is empty
   * (STOP_EN=0). Official driver does not configure this bit - when
   * recording only RX is enabled, TX has no data stream, if FIFO empty clock
   * Stop -> RX (SIG_LOOPBACK shares TX clock) receives no data.
   */

  modifyreg32(I2S_TX_CONF_REG(0), I2S_TX_STOP_EN, 0);
  modifyreg32(I2S_TX_CONF_REG(0), 0, I2S_TX_UPDATE);
    {
      int sync_wait = 0;

      while ((getreg32(I2S_TX_CONF_REG(0)) & I2S_TX_UPDATE) &&
             sync_wait++ < 100000)
        {
        }
    }

  /* ⚠️ 2026-08-25 Root cause fix: TX_START must be explicitly set!
   * Official driver only sets TX_START when TX data is queued (esp32s3_i2s.c
   * L708, i2s_txdma_start) → record only reads RX (no TX data stream) →
   * TX_START=0 → TX serializer not working -> BCLK/WS not output -> ES7210
   * no clock -> RX 3s timeout (r=-110, actual test TX_CONF=0x08089200
   * bit2=0). Setting it + STOP_EN=0 → BCLK/WS continues outputting when
   * FIFO empty, RX clock stable.
   */

  modifyreg32(I2S_TX_CONF_REG(0), 0, I2S_TX_START);

  /* Drain residual semaphores (official driver may have completion events at
   * startup)
   */

  while (nxsem_trywait(&g_tx_done_sem) == 0);

  return 0;
}

int hal_i2s_start_tx_clock(void)
{
  /* In official driver initialization, TX channel already started (clock
   * continuously outputs, STOP_EN=0)
   */

  return 0;
}

/****************************************************************************
 * TX playback (2026-09-02 windowed pipeline—root cause fix for
 * "squeaky-beep-beep" issue)
 *
 * Background: Old version queued per 2044B block "queue → wait for EOF →
 * HPWORK worker callback → re-queue",
 * Between each block, DMA necessarily idles (worker round-trip > FIFO
 * margin), 1kHz pure tone is cut into
 * ~21ms segment, beat drop between segments -> hearing "squeaky bleeps".
 * 08-25 verified clean tone is
 * Single large DMA chain (no block gaps); after 09-02 chunking + apb
 * compression to 2044B, audio quality has never
 * Verified — this fix.
 *
 * Mechanism: queue HAL_TX_MAX_INFLIGHT blocks (official driver container
 * pool=4, keep 1
 * for RX boundary) then wait for completion -> after block EOF, official
 * driver ISR directly continues chain from pending queue
 * Next block (esp32s3_i2s.c i2s_tx_schedule), no worker round-trip idle
 * window → DMA
 * Continuous. Blocks only when window full waiting for a slot to free up
 * (official driver also blocks in i2s_send when pool full,
 * natural flow control, no deadlock).
 *
 * Note: i2s_send queues data via memcpy to internal buffer (txdma_setup),
 * Thus, buf can be safely reused after calling hal_i2s_write_async—tone
 * playback relies on this.
 * Implements "play previous block while generating next", eliminating block
 * boundary gaps.
 ****************************************************************************/

/* Queue a TX block (≤2044B, frame-aligned) into the official driver pend
 * queue, without waiting for completion. Returns OK / negative errno.
 * Official driver blocks internally when container pool is full.
 */

static int hal_tx_queue_block(FAR const void *buf, uint32_t bytes)
{
  struct audio_buf_desc_s desc;
  FAR struct ap_buffer_s *apb = NULL;
  int ret;

  memset(&desc, 0, sizeof(desc));
  desc.numbytes = bytes;
  desc.u.pbuffer = &apb;
  ret = apb_alloc(&desc);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(apb->samp, buf, bytes);
  apb->nbytes = bytes;
  apb->curbyte = 0;

  ret = g_dev->ops->i2s_send(g_dev, apb, hal_i2s_tx_cb, NULL, SEC2TICK(5));
  if (ret < 0)
    {
      /* Not enqueued: release our reference (failure path driver may have
       * already referenced)
       */

      printf("[I2S] i2s_send enqueue failed: %d (%u B)\n",
             ret, (unsigned)bytes);
      apb_free(apb);
      return ret;
    }

  s_tx_inflight++;
  return OK;
}

/* Wait until the number of in-flight TX blocks drops to want (want=0 to
 * drain all). Timeout indicates driver/clock anomaly: Clear semaphore to
 * prevent errors, then return -ETIMEDOUT.
 */

static int hal_tx_wait(int want)
{
  while (s_tx_inflight > want)
    {
      if (nxsem_tickwait(&g_tx_done_sem, SEC2TICK(5)) < 0)
        {
          printf("[I2S] TX drain timeout (inflight=%d)!\n", s_tx_inflight);
          while (nxsem_trywait(&g_tx_done_sem) == 0);
          s_tx_inflight = 0;
          return -ETIMEDOUT;
        }

      s_tx_inflight--;
    }

  return OK;
}

int hal_i2s_write_async(const void *buf, uint32_t bytes)
{
  uint32_t sent = 0;
  FAR const uint8_t *p = buf;
  int err;

  /* ⚠️ 2026-09-02: Before TX playback, [synchronously] stop prefetch
   * chain (-5 fix, idempotent) —— Ring FIFO prefetch auto-renewal keeps
   * RX DMA active; the official driver's i2s_send and TX fails to start
   * during RX concurrency. Time-slicing: stop RX during playback; read_slot
   * auto-restarts during recording.
   */

  hal_i2s_rx_prefetch_stop();

  while (sent < bytes)
    {
      uint32_t chunk = MIN(bytes - sent, 2044);   /* 2044B = heap capacity limit (see pre-read comments) */

      chunk -= (chunk % HAL_FRAME_BYTES);   /* Align to frame (4B) */

      if (chunk == 0)
        {
          break;
        }

      /* Window full → wait for a block to complete and free space (ISR has
       * seamlessly queued the next block, no gap)
       */

      if (s_tx_inflight >= HAL_TX_MAX_INFLIGHT)
        {
          err = hal_tx_wait(HAL_TX_MAX_INFLIGHT - 1);
          if (err < 0)
            {
              return err;
            }
        }

      err = hal_tx_queue_block(p + sent, chunk);
      if (err == -ENOMEM)
        {
          /* Heap pressure: drain in-flight items (reclaim apb/container)
           * then retry once
           */

          hal_tx_wait(0);
          err = hal_tx_queue_block(p + sent, chunk);
        }

      if (err < 0)
        {
          return err;
        }

      sent += chunk;
    }

  return (int)sent;
}

int hal_i2s_write_flush(void)
{
  return hal_tx_wait(0);
}

int hal_i2s_write(const void *buf, uint32_t bytes)
{
  int ret = hal_i2s_write_async(buf, bytes);

  if (ret < 0)
    {
      return ret;
    }

  return hal_i2s_write_flush() < 0 ? -ETIMEDOUT : ret;
}

int hal_i2s_read(void *buf, uint32_t bytes)
{
  return hal_i2s_read_slot(buf, bytes, 0);
}

int hal_i2s_read_slot(void *buf, uint32_t bytes, int slot)
{
  FAR int16_t *p = (FAR int16_t *)buf;
  uint32_t want_samp = bytes / 2;
  uint32_t got = 0;
  irqstate_t flags;

  /* Ensure pre-read chain is running and full (depth 2: first call /
   * auto-fill after playback restart)
   */

  hal_i2s_rx_prefetch_start(slot);

  /* Get data from ring FIFO: use existing first, if insufficient wait for
   * prefetch callback (auto-renew chain for continuous supply)
   */

  while (got < want_samp)
    {
      flags = enter_critical_section();
      while (s_rx_fifo_cnt > 0 && got < want_samp)
        {
          p[got++] = s_rx_fifo[s_rx_fifo_rd];
          s_rx_fifo_rd = (s_rx_fifo_rd + 1) & HAL_RX_FIFO_MASK;
          s_rx_fifo_cnt--;
        }

      leave_critical_section(flags);

      if (got >= want_samp)
        {
          break;
        }

      if (nxsem_tickwait(&s_rx_prefetch_sem, SEC2TICK(3)) < 0)
        {
          printf("[I2S] RX FIFO timeout (chain broken? TX_CONF=0x%08x "
                 "RX_CONF=0x%08x)\n",
                 (unsigned)getreg32(I2S_TX_CONF_REG(0)),
                 (unsigned)getreg32(I2S_RX_CONF_REG(0)));
          break;
        }
    }

  return (int)(got * 2);
}

uint32_t hal_i2s_rx_drop_count(void)
{
  return s_rx_fifo_drop;
}

int hal_i2s_rx_set_channel(int slot)
{
  /* Official driver RX is always 2-slot stereo; slot selection completes in
   * read_slot
   */

  (void)slot;
  return 0;
}

/****************************************************************************
 * Diagnostics: I2S TX status dump (for silent troubleshooting)
 *
 * Diagnosis when playback "completes" but no sound:
 * - TX_CONF bit2=TX_START(1=start), bit13=TX_STOP_EN(0=keep clock),
 * bit27=SIG_LOOPBACK；
 * - STATE bit0=TX_IDLE(1=serializer idle—normal after playback ends,
 * should be 0 during playback).
 ****************************************************************************/

void hal_i2s_dump_tx(void)
{
  uint32_t conf;
  uint32_t raw;

  printf("[I2S] TX_CONF=0x%08x TX_CONF1=0x%08x STATE=0x%08x "
         "TX_CLKM=0x%08x\n",
         (unsigned)getreg32(I2S_TX_CONF_REG(0)),
         (unsigned)getreg32(I2S_TX_CONF1_REG(0)),
         (unsigned)getreg32(I2S_STATE_REG(0)),
         (unsigned)getreg32(I2S_TX_CLKM_CONF_REG(0)));
  conf = getreg32(I2S_TX_CONF_REG(0));

  printf("[I2S] TX_START=%u TX_STOP_EN=%u TX_IDLE=%u SIG_LOOPBACK=%u\n",
         (unsigned)((conf & I2S_TX_START) ? 1 : 0),
         (unsigned)((conf & I2S_TX_STOP_EN) ? 1 : 0),
         (unsigned)((getreg32(I2S_STATE_REG(0)) & I2S_TX_IDLE) ? 1 : 0),
         (unsigned)((conf & I2S_SIG_LOOPBACK) ? 1 : 0));

  /* ⚠️ 2026-09-02 Added RX frame structure (ES7210 4 slots/64×FS
   * criteria): RX_TDM_CTRL bits[19:16]=TOT_CHAN_NUM(3=4 slots) lower 16
   * bits=channel enable; RX_CLKM divider should set BCLK=1.536MHz
   * (24k×4×16).
   */

  printf("[I2S] RX_TDM_CTRL=0x%08x RX_CONF=0x%08x RX_CLKM=0x%08x\n",
         (unsigned)getreg32(I2S_RX_TDM_CTRL_REG(0)),
         (unsigned)getreg32(I2S_RX_CONF_REG(0)),
         (unsigned)getreg32(I2S_RX_CLKM_CONF_REG(0)));

  /* ⚠️ 2026-09-02 Added I2S interrupt state (playback underrun
   * criterion): INT_RAW bit3=TX_HUNG (TX FIFO idle over threshold = DMA data
   * supply during playback bit1=TX_DONE, bit0=RX_DONE. Just after playback
   * ends, TX_HUNG=1 indicates Data stream has holes (gaps between blocks);
   * continuous playback should be all zeros.
   */

  raw = getreg32(I2S_INT_RAW_REG(0));

  printf("[I2S] INT_RAW=0x%08x (TX_HUNG=%u TX_DONE=%u RX_DONE=%u)\n",
         (unsigned)raw,
         (unsigned)((raw & I2S_TX_HUNG_INT_RAW) ? 1 : 0),
         (unsigned)((raw & I2S_TX_DONE_INT_RAW) ? 1 : 0),
         (unsigned)((raw & I2S_RX_DONE_INT_RAW) ? 1 : 0));
}
