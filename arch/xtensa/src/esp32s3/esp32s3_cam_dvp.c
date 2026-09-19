/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_cam_dvp.c
 *
 * ESP32-S3 LCD_CAM peripheral in CAM (input) mode: capture DVP 8-bit
 * parallel camera frames (OV3660 RGB565) via GDMA RX.
 *
 * Written NuttX-style, reusing the same GDMA + LCD_CAM infrastructure as
 * esp32s3_lcd.c (LCD is the same peripheral in output mode).
 *
 * Capture model (full-frame, modeled on IDF dvp_spi_lcd + esp_cam_ctlr_dvp):
 * - XMCLK(IO39) drives the sensor clock (board-layer LEDC, 20MHz)
 * - DMA RX chain covers one RGB565 frame (10 x 3840B desc = 38400B)
 * - cam_vs_eof_en = 1: VSYNC generates in_suc_eof = frame complete
 * - the GDMA EOF ISR re-arms the chain at every frame boundary (the
 * official start_trans() per-frame model): each DMA window is exactly
 * one frame, aligned to rxbuf[0] (this fixed the "pattern" misalignment)
 * - capture() masks the EOF IRQ, waits for the next EOF, freezes the
 * DMA and reads the stable frame; optional software RGB565 byte swap
 * (sensor sends high byte first, LVGL wants little-endian)
 * - GDMA RX channel IRQ (independent of LCD_CAM peripheral IRQ); the
 * LCD_CAM IRQ is used for the VSYNC frame-boundary counter (this
 * board's LCD is SPI-based, so ESP32S3_IRQ_LCD_CAM is free)
 *
 * [DIAG] Camera bring-up. Keep until capture is fully working.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "xtensa.h"
#include "esp32s3_gpio.h"
#include "esp32s3_dma.h"
#include "esp32s3_irq.h"
#include "esp32s3_spiram.h"
#include "hardware/esp32s3_soc.h"
#include "rom/cache.h"

/* 2026-09-09: ROM cache helper used by the I2S PSRAM DMA path; not declared
 * in the esp32s3 rom/cache.h header.
 */

extern int rom_cache_writeback_addr(uint32_t addr, uint32_t size)
  __asm__("rom_Cache_WriteBack_Addr");
extern int cache_invalidate_addr(uint32_t addr, uint32_t size)
  __asm__("Cache_Invalidate_Addr");
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_lcd_cam.h"
#include "hardware/esp32s3_dma.h"

#ifdef CONFIG_ESP32S3_CAM_DVP

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/* DVP pin mapping (CHQ ESP32-S3-BOX V2.0, corrected 2026-08-28 to match
 * the WORKING IDF reference esp32-camera config):
 *   Y2=IO12 Y3=IO10 Y4=IO9 Y5=IO11 Y6=IO13 Y7=IO21 Y8=IO38 Y9=IO40
 *   PCLK=IO14 HREF=IO41 VSYNC=IO42 XMCLK=IO39
 * Y2..Y9 are the sensor's 8-bit data (D2..D9) and must land on
 * CAM_DATA_IN0..IN7 IN ORDER (Y2->bit0 ... Y9->bit7).  The old mapping
 * had Y2/Y4/Y5 swapped -> bit-scrambled bytes -> garbage colors.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAM_DVP_PCLK_PIN   14
#define CAM_DVP_HREF_PIN   41
#define CAM_DVP_VSYNC_PIN  42

/* CAM signal indices (soc/gpio_sig_map.h) */

#define SIG_CAM_PCLK       149   /* CAM_PCLK_IDX */
#define SIG_CAM_H_ENABLE   150   /* CAM_H_ENABLE_IDX (HREF) */
#define SIG_CAM_V_SYNC     152   /* CAM_V_SYNC_IDX (VSYNC) */

/* XMCLK: NOT configured here. The board layer (esp32s3_board_camera.c)
 * drives IO39 with LEDC PWM at 20MHz via board_camera_xmclk_init() (called
 * from board_camera_sccb_probe -> camera_init).
 */

/* Full-frame capture (vs_eof=1), one RGB565 160x120 frame per DMA window:
 * - descriptor chain covers exactly one frame (10 x 3840B desc,
 * line-aligned)
 * - cam_vs_eof_en = 1: VSYNC triggers in_suc_eof = frame complete
 * - GDMA EOF ISR re-arms the chain at every frame boundary (official
 * esp_cam_ctlr_dvp_start_trans() model)
 */

/* 2026-09-10: Frame size changed to [runtime selectable]: preview 160x120 /
 * capture 640x480. DMA links and dual PSRAM buffers are allocated based on
 * maximum frame size; when vs_eof_en=1, VSYNC serves as frame boundary.
 * boundary (EOF prematurely ends this transfer), so the chain being longer
 * than the current frame is harmless — DMA is not used at all gets
 * truncated by EOF. Switching modes only changes how many bytes the CPU
 * reads back (g_frame_size).
 */

#define CAM_FRAME_BPP     2
#define CAM_MAX_W         640
#define CAM_MAX_H         480
/* CAM_MAX_SIZE = 640 x 480 x 2 = 614400; CAM_LINE_BYTES = 640 x 2 = 1280.
 * CAM_RX_CHUNK is 3 lines at 640w / 12 lines at 160w (both 32-byte aligned).
 * CAM_RX_DESC_NUM >= ceil(614400 / 4032) = 153.
 */

#define CAM_MAX_SIZE      (CAM_MAX_W * CAM_MAX_H * CAM_FRAME_BPP)
#define CAM_LINE_BYTES    (CAM_MAX_W * CAM_FRAME_BPP)
#define CAM_RX_CHUNK      3840
#define CAM_RX_DESC_NUM   160
#define CAM_RX_BUF_SIZE   CAM_MAX_SIZE

/* AEC/AGC/AWB convergence warm-up frame count.
 *
 * Zenn's proof of root cause for 'NuttX ESP32-S3 camera all green': OV
 * series sensors' Auto-exposure/gain/white balance need **dozens of frames**
 * to converge. Cold frames (first few frames after STREAMON) will be "dark +
 * green tint". Official method = idle 24 frames to discard, then capture,
 * zero post-processing to achieve natural color. We tested that the G
 * channel slowly dropped from 50 to 42 (AWB still converging), whereas
 * capture only waits 2 frames to grab a frame → it always grabs a 'cold
 * frame'. OV3660 measured convergence is slower, warm-up takes 40 frames
 * (~1.2s @34FPS / ~2.3s @17.8FPS).
 */

#define CAM_WARMUP_FRAMES 40

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s3_cam_s
{
  int          dma_channel;          /* NuttX GDMA channel */
  bool         initialized;          /* init done once (idempotent) */
  bool         started;
  bool         warmed_up;            /* AEC/AWB warm-up done only once (reused for preview) */
  struct esp32s3_dmadesc_s dmadesc[CAM_RX_DESC_NUM];

  /* completed frames (ISR-only inc; vs_eof=1: one per VSYNC EOF) */

  volatile uint32_t block_done;
  volatile uint32_t vsync_count;     /* VSYNC pulses seen (LCD_CAM ISR) */
  volatile uint32_t frame_pending;   /* VSYNC seen since capture start */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32s3_cam_s g_cam;

/* DMA RX + shadow buffers (38400B each) relocated to reserved top of PSRAM
 * since 2026-09-09. (esp32s3_psram_static_alloc), no longer occupying
 * internal DRAM: internal DRAM is occupied by WiFi (esp_wifi can only
 * allocate internally) can lead to ic_ebuf_alloc returning NULL when
 * combined with other static exhaustion, Data frame cannot be sent. The DMA
 * chain itself still uses internal descriptors; rxbuf/shadow data resides in
 * PSRAM Directly accessible via GDMA (consistent with I2S recording PSRAM
 * path); CPU reads back after cache operation. failure (cam_dvp_sync_rxbuf).
 * Allocation is done once in cam_dvp_dma_init().
 */

static uint8_t *g_cam_rxbuf;
static uint8_t *g_cam_shadow;

/* Preview stream mode switch: after esp32s3_cam_dvp_set_stream(true),
 * capture proceeds with rolling snapshots
 */

static bool g_stream_mode = false;
static bool g_stream_armed = false;
static uint8_t *g_last_frame = NULL;   /* get_frame returns frame: stream=shadow */

/* Software RGB565 byte swap, default ON.
 *
 * 2026-08-29 corrected (definitively, by sensor color bar): OV3660 RGB565
 * (0x4300=0x61) outputs BIG-ENDIAN (high byte first). PROOF = `plant cam
 * bar` (0x503D=0x80 standard 8-color bar): captured bars match [white yellow
 * cyan green magenta red blue black] ONLY in the byte-swapped (SWP) decode;
 * the little-endian decode is garbage. The earlier "LITTLE-ENDIAN"
 * conclusion (debug log 2.38) rested on ambiguous tests: a WHITE WALL is
 * 0xFFFF and a COVERED lens is ~0x0000 — both are byte-swap symmetric, and
 * contours only prove luminance structure, so neither could detect byte
 * order. The original datasheet reading "high byte first" (debug log 2.21)
 * was right all along. Toggle at runtime with `plant cam swap on|off`.
 */

static bool g_cam_byte_swap = true;

/* Silent mode (for preview): disables per-frame capture logging to avoid
 * screen flicker and slowdown.
 */

static bool g_cam_quiet = false;

/* Current frame geometry (runtime switch) */

static uint32_t g_frame_w = 160;
static uint32_t g_frame_h = 120;
static uint32_t g_frame_size = 160 * 120 * CAM_FRAME_BPP;

#define CAM_LOG(...) \
  do { if (!g_cam_quiet) printf(__VA_ARGS__); } while (0)

/****************************************************************************
 * Name: cam_dvp_sync_rxbuf
 *
 * Description:
 * The rxbuf on PSRAM is written directly by GDMA (bypassing CPU cache).
 * After DMA completes a frame,
 * CPU must invalidate that cache line before reading, otherwise CPU will
 * read stale cached data
 * (Same processing as I2S recording). Skipped when internal SRAM pointer is
 * used, behavior identical to original.
 ****************************************************************************/

static void cam_dvp_sync_rxbuf(void)
{
  if (esp32s3_ptr_extram(g_cam_rxbuf))
    {
      cache_invalidate_addr((uint32_t)g_cam_rxbuf, g_frame_size);
    }
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void cam_swap_rgb565(uint8_t *buf, size_t len)
{
  size_t i;

  for (i = 0; i + 1 < len; i += 2)
    {
      uint8_t t = buf[i];

      buf[i]     = buf[i + 1];
      buf[i + 1] = t;
    }
}

/* Print a few pixels at byte offset `off` decoded BOTH ways as RGB565:
 * [raw_le ...] = interpretation WITHOUT swap (wrong for OV3660: it is
 * BIG-ENDIAN, high byte first — see 2.49 color bar proof) [swp ...] =
 * interpretation WITH swap (CORRECT) White bar: swp should be near-white.
 */

static void cam_diag_pixels(const uint8_t *fp, int off)
{
  int p;
  int row = off / CAM_LINE_BYTES;

  printf("[Cam-DVP] row %d @%d:", row, off);
  for (p = 0; p < 4; p++)
    {
      uint16_t raw_le = fp[off + p * 2] |
                        (uint16_t)(fp[off + p * 2 + 1] << 8);
      uint16_t swp    = fp[off + p * 2 + 1] |
                        (uint16_t)(fp[off + p * 2] << 8);

      printf(" [%04x|%04x r%02u g%02u b%02u | r%02u g%02u b%02u]",
             raw_le, swp,
             (raw_le >> 11) & 0x1f, (raw_le >> 5) & 0x3f, raw_le & 0x1f,
             (swp >> 11) & 0x1f, (swp >> 5) & 0x3f, swp & 0x1f);
    }

  printf("\n");
}

static inline uint32_t cam_dvp_getreg(uint32_t reg)
{
  return getreg32(reg);
}

static inline void cam_dvp_putreg(uint32_t reg, uint32_t val)
{
  putreg32(val, reg);
}

/* Per-channel GDMA register accessor (register sets are 0xC8 apart). */

static uint32_t cam_gdma_reg(uint32_t base_ch0, int ch)
{
  return base_ch0 + (uint32_t)ch * GDMA_REG_OFFSET;
}

/****************************************************************************
 * Name: cam_dvp_gpio_config
 *
 * XMCLK(IO39) is driven by the board layer via LEDC PWM 20MHz
 * (board_camera_xmclk_init, called from camera_init -> probe).  We must
 * NOT touch IO39 here, otherwise the sensor clock dies.
 ****************************************************************************/

static void cam_dvp_gpio_config(void)
{
  /* DVP data pins -> CAM_DATA_IN0..IN7.
   *
   * IMPORTANT (2026-08-28): mapping corrected to match the WORKING IDF
   * reference (esp32s3/plant-companion/components/camera_capture/):
   * pin_d0=12 pin_d1=10 pin_d2=9 pin_d3=11 pin_d4=13 pin_d5=21 pin_d6=38
   * pin_d7=40 (esp32-camera pin_dN -> CAM_DATA_INN) i.e. sensor
   * D2(Y2)->bit0, D3(Y3)->bit1, D4(Y4)->bit2, D5(Y5)->bit3, D6(Y6)->bit4,
   * D7(Y7)->bit5, D8(Y8)->bit6, D9(Y9)->bit7. The previous mapping fed
   * D2/D4/D5 to the wrong bit positions -> every received byte was
   * bit-scrambled -> RGB565 colors were garbage while pure-white pixels
   * (all-1 bits) still survived as 0xFFFF (the "ff ff" frames we saw) and
   * dark pixels stayed ~0. That exactly matches the observed "colorful
   * chaos" that no register fix could cure.
   */

  /* Y2 (sensor D2) -> CAM_DATA_IN0_IDX (bit 0) */

  esp32s3_configgpio(12, INPUT);
  esp32s3_gpio_matrix_in(12, 133, 0);

  /* Y3 (sensor D3) -> CAM_DATA_IN1_IDX (bit 1) */

  esp32s3_configgpio(10, INPUT);
  esp32s3_gpio_matrix_in(10, 134, 0);

  /* Y4 (sensor D4) -> CAM_DATA_IN2_IDX (bit 2) */

  esp32s3_configgpio(9, INPUT);
  esp32s3_gpio_matrix_in(9, 135, 0);

  /* Y5 (sensor D5) -> CAM_DATA_IN3_IDX (bit 3) */

  esp32s3_configgpio(11, INPUT);
  esp32s3_gpio_matrix_in(11, 136, 0);

  /* Y6 (sensor D6) -> CAM_DATA_IN4_IDX (bit 4) */

  esp32s3_configgpio(13, INPUT);
  esp32s3_gpio_matrix_in(13, 137, 0);

  /* Y7 (sensor D7) -> CAM_DATA_IN5_IDX (bit 5) */

  esp32s3_configgpio(21, INPUT);
  esp32s3_gpio_matrix_in(21, 138, 0);

  /* Y8 (sensor D8) -> CAM_DATA_IN6_IDX (bit 6) */

  esp32s3_configgpio(38, INPUT);
  esp32s3_gpio_matrix_in(38, 139, 0);

  /* Y9 (sensor D9) -> CAM_DATA_IN7_IDX (bit 7) */

  esp32s3_configgpio(40, INPUT);
  esp32s3_gpio_matrix_in(40, 140, 0);

  esp32s3_configgpio(CAM_DVP_PCLK_PIN, INPUT);
  esp32s3_gpio_matrix_in(CAM_DVP_PCLK_PIN, SIG_CAM_PCLK, 0);

  esp32s3_configgpio(CAM_DVP_HREF_PIN, INPUT);
  esp32s3_gpio_matrix_in(CAM_DVP_HREF_PIN, SIG_CAM_H_ENABLE, 0);

  esp32s3_configgpio(CAM_DVP_VSYNC_PIN, INPUT);
  esp32s3_gpio_matrix_in(CAM_DVP_VSYNC_PIN, SIG_CAM_V_SYNC, 1);

  /* XMCLK(IO39) is NOT touched here: board layer drives it with LEDC PWM
   * 20MHz (board_camera_xmclk_init via camera_init/probe).
   *
   * VSYNC inverted (last arg = 1): esp32-camera cam_init() sets
   * cam_obj->vsync_invert = true; OV3660 drives VSYNC low during the active
   * frame, inverting aligns CAM EOF with the real frame start.
   */

  lcdinfo("DVP pins routed to LCD_CAM CAM input (XMCLK=IO39 via LEDC)\n");
}

/****************************************************************************
 * Name: cam_dvp_hw_init
 ****************************************************************************/

static void cam_dvp_hw_init(void)
{
  uint32_t regval;

  /* Enable LCD_CAM peripheral clock DIRECTLY via SYSTEM register. This
   * board's LCD uses SPI (CONFIG_ESP32S3_LCD=n) so esp32s3_lcd.c is not
   * compiled and nobody enables the LCD_CAM clock. We bypass
   * periph_module_enable() (its spinlock path is suspect in this build) and
   * set the clock-enable + clear-reset bits directly.
   */

  modifyreg32(SYSTEM_PERIP_CLK_EN1_REG, 0, SYSTEM_LCD_CAM_CLK_EN);
  modifyreg32(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_LCD_CAM_RST, 0);

  /* Whole-register reset, exactly like IDF ll_cam_config():
   * LCD_CAM.cam_ctrl.val = 0; LCD_CAM.cam_ctrl1.val = 0;
   * LCD_CAM.cam_rgb_yuv.val = 0; This avoids stale bits (e.g. cam_start,
   * bytelen) left by previous read-modify-write passes.
   */

  cam_dvp_putreg(LCD_CAM_CAM_CTRL_REG, 0);
  cam_dvp_putreg(LCD_CAM_CAM_CTRL1_REG, 0);
  cam_dvp_putreg(LCD_CAM_CAM_RGB_YUV_REG, 0);

  /* cam_ctrl (IDF order):
   *   cam_clkm_div_b/a = 0
   *   cam_clkm_div_num = 160M / 20M = 8   (xclk = PLL160M/8)
   *   cam_clk_sel      = 3                (PLL160M source)
   *   cam_stop_en      = 0
   *   cam_vsync_filter_thres = 4
   *   cam_update       = 0 (armed later in start)
   *   cam_byte_order   = 0, cam_bit_order = 0
   *   cam_line_int_en  = 0
   *   cam_vs_eof_en    = 0  (EOF by cam_rec_data_bytelen, like esp32-camera
   *                          ll_cam_config.  vs_eof=1 fires mid-frame,
   *                          data starts at wrong offset -> looked like
   *                          a byte-order bug!)
   */

  regval = 0;
  regval |= (8 << LCD_CAM_CAM_CLKM_DIV_NUM_S);   /* 160M/8 = 20M XMCLK */
  regval |= (3 << LCD_CAM_CAM_CLK_SEL_S);        /* PLL160M */
  regval |= (4 << LCD_CAM_CAM_VSYNC_FILTER_THRES_S);
  /* vs_eof=1: VSYNC EOF = frame complete (official dvp_spi_lcd cam_hal_init:
   * cam_ll_enable_vsync_generate _eof(hw,1))
   */

  regval |= (1 << LCD_CAM_CAM_VS_EOF_EN_S);
  /* cam_byte_order=0: official rejects byte swap for 8-bit data ("byte swap
   * is not supported when cam_data_width is 8")
   */

  cam_dvp_putreg(LCD_CAM_CAM_CTRL_REG, regval);

  /* cam_ctrl1 (IDF order):
   *   cam_rec_data_bytelen = 3839 (compat only; vs_eof=1 makes VSYNC the
   *                                           EOF source)
   *   cam_line_int_num = 0
   *   cam_clk_inv = 0 (PCLK not inverted)
   *   cam_vsync_filter_en = 1
   *   cam_2byte_en = 0 (8-bit data)
   *   cam_de_inv / hsync_inv / vsync_inv = 0
   *   cam_vh_de_mode_en = 0
   */

  regval = 0;
  regval |= ((CAM_RX_CHUNK - 1) << LCD_CAM_CAM_REC_DATA_BYTELEN_S);
  regval |= (1 << LCD_CAM_CAM_VSYNC_FILTER_EN_S);
  cam_dvp_putreg(LCD_CAM_CAM_CTRL1_REG, regval);

  printf("[Cam-DVP] CAM hw configured (8-bit DVP, PLL160M/8, "
         "vs_eof=1, bytelen=%d compat)\n", CAM_RX_CHUNK - 1);
}

/****************************************************************************
 * Name: cam_dvp_dma_init
 ****************************************************************************/

static int cam_dvp_dma_init(void)
{
  struct esp32s3_cam_s *priv = &g_cam;

  /* burst=true -> esp32s3_dma_setup() uses ESP32S3_DMA_BUFLEN_MAX_4B_ALIGNED
   * (4092) per descriptor, exactly matching LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE
   * in IDF's esp32-camera. With burst=false the descriptor would be 4095
   * bytes while bytelen is 4091 -> EOF lands 3 bytes early, corrupting the
   * stream.
   */

  priv->dma_channel = esp32s3_dma_request(ESP32S3_DMA_PERIPH_LCDCAM,
                                          10, 1, true);
  if (priv->dma_channel < 0)
    {
      printf("[Cam-DVP] GDMA RX alloc FAILED\n");
      return -ENODEV;
    }

  printf("[Cam-DVP] dma_channel=%d\n", priv->dma_channel);

#ifdef CONFIG_ESP32S3_SPIRAM_COMMON_HEAP
  g_cam_rxbuf = esp32s3_psram_static_alloc(CAM_RX_BUF_SIZE);
  g_cam_shadow = esp32s3_psram_static_alloc(CAM_MAX_SIZE);
#else
  static uint8_t rxbuf_fallback[CAM_RX_BUF_SIZE]
    __attribute__((aligned(64)));
  static uint8_t shadow_fallback[160 * 120 * CAM_FRAME_BPP]
    __attribute__((aligned(64)));

  g_cam_rxbuf = rxbuf_fallback;
  g_cam_shadow = shadow_fallback;
#endif
  if (g_cam_rxbuf == NULL || g_cam_shadow == NULL)
    {
      printf("[Cam-DVP] psram carve alloc FAILED\n");
      return -ENOMEM;
    }

  printf("[Cam-DVP] rxbuf=%p size=%u (psram carve)\n",
         g_cam_rxbuf, CAM_RX_BUF_SIZE);
  memset(g_cam_rxbuf, 0, CAM_RX_BUF_SIZE);
  if (esp32s3_ptr_extram(g_cam_rxbuf))
    {
      rom_cache_writeback_addr((uint32_t)g_cam_rxbuf, CAM_RX_BUF_SIZE);
    }

  printf("[Cam-DVP] dma_setup call (%u desc x %uB)...\n",
         CAM_RX_DESC_NUM, CAM_RX_CHUNK);
  esp32s3_dma_setup(priv->dmadesc, CAM_RX_DESC_NUM,
                    g_cam_rxbuf, CAM_RX_BUF_SIZE,
                    false, priv->dma_channel);

  /* Linear chain (official dvp_spi_lcd + esp_cam_ctlr_dvp): the GDMA EOF ISR
   * re-loads the chain at every frame boundary (per-frame start), so the DMA
   * always captures exactly one frame per VSYNC window.
   */

  printf("[Cam-DVP] dma_setup done (linear chain, ISR re-arm per frame)\n");

  return OK;
}

/****************************************************************************
 * Name: cam_dvp_gdma_isr
 *
 * GDMA RX channel interrupt: in_suc_eof fires once per frame (vs_eof=1,
 * VSYNC-driven). ISR does NOT touch NuttX semaphores/syslog (IRAM-safe):
 * it increments a volatile counter and RE-ARMS the DMA chain at the frame
 * boundary, exactly like the official driver's per-frame start_trans().
 *
 * Re-arm is DMA-only (stop -> in_rst -> reload link -> start), inline
 * register writes so the ISR stays IRAM-safe. We deliberately do NOT
 * pulse cam_reset / cam_afifo_reset here: cam_reset would clear the
 * LCD_CAM INT_ENA (debug log 2.29) and afifo_reset would drop FIFO
 * continuity; the CAM peripheral keeps streaming on its own once started.
 *
 * This per-frame re-arm is the key fix for the "garbled colors" (garbled
 * colors):
 * previously the chain was only re-armed from capture() at an arbitrary
 * phase, so the DMA window [arm -> next VSYNC] was a partial frame that
 * changed every capture. Re-arming at every VSYNC EOF makes each window
 * exactly one frame, aligned to the frame start.
 ****************************************************************************/

static int IRAM_ATTR cam_dvp_gdma_isr(int irq, void *context, void *arg)
{
  struct esp32s3_cam_s *priv = &g_cam;
  int ch = priv->dma_channel;
  uint32_t st;
  uint32_t regval;

  st = cam_dvp_getreg(cam_gdma_reg(DMA_IN_INT_ST_CH0_REG, ch));
  if (st != 0)
    {
      cam_dvp_putreg(cam_gdma_reg(DMA_IN_INT_CLR_CH0_REG, ch), st);
    }

  if (st & DMA_IN_SUC_EOF_CH0_INT_ST_M)
    {
      priv->block_done++;

      /* 1. stop DMA RX (self-clearing bit) */

      regval = cam_dvp_getreg(cam_gdma_reg(DMA_IN_LINK_CH0_REG, ch));
      regval |= DMA_INLINK_STOP_CH0_M;
      cam_dvp_putreg(cam_gdma_reg(DMA_IN_LINK_CH0_REG, ch), regval);

      /* 2. reset RX FSM + FIFO pointer (in_rst pulse) */

      regval = cam_dvp_getreg(cam_gdma_reg(DMA_IN_CONF0_CH0_REG, ch));
      regval |= DMA_IN_RST_CH0_M;
      cam_dvp_putreg(cam_gdma_reg(DMA_IN_CONF0_CH0_REG, ch), regval);
      regval &= ~DMA_IN_RST_CH0_M;
      cam_dvp_putreg(cam_gdma_reg(DMA_IN_CONF0_CH0_REG, ch), regval);

      /* 3. reload the descriptor link base (preserve other LINK bits) */

      regval = cam_dvp_getreg(cam_gdma_reg(DMA_IN_LINK_CH0_REG, ch));
      regval &= ~DMA_INLINK_ADDR_CH0;
      regval |= (uint32_t)priv->dmadesc & DMA_INLINK_ADDR_CH0;
      cam_dvp_putreg(cam_gdma_reg(DMA_IN_LINK_CH0_REG, ch), regval);

      /* 4. start RX again */

      regval = cam_dvp_getreg(cam_gdma_reg(DMA_IN_LINK_CH0_REG, ch));
      regval |= DMA_INLINK_START_CH0_M;
      cam_dvp_putreg(cam_gdma_reg(DMA_IN_LINK_CH0_REG, ch), regval);
    }

  return 0;
}

/****************************************************************************
 * Name: cam_dvp_vsync_isr
 *
 * LCD_CAM peripheral interrupt: cam_vsync marks frame boundaries.  With
 * vs_eof=0 the DMA EOF is data-driven (every 3840B) and does NOT align
 * to frames - without a VSYNC marker, a 38400B collection can span two
 * frames -> horizontal banding.  ISR only clears the flag and counts.
 ****************************************************************************/

static int IRAM_ATTR cam_dvp_vsync_isr(int irq, void *context, void *arg)
{
  struct esp32s3_cam_s *priv = &g_cam;
  uint32_t st;

  st = cam_dvp_getreg(LCD_CAM_LC_DMA_INT_ST_REG);
  if (st & LCD_CAM_CAM_VSYNC_INT_ST_M)
    {
      cam_dvp_putreg(LCD_CAM_LC_DMA_INT_CLR_REG,
                     LCD_CAM_CAM_VSYNC_INT_ST_M);
      priv->vsync_count++;
      priv->frame_pending = 1;
    }

  return 0;
}

/****************************************************************************
 * Name: cam_dvp_arm
 *
 * Re-arm the DMA chain + CAM start.  Follows esp32-camera ll_cam_start()
 * exactly (IDF-proven sequence).  Called on first start and after every
 * completed frame (per-frame re-arm, like the dvp_spi_lcd example).
 ****************************************************************************/

static void cam_dvp_arm(void)
{
  struct esp32s3_cam_s *priv = &g_cam;
  uint32_t regval;

  /* 1. cam_start = 0 (clear leftover start bit) */

  regval = cam_dvp_getreg(LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_START_M;
  cam_dvp_putreg(LCD_CAM_CAM_CTRL1_REG, regval);

  /* 2. CAM + AFIFO reset pulses (WO bits) */

  regval = cam_dvp_getreg(LCD_CAM_CAM_CTRL1_REG);
  regval |= LCD_CAM_CAM_RESET_M;
  cam_dvp_putreg(LCD_CAM_CAM_CTRL1_REG, regval);
  regval &= ~LCD_CAM_CAM_RESET_M;
  cam_dvp_putreg(LCD_CAM_CAM_CTRL1_REG, regval);

  regval |= LCD_CAM_CAM_AFIFO_RESET_M;
  cam_dvp_putreg(LCD_CAM_CAM_CTRL1_REG, regval);
  regval &= ~LCD_CAM_CAM_AFIFO_RESET_M;
  cam_dvp_putreg(LCD_CAM_CAM_CTRL1_REG, regval);

  /* 3. Set bytelen (kept for compatibility; EOF driven by VSYNC) */

  regval = cam_dvp_getreg(LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_REC_DATA_BYTELEN_M;
  regval |= ((CAM_RX_CHUNK - 1) << LCD_CAM_CAM_REC_DATA_BYTELEN_S);
  cam_dvp_putreg(LCD_CAM_CAM_CTRL1_REG, regval);

  /* 4. Load chain (in_rst pulse + link addr) and start DMA (link start) */

  esp32s3_dma_load(priv->dmadesc, priv->dma_channel, false);
  esp32s3_dma_enable(priv->dma_channel, false);

  /* 5. cam_ctrl.cam_update = 1 (update lives in cam_ctrl) */

  regval = cam_dvp_getreg(LCD_CAM_CAM_CTRL_REG);
  regval |= LCD_CAM_CAM_UPDATE_REG_M;
  cam_dvp_putreg(LCD_CAM_CAM_CTRL_REG, regval);

  /* 6. cam_ctrl1.cam_start = 1 */

  regval = cam_dvp_getreg(LCD_CAM_CAM_CTRL1_REG);
  regval |= LCD_CAM_CAM_START_M;
  cam_dvp_putreg(LCD_CAM_CAM_CTRL1_REG, regval);
}

/****************************************************************************
 * Name: cam_dvp_enable_ints
 *
 * Re-enable the two capture interrupts.  cam_dvp_arm() pulses cam_reset,
 * which resets the whole LCD_CAM peripheral and therefore clears
 * LC_DMA_INT_ENA (debug log 2.29) — call this after every cam_dvp_arm().
 ****************************************************************************/

static void cam_dvp_enable_ints(void)
{
  struct esp32s3_cam_s *priv = &g_cam;
  uint32_t regval;

  /* GDMA RX in_suc_eof interrupt (per-frame re-arm driver) */

  regval = cam_dvp_getreg(cam_gdma_reg(DMA_IN_INT_ENA_CH0_REG,
                                       priv->dma_channel));
  regval |= DMA_IN_SUC_EOF_CH0_INT_ENA_M;
  cam_dvp_putreg(cam_gdma_reg(DMA_IN_INT_ENA_CH0_REG, priv->dma_channel),
                 regval);

  /* LCD_CAM cam_vsync interrupt (frame boundary diagnostics) */

  regval = cam_dvp_getreg(LCD_CAM_LC_DMA_INT_ENA_REG);
  regval |= LCD_CAM_CAM_VSYNC_INT_ENA_M;
  cam_dvp_putreg(LCD_CAM_LC_DMA_INT_ENA_REG, regval);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int esp32s3_cam_dvp_init(void)
{
  struct esp32s3_cam_s *priv = &g_cam;
  uint32_t regval;
  int ret;

  /* Idempotent: only initialize once. Repeated calls (e.g. each
   * camera_capture_frame) must NOT re-request DMA channels or re-memalign
   * the frame buffer - that leaks channels/memory.
   */

  if (priv->initialized)
    {
      return OK;
    }

  memset(priv, 0, sizeof(*priv));

  cam_dvp_gpio_config();
  cam_dvp_hw_init();

  ret = cam_dvp_dma_init();
  if (ret < 0)
    {
      return ret;
    }

  /* GDMA RX channel interrupt only (peripheral IRQ 66 + chan). LCD_CAM
   * peripheral IRQ is NOT used -> no conflict with LCD driver. Must map the
   * peripheral IRQ (esp32s3_setup_irq) BEFORE irq_attach + up_enable_irq,
   * otherwise g_irqmap[irq] is IRQ_UNMAPPED and up_enable_irq writes a
   * garbage CPUINT -> hangs.
   */

  printf("[Cam-DVP] setup_irq call (periph=%d)...\n",
         ESP32S3_PERIPH_DMA_IN_CH0 + priv->dma_channel);
  esp32s3_setup_irq(this_cpu(),
                    ESP32S3_PERIPH_DMA_IN_CH0 + priv->dma_channel,
                    ESP32S3_INT_PRIO_DEF,
                    ESP32S3_CPUINT_LEVEL);
  printf("[Cam-DVP] setup_irq done\n");

  printf("[Cam-DVP] irq_attach call (irq=%d)...\n",
         ESP32S3_IRQ_DMA_IN_CH0 + priv->dma_channel);
  ret = irq_attach(ESP32S3_IRQ_DMA_IN_CH0 + priv->dma_channel,
                   cam_dvp_gdma_isr, NULL);
  printf("[Cam-DVP] irq_attach done (ret=%d)\n", ret);
  if (ret < 0)
    {
      printf("[Cam-DVP] gdma irq attach FAILED (%d)\n", ret);
      return ret;
    }

  up_enable_irq(ESP32S3_IRQ_DMA_IN_CH0 + priv->dma_channel);
  printf("[Cam-DVP] irq enabled\n");

  /* Enable GDMA in_suc_eof interrupt */

  regval = cam_dvp_getreg(cam_gdma_reg(DMA_IN_INT_ENA_CH0_REG,
                                       priv->dma_channel));
  regval |= DMA_IN_SUC_EOF_CH0_INT_ENA_M;
  cam_dvp_putreg(cam_gdma_reg(DMA_IN_INT_ENA_CH0_REG, priv->dma_channel),
                 regval);

  printf("[Cam-DVP] init done: irq=%d\n",
         ESP32S3_IRQ_DMA_IN_CH0 + priv->dma_channel);

  /* LCD_CAM peripheral VSYNC interrupt: marks frame boundaries (needed
   * because vs_eof=0 DMA EOFs are data-driven, not frame-aligned).
   * esp32s3_lcd.c is NOT compiled (CONFIG_ESP32S3_LCD=n) so this IRQ is
   * free.
   */

  printf("[Cam-DVP] vsync irq setup (periph=%d)...\n",
         ESP32S3_PERIPH_LCD_CAM);
  esp32s3_setup_irq(this_cpu(),
                    ESP32S3_PERIPH_LCD_CAM,
                    ESP32S3_INT_PRIO_DEF,
                    ESP32S3_CPUINT_LEVEL);
  ret = irq_attach(ESP32S3_IRQ_LCD_CAM, cam_dvp_vsync_isr, NULL);
  printf("[Cam-DVP] vsync irq_attach done (ret=%d)\n", ret);
  if (ret < 0)
    {
      printf("[Cam-DVP] vsync irq attach FAILED (%d)\n", ret);
      return ret;
    }

  up_enable_irq(ESP32S3_IRQ_LCD_CAM);

  /* Enable cam_vsync interrupt in LCD_CAM */

  regval = cam_dvp_getreg(LCD_CAM_LC_DMA_INT_ENA_REG);
  regval |= LCD_CAM_CAM_VSYNC_INT_ENA_M;
  cam_dvp_putreg(LCD_CAM_LC_DMA_INT_ENA_REG, regval);
  printf("[Cam-DVP] vsync irq enabled\n");

  priv->initialized = true;
  lcdinfo("cam_dvp: DVP capture initialized\n");

  return OK;
}

int esp32s3_cam_dvp_start(void)
{
  struct esp32s3_cam_s *priv = &g_cam;

  /* 2026-09-18: Before each stream start, reclaim DVP pins from UART0.
   *
   * When leaving capture page, apps side calls esp32s3_uart0_reclaim_pins()
   * to reclaim IO42(TX)/IO40(RX) returned to UART0 (needed for soil sensor),
   * and esp32s3_cam_dvp_init() carries the initialized guard, won't run
   * cam_dvp_gpio_config() again when entering the photo page the second
   * time. As a result: the second time and Previously, entering the photo
   * capture page, VSYNC(IO42) was constantly held by UART0's TX, preventing
   * detection of frame boundaries, capture waits 10-second timeout, preview
   * thread exits, viewfinder goes blank, only rebooting board can
   * restoration (user feedback: "works initially after restart, fails after
   * running for a while").
   *
   * GPIO matrix is last-writer-wins, rerouting once here is sufficient; only
   * touches pins, Does not disturb DMA or interrupts, can be safely called
   * repeatedly.
   */

  cam_dvp_gpio_config();

  if (priv->started)
    {
      return OK;
    }

  /* arm the DMA chain + CAM once (full start sequence); cam_reset inside arm
   * clears the INT_ENAs, so re-enable both interrupts afterwards
   */

  cam_dvp_arm();
  cam_dvp_enable_ints();

  priv->started = true;
  printf("[Cam-DVP] capture started\n");

  return OK;
}

int esp32s3_cam_dvp_stop(void)
{
  struct esp32s3_cam_s *priv = &g_cam;
  uint32_t regval;

  if (!priv->started)
    {
      return OK;
    }

  /* Mask the GDMA EOF interrupt first: the ISR re-arms the chain at every
   * frame boundary and would otherwise restart a disabled DMA.
   */

  regval = cam_dvp_getreg(cam_gdma_reg(DMA_IN_INT_ENA_CH0_REG,
                                       priv->dma_channel));
  regval &= ~DMA_IN_SUC_EOF_CH0_INT_ENA_M;
  cam_dvp_putreg(cam_gdma_reg(DMA_IN_INT_ENA_CH0_REG, priv->dma_channel),
                 regval);

  /* cam_ll_stop(): cam_ctrl1.cam_start = 0; cam_ctrl.cam_update = 1 */

  regval = cam_dvp_getreg(LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_START_M;
  cam_dvp_putreg(LCD_CAM_CAM_CTRL1_REG, regval);

  regval = cam_dvp_getreg(LCD_CAM_CAM_CTRL_REG);
  regval |= LCD_CAM_CAM_UPDATE_REG_M;
  cam_dvp_putreg(LCD_CAM_CAM_CTRL_REG, regval);

  esp32s3_dma_disable(priv->dma_channel, false);
  priv->started = false;
  g_stream_armed = false;

  lcdinfo("cam_dvp: capture stopped\n");

  return OK;
}

/****************************************************************************
 * Name: esp32s3_cam_dvp_capture
 *
 * Capture one complete RGB565 frame (vs_eof=1: VSYNC-driven EOF = frame
 * complete).  The GDMA ISR re-arms the chain at every frame boundary, so
 * the DMA always captures exactly one aligned frame per VSYNC.
 *
 * Flow:
 *   1. align  — wait for one EOF with the ISR active (guarantees the DMA
 *               is mid-capture of a complete frame when we mask below)
 *   2. mask   — disable the GDMA EOF interrupt + clear pending bits, so
 *               the ISR cannot re-arm (and overwrite) the frame we read
 *   3. wait   — for the next (post-mask) EOF; the DMA stops at this frame
 *               boundary with one complete frame in rxbuf
 *   4. freeze — esp32s3_dma_disable (already stopped at EOF, safety)
 *   5. sink   — optional RGB565 byte swap, then hand frame to the sink
 *   6. re-arm — cam_dvp_arm() + re-enable both interrupts for next capture
 *
 * Input Parameters:
 *   sink     - frame callback (called once with g_frame_size bytes)
 *   sink_arg - opaque arg passed to sink
 *   eoi_out  - set to true when the frame is delivered (may be NULL)
 *
 * Returned Value: g_frame_size on success; negated errno on failure.
 ****************************************************************************/

int esp32s3_cam_dvp_capture(void (*sink)(const uint8_t *data, size_t len,
                                          void *arg),
                            void *sink_arg,
                            volatile bool *eoi_out)
{
  struct esp32s3_cam_s *priv = &g_cam;
  int ch = priv->dma_channel;
  irqstate_t flags;
  uint32_t seen;
  uint32_t stall = 0;
  uint32_t regval;
  uint32_t raww;

  if (sink == NULL && eoi_out == NULL)
    {
      return -EINVAL;
    }

  CAM_LOG("[Cam-DVP] capture enter (vs_eof=1): block_done=%u vsync=%u\n",
          priv->block_done, priv->vsync_count);

  /* Warm-up (first time only): AEC/AGC/AWB needs dozens of frames to
   * converge. DMA must keep running, During warm-up, ISR continuously
   * re-arms, rxbuf gets overwritten (we read only during frame capture).
   * warm-up only occurs during the first capture — continuous preview
   * reuses the converged state.
   */

  if (!priv->warmed_up)
    {
      cam_dvp_arm();
      cam_dvp_enable_ints();
      g_stream_armed = true;

        {
          int w;

          for (w = 0; w < CAM_WARMUP_FRAMES; w++)
            {
              seen = priv->block_done;
              stall = 0;
              while (priv->block_done == seen)
                {
                  up_udelay(2000);
                  stall++;
                  if (stall > 5000)
                    {
                      CAM_LOG("[Cam-DVP] warmup timeout\n");
                      return -ETIMEDOUT;
                    }
                }
            }
        }

      priv->warmed_up = true;
      CAM_LOG("[Cam-DVP] warmup done (%d frames, AEC converged)\n",
              CAM_WARMUP_FRAMES);
    }

  /* Preview stream mode: rolling capture + snapshot. Wait for the next EOF
   * (frame just completed), immediately rxbuf copied to shadow (and
   * byte-swapped on shadow), DMA does not stop, ISR continuous re-arming —
   * next capture requires no re-arm/realignment, only waiting for one frame
   * period; When LCD reads shadow, DMA is writing rxbuf, no interference, no
   * tearing.
   */

  if (g_stream_mode)
    {
      if (!g_stream_armed)
        {
          cam_dvp_arm();
          cam_dvp_enable_ints();
          g_stream_armed = true;

          /* The first window after arming is a half-frame: digest one EOF
           * first, then all subsequent are full frames
           */

          seen = priv->block_done;
          stall = 0;
          while (priv->block_done == seen)
            {
              up_udelay(2000);
              stall++;
              if (stall > 5000)
                {
                  CAM_LOG("[Cam-DVP] stream arm align timeout\n");
                  return -ETIMEDOUT;
                }
            }
        }

      seen = priv->block_done;
      stall = 0;
      while (priv->block_done == seen)
        {
          up_udelay(150);
          stall++;
          if ((stall % 2000) == 0)
            {
              CAM_LOG("[Cam-DVP] stream frame wait... stall=%u\n", stall);
            }

          if (stall > 80000)
            {
              CAM_LOG("[Cam-DVP] stream frame timeout\n");
              return -ETIMEDOUT;
            }
        }

      cam_dvp_sync_rxbuf();
      memcpy(g_cam_shadow, g_cam_rxbuf, g_frame_size);
      if (g_cam_byte_swap)
        {
          cam_swap_rgb565(g_cam_shadow, g_frame_size);
        }

      g_last_frame = g_cam_shadow;

      if (eoi_out != NULL)
        {
          *eoi_out = true;
        }

      CAM_LOG("[Cam-DVP] stream frame done: %u bytes\n",
              (unsigned)g_frame_size);
      return (int)g_frame_size;
    }

  /* === Freeze mode (capture/colorbars/single frame): stops DMA after
   * grabbing, ensures frame stability ===
   */

  if (!g_stream_armed)
    {
      cam_dvp_arm();
      cam_dvp_enable_ints();
      g_stream_armed = true;

      /* First window after arm is half-frame: waiting for its self-healing
       * EOF (alignment)
       */

      seen = priv->block_done;
      stall = 0;
      while (priv->block_done == seen)
        {
          up_udelay(2000);
          stall++;
          if ((stall % 500) == 0)
            {
              CAM_LOG("[Cam-DVP] align wait... stall=%u\n", stall);
            }

          if (stall > 5000)
            {
              CAM_LOG("[Cam-DVP] align timeout, no EOF\n");
              return -ETIMEDOUT;
            }
        }

      CAM_LOG("[Cam-DVP] aligned (block_done=%u)\n", priv->block_done);
    }

  /* 2. Mask the GDMA EOF interrupt and clear any pending EOF, so no ISR
   * re-arm can overwrite the frame while we read it.
   */

  flags = enter_critical_section();

  regval = cam_dvp_getreg(cam_gdma_reg(DMA_IN_INT_ENA_CH0_REG, ch));
  regval &= ~DMA_IN_SUC_EOF_CH0_INT_ENA_M;
  cam_dvp_putreg(cam_gdma_reg(DMA_IN_INT_ENA_CH0_REG, ch), regval);

  cam_dvp_putreg(cam_gdma_reg(DMA_IN_INT_CLR_CH0_REG, ch),
                 DMA_IN_SUC_EOF_CH0_INT_ST_M);

  leave_critical_section(flags);

  /* 3. Wait for the NEXT (post-mask) EOF: the DMA stops at this frame
   * boundary with one complete frame in rxbuf (it was capturing from the
   * last ISR re-arm at the previous boundary).
   */

  stall = 0;
  for (; ; )
    {
      raww = cam_dvp_getreg(cam_gdma_reg(DMA_IN_INT_RAW_CH0_REG, ch));
      if (raww & DMA_IN_SUC_EOF_CH0_INT_ST_M)
        {
          break;
        }

      up_udelay(2000);
      stall++;
      if ((stall % 500) == 0)
        {
          CAM_LOG("[Cam-DVP] waiting frame EOF... stall=%u\n", stall);
        }

      if (stall > 5000)
        {
          CAM_LOG("[Cam-DVP] frame EOF timeout\n");

          /* Restore: re-arm + re-enable interrupts so the next capture can
           * retry (the EOF IRQ is still masked here).
           */

          cam_dvp_arm();
          cam_dvp_enable_ints();
          g_stream_armed = true;
          return -ETIMEDOUT;
        }
    }

  cam_dvp_putreg(cam_gdma_reg(DMA_IN_INT_CLR_CH0_REG, ch),
                 DMA_IN_SUC_EOF_CH0_INT_ST_M);

  /* 4. Freeze the DMA (it stopped at the EOF; belt & suspenders) */

  esp32s3_dma_disable(priv->dma_channel, false);
  g_stream_armed = false;

  /* Frame directly written to PSRAM by DMA: CPU invalidates cache before
   * reading (no-op for internal SRAM)
   */

  cam_dvp_sync_rxbuf();

  /* 5. Diagnostics on the RAW frame (before any swap):
   * [xxxx|yyyy r..g..b.. | r..g..b..] = pixel value as little-endian vs
   * swapped; the first RGB triplet is what LVGL shows WITHOUT swap (CORRECT
   * for OV3660), the second is what it shows WITH swap.
   */

  CAM_LOG("[Cam-DVP] frame done: %u bytes (swap=%s)\n",
         (unsigned)g_frame_size, g_cam_byte_swap ? "on" : "off");

  if (sink != NULL)
    {
      int k;
      const uint8_t *fp = g_cam_rxbuf;

      CAM_LOG("[Cam-DVP] frame head:");
      for (k = 0; k < 16; k++)
        {
          printf(" %02x", fp[k]);
        }

      printf("\n");

      cam_diag_pixels(fp, 0);
      cam_diag_pixels(fp, 320);
      cam_diag_pixels(fp, 16000);
    }

  /* Optional RGB565 byte swap (sensor sends high byte first, LVGL wants
   * little-endian), then hand the frame to the sink.
   */

  if (g_cam_byte_swap)
    {
      cam_swap_rgb565(g_cam_rxbuf, g_frame_size);
    }

  /* get_frame() must point to [this specific frame captured]. Fixed on
   * 2026-09-10: This line was originally Indentation error in
   * g_cam_byte_swap branch — once byte swap is disabled (`plant cam swap`
   * or default changed elsewhere), g_last_frame retains previous value
   * (likely for preview shadow), the caller gets the previous frame/another
   * buffer by calling get_frame(): desaturated on A, but upload read from B,
   * resulting in "changes appeared to have no effect".
   */

  g_last_frame = g_cam_rxbuf;

  if (sink != NULL)
    {
      sink(g_cam_rxbuf, g_frame_size, sink_arg);
    }

  /* 6. End [no re-arm]: DMA stops at frame boundary, rxbuf remains stable.
   * this is key for tear-free preview — if cam_dvp_arm() restarts DMA
   * here, the next frame immediately overwrites rxbuf, LVGL reads half-new
   * half-old during rendering → random stripes. The next capture start
   * will re-arm.
   */

  if (eoi_out != NULL)
    {
      *eoi_out = true;
    }

  CAM_LOG("[Cam-DVP] capture done\n");
  return (int)g_frame_size;
}

/****************************************************************************
 * Name: esp32s3_cam_dvp_diag
 *
 * Layered hardware diagnosis.  Prints the raw register state of every
 * link in the capture chain so a single call pinpoints which layer is
 * broken (camera output / GPIO routing / CAM peripheral / clock / GDMA).
 *
 * Layers:
 *   L1 camera: PCLK/VSYNC/HREF GPIO input levels (static sample)
 *   L2 CAM ctrl: cam_ctrl / cam_ctrl1 (mode, start, bytelen)
 *   L3 CAM clock: lcd_clock (clk_en, cam_clk_sel)
 *   L4 CAM int: lc_dma_int_raw (cam_vsync/hs raw bits = VSYNC arriving?)
 *   L5 GDMA: int_raw/int_st, conf0, periph sel, state, dscr
 ****************************************************************************/

void esp32s3_cam_dvp_diag(void)
{
  struct esp32s3_cam_s *priv = &g_cam;
  int ch = priv->dma_channel;
  uint32_t v;

  printf("\n===== CAM-DVP LAYER DIAG =====\n");

  printf("L0 swap: rgb565 byte swap = %s\n",
         g_cam_byte_swap ? "ON" : "OFF");

  /* L0b: frame rate (block_done delta over 1s; needs init (capture) first).
   * NOTE: vsync_count fires ~2.3x per frame (OV3660 VSYNC edges + filter),
   * use block_done (GDMA EOF = 1/frame) for the real rate.
   */

  if (priv->initialized)
    {
      uint32_t v0 = priv->block_done;

      up_udelay(1000000);
      printf("L0b frame rate: %u FPS (block_done %u -> %u, vsync %u)\n",
             priv->block_done - v0, v0, priv->block_done,
             priv->vsync_count);
    }
  else
    {
      printf("L0b frame rate: not initialized (run 'plant cam capture' "
             "once first)\n");
    }

  /* L1: camera GPIO levels (static; PCLK toggles fast so sample a few) */

  printf("L1 GPIO: PCLK(14)=%d HREF(41)=%d VSYNC(42)=%d "
         "D0(12)=%d D7(40)=%d\n",
         esp32s3_gpioread(14), esp32s3_gpioread(41), esp32s3_gpioread(42),
         esp32s3_gpioread(12), esp32s3_gpioread(40));

  /* PCLK toggle test: 20 samples over ~2ms - any change = clock running */

    {
      int s;
      int pclk_prev = -1;
      int pclk_changes = 0;

      for (s = 0; s < 20; s++)
        {
          int p = esp32s3_gpioread(14);

          if (pclk_prev >= 0 && p != pclk_prev)
            {
              pclk_changes++;
            }

          pclk_prev = p;
          up_udelay(100);
        }

      printf("L1 PCLK toggles in 2ms: %d/20 samples\n", pclk_changes);
    }

  /* All 8 data pins + HREF + VSYNC raw sample (sensor output check; D0-D7 =
   * Y2-Y9 = IO12,10,9,11,13,21,38,40 in bit order)
   */

  printf("L1 D0-7(12,10,9,11,13,21,38,40)=%d%d%d%d%d%d%d%d "
         "HREF=%d VSYNC=%d\n",
         esp32s3_gpioread(12), esp32s3_gpioread(10), esp32s3_gpioread(9),
         esp32s3_gpioread(11), esp32s3_gpioread(13), esp32s3_gpioread(21),
         esp32s3_gpioread(38), esp32s3_gpioread(40),
         esp32s3_gpioread(41), esp32s3_gpioread(42));

  /* L2: CAM control registers */

  v = cam_dvp_getreg(LCD_CAM_CAM_CTRL_REG);
  printf("L2 cam_ctrl  = 0x%08x (stop_en=%d vs_eof_en=%d clk_sel=%d)\n",
         v, (v >> 0) & 1, (v >> 8) & 1, (v >> 29) & 3);

  v = cam_dvp_getreg(LCD_CAM_CAM_CTRL1_REG);
  printf("L2 cam_ctrl1 = 0x%08x (start=%d 2byte=%d bytelen=%d)\n",
         v, (v >> 29) & 1, (v >> 24) & 1, v & 0xffff);

  /* L3: clock */

  v = cam_dvp_getreg(LCD_CAM_LCD_CLOCK_REG);
  printf("L3 lcd_clock = 0x%08x (clk_en=%d clk_sel=%d div_num=%d)\n",
         v, (v >> 31) & 1, (v >> 29) & 3, (v >> 9) & 0xff);

  /* L4: CAM interrupt raw (did VSYNC arrive?) */

  v = cam_dvp_getreg(LCD_CAM_LC_DMA_INT_RAW_REG);
  printf("L4 int_raw   = 0x%08x (lcd_vsync=%d cam_vsync=%d cam_hs=%d)\n",
         v, (v >> 0) & 1, (v >> 2) & 1, (v >> 3) & 1);

  /* L5: GDMA RX channel state */

  if (ch >= 0)
    {
      printf("L5 DMA chan=%d\n", ch);
      v = cam_dvp_getreg(cam_gdma_reg(DMA_IN_INT_RAW_CH0_REG, ch));
      printf("L5 in_int_raw = 0x%08x (suc_eof=%d err_eof=%d done=%d)\n",
             v, (v >> 1) & 1, (v >> 2) & 1, (v >> 0) & 1);
      v = cam_dvp_getreg(cam_gdma_reg(DMA_IN_CONF0_CH0_REG, ch));
      printf("L5 in_conf0   = 0x%08x (mem_trans_en=%d burst=%d)\n",
             v, (v >> 0) & 1, (v >> 5) & 1);
      v = cam_dvp_getreg(cam_gdma_reg(DMA_IN_PERI_SEL_CH0_REG, ch));
      printf("L5 in_peri_sel= 0x%08x (%d)\n", v, v & 0x1f);
      v = cam_dvp_getreg(cam_gdma_reg(DMA_IN_STATE_CH0_REG, ch));
      printf("L5 in_state   = 0x%08x\n", v);
      v = cam_dvp_getreg(cam_gdma_reg(DMA_IN_LINK_CH0_REG, ch));
      printf("L5 in_link    = 0x%08x (start=%d stop=%d)\n",
             v, (v >> 22) & 1, (v >> 21) & 1);
    }
  else
    {
      printf("L5 DMA chan not allocated\n");
    }

  /* L6: per-descriptor status (did DMA actually write bytes?) */

  if (priv->initialized)
    {
      int i;

      printf("L6 desc ctrl (buf_len=%d):\n", ESP32S3_DMA_CTRL_BUFLEN_V);
      for (i = 0; i < CAM_RX_DESC_NUM; i++)
        {
          uint32_t c = priv->dmadesc[i].ctrl;

          printf("L6  desc[%d] ctrl=0x%08x owner=%d eof=%d err=%d "
                 "datalen=%d\n",
                 i, c, (c >> 31) & 1, (c >> 30) & 1, (c >> 28) & 1,
                 (c >> ESP32S3_DMA_CTRL_DATALEN_S) &
                 ESP32S3_DMA_CTRL_DATALEN_V);
        }
    }
  else
    {
      printf("L6 not initialized yet\n");
    }

  printf("===== DIAG END =====\n");
}

/****************************************************************************
 * Name: esp32s3_cam_dvp_get_frame
 *
 * Return the internal DMA RX buffer (static, CAM_RX_BUF_SIZE bytes).
 * The latest captured frame lives in its first g_frame_size bytes.
 * Used by the UI to display the frame without a second buffer copy.
 ****************************************************************************/

uint8_t *esp32s3_cam_dvp_get_frame(void)
{
  return g_last_frame != NULL ? g_last_frame : g_cam_rxbuf;
}

/****************************************************************************
 * Name: esp32s3_cam_dvp_set_framesize
 *
 * Runtime frame geometry switching (preview 160x120 / capture 640x480).
 *
 * DMA chain and rxbuf/shadow are allocated with CAM_MAX_SIZE, and
 * vs_eof_en=1 makes VSYNC
 * becomes a frame boundary (EOF prematurely ends transfer), so here there is
 * no need to reconfigure the DMA chain — only change
 * CPU read-back length and write-back length. The caller is responsible for
 * switching the sensor to the same resolution first (via SCCB),
 * And ensure capture is stopped before calling (no concurrent frame
 * grabbing).
 *
 * ⚠️ Do NOT clear warmed_up here: both modes use the same sensor window
 * (start/end both 0,0→2079,1547), only output scaling differs, scene
 * average brightness unchanged
 * → AEC/AGC/AWB remains effective, no need to re-run the 40-frame warm-up
 * (otherwise, on every capture, every return
 * Preview will all wait about an extra 9 seconds). Switching timing
 * stability is handled by two inherent aspects of the freeze path.
 * Frame period alignment (see the two-step align + post-mask EOF in
 * capture).
 *
 * Input Parameters:
 * w, h - target pixel dimensions (<= CAM_MAX_W / CAM_MAX_H)
 *
 * Returned Value: 0 success; negative errno for failure
 ****************************************************************************/

int esp32s3_cam_dvp_set_framesize(uint32_t w, uint32_t h)
{
  if (w == 0 || h == 0 || w > CAM_MAX_W || h > CAM_MAX_H)
    {
      printf("[Cam-DVP] set_framesize %ux%u out of range (max %dx%d)\n",
             (unsigned)w, (unsigned)h, CAM_MAX_W, CAM_MAX_H);
      return -EINVAL;
    }

  g_frame_w = w;
  g_frame_h = h;
  g_frame_size = w * h * CAM_FRAME_BPP;

  printf("[Cam-DVP] framesize -> %ux%u (%u bytes)\n",
         (unsigned)w, (unsigned)h, (unsigned)g_frame_size);
  return OK;
}

/****************************************************************************
 * Name: esp32s3_cam_dvp_get_framesize
 ****************************************************************************/

void esp32s3_cam_dvp_get_framesize(uint32_t *w, uint32_t *h)
{
  if (w != NULL) *w = g_frame_w;
  if (h != NULL) *h = g_frame_h;
}

/****************************************************************************
 * Name: esp32s3_cam_dvp_set_swap / esp32s3_cam_dvp_get_swap
 *
 * Runtime toggle for the software RGB565 byte swap (`plant cam swap`).
 * Default ON (OV3660 0x4300=0x61 sends high byte first; LVGL wants
 * little-endian).  Flip to OFF if a capture ever shows swapped R/B.
 ****************************************************************************/

void esp32s3_cam_dvp_set_swap(bool on)
{
  g_cam_byte_swap = on;
  printf("[Cam-DVP] rgb565 byte swap %s\n", on ? "ON" : "OFF");
}

bool esp32s3_cam_dvp_get_swap(void)
{
  return g_cam_byte_swap;
}

/****************************************************************************
 * Name: esp32s3_cam_dvp_set_quiet
 *
 * Silent mode: disable capture's per-frame logging (used by the preview
 * thread, to avoid flooding the log and slowing things down).
 ****************************************************************************/

void esp32s3_cam_dvp_set_quiet(bool quiet)
{
  g_cam_quiet = quiet;
}

/****************************************************************************
 * Name: esp32s3_cam_dvp_set_stream / esp32s3_cam_dvp_get_stream
 *
 * Preview stream mode switch. When enabled, capture() follows the "rolling
 * snapshot" path: DMA continuous acquisition,
 * Frame complete then copied to shadow (no freeze, no re-arm), preview wait
 * reduced from ~2 frame periods
 * reduces to about one frame period; LCD/UI reads from shadow while DMA
 * writes to rxbuf, without mutual interference.
 * Single-frame paths like photo/color bars remain disabled (freeze mode, for
 * stable, zero-copy frames).
 ****************************************************************************/

void esp32s3_cam_dvp_set_stream(bool on)
{
  if (g_stream_mode == on)
    {
      return;
    }

  g_stream_mode = on;
  g_stream_armed = false;
  g_last_frame = on ? g_cam_shadow : g_cam_rxbuf;
  printf("[Cam-DVP] stream mode %s\n", on ? "ON (rolling snapshot)" : "OFF");
}

bool esp32s3_cam_dvp_get_stream(void)
{
  return g_stream_mode;
}
#endif /* CONFIG_ESP32S3_CAM_DVP */
