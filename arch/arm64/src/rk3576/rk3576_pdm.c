/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_pdm.c
 *
 * RK3576 PDM capture for the openvela AMP slave.
 *
 * IMPORTANT: the RK3576 does NOT use the classic Rockchip PDM block.  Its
 * device tree says "rockchip,rk3576-pdm", which only
 * sound/soc/rockchip/rockchip_pdm_v2.{c,h} matches — a 2024 IP revision with
 * a completely different register map from rockchip_pdm.h.  The give-away on
 * hardware is PDM_V2_VERSION at offset 0x38 reading 0x23023576; with the old
 * map, every functional register reads back zero and the FIFO never fills.
 *
 * As with the SAI we do not use DMA (the three PL330 DMACs belong to Linux),
 * so a polling thread drains the FIFO.  Unlike an I2S MEMS mic, a PDM mic
 * returns a raw 1-bit oversampled stream; the CIC decimation filter that
 * turns it into PCM lives inside this block, so there is no DSP to do here.
 *
 * Clocking.  Everything comes from the 24 MHz crystal, never from the audio
 * PLLs: those belong to Linux, which retunes them whenever it plays audio and
 * would drag our sample rate along.  The v2 driver's own reference table has
 * an entry for exactly this case — { clk 24000000, clk_out 2400000 } — so the
 * rate lands on a round number instead of the SAI path's 15625 Hz:
 *
 *   mclk_pdm1    = xin24m / 1  = 24 MHz     (filter clock)
 *   clk_pdm1_out = xin24m / 10 = 2.4 MHz    (clock driven onto the mic)
 *   sample rate  = 2.4 MHz / (2 * CIC_RATIO) = 16000 Hz
 *
 * 2.4 MHz sits in the normal-mode clock range of every PDM MEMS mic.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <errno.h>
#include <stdint.h>

#include <arch/chip/chip.h>

#include "arm64_internal.h"
#include "rk3576_pdm.h"
#include "rk3576_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets (sound/soc/rockchip/rockchip_pdm_v2.h) */

#define PDM_SYSCONFIG           0x0000
#define PDM_CTRL                0x0004
#define PDM_FILTER_CTRL         0x0008
#define PDM_FIFO_CTRL           0x000c
#define PDM_DATA_VALID          0x0010
#define PDM_RXFIFO_DATA         0x0014
#define PDM_VERSION             0x0038

#define PDM_VERSION_RK3576      0x23023576

/* SYSCONFIG */

#define PDM_NUM_START           (1u << 3)   /* filter/decimator running   */
#define PDM_RX_START            (1u << 2)   /* receiver running           */
#define PDM_RX_CLR_WR           (1u << 0)   /* flush FIFO (self clearing) */
#define PDM_SYSCONFIG_RUN_MSK   (PDM_NUM_START | PDM_RX_START)

/* CTRL */

#define PDM_RX_PATH_SEL(x, v)   ((v) << (13 + (x) * 2))
#define PDM_RX_PATH_SEL_MSK(x)  (3u << (13 + (x) * 2))
#define PDM_CKP_NORMAL          (0u << 12)
#define PDM_CKP_MSK             (1u << 12)
#define PDM_SJM_SEL_L           (1u << 11)  /* sample left justified      */
#define PDM_SJM_SEL_MSK         (1u << 11)
#define PDM_PATH_MSK            (0xfu << 7)
#define PDM_PATH0_EN            (1u << 7)
#define PDM_VDW(x)              (((x) - 1) << 0)
#define PDM_VDW_MSK             (0x1fu << 0)

/* FILTER_CTRL */

#define PDM_GAIN_MSK            (0xffu << 23)
#define PDM_GAIN_24DB           (239u << 23)
#define PDM_HPF_L_EN            (1u << 22)
#define PDM_HPF_R_EN            (1u << 21)
#define PDM_HPF_MSK             ((1u << 22) | (1u << 21))
#define PDM_HPF_FREQ_60         (1u << 19)
#define PDM_HPF_FREQ_MSK        (3u << 19)
#define PDM_CIC_SCALE(x)        ((x) << 10)
#define PDM_CIC_SCALE_MSK       (0x7fu << 10)
#define PDM_CIC_RATIO(x)        (((x) - 1) << 1)
#define PDM_CIC_RATIO_MSK       (0x1ffu << 1)

/* FIFO_CTRL: RFL is the current fill level, RXOI a sticky overrun flag and
 * RXOIC its write-1-to-clear.  Interrupts (RXOIE/RXFTIE) stay off: we poll.
 */

#define PDM_FIFO_LEVEL(x)       (((x) >> 20) & 0xff)
#define PDM_RXOI                (1u << 11)
#define PDM_RXOIC               (1u << 9)
#define PDM_DMA_RD_EN           (1u << 12)
#define PDM_DMA_RD_MSK          (1u << 12)
#define PDM_RXOIE               (1u << 1)
#define PDM_RXFTIE              (1u << 0)

/* Clock plan, see the file header */

#define PDM_XIN_HZ              24000000
#define PDM_MCLK_DIV            1
#define PDM_OUT_DIV             10
#define PDM_OUT_HZ              (PDM_XIN_HZ / PDM_OUT_DIV)      /* 2400000 */

/* rockchip_pdm_v2.c: ratio = clk_out / rate / 2, and its scale table pairs
 * ratio 75 with 57.  Keep the pair together — the scale compensates the CIC's
 * gain, so a mismatched value clips or mutes.
 */

#define PDM_CIC_RATIO_VAL       75
#define PDM_CIC_SCALE_VAL       57
#define PDM_SAMPLE_RATE         (PDM_OUT_HZ / (2 * PDM_CIC_RATIO_VAL))

/* Valid data width.  24 rather than 16 on purpose: at 24 bits one sample
 * cannot share a 32-bit FIFO word with the next one, so draining stays
 * unambiguous.  Left justified, so the top 16 bits are the PCM we want.
 */

#define PDM_VDW_BITS            24

/* CRU bits for PDM1 (drivers/clk/rockchip/clk-rk3576.c)
 *   MCLK_PDM1     CLKSEL_CON51 mux[7:5] div[4:0]   gate CON9 bit8
 *   CLK_PDM1      CLKSEL_CON50 mux[11:9] div[8:0]  gate CON9 bit5
 *   HCLK_PDM1                                      gate CON9 bit7
 *   CLK_PDM1_OUT  (child of clk_pdm1)              gate CON3 bit5
 * audio_frac_int_p mux 0 = xin24m.
 */

#define PDM_MCLK_CLKSEL_CON     51
#define PDM_MCLK_MUX_SHIFT      5
#define PDM_MCLK_MUX_WIDTH      3
#define PDM_MUX_XIN24M          0
#define PDM_MCLK_DIV_SHIFT      0
#define PDM_MCLK_DIV_WIDTH      5

#define PDM_OUT_CLKSEL_CON      50
#define PDM_OUT_MUX_SHIFT       9
#define PDM_OUT_MUX_WIDTH       3
#define PDM_OUT_DIV_SHIFT       0
#define PDM_OUT_DIV_WIDTH       9

#define PDM_GATE_CON            9
#define PDM_GATE_CLK_BIT        5
#define PDM_GATE_HCLK_BIT       7
#define PDM_GATE_MCLK_BIT       8
#define PDM_OUT_GATE_CON        3
#define PDM_OUT_GATE_BIT        5

/* pdm1m1 pins, all function 3 on bank 4 */

#define PDM_PIN_CLK0            RK3576_PIN_A(6)
#define PDM_PIN_CLK1            RK3576_PIN_B(0)
#define PDM_PIN_SDI1            RK3576_PIN_B(2)
#define PDM_PIN_FUNC            3

/* SDI1 feeds internal path 0 */

#define PDM_SDI_INDEX           1

/* rockchip_pdm_v2.c waits this long after RX_START for the mic to wake and
 * the filter to settle (PDM_V2_START_DELAY_MS_DEFAULT).
 */

#define PDM_START_DELAY_MS      20

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_pdm_overruns;
static uint32_t g_pdm_maxfifo;
static uint32_t g_pdm_slot;             /* next FIFO word's channel, 0 or 1 */
static int      g_pdm_channel;          /* channel handed out as mono       */
static uint16_t g_pdm_peak[2];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t pdm_getreg(unsigned int off)
{
  return getreg32(RK3576_PDM1_BASE + off);
}

static inline void pdm_putreg(unsigned int off, uint32_t val)
{
  putreg32(val, RK3576_PDM1_BASE + off);
}

/****************************************************************************
 * Name: pdm_modifyreg
 *
 * Description:
 *   Read-modify-write, mirroring the regmap_update_bits() calls in the Linux
 *   driver.  Several fields in these registers (the SYSCONFIG memory/filter
 *   gates in particular) are left at their reset value by Rockchip's driver,
 *   so a blind full-register write would clobber them.
 *
 ****************************************************************************/

static void pdm_modifyreg(unsigned int off, uint32_t mask, uint32_t val)
{
  pdm_putreg(off, (pdm_getreg(off) & ~mask) | (val & mask));
}

/****************************************************************************
 * Name: rk3576_pdm_clk_init
 *
 * Description:
 *   Point PDM1's filter clock and its microphone clock output at the 24 MHz
 *   crystal, and ungate everything the block needs.
 *
 ****************************************************************************/

static void rk3576_pdm_clk_init(void)
{
  /* Filter clock: xin24m / 1 */

  rk3576_clk_set_mux_div(PDM_MCLK_CLKSEL_CON,
                         PDM_MCLK_MUX_SHIFT, PDM_MCLK_MUX_WIDTH,
                         PDM_MUX_XIN24M,
                         PDM_MCLK_DIV_SHIFT, PDM_MCLK_DIV_WIDTH,
                         PDM_MCLK_DIV);

  /* Microphone clock: xin24m / 10 = 2.4 MHz.  clk_pdm1_out is a plain gate
   * on top of clk_pdm1, so this divider sets what reaches the pin.
   */

  rk3576_clk_set_mux_div(PDM_OUT_CLKSEL_CON,
                         PDM_OUT_MUX_SHIFT, PDM_OUT_MUX_WIDTH,
                         PDM_MUX_XIN24M,
                         PDM_OUT_DIV_SHIFT, PDM_OUT_DIV_WIDTH,
                         PDM_OUT_DIV);

  rk3576_clk_gate(PDM_GATE_CON, PDM_GATE_MCLK_BIT, true);
  rk3576_clk_gate(PDM_GATE_CON, PDM_GATE_HCLK_BIT, true);
  rk3576_clk_gate(PDM_GATE_CON, PDM_GATE_CLK_BIT, true);
  rk3576_clk_gate(PDM_OUT_GATE_CON, PDM_OUT_GATE_BIT, true);
}

/****************************************************************************
 * Name: rk3576_pdm_hw_reassert
 *
 * Description:
 *   (Re)claim everything the Linux boot can silently take away: the CRU
 *   mux/div/gate settings and the IOC pin functions.  This core starts
 *   before Linux, whose boot both re-parents parts of the shared clock
 *   tree and gates "unused" clocks (clk_disable_unused) — including
 *   clk_pdm1_out, the 2.4 MHz clock that drives the microphone.  The
 *   controller keeps ticking on mclk and decimates an idle data line into
 *   perfect silence, so the theft is invisible except for the audio being
 *   gone.  All writes are idempotent; called before every capture start.
 *
 ****************************************************************************/

static void rk3576_pdm_hw_reassert(void)
{
  rk3576_pdm_clk_init();

  /* pdm1m1: both clock outputs plus the sdi1 data line */

  rk3576_iomux(RK3576_GPIO_BANK4, PDM_PIN_CLK0, PDM_PIN_FUNC);
  rk3576_iomux(RK3576_GPIO_BANK4, PDM_PIN_CLK1, PDM_PIN_FUNC);
  rk3576_iomux(RK3576_GPIO_BANK4, PDM_PIN_SDI1, PDM_PIN_FUNC);
  rk3576_pull(RK3576_GPIO_BANK4, PDM_PIN_SDI1, RK3576_PULL_NONE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rk3576_pdm_capture_init(void)
{
  rk3576_pdm_hw_reassert();

  /* Refuse to run against an unknown register map rather than silently
   * capturing nothing: this offset is the one thing that distinguishes the
   * v2 block from the classic one.
   */

  if (pdm_getreg(PDM_VERSION) != PDM_VERSION_RK3576)
    {
      return -ENODEV;
    }

  /* Halt receiver and filter, and flush, while we program */

  pdm_modifyreg(PDM_SYSCONFIG, PDM_SYSCONFIG_RUN_MSK | PDM_RX_CLR_WR,
                PDM_RX_CLR_WR);

  /* One stereo path fed from sdi1, 24-bit samples left justified in the
   * FIFO word, clock not inverted.
   */

  pdm_modifyreg(PDM_CTRL,
                PDM_RX_PATH_SEL_MSK(0) | PDM_CKP_MSK | PDM_SJM_SEL_MSK |
                PDM_PATH_MSK | PDM_VDW_MSK,
                PDM_RX_PATH_SEL(0, PDM_SDI_INDEX) | PDM_CKP_NORMAL |
                PDM_SJM_SEL_L | PDM_PATH0_EN | PDM_VDW(PDM_VDW_BITS));

  /* Decimation and conditioning.  The 24 dB gain is Rockchip's default and
   * matters here: a PDM mic's raw CIC output is quiet.
   */

  pdm_modifyreg(PDM_FILTER_CTRL,
                PDM_GAIN_MSK | PDM_HPF_MSK | PDM_HPF_FREQ_MSK |
                PDM_CIC_SCALE_MSK | PDM_CIC_RATIO_MSK,
                PDM_GAIN_24DB | PDM_HPF_L_EN | PDM_HPF_R_EN |
                PDM_HPF_FREQ_60 |
                PDM_CIC_SCALE(PDM_CIC_SCALE_VAL) |
                PDM_CIC_RATIO(PDM_CIC_RATIO_VAL));

  /* No DMA, no interrupts: this core polls */

  pdm_modifyreg(PDM_FIFO_CTRL,
                PDM_DMA_RD_MSK | PDM_RXOIE | PDM_RXFTIE, 0);

  g_pdm_overruns = 0;
  g_pdm_maxfifo  = 0;
  g_pdm_slot     = 0;
  g_pdm_channel  = 0;
  g_pdm_peak[0]  = 0;
  g_pdm_peak[1]  = 0;

  return PDM_SAMPLE_RATE;
}

void rk3576_pdm_start(void)
{
  /* Linux may have re-parented or gated our clocks (or re-muxed the pins)
   * since the last start; take them back first.
   */

  rk3576_pdm_hw_reassert();

  /* Receiver first, then let the mic wake up and the CIC filter settle
   * before the decimator starts emitting samples.
   */

  pdm_modifyreg(PDM_SYSCONFIG, PDM_RX_START, PDM_RX_START);
  up_mdelay(PDM_START_DELAY_MS);

  pdm_modifyreg(PDM_SYSCONFIG, PDM_NUM_START, PDM_NUM_START);

  /* Drop whatever settled into the FIFO and realign to channel 0 */

  pdm_modifyreg(PDM_SYSCONFIG, PDM_RX_CLR_WR, PDM_RX_CLR_WR);
  pdm_modifyreg(PDM_FIFO_CTRL, PDM_RXOIC, PDM_RXOIC);
  g_pdm_slot = 0;
}

void rk3576_pdm_stop(void)
{
  pdm_modifyreg(PDM_SYSCONFIG, PDM_SYSCONFIG_RUN_MSK | PDM_RX_CLR_WR,
                PDM_RX_CLR_WR);
}

size_t rk3576_pdm_read(int16_t *buf, size_t nsamples)
{
  uint32_t fifoctrl = pdm_getreg(PDM_FIFO_CTRL);
  uint32_t level    = PDM_FIFO_LEVEL(fifoctrl);
  size_t   got      = 0;

  if (level > g_pdm_maxfifo)
    {
      g_pdm_maxfifo = level;
    }

  if ((fifoctrl & PDM_RXOI) != 0)
    {
      /* Samples were lost, so the L/R phase is no longer trustworthy. */

      g_pdm_overruns++;
      g_pdm_slot = 0;
      pdm_modifyreg(PDM_FIFO_CTRL, PDM_RXOIC, PDM_RXOIC);
    }

  while (got < nsamples && level != 0)
    {
      uint32_t raw = pdm_getreg(PDM_RXFIFO_DATA);
      int16_t  s   = (int16_t)(raw >> 16);
      uint16_t mag = (uint16_t)(s < 0 ? -(int32_t)s : s);

      if (mag > g_pdm_peak[g_pdm_slot])
        {
          g_pdm_peak[g_pdm_slot] = mag;
        }

      /* The two channels alternate in the FIFO; hand out only ours. */

      if ((int)g_pdm_slot == g_pdm_channel)
        {
          buf[got++] = s;
        }

      g_pdm_slot ^= 1;
      level--;
    }

  return got;
}

void rk3576_pdm_set_channel(int ch)
{
  g_pdm_channel = (ch != 0) ? 1 : 0;
}

void rk3576_pdm_peaks(uint16_t *ch0, uint16_t *ch1)
{
  *ch0 = g_pdm_peak[0];
  *ch1 = g_pdm_peak[1];

  g_pdm_peak[0] = 0;
  g_pdm_peak[1] = 0;
}

uint32_t rk3576_pdm_overruns(void)
{
  return g_pdm_overruns;
}

uint32_t rk3576_pdm_maxfifo(void)
{
  return g_pdm_maxfifo;
}
