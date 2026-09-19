/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_sai.c
 *
 * RK3576 SAI capture (I2S master) for the openvela AMP slave.
 *
 * Register layout and the I2S timing recipe follow the Linux driver this
 * core coexists with: sound/soc/rockchip/rockchip_sai.{c,h}.  Unlike Linux
 * we do not use DMA: the RK3576 SAI has no "FIFO half full" interrupt (only
 * overrun / frame-sync errors), and the three PL330 DMACs are all owned by
 * Linux.  Since this core is dedicated to the RTOS, draining the FIFO from a
 * polling thread is both simpler and perfectly affordable.
 *
 * Clocking (all derived from the 24 MHz crystal so Linux can never change it
 * underneath us — the audio PLLs belong to Linux):
 *
 *   mclk_sai2_2ch_src = xin24m / SAI_MCLK_DIV
 *   mclk_sai2_2ch     = mclk_sai2_2ch_src        (mux 0)
 *   SCLK              = mclk / MDIV              (MDIV = 1)
 *   LRCK (fs)         = SCLK / 64                (2 slots x 32 bit)
 *
 * With SAI_MCLK_DIV = 24 that gives SCLK = 1 MHz and fs = 15625 Hz, i.e.
 * 2.3% below the nominal 16 kHz.  A MEMS mic does not care (it is a slave to
 * our clocks) and the exact rate is reported to the host, so playback stays
 * in tune.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>

#include <arch/chip/chip.h>

#include "arm64_internal.h"
#include "rk3576_sai.h"
#include "rk3576_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets (sound/soc/rockchip/rockchip_sai.h) */

#define SAI_TXCR                0x0000
#define SAI_FSCR                0x0004
#define SAI_RXCR                0x0008
#define SAI_MONO_CR             0x000c
#define SAI_XFER                0x0010
#define SAI_CLR                 0x0014
#define SAI_CKR                 0x0018
#define SAI_RXFIFOLR            0x0020
#define SAI_DMACR               0x0024
#define SAI_INTCR               0x0028
#define SAI_INTSR               0x002c
#define SAI_RXDR                0x0034
#define SAI_PATH_SEL            0x0038
#define SAI_RX_SHIFT            0x0068

/* RXCR / TXCR fields */

#define SAI_XCR_VDW(x)          (((x) - 1) << 0)    /* valid data width  */
#define SAI_XCR_SBW(x)          (((x) - 1) << 5)    /* slot bit width    */
#define SAI_XCR_VDJ_L           (1u << 10)          /* valid data left-justified */
#define SAI_XCR_SNB(x)          (((x) - 1) << 11)   /* slots per frame   */
#define SAI_XCR_CSR(x)          (((x) - 1) << 20)   /* number of SD lines*/
#define SAI_XCR_EDGE_SHIFT_1    (1u << 22)          /* I2S: shift 1 SCLK */

/* FSCR */

#define SAI_FSCR_FW(x)          (((x) - 1) << 0)    /* frame width       */
#define SAI_FSCR_FPW(x)         (((x) - 1) << 12)   /* frame pulse width */
#define SAI_FSCR_EDGE_DUAL      (1u << 24)

/* XFER */

#define SAI_XFER_CLK_EN         (1u << 0)
#define SAI_XFER_FSS_EN         (1u << 1)
#define SAI_XFER_RXS_EN         (1u << 3)

/* CLR */

#define SAI_CLR_RXC             (1u << 1)
#define SAI_CLR_FCR             (1u << 3)

/* CKR */

#define SAI_CKR_MDIV(x)         (((x) - 1) << 3)
#define SAI_CKR_MSS_MASTER      0

/* INTCR / INTSR */

#define SAI_INTCR_RXOIE         (1u << 17)
#define SAI_INTCR_RXOIC         (1u << 18)
#define SAI_INTSR_RXOI_ACT      (1u << 17)

/* RX_SHIFT: I2S needs the data shifted right by 2 SCLKs */

#define SAI_XSHIFT_RIGHT(x)     (x)

/* RXFIFOLR: level of SDI0 in the low 6 bits */

#define SAI_FIFOLR_XFL0(v)      ((v) & 0x3f)

/* Frame geometry: 2 slots (L/R) of 32 bit -> 64 SCLK per frame */

#define SAI_SLOT_BITS           32
#define SAI_SLOTS               2
#define SAI_FRAME_BITS          (SAI_SLOT_BITS * SAI_SLOTS)

/* Clock plan, see the file header */

#define SAI_XIN_HZ              24000000
#define SAI_MCLK_DIV            24
#define SAI_MCLK_HZ             (SAI_XIN_HZ / SAI_MCLK_DIV)
#define SAI_SAMPLE_RATE         (SAI_MCLK_HZ / SAI_FRAME_BITS)

/* CRU bits for SAI2 (drivers/clk/rockchip/clk-rk3576.c)
 *   MCLK_SAI2_2CH_SRC  CLKSEL_CON47 mux[10:8] div[7:0]  gate CON8 bit7
 *   MCLK_SAI2_2CH      CLKSEL_CON47 mux[12:11]          gate CON8 bit8
 *   HCLK_SAI2_2CH                                       gate CON8 bit10
 * audio_frac_int_p mux 0 = xin24m.
 */

#define SAI2_CLKSEL_CON         47
#define SAI2_SRC_MUX_SHIFT      8
#define SAI2_SRC_MUX_WIDTH      3
#define SAI2_SRC_MUX_XIN24M     0
#define SAI2_SRC_DIV_SHIFT      0
#define SAI2_SRC_DIV_WIDTH      8
#define SAI2_MCLK_MUX_SHIFT     11
#define SAI2_MCLK_MUX_WIDTH     2
#define SAI2_MCLK_MUX_SRC       0
#define SAI2_GATE_CON           8
#define SAI2_GATE_SRC_BIT       7
#define SAI2_GATE_MCLK_BIT      8
#define SAI2_GATE_HCLK_BIT      10

/* sai2m0 pins, all function 4 on bank 1 */

#define SAI2_PIN_SCLK           RK3576_PIN_D(1)
#define SAI2_PIN_LRCK           RK3576_PIN_D(2)
#define SAI2_PIN_SDI            RK3576_PIN_D(3)
#define SAI2_PIN_FUNC           4

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_sai_overruns;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t sai_getreg(unsigned int off)
{
  return getreg32(RK3576_SAI2_BASE + off);
}

static inline void sai_putreg(unsigned int off, uint32_t val)
{
  putreg32(val, RK3576_SAI2_BASE + off);
}

/****************************************************************************
 * Name: rk3576_sai_clk_init
 *
 * Description:
 *   Route SAI2's mclk to the 24 MHz crystal divided down to SAI_MCLK_HZ and
 *   ungate everything the block needs.
 *
 ****************************************************************************/

static void rk3576_sai_clk_init(void)
{
  /* mclk source: xin24m / SAI_MCLK_DIV */

  rk3576_clk_set_mux_div(SAI2_CLKSEL_CON,
                         SAI2_SRC_MUX_SHIFT, SAI2_SRC_MUX_WIDTH,
                         SAI2_SRC_MUX_XIN24M,
                         SAI2_SRC_DIV_SHIFT, SAI2_SRC_DIV_WIDTH,
                         SAI_MCLK_DIV);

  /* mclk mux: take the divider output (not an external mclk-in pin) */

  rk3576_clk_set_mux_div(SAI2_CLKSEL_CON,
                         SAI2_MCLK_MUX_SHIFT, SAI2_MCLK_MUX_WIDTH,
                         SAI2_MCLK_MUX_SRC,
                         0, 0, 0);

  rk3576_clk_gate(SAI2_GATE_CON, SAI2_GATE_SRC_BIT, true);
  rk3576_clk_gate(SAI2_GATE_CON, SAI2_GATE_MCLK_BIT, true);
  rk3576_clk_gate(SAI2_GATE_CON, SAI2_GATE_HCLK_BIT, true);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rk3576_sai_capture_init(void)
{
  uint32_t rxcr;

  rk3576_sai_clk_init();

  /* sai2m0: SCLK / LRCK / SDI on the 40-pin header */

  rk3576_iomux(RK3576_GPIO_BANK1, SAI2_PIN_SCLK, SAI2_PIN_FUNC);
  rk3576_iomux(RK3576_GPIO_BANK1, SAI2_PIN_LRCK, SAI2_PIN_FUNC);
  rk3576_iomux(RK3576_GPIO_BANK1, SAI2_PIN_SDI, SAI2_PIN_FUNC);
  rk3576_pull(RK3576_GPIO_BANK1, SAI2_PIN_SDI, RK3576_PULL_NONE);

  /* Hold everything stopped while we program it */

  sai_putreg(SAI_XFER, 0);
  sai_putreg(SAI_CLR, SAI_CLR_RXC | SAI_CLR_FCR);

  /* Receiver: 2 slots x 32 bit, 32 valid bits, MSB first, left justified,
   * one SD line, I2S edge shift.
   */

  rxcr = SAI_XCR_VDW(32) | SAI_XCR_SBW(SAI_SLOT_BITS) |
         SAI_XCR_SNB(SAI_SLOTS) | SAI_XCR_CSR(1) |
         SAI_XCR_VDJ_L | SAI_XCR_EDGE_SHIFT_1;
  sai_putreg(SAI_RXCR, rxcr);

  /* Frame: 64 SCLK wide, LRCK low for the first half (I2S) */

  sai_putreg(SAI_FSCR, SAI_FSCR_FW(SAI_FRAME_BITS) |
                       SAI_FSCR_FPW(SAI_FRAME_BITS / 2) |
                       SAI_FSCR_EDGE_DUAL);

  /* I2S puts the MSB one SCLK after the LRCK edge */

  sai_putreg(SAI_RX_SHIFT, SAI_XSHIFT_RIGHT(2));

  /* We are the clock master; SCLK = mclk / 1 */

  sai_putreg(SAI_CKR, SAI_CKR_MDIV(1) | SAI_CKR_MSS_MASTER);

  /* RX data comes from SDI0 (PATH_SEL RX path 0 = 0), no DMA */

  sai_putreg(SAI_PATH_SEL, 0);
  sai_putreg(SAI_DMACR, 0);

  /* Only care about overruns; there is no FIFO-level interrupt */

  sai_putreg(SAI_INTCR, SAI_INTCR_RXOIC | SAI_INTCR_RXOIE);

  /* Leave the receiver stopped: with nobody draining the FIFO it would
   * overrun continuously.  rk3576_sai_start() turns it on.
   */

  g_sai_overruns = 0;

  return SAI_SAMPLE_RATE;
}

void rk3576_sai_start(void)
{
  /* Flush anything stale, then run clocks + frame sync + receiver */

  sai_putreg(SAI_CLR, SAI_CLR_RXC | SAI_CLR_FCR);
  sai_putreg(SAI_INTCR, sai_getreg(SAI_INTCR) | SAI_INTCR_RXOIC);
  sai_putreg(SAI_XFER, SAI_XFER_CLK_EN | SAI_XFER_FSS_EN | SAI_XFER_RXS_EN);
}

void rk3576_sai_stop(void)
{
  sai_putreg(SAI_XFER, 0);
  sai_putreg(SAI_CLR, SAI_CLR_RXC | SAI_CLR_FCR);
}

size_t rk3576_sai_read(int16_t *buf, size_t nsamples)
{
  size_t got = 0;

  /* Note an overrun and clear it; the data already in the FIFO is still
   * good, we just lost whatever did not fit.
   */

  if ((sai_getreg(SAI_INTSR) & SAI_INTSR_RXOI_ACT) != 0)
    {
      g_sai_overruns++;
      sai_putreg(SAI_INTCR, sai_getreg(SAI_INTCR) | SAI_INTCR_RXOIC);
    }

  while (got < nsamples)
    {
      uint32_t raw;

      if (SAI_FIFOLR_XFL0(sai_getreg(SAI_RXFIFOLR)) == 0)
        {
          break;
        }

      raw = sai_getreg(SAI_RXDR);

      /* One FIFO word per frame (the controller only latches the enabled
       * slot, measured on hardware), and a 24-bit MEMS mic left-justifies
       * its sample in the 32-bit slot — so the top 16 bits are exactly the
       * 16-bit PCM we want.
       */

      buf[got++] = (int16_t)(raw >> 16);
    }

  return got;
}

uint32_t rk3576_sai_overruns(void)
{
  return g_sai_overruns;
}
