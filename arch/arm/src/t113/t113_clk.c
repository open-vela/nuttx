/****************************************************************************
 * arch/arm/src/t113/t113_clk.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "hardware/t113_ccu.h"
#include "hardware/t113_clk.h"
#include "t113_clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PLL control register bit definitions */

#define PLL_EN          (1 << 31)  /* PLL enable */
#define PLL_LDO_EN      (1 << 30)  /* PLL LDO enable */
#define PLL_LOCK_EN     (1 << 29)  /* Lock enable */
#define PLL_LOCK        (1 << 28)  /* PLL locked (read-only) */
#define PLL_OUT_EN      (1 << 27)  /* PLL output gating */

/* PLL_CPUX N-factor field: bits[15:8] */

#define PLL_CPUX_N_SHIFT  8
#define PLL_CPUX_N_MASK   (0xff << PLL_CPUX_N_SHIFT)

/* PLL_PERI0 N-factor field: bits[15:8] */

#define PLL_PERI0_N_SHIFT 8
#define PLL_PERI0_N_MASK  (0xff << PLL_PERI0_N_SHIFT)

/* PSI/APB clock source select: bits[25:24] */

#define CLK_SRC_SHIFT    24
#define CLK_SRC_MASK     (0x3 << CLK_SRC_SHIFT)
#define CLK_SRC_HOSC     (0x0 << CLK_SRC_SHIFT)
#define CLK_SRC_PSI_AHB  (0x2 << CLK_SRC_SHIFT)
#define CLK_SRC_PLL_PERI (0x3 << CLK_SRC_SHIFT)

/* PSI_CLK / APBx_CLK divider fields */

#define CLK_DIV_N_SHIFT  8
#define CLK_DIV_N_MASK   (0x3 << CLK_DIV_N_SHIFT)
#define CLK_DIV_M_SHIFT  0
#define CLK_DIV_M_MASK   (0x1f << CLK_DIV_M_SHIFT) /* 5-bit for APB */

/* PLL lock timeout: ~10ms at 24MHz HOSC (worst case CPU clock) */

#define PLL_LOCK_TIMEOUT 1000000

/* MBUS domain reset bit */

#define MBUS_RST         (1 << 30)

/* DMA BGR register bits */

#define DMA_RST          (1 << 16)
#define DMA_GATING       (1 << 0)

/* DE / DPSS-TOP / DSI / TCON-LCD0 CCU register addresses */

#define T113_CCU_DE_CLK         (T113_CCU_BASE + 0x0600) /* DE0 func clock  */
#define T113_CCU_DE_BGR         (T113_CCU_BASE + 0x060c) /* DE0 bus gate+rst */
#define T113_CCU_DPSS_TOP_BGR   (T113_CCU_BASE + 0x0abc) /* DPSS gate+rst   */
#define T113_CCU_DSI_CLK        (T113_CCU_BASE + 0x0b24) /* DSI func clock  */
#define T113_CCU_DSI_BGR        (T113_CCU_BASE + 0x0b4c) /* DSI bus gate+rst */
#define T113_CCU_TCON_LCD0_CLK  (T113_CCU_BASE + 0x0b60) /* TCON func clock */
#define T113_CCU_TCON_LCD0_BGR  (T113_CCU_BASE + 0x0b7c) /* TCON bus gate+rst */

/* Functional clock gate bit (bit31) and bus clock gate bit (bit0) */

#define CLK_FUNC_GATE    (1u << 31)
#define CLK_BUS_GATE     (1u << 0)

/* Reset bit: bit16, 0 = asserted, 1 = de-asserted */

#define CLK_RST_BIT      (1u << 16)

/* SMHC_BGR_REG bit layout (CCU+0x84C) */

#define SMHC_BGR_GATING(b)    (1u << (b))        /* bit 0/1/2: SMHCn bus gate */
#define SMHC_BGR_RST(b)       (1u << (16 + (b))) /* bit 16/17/18: SMHCn reset */

/* SMHCn_CLK_REG bit layout (CCU+0x830/0x834/0x838) */

#define SMHC_CLK_GATING       (1u << 31)
#define SMHC_CLK_SRC_HOSC     (0u << 24)         /* 24 MHz */
#define SMHC_CLK_SRC_PLL_PERI (1u << 24)         /* 600 MHz */
#define SMHC_CLK_FACTOR_N(n)  (((n) & 0x3) << 8)
#define SMHC_CLK_FACTOR_M(m)  (((m) & 0xf) << 0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clk_sdelay
 *
 * Description:
 *   Simple inline delay loop (matches boot0 sdelay).
 *   Each iteration ~1 cycle on Cortex-A7, used for short
 *   PLL stabilization waits.
 *
 ****************************************************************************/

static inline void clk_sdelay(uint32_t loops)
{
  __asm__ __volatile__("1:\n"
                       "subs %0, %1, #1\n"
                       "bne 1b"
                       : "=r"(loops)
                       : "0"(loops));
}

/****************************************************************************
 * Name: clk_set_pll_cpux
 *
 * Description:
 *   Configure PLL_CPUX to produce T113_PLL_CPUX_FREQUENCY.
 *   Idempotent: skips if PLL is already enabled with correct N.
 *
 *   Configures PLL_CPUX, AXI dividers, and enables AXI clock gating.
 *
 ****************************************************************************/

static void clk_set_pll_cpux(void)
{
  uint32_t val;
  uint32_t want_mask;
  uint32_t want_value;
  int timeout;

  /* Check if already configured with correct N, M, and P and enabled.
   * xfel may leave the PLL running with a non-default P divider, so
   * validate all factors before skipping reconfigure.
   */

  val = getreg32(T113_CCU_PLL_CPUX);
  want_mask  = PLL_EN | PLL_CPUX_N_MASK | (0x3 << 16) | (0x3 << 0);
  want_value = PLL_EN | (T113_PLL_CPUX_N << PLL_CPUX_N_SHIFT) |
               (0 << 16) | (0 << 0);
  if ((val & want_mask) == want_value)
    {
      return;
    }

  /* Switch CPU clock source to PLL_PERI(1x) while
   * reconfiguring PLL_CPUX
   */

  putreg32((4 << 24) | (1 << 0), T113_CCU_CPU_AXI_CFG);
  clk_sdelay(10);

  /* Disable PLL output gating */

  val = getreg32(T113_CCU_PLL_CPUX);
  val &= ~PLL_OUT_EN;
  putreg32(val, T113_CCU_PLL_CPUX);

  /* Enable PLL LDO */

  val = getreg32(T113_CCU_PLL_CPUX);
  val |= PLL_LDO_EN;
  putreg32(val, T113_CCU_PLL_CPUX);
  clk_sdelay(5);

  /* Set N factor for target frequency */

  val = getreg32(T113_CCU_PLL_CPUX);
  val &= ~((0x3 << 16) | PLL_CPUX_N_MASK | (0x3 << 0));
  val |= (T113_PLL_CPUX_N << PLL_CPUX_N_SHIFT);
  putreg32(val, T113_CCU_PLL_CPUX);

  /* Enable lock detect */

  val = getreg32(T113_CCU_PLL_CPUX);
  val |= PLL_LOCK_EN;
  putreg32(val, T113_CCU_PLL_CPUX);

  /* Enable PLL */

  val = getreg32(T113_CCU_PLL_CPUX);
  val |= PLL_EN;
  putreg32(val, T113_CCU_PLL_CPUX);

  /* Wait for PLL to lock (with timeout) */

  timeout = PLL_LOCK_TIMEOUT;
  while (!(getreg32(T113_CCU_PLL_CPUX) & PLL_LOCK) && --timeout);
  if (timeout <= 0)
    {
      /* PLL_CPUX failed to lock; we cannot safely continue running
       * on an unlocked PLL. Hard-hang so the failure is visible
       * (DEBUGASSERT would compile out in release builds).
       */

      for (; ; );
    }

  clk_sdelay(20);

  /* Enable PLL output gating */

  val = getreg32(T113_CCU_PLL_CPUX);
  val |= PLL_OUT_EN;
  putreg32(val, T113_CCU_PLL_CPUX);

  /* Disable lock detect */

  val = getreg32(T113_CCU_PLL_CPUX);
  val &= ~PLL_LOCK_EN;
  putreg32(val, T113_CCU_PLL_CPUX);
  clk_sdelay(1);

  /* Switch CPU to PLL_CPUX: src=PLL_CPUX(3), P=0, AXI_DIV=1 */

  val = getreg32(T113_CCU_CPU_AXI_CFG);
  val &= ~(0x07 << 24 | 0x3 << 16 | 0x3 << 8 | 0xf << 0);
  val |= (0x03 << 24 | 0x0 << 16 | 0x1 << 8 | 0x1 << 0);
  putreg32(val, T113_CCU_CPU_AXI_CFG);
  clk_sdelay(1);
}

/****************************************************************************
 * Name: clk_set_pll_periph0
 *
 * Description:
 *   Configure PLL_PERIPH0 with N from t113_clk.h.
 *   Idempotent: skips if PLL is enabled AND N matches target.
 *
 *   Configures PLL_PERIPH0 N, enables the PLL, and waits for lock.
 *
 ****************************************************************************/

static void clk_set_pll_periph0(void)
{
  uint32_t val;
  int timeout;

  /* PLL_PERIPH0 already at target N -- skip.
   * Must check N value, not just PLL_EN, because xfel may have
   * configured PLL with a different N.
   */

  val = getreg32(T113_CCU_PLL_PERIPH0);
  if ((val & PLL_EN) &&
      ((val & PLL_PERI0_N_MASK) ==
       (T113_PLL_PERI0_N << PLL_PERI0_N_SHIFT)))
    {
      return;
    }

  /* Switch PSI clock source to HOSC while reconfiguring PLL */

  val = getreg32(T113_CCU_PSI_CLK);
  val &= ~CLK_SRC_MASK;
  putreg32(val, T113_CCU_PSI_CLK);

  /* Set N=99 (0x63) -> PLL_PERI(1X) = 24*(99+1)/2/2 = 600 MHz */

  putreg32(T113_PLL_PERI0_N << PLL_PERI0_N_SHIFT,
           T113_CCU_PLL_PERIPH0);

  /* Enable lock detect */

  val = getreg32(T113_CCU_PLL_PERIPH0);
  val |= PLL_LOCK_EN;
  putreg32(val, T113_CCU_PLL_PERIPH0);

  /* Enable PLL */

  val = getreg32(T113_CCU_PLL_PERIPH0);
  val |= PLL_EN;
  putreg32(val, T113_CCU_PLL_PERIPH0);

  /* Wait for lock (with timeout) */

  timeout = PLL_LOCK_TIMEOUT;
  while (!(getreg32(T113_CCU_PLL_PERIPH0) & PLL_LOCK) &&
         --timeout);
  if (timeout <= 0)
    {
      /* PLL_PERIPH0 failed to lock; downstream bus/peripheral
       * clocks depend on it. Hard-hang so the failure is visible
       * (DEBUGASSERT would compile out in release builds).
       */

      for (; ; );
    }

  clk_sdelay(20);

  /* Disable lock detect */

  val = getreg32(T113_CCU_PLL_PERIPH0);
  val &= ~PLL_LOCK_EN;
  putreg32(val, T113_CCU_PLL_PERIPH0);
}

/****************************************************************************
 * Name: clk_set_psi_ahb
 *
 * Description:
 *   Configure PSI/AHB clock = PLL_PERI0(1X) / (M+1) = 200 MHz.
 *
 *   Selects PLL_PERI0(1X) as PSI/AHB source and programs the M divider.
 *
 ****************************************************************************/

static void clk_set_psi_ahb(void)
{
  uint32_t val;

  /* Set dividers first: N=0, M=T113_PSI_DIV_M */

  putreg32((T113_PSI_DIV_M << CLK_DIV_M_SHIFT) |
           (0 << CLK_DIV_N_SHIFT),
           T113_CCU_PSI_CLK);

  /* Switch source to PLL_PERI0(1X) -- RMW so we don't stomp any
   * existing bits that happen to fall inside the source field.
   */

  val = getreg32(T113_CCU_PSI_CLK);
  val &= ~CLK_SRC_MASK;
  val |= CLK_SRC_PLL_PERI;
  putreg32(val, T113_CCU_PSI_CLK);
  clk_sdelay(1);
}

/****************************************************************************
 * Name: clk_set_apb0
 *
 * Description:
 *   Configure APB0 = PLL_PERI0(1X) / 2^N / (M+1) = 100 MHz.
 *
 *   Selects PLL_PERI0(1X) as APB0 source and programs N/M dividers.
 *
 ****************************************************************************/

static void clk_set_apb0(void)
{
  uint32_t val;

  /* Set dividers first: N=T113_APB0_DIV_N, M=T113_APB0_DIV_M */

  putreg32((T113_APB0_DIV_M << CLK_DIV_M_SHIFT) |
           (T113_APB0_DIV_N << CLK_DIV_N_SHIFT),
           T113_CCU_APB0_CLK);

  /* Switch source to PLL_PERI0(1X) -- clear the source-field mask
   * before ORing in the new source, so any pre-existing bits in
   * that field (e.g. after a warm boot) are replaced, not merged.
   */

  val = getreg32(T113_CCU_APB0_CLK);
  val &= ~CLK_SRC_MASK;
  val |= CLK_SRC_PLL_PERI;
  putreg32(val, T113_CCU_APB0_CLK);
  clk_sdelay(1);
}

/****************************************************************************
 * Name: clk_set_apb1
 *
 * Description:
 *   Configure APB1 = PLL_PERI0(1X) / 2^N / (M+1) = 100 MHz.
 *   APB1 drives UART, TWI (I2C), and CAN peripherals.
 *
 *   This is NEW -- boot0 does not configure APB1, leaving it at
 *   the reset default of 24 MHz (HOSC).
 *
 ****************************************************************************/

static void clk_set_apb1(void)
{
  uint32_t val;

  /* Set dividers first: N=T113_APB1_DIV_N, M=T113_APB1_DIV_M */

  putreg32((T113_APB1_DIV_M << CLK_DIV_M_SHIFT) |
           (T113_APB1_DIV_N << CLK_DIV_N_SHIFT),
           T113_CCU_APB1_CLK);

  /* Switch source to PLL_PERI0(1X) -- clear the source-field mask
   * before ORing in the new source, so any pre-existing bits in
   * that field (e.g. after a warm boot) are replaced, not merged.
   */

  val = getreg32(T113_CCU_APB1_CLK);
  val &= ~CLK_SRC_MASK;
  val |= CLK_SRC_PLL_PERI;
  putreg32(val, T113_CCU_APB1_CLK);
  clk_sdelay(1);
}

/****************************************************************************
 * Name: clk_set_mbus
 *
 * Description:
 *   Reset MBUS domain and enable DMA master clock gating.
 *
 *   Resets the MBUS domain and enables DMA master clock gating.
 *
 ****************************************************************************/

static void clk_set_mbus(void)
{
  uint32_t val;

  /* Reset MBUS domain */

  val = getreg32(T113_CCU_MBUS_CLK);
  val |= MBUS_RST;
  putreg32(val, T113_CCU_MBUS_CLK);
  clk_sdelay(1);

  /* Enable MBUS master clock gating */

  putreg32(T113_CCU_MBUS_DMA_GATING, T113_CCU_MBUS_MAT);
}

/****************************************************************************
 * Name: clk_set_dma
 *
 * Description:
 *   De-assert DMA reset and enable DMA bus clock gating.
 *
 *   De-asserts DMA reset and enables DMA bus clock gating via the CCU.
 *
 ****************************************************************************/

static void clk_set_dma(void)
{
  /* De-assert DMA reset */

  putreg32(getreg32(T113_CCU_DMA_BGR) | DMA_RST,
           T113_CCU_DMA_BGR);
  clk_sdelay(20);

  /* Enable DMA bus clock gating */

  putreg32(getreg32(T113_CCU_DMA_BGR) | DMA_GATING,
           T113_CCU_DMA_BGR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_apb1_freq
 *
 * Description:
 *   Return the live APB1 bus-clock frequency by decoding the APB1_CLK
 *   register, rather than assuming the compile-time T113_APB1_FREQUENCY.
 *
 *   APB1 feeds UART/TWI/CAN.  When a foreign master (e.g. Linux remoteproc)
 *   owns the clock tree it may leave APB1 at a different rate than a NuttX
 *   master would program, and the slave must not reprogram APB1 because it
 *   is shared (Linux's own UART0 console hangs off it).  Reading the rate
 *   keeps the UART baud divisor correct without touching the shared clock.
 *
 *   APB1_CLK layout (0x524): src bits[25:24], pre-divide N=2^bits[9:8],
 *   post-divide (M+1)=bits[4:0]+1.  freq = parent / 2^N / (M+1).
 *
 ****************************************************************************/

uint32_t t113_apb1_freq(void)
{
  uint32_t reg = getreg32(T113_CCU_APB1_CLK);
  uint32_t n   = (reg & CLK_DIV_N_MASK) >> CLK_DIV_N_SHIFT;
  uint32_t m   = (reg & CLK_DIV_M_MASK) >> CLK_DIV_M_SHIFT;
  uint32_t parent;

  switch (reg & CLK_SRC_MASK)
    {
      case CLK_SRC_PLL_PERI:           /* 3: PLL_PERI0(1X) 600 MHz */
        parent = T113_PLL_PERI0_1X;
        break;

      case CLK_SRC_PSI_AHB:            /* 2: PSI/AHB */
        parent = T113_AHB_FREQUENCY;
        break;

      default:                         /* 0: HOSC 24 MHz, 1: LOSC (n/a here) */
        parent = T113_HOSC_FREQUENCY;
        break;
    }

  return parent / (1u << n) / (m + 1);
}

/****************************************************************************
 * Name: t113_clk_init
 *
 * Description:
 *   Initialize the T113 system clock tree:
 *     PLL_CPUX  -> 1008 MHz (CPU core clock)
 *     PLL_PERI0 -> 600 MHz (1x) / 1200 MHz (2x)
 *     PSI/AHB   -> 200 MHz
 *     APB0      -> 100 MHz
 *     APB1      -> 100 MHz (UART/TWI/CAN bus clock)
 *     MBUS      -> reset + DMA gating
 *     DMA       -> reset + gating
 *
 *   Must be called before t113_lowsetup() so that UART baud rate
 *   divisor calculation uses the correct APB1 frequency.
 *
 ****************************************************************************/

void t113_clk_init(void)
{
#ifdef CONFIG_T113_RPTUN_SLAVE
  /* rptun slave: the master has already configured PLL_CPUX / PLL_PERI0 /
   * PSI/AHB / APB / MBUS / DMA gating.  Re-running the sequence on
   * a live clock tree would briefly squeeze APB1 down to HOSC and
   * stall every active peripheral on the slave side (UART2 console
   * loses sync, DMAC pending transfers stall).  Inherit silently.
   *
   * Under a Linux master APB1 is shared with Linux's own UART0 console,
   * so the slave must NOT reprogram it here -- the UART driver instead
   * reads the live APB1 rate at runtime (t113_apb1_freq) to compute its
   * baud divisor.
   */

  return;
#else
  clk_set_pll_cpux();
  clk_set_pll_periph0();
  clk_set_psi_ahb();
  clk_set_apb0();
  clk_set_apb1();
  clk_set_mbus();
  clk_set_dma();
#endif
}

/****************************************************************************
 * Name: t113_clk_enable
 *
 * Description:
 *   Enable the clock gate for the given clock ID.
 *   For functional clocks (DE, DSI, TCON_LCD0) this sets bit31 only;
 *   parent/divider configuration is left to the driver via direct CCU
 *   register writes.
 *   For bus clocks (*_BUS) this sets bit0.
 *
 ****************************************************************************/

void t113_clk_enable(enum t113_clk_id_e id)
{
  switch (id)
    {
      case T113_CLK_DE:
        modifyreg32(T113_CCU_DE_CLK, 0, CLK_FUNC_GATE);
        break;

      case T113_CLK_DE_BUS:
        modifyreg32(T113_CCU_DE_BGR, 0, CLK_BUS_GATE);
        break;

      case T113_CLK_DPSS_TOP:
        modifyreg32(T113_CCU_DPSS_TOP_BGR, 0, CLK_BUS_GATE);
        break;

      case T113_CLK_DSI:
        modifyreg32(T113_CCU_DSI_CLK, 0, CLK_FUNC_GATE);
        break;

      case T113_CLK_DSI_BUS:
        modifyreg32(T113_CCU_DSI_BGR, 0, CLK_BUS_GATE);
        break;

      case T113_CLK_TCON_LCD0:
        modifyreg32(T113_CCU_TCON_LCD0_CLK, 0, CLK_FUNC_GATE);
        break;

      case T113_CLK_TCON_LCD0_BUS:
        modifyreg32(T113_CCU_TCON_LCD0_BGR, 0, CLK_BUS_GATE);
        break;
    }
}

/****************************************************************************
 * Name: t113_clk_disable
 *
 * Description:
 *   Disable (gate off) the clock for the given clock ID.
 *
 ****************************************************************************/

void t113_clk_disable(enum t113_clk_id_e id)
{
  switch (id)
    {
      case T113_CLK_DE:
        modifyreg32(T113_CCU_DE_CLK, CLK_FUNC_GATE, 0);
        break;

      case T113_CLK_DE_BUS:
        modifyreg32(T113_CCU_DE_BGR, CLK_BUS_GATE, 0);
        break;

      case T113_CLK_DPSS_TOP:
        modifyreg32(T113_CCU_DPSS_TOP_BGR, CLK_BUS_GATE, 0);
        break;

      case T113_CLK_DSI:
        modifyreg32(T113_CCU_DSI_CLK, CLK_FUNC_GATE, 0);
        break;

      case T113_CLK_DSI_BUS:
        modifyreg32(T113_CCU_DSI_BGR, CLK_BUS_GATE, 0);
        break;

      case T113_CLK_TCON_LCD0:
        modifyreg32(T113_CCU_TCON_LCD0_CLK, CLK_FUNC_GATE, 0);
        break;

      case T113_CLK_TCON_LCD0_BUS:
        modifyreg32(T113_CCU_TCON_LCD0_BGR, CLK_BUS_GATE, 0);
        break;
    }
}

/****************************************************************************
 * Name: t113_clk_reset_assert
 *
 * Description:
 *   Assert (hold in reset) the peripheral identified by id.
 *   Clears bit16 of the corresponding BGR register (0 = asserted).
 *
 ****************************************************************************/

void t113_clk_reset_assert(enum t113_rst_id_e id)
{
  switch (id)
    {
      case T113_RST_DE:
        modifyreg32(T113_CCU_DE_BGR, CLK_RST_BIT, 0);
        break;

      case T113_RST_DPSS_TOP:
        modifyreg32(T113_CCU_DPSS_TOP_BGR, CLK_RST_BIT, 0);
        break;

      case T113_RST_DSI:
        modifyreg32(T113_CCU_DSI_BGR, CLK_RST_BIT, 0);
        break;

      case T113_RST_TCON_LCD0:
        modifyreg32(T113_CCU_TCON_LCD0_BGR, CLK_RST_BIT, 0);
        break;
    }
}

/****************************************************************************
 * Name: t113_clk_reset_deassert
 *
 * Description:
 *   De-assert (release from reset) the peripheral identified by id.
 *   Sets bit16 of the corresponding BGR register (1 = running).
 *
 ****************************************************************************/

void t113_clk_reset_deassert(enum t113_rst_id_e id)
{
  switch (id)
    {
      case T113_RST_DE:
        modifyreg32(T113_CCU_DE_BGR, 0, CLK_RST_BIT);
        break;

      case T113_RST_DPSS_TOP:
        modifyreg32(T113_CCU_DPSS_TOP_BGR, 0, CLK_RST_BIT);
        break;

      case T113_RST_DSI:
        modifyreg32(T113_CCU_DSI_BGR, 0, CLK_RST_BIT);
        break;

      case T113_RST_TCON_LCD0:
        modifyreg32(T113_CCU_TCON_LCD0_BGR, 0, CLK_RST_BIT);
        break;
    }
}

/****************************************************************************
 * Name: t113_smhc_clk_enable
 *
 * Description:
 *   Bring SMHCn out of reset, ungate its APB clock, select PLL_PERI(1X)
 *   as the module clock source, and program the divider for freq_hz.
 *
 *   Divider math (parent/(1<<N)/(M+1)/2 because the controller always
 *   inserts a hardware /2 before the pad):
 *     SDCLK = parent / (1 << N) / (M + 1) / 2
 *   For freq_hz = 25 MHz:  src=PLL_PERI(1X), N=0, M=11 -> 600/1/12/2
 *   = 25 MHz.
 *
 ****************************************************************************/

int t113_smhc_clk_enable(int bus, uint32_t freq_hz)
{
  uint32_t smhc_clk_reg;
  uint32_t v;
  uint32_t src;
  uint32_t n;
  uint32_t m;
  uint32_t parent;
  uint32_t cclk;
  uint32_t div;
  uint32_t actual;

  if (bus < 0 || bus > 2)
    {
      return -EINVAL;
    }

  if (freq_hz == 0)
    {
      return -EINVAL;
    }

  smhc_clk_reg = T113_CCU_MMC0_CLK + bus * 4;

  /* The controller needs an 8-branch divider dispatch
   * (div > 128 / 64 / 32 / 16 / else) that splits the divisor between
   * FACTOR_N (shift, 1/2/4/8) and FACTOR_M (M+1, 1..16); a naive loop
   * that only escalates N while div > 16 stops short on the HOSC path
   * and cannot reach the SD-spec-required 400 kHz init clock.
   *
   * Source rule: PLL_PERI(1X)=600MHz when cclk > HOSC/2 = 12 MHz,
   * otherwise HOSC=24 MHz.
   *
   * Pre-double cclk to account for the controller's internal 2:1 pad
   * divider, then use rounded div = (2*sclk + cclk) / (2*cclk) --
   * round-to-nearest behaviour on non-round targets.
   */

  if (freq_hz > 12000000u)
    {
      src    = SMHC_CLK_SRC_PLL_PERI;
      parent = 600000000u;
    }
  else
    {
      src    = SMHC_CLK_SRC_HOSC;
      parent = 24000000u;
    }

  cclk = freq_hz * 2u;
  div  = (2u * parent + cclk) / (2u * cclk);
  if (div == 0)
    {
      div = 1;
    }

  /* 8-branch dispatch using floor shifts (div >> n), not ceiling.  The
   * resulting SDCLK can be slightly above the requested freq_hz on
   * non-round targets; this matches the controller's intended behaviour.
   */

  if (div > 128)
    {
      n = 3;
      m = 16;
    }
  else if (div > 64)
    {
      n = 3;
      m = div >> 3;
    }
  else if (div > 32)
    {
      n = 2;
      m = div >> 2;
    }
  else if (div > 16)
    {
      n = 1;
      m = div >> 1;
    }
  else
    {
      n = 0;
      m = div;
    }

  m = m - 1;

  /* Sanity bound: compute the SDCLK we just landed on and make sure it
   * stays in [requested/4, requested*2].  The upper bound catches the
   * div>128 clamp (we cannot actually reach the target, so refuse to
   * overclock the card).  The lower bound is generous -- the dispatch
   * can round down by up to 2x on awkward targets but never 4x.
   */

  actual = parent / (1u << n) / (m + 1u) / 2u;
  if (actual > freq_hz * 2u || actual < freq_hz / 4u)
    {
      return -ERANGE;
    }

  /* Step 1: enable the SMHCn bus gate. */

  v  = getreg32(T113_CCU_MMC_BGR);
  v |= SMHC_BGR_GATING(bus);
  putreg32(v, T113_CCU_MMC_BGR);

  /* Step 2: de-assert the SMHCn reset. */

  v  = getreg32(T113_CCU_MMC_BGR);
  v |= SMHC_BGR_RST(bus);
  putreg32(v, T113_CCU_MMC_BGR);

  /* Step 3: program SMHCn_CLK_REG with gating cleared while the divider
   * is updated, then re-enable gating.
   */

  putreg32(0, smhc_clk_reg);
  putreg32(src | SMHC_CLK_FACTOR_N(n) | SMHC_CLK_FACTOR_M(m),
           smhc_clk_reg);

  v  = getreg32(smhc_clk_reg);
  v |= SMHC_CLK_GATING;
  putreg32(v, smhc_clk_reg);

  /* Phase compensation fields in SMHC_NTSR are left at the SoC reset
   * value (0x81710000, sample phase 90 deg) for default-speed 25 MHz
   * operation.
   */

  return 0;
}

/****************************************************************************
 * Name: t113_smhc_clk_disable
 *
 * Description:
 *   Gate the SMHCn bus clock and hold the module in reset.
 *
 ****************************************************************************/

void t113_smhc_clk_disable(int bus)
{
  uint32_t smhc_clk_reg;
  uint32_t v;

  if (bus < 0 || bus > 2)
    {
      return;
    }

  smhc_clk_reg = T113_CCU_MMC0_CLK + bus * 4;

  /* Gate the module clock first, then drop the bus gate and assert reset. */

  v  = getreg32(smhc_clk_reg);
  v &= ~SMHC_CLK_GATING;
  putreg32(v, smhc_clk_reg);

  v  = getreg32(T113_CCU_MMC_BGR);
  v &= ~(SMHC_BGR_GATING(bus) | SMHC_BGR_RST(bus));
  putreg32(v, T113_CCU_MMC_BGR);
}

/****************************************************************************
 * Audio clock provider (PLL_AUDIO1 refcount + per-consumer divider)
 *
 * PLL_AUDIO1 is the shared parent for the PDM DMIC controller, the audio
 * codec ADC and the audio codec DAC.  We bring it up on the first
 * t113_audio_clk_request() and tear it down once the last consumer has
 * released, so power is only drawn while audio is in use.
 *
 * Each consumer owns its own M/N divider register sourced from
 * PLL_AUDIO1(DIV5) (per user manual sections 3.3.6.81/83/84):
 *   DMIC                 -> 0x0A40 DMIC_CLK_REG
 *   audio codec DAC      -> 0x0A50 AUDIO_CODEC_DAC_CLK_REG
 *   audio codec ADC      -> 0x0A54 AUDIO_CODEC_ADC_CLK_REG
 *
 * PLL_AUDIO1 frequency choice:
 *   Mainline Linux on the sister sun20i-d1 chip uses PLL_AUDIO1(DIV5)
 *   at exactly 614.4 MHz to feed the 48 kHz audio family
 *   (drivers/clk/sunxi-ng/ccu-sun20i-d1.c, around the pll_audio1_div5_clk
 *   definition).  We follow the same strategy.
 *
 *     PLL_AUDIO1       = 24 MHz * (N+1) / (M+1)
 *     PLL_AUDIO1(DIV5) = PLL_AUDIO1 / (P1+1)
 *
 *   Using the silicon reset factors -- N field = 0x7F, M field = 0,
 *   P1 field = 4 -- gives:
 *     PLL_AUDIO1       = 24 MHz * 128 / 1   = 3072 MHz
 *     PLL_AUDIO1(DIV5) = 3072 MHz / 5       = 614.4 MHz
 *
 *   Each audio consumer then divides DIV5 by 25 (FACTOR_N=0 (/1),
 *   FACTOR_M=24 (=25)) to land at 24.576 MHz exactly -- no SDM, no
 *   fractional-N, no per-silicon tuning.  This avoids the Pattern0
 *   semantics ambiguity that bit the earlier PLL_AUDIO0 attempt on
 *   T113-S3 silicon (sun55iw3 patterns overshooting at our operating
 *   point because Allwinner's SDM patterns are point-specific).
 *
 *   See user manual section 3.3.4.2 (page 61) for general PLL bring-up
 *   sequence and section 3.3.6.8 PLL_AUDIO1_CTRL_REG (page 84) for the
 *   register layout.
 *
 * Sample-rate family note:
 *   614.4 MHz / 25 = 24.576 MHz feeds the 8/16/32/48/96/192 kHz
 *   family cleanly -- the codec/DMIC controllers divide internally to
 *   reach the actual sample rate.  The 11.025/22.05/44.1/88.2 kHz
 *   family would want 22.5792 MHz, which neither PLL_AUDIO1 (integer
 *   only) nor 614.4 MHz / integer can produce.  That family is out of
 *   scope for the current driver and would require re-introducing
 *   PLL_AUDIO0 with a properly characterised SDM pattern.
 *
 * Codec ADC + DAC rate constraint:
 *   The codec analog block shares a single sample-rate domain.  Even
 *   though ADC and DAC each have their own CCU divider register, when
 *   both are active simultaneously they must agree on the sample rate;
 *   the second consumer to request a different rate is rejected with
 *   -EINVAL.  DMIC is independent of the codec.  Today every consumer
 *   gets the same 24.576 MHz module clock regardless of target_rate, so
 *   the rate-agreement check is conservatively kept for future SR
 *   diversity.
 *
 ****************************************************************************/

/* PLL lock poll uses up_udelay(); 100us * 100 = 10ms timeout */

#define AUDIO_PLL_LOCK_POLL_US      100
#define AUDIO_PLL_LOCK_POLL_LOOPS   100

/* PLL_AUDIO1 target factors:
 *     24 MHz * (N+1) / (M+1) / (P1+1)
 *   = 24 MHz * 128   / 1     / 5      = 614.4 MHz on the DIV5 tap
 */

#define AUDIO_PLL_TARGET_N          127        /* N field, N actual = 128 */
#define AUDIO_PLL_TARGET_P1         4          /* P1 field, P1 actual = 5 */
#define AUDIO_PLL_DIV5_HZ           614400000u /* PLL_AUDIO1(DIV5) output */

/* Module-clock target fed to every audio consumer.  614.4 MHz / 25
 * yields 24.576 MHz exactly with FACTOR_N=0 (/1), FACTOR_M=24 (=25).
 */

#define AUDIO_MODULE_CLK_HZ         24576000u

/* Bit field that uniquely identifies the target PLL programming so that
 * a warm-boot (PLL already up at the right factors) can skip reconfigure.
 * SDM_EN must be 0 in the target value -- if a previous boot left SDM
 * armed we must reprogram to clear it.  M field bit 1 must be 0 too
 * (M actual = 1).
 */

#define AUDIO_PLL_TARGET_MASK       (T113_CCU_PLL_AUDIO1_EN |     \
                                     T113_CCU_PLL_AUDIO1_SDM_EN | \
                                     T113_CCU_PLL_AUDIO1_N_MASK | \
                                     T113_CCU_PLL_AUDIO1_P1_MASK | \
                                     T113_CCU_PLL_AUDIO1_M)

#define AUDIO_PLL_TARGET_VALUE      (T113_CCU_PLL_AUDIO1_EN |     \
                                     T113_CCU_PLL_AUDIO1_N(AUDIO_PLL_TARGET_N) | \
                                     T113_CCU_PLL_AUDIO1_P1(AUDIO_PLL_TARGET_P1))

static struct
{
  mutex_t  lock;
  int      refcount;
  uint32_t consumer_mask;     /* bit per consumer */
  uint32_t consumer_rate[T113_AUDIO_CONSUMER_NUM];
} g_audio_clk =
{
  .lock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Name: audio_pll_enable_locked
 *
 * Description:
 *   Configure PLL_AUDIO1 to 3072 MHz (614.4 MHz on the DIV5 tap) in
 *   plain integer mode and wait for the PLL to lock.  Idempotent: if
 *   the PLL is already running with the target N/M/P1 factors and SDM
 *   disabled, return success immediately.
 *
 *   Programming sequence follows user manual section 3.3.4.3 / 3.3.4.4
 *   (general-PLL configure + enable, page 62-63):
 *     1) write N / M / P1 factors with SDM_EN cleared
 *     2) re-arm lock detect (LOCK_EN 0 -> 1)
 *     3) enable PLL
 *     4) wait for LOCK
 *
 *   The Pattern0 register is left at its reset value (0x0); SDM stays
 *   disabled -- 614.4 MHz / 25 = 24.576 MHz exact, so we don't need it.
 *
 *   Caller must hold g_audio_clk.lock.
 *
 * Returned Value:
 *   0 on success or -ETIMEDOUT if the PLL fails to lock within 10ms.
 *
 ****************************************************************************/

static int audio_pll_enable_locked(void)
{
  uint32_t val;
  int      i;

  /* Already programmed and enabled with the right factors and SDM
   * disabled -- skip.
   */

  val = getreg32(T113_CCU_PLL_AUDIO1);
  if ((val & AUDIO_PLL_TARGET_MASK) == AUDIO_PLL_TARGET_VALUE)
    {
      return 0;
    }

  /* Disable PLL output gating before reprogramming factors */

  val = getreg32(T113_CCU_PLL_AUDIO1);
  val &= ~T113_CCU_PLL_AUDIO1_OUT_EN;
  putreg32(val, T113_CCU_PLL_AUDIO1);

  /* Make sure the LDO is enabled */

  val = getreg32(T113_CCU_PLL_AUDIO1);
  val |= T113_CCU_PLL_AUDIO1_LDO_EN;
  putreg32(val, T113_CCU_PLL_AUDIO1);
  up_udelay(5);

  /* Program N / M / P1 (target factors) and clear SDM_EN.  Clear EN
   * during the write so we re-arm the lock detector cleanly afterwards.
   * P0 is left untouched (the DIV2 tap is unused by audio consumers).
   */

  val = getreg32(T113_CCU_PLL_AUDIO1);
  val &= ~(T113_CCU_PLL_AUDIO1_EN |
           T113_CCU_PLL_AUDIO1_SDM_EN |
           T113_CCU_PLL_AUDIO1_N_MASK |
           T113_CCU_PLL_AUDIO1_P1_MASK |
           T113_CCU_PLL_AUDIO1_M);
  val |= T113_CCU_PLL_AUDIO1_N(AUDIO_PLL_TARGET_N) |
         T113_CCU_PLL_AUDIO1_P1(AUDIO_PLL_TARGET_P1);
  putreg32(val, T113_CCU_PLL_AUDIO1);

  /* Re-arm lock detect (write 0 then 1 per user manual section 3.3.4.3) */

  val = getreg32(T113_CCU_PLL_AUDIO1);
  val &= ~T113_CCU_PLL_AUDIO1_LOCK_EN;
  putreg32(val, T113_CCU_PLL_AUDIO1);

  val |= T113_CCU_PLL_AUDIO1_LOCK_EN;
  putreg32(val, T113_CCU_PLL_AUDIO1);

  /* Enable PLL */

  val = getreg32(T113_CCU_PLL_AUDIO1);
  val |= T113_CCU_PLL_AUDIO1_EN;
  putreg32(val, T113_CCU_PLL_AUDIO1);

  /* Poll LOCK with 10ms timeout */

  for (i = 0; i < AUDIO_PLL_LOCK_POLL_LOOPS; i++)
    {
      if (getreg32(T113_CCU_PLL_AUDIO1) & T113_CCU_PLL_AUDIO1_LOCK)
        {
          break;
        }

      up_udelay(AUDIO_PLL_LOCK_POLL_US);
    }

  if (!(getreg32(T113_CCU_PLL_AUDIO1) & T113_CCU_PLL_AUDIO1_LOCK))
    {
      /* Lock failed; leave the PLL gated and report timeout */

      val = getreg32(T113_CCU_PLL_AUDIO1);
      val &= ~T113_CCU_PLL_AUDIO1_EN;
      putreg32(val, T113_CCU_PLL_AUDIO1);
      return -ETIMEDOUT;
    }

  /* Disable lock detect (post-lock; per user manual recommendation) */

  val = getreg32(T113_CCU_PLL_AUDIO1);
  val &= ~T113_CCU_PLL_AUDIO1_LOCK_EN;
  putreg32(val, T113_CCU_PLL_AUDIO1);

  /* Re-enable output gating */

  val = getreg32(T113_CCU_PLL_AUDIO1);
  val |= T113_CCU_PLL_AUDIO1_OUT_EN;
  putreg32(val, T113_CCU_PLL_AUDIO1);

  return 0;
}

/****************************************************************************
 * Name: audio_pll_disable_locked
 *
 * Description:
 *   Gate the PLL_AUDIO1 output and disable the PLL.  Caller must hold
 *   g_audio_clk.lock.
 *
 ****************************************************************************/

static void audio_pll_disable_locked(void)
{
  uint32_t val;

  val = getreg32(T113_CCU_PLL_AUDIO1);
  val &= ~(T113_CCU_PLL_AUDIO1_OUT_EN | T113_CCU_PLL_AUDIO1_EN);
  putreg32(val, T113_CCU_PLL_AUDIO1);
}

/****************************************************************************
 * Name: audio_clk_compute_div
 *
 * Description:
 *   Compute integer factor_n (encoded 0..3 -> /1,/2,/4,/8) and factor_m
 *   (M-1 in [0..31]) so that PLL_AUDIO1(DIV5) / N / M == module_rate.
 *
 *   module_rate is the desired controller-input module clock.  Today
 *   every audio consumer wants the same 24.576 MHz module clock; the
 *   codec / DMIC controller divides it internally to reach the audio
 *   sample rate.  614.4 MHz / 1 / 25 = 24.576 MHz lands on FACTOR_N=0,
 *   FACTOR_M=24.
 *
 * Returned Value:
 *   0 on success with *factor_n / *factor_m populated; -EINVAL if the
 *   target rate cannot be expressed exactly with the available divider.
 *
 ****************************************************************************/

static int audio_clk_compute_div(uint32_t module_rate,
                                 uint32_t *factor_n, uint32_t *factor_m)
{
  uint32_t parent = AUDIO_PLL_DIV5_HZ;
  uint32_t n;
  uint32_t div;

  if (module_rate == 0 || module_rate > parent)
    {
      return -EINVAL;
    }

  /* Try N = /1, /2, /4, /8 and pick the smallest N that yields an
   * integer M in [1..32] with parent / (1<<n) / m == module_rate.
   */

  for (n = 0; n < 4; n++)
    {
      uint32_t step = parent >> n;

      if (step % module_rate != 0)
        {
          continue;
        }

      div = step / module_rate;
      if (div >= 1 && div <= 32)
        {
          *factor_n = n;
          *factor_m = div - 1;
          return 0;
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: audio_consumer_div_configure
 *
 * Description:
 *   Configure the M/N divider register for one consumer, select
 *   PLL_AUDIO1(DIV5) as source, enable the module clock, and de-assert
 *   the peripheral reset / open the bus gate.
 *
 *   For DMIC the controller has its own internal sample-rate divider
 *   (sun50i-dmic SR field) and accepts the 24.576 MHz parent directly,
 *   same as the codec.  target_rate is therefore unused today; it is
 *   kept on the signature so the codec analog block can reject a
 *   conflicting ADC/DAC rate request, and so a future SR family
 *   selection (eg 22.5792 MHz from PLL_AUDIO0) can land here without
 *   touching every caller.
 *
 *   Caller must hold g_audio_clk.lock.
 *
 ****************************************************************************/

static int audio_consumer_div_configure(enum t113_audio_consumer_e c,
                                        uint32_t target_rate)
{
  uint32_t clk_reg;
  uint32_t bgr_reg;
  uint32_t bgr_bits;
  uint32_t factor_n;
  uint32_t factor_m;
  uint32_t v;
  int      ret;

  UNUSED(target_rate);

  /* Map consumer -> CLK / BGR registers.  Every consumer wants the same
   * 24.576 MHz module clock from PLL_AUDIO1(DIV5); the controllers
   * divide internally to reach the audio sample rate.
   */

  switch (c)
    {
      case T113_AUDIO_CONSUMER_DMIC:
        clk_reg  = T113_CCU_DMIC_CLK_REG;
        bgr_reg  = T113_CCU_DMIC_BGR;
        bgr_bits = T113_CCU_DMIC_GATING | T113_CCU_DMIC_RST;
        break;

      case T113_AUDIO_CONSUMER_CODEC_ADC:
        clk_reg  = T113_CCU_AUDIO_CODEC_ADC_CLK;
        bgr_reg  = T113_CCU_AUDIO_BGR;
        bgr_bits = T113_CCU_AUDIO_CODEC_GATING |
                   T113_CCU_AUDIO_CODEC_RST;
        break;

      case T113_AUDIO_CONSUMER_CODEC_DAC:
        clk_reg  = T113_CCU_AUDIO_CODEC_DAC_CLK;
        bgr_reg  = T113_CCU_AUDIO_BGR;
        bgr_bits = T113_CCU_AUDIO_CODEC_GATING |
                   T113_CCU_AUDIO_CODEC_RST;
        break;

      default:
        return -EINVAL;
    }

  ret = audio_clk_compute_div(AUDIO_MODULE_CLK_HZ, &factor_n, &factor_m);
  if (ret < 0)
    {
      return ret;
    }

  /* Program divider with module gating off, then re-enable the module
   * clock gate.  Source select 010 = PLL_AUDIO1(DIV5) at 614.4 MHz.
   */

  putreg32(T113_CCU_AUDIO_CLK_SRC_PLL_A1D5 |
           T113_CCU_AUDIO_CLK_FACTOR_N(factor_n) |
           T113_CCU_AUDIO_CLK_FACTOR_M(factor_m),
           clk_reg);

  v  = getreg32(clk_reg);
  v |= T113_CCU_AUDIO_CLK_GATING;
  putreg32(v, clk_reg);

  /* Open bus gate and de-assert reset.  For the codec these bits are
   * shared between ADC and DAC consumers; setting them again when the
   * other consumer is already up is harmless.
   */

  v  = getreg32(bgr_reg);
  v |= bgr_bits;
  putreg32(v, bgr_reg);

  return 0;
}

/****************************************************************************
 * Name: audio_consumer_div_disable
 *
 * Description:
 *   Gate the consumer divider clock and, if no other consumer of the
 *   shared BGR remains, drop the bus gate.  Reset is left de-asserted to
 *   avoid disturbing any in-flight state of the other consumer (the
 *   codec ADC and DAC share AUDIO_CODEC_BGR; tearing one down must not
 *   reset the other).
 *
 *   Caller must hold g_audio_clk.lock.
 *
 ****************************************************************************/

static void audio_consumer_div_disable(enum t113_audio_consumer_e c)
{
  uint32_t clk_reg;
  uint32_t v;
  bool     codec_other_active;

  switch (c)
    {
      case T113_AUDIO_CONSUMER_DMIC:
        clk_reg = T113_CCU_DMIC_CLK_REG;

        /* Gate the module clock then drop the BGR (DMIC has no peer). */

        v  = getreg32(clk_reg);
        v &= ~T113_CCU_AUDIO_CLK_GATING;
        putreg32(v, clk_reg);

        v  = getreg32(T113_CCU_DMIC_BGR);
        v &= ~(T113_CCU_DMIC_GATING | T113_CCU_DMIC_RST);
        putreg32(v, T113_CCU_DMIC_BGR);
        return;

      case T113_AUDIO_CONSUMER_CODEC_ADC:
        clk_reg            = T113_CCU_AUDIO_CODEC_ADC_CLK;
        codec_other_active = (g_audio_clk.consumer_mask &
                              (1u << T113_AUDIO_CONSUMER_CODEC_DAC)) != 0;
        break;

      case T113_AUDIO_CONSUMER_CODEC_DAC:
        clk_reg            = T113_CCU_AUDIO_CODEC_DAC_CLK;
        codec_other_active = (g_audio_clk.consumer_mask &
                              (1u << T113_AUDIO_CONSUMER_CODEC_ADC)) != 0;
        break;

      default:
        return;
    }

  /* Always gate this consumer's module clock divider */

  v  = getreg32(clk_reg);
  v &= ~T113_CCU_AUDIO_CLK_GATING;
  putreg32(v, clk_reg);

  /* Drop the codec BGR only when the peer codec consumer is already gone */

  if (!codec_other_active)
    {
      v  = getreg32(T113_CCU_AUDIO_BGR);
      v &= ~(T113_CCU_AUDIO_CODEC_GATING | T113_CCU_AUDIO_CODEC_RST);
      putreg32(v, T113_CCU_AUDIO_BGR);
    }
}

/****************************************************************************
 * Name: t113_audio_clk_request
 ****************************************************************************/

int t113_audio_clk_request(enum t113_audio_consumer_e c,
                           uint32_t target_rate)
{
  uint32_t peer_mask;
  uint32_t peer_rate;
  int      ret = 0;

  if ((unsigned)c >= T113_AUDIO_CONSUMER_NUM || target_rate == 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&g_audio_clk.lock);

  if (g_audio_clk.consumer_mask & (1u << c))
    {
      /* Duplicate request from same consumer */

      ret = -EBUSY;
      goto out;
    }

  /* Codec ADC/DAC share the analog block; if one is already active the
   * other must agree on the sample rate.
   */

  if (c == T113_AUDIO_CONSUMER_CODEC_ADC)
    {
      peer_mask = 1u << T113_AUDIO_CONSUMER_CODEC_DAC;
      peer_rate = g_audio_clk.consumer_rate[T113_AUDIO_CONSUMER_CODEC_DAC];
      if ((g_audio_clk.consumer_mask & peer_mask) &&
          peer_rate != target_rate)
        {
          ret = -EINVAL;
          goto out;
        }
    }
  else if (c == T113_AUDIO_CONSUMER_CODEC_DAC)
    {
      peer_mask = 1u << T113_AUDIO_CONSUMER_CODEC_ADC;
      peer_rate = g_audio_clk.consumer_rate[T113_AUDIO_CONSUMER_CODEC_ADC];
      if ((g_audio_clk.consumer_mask & peer_mask) &&
          peer_rate != target_rate)
        {
          ret = -EINVAL;
          goto out;
        }
    }

  if (g_audio_clk.refcount == 0)
    {
      ret = audio_pll_enable_locked();
      if (ret < 0)
        {
          goto out;
        }
    }

  ret = audio_consumer_div_configure(c, target_rate);
  if (ret < 0)
    {
      if (g_audio_clk.refcount == 0)
        {
          audio_pll_disable_locked();
        }

      goto out;
    }

  g_audio_clk.refcount++;
  g_audio_clk.consumer_mask  |= (1u << c);
  g_audio_clk.consumer_rate[c] = target_rate;

out:
  nxmutex_unlock(&g_audio_clk.lock);
  return ret;
}

/****************************************************************************
 * Name: t113_audio_clk_release
 ****************************************************************************/

int t113_audio_clk_release(enum t113_audio_consumer_e c)
{
  if ((unsigned)c >= T113_AUDIO_CONSUMER_NUM)
    {
      return -EINVAL;
    }

  nxmutex_lock(&g_audio_clk.lock);

  if (!(g_audio_clk.consumer_mask & (1u << c)))
    {
      nxmutex_unlock(&g_audio_clk.lock);
      return -EINVAL;
    }

  audio_consumer_div_disable(c);

  g_audio_clk.consumer_mask  &= ~(1u << c);
  g_audio_clk.consumer_rate[c] = 0;
  g_audio_clk.refcount--;

  if (g_audio_clk.refcount == 0)
    {
      audio_pll_disable_locked();
    }

  nxmutex_unlock(&g_audio_clk.lock);
  return 0;
}
