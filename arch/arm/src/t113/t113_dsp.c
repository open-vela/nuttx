/****************************************************************************
 * arch/arm/src/t113/t113_dsp.c
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
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "hardware/t113_ccu.h"
#include "hardware/t113_dsp.h"
#include "t113_dsp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void t113_setbits32(uint32_t addr, uint32_t mask,
                                  uint32_t bits)
{
  putreg32((getreg32(addr) & ~mask) | (bits & mask), addr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_sramc_remap
 *
 * Description:
 *   Set SRAMC remap bit.  value=1 means AP owns the SRAM; value=0 hands
 *   it to the DSP.  Must be called with 1 before touching DSP clock/reset
 *   registers, and with 0 just before releasing RUN_STALL.
 *
 ****************************************************************************/

void t113_sramc_remap(int value)
{
  t113_setbits32(T113_SRAMC_REMAP_REG,
                 T113_SRAMC_REMAP_ENABLE,
                 value ? T113_SRAMC_REMAP_ENABLE : 0u);
}

/****************************************************************************
 * Name: t113_dsp_clk_set
 *
 * Description:
 *   Configure DSP root clock.  Programs PERI2X (1.2 GHz) / 2 = 600 MHz.
 *
 * Returns the resulting rate in Hz (600000000).
 *
 ****************************************************************************/

uint32_t t113_dsp_clk_set(uint32_t freq_hz)
{
  (void)freq_hz;

  t113_setbits32(T113_CCU_DSP_CLK_REG,
                 T113_CCU_DSP_CLK_SRC_MASK | 0x1fu,
                 T113_CCU_DSP_CLK_SRC_PERI2X | T113_CCU_DSP_CLK_FACTOR_M(2));

  t113_setbits32(T113_CCU_DSP_CLK_REG,
                 T113_CCU_DSP_CLK_GATING,
                 T113_CCU_DSP_CLK_GATING);

  return 600000000u;
}

/****************************************************************************
 * Name: t113_dsp_release
 *
 * Description:
 *   Release HiFi4 DSP from reset and start it executing at `entry`.
 *   Called by the rptun start op after the framework has loaded DSP
 *   firmware via the remoteproc loader.
 *
 *   Sequence matches the T113 datasheet boot order:
 *
 *   1.  SRAMC remap = 1 (AP owns SRAM while we program clocks / resets)
 *   2.  Configure DSP root clock (PERI2X / 2 = 600 MHz) + gate on
 *   3.  Enable CFG bus gating
 *   4.  De-assert CFG + DBG resets
 *   5.  Write firmware entry to ALT_RESET_VEC
 *   6.  START_VEC_SEL = 1
 *   7.  RUN_STALL = 1
 *   8.  DSP_CLKEN = 1
 *   9.  De-assert DSP core reset
 *   10. SRAMC remap = 0 (DSP owns SRAM)
 *   11. RUN_STALL = 0 -- DSP starts fetching from `entry`
 *
 * Returns 0 on success.
 *
 ****************************************************************************/

int t113_dsp_release(uint32_t entry)
{
  /* 0. Enable UART2 bus clock and de-assert reset so the DSP firmware
   *    can use UART2 for debug output.
   */

  t113_setbits32(T113_CCU_UART_BGR_REG,
                 T113_CCU_UART2_RST,
                 0);
  t113_setbits32(T113_CCU_UART_BGR_REG,
                 T113_CCU_UART2_GATING | T113_CCU_UART2_RST,
                 T113_CCU_UART2_GATING | T113_CCU_UART2_RST);

  /* 1. AP owns SRAM while we set up clocks and resets (idempotent). */

  t113_sramc_remap(1);

  /* 2. Configure DSP root clock */

  t113_dsp_clk_set(0);

  /* 3. Enable CFG bus gating so DSP_CTRL_REG0 / ALT_RESET_VEC are writable */

  t113_setbits32(T113_CCU_DSP_BGR_REG,
                 T113_CCU_DSP_BGR_CFG_GATING,
                 T113_CCU_DSP_BGR_CFG_GATING);

  /* 4. De-assert CFG and DBG resets */

  t113_setbits32(T113_CCU_DSP_BGR_REG,
                 T113_CCU_DSP_BGR_CFG_RST | T113_CCU_DSP_BGR_DBG_RST,
                 T113_CCU_DSP_BGR_CFG_RST | T113_CCU_DSP_BGR_DBG_RST);

  /* 5. Write firmware entry to ALT reset vector */

  putreg32(entry, T113_DSP_ALT_RESET_VEC_REG);

  /* 6. Select ALT reset vector */

  t113_setbits32(T113_DSP_CTRL_REG0_REG,
                 T113_DSP_CTRL_START_VEC_SEL,
                 T113_DSP_CTRL_START_VEC_SEL);

  /* 7. Assert RUN_STALL to keep DSP halted while we finish setup */

  t113_setbits32(T113_DSP_CTRL_REG0_REG,
                 T113_DSP_CTRL_RUNSTALL,
                 T113_DSP_CTRL_RUNSTALL);

  /* 8. Enable DSP internal clock gate */

  t113_setbits32(T113_DSP_CTRL_REG0_REG,
                 T113_DSP_CTRL_DSP_CLKEN,
                 T113_DSP_CTRL_DSP_CLKEN);

  /* 9. De-assert DSP core reset */

  t113_setbits32(T113_CCU_DSP_BGR_REG,
                 T113_CCU_DSP_BGR_DSP0_RST,
                 T113_CCU_DSP_BGR_DSP0_RST);

  /* 10. Hand SRAM ownership to DSP before releasing it */

  t113_sramc_remap(0);

  /* 11. Release RUN_STALL -- DSP starts fetching from `entry` */

  t113_setbits32(T113_DSP_CTRL_REG0_REG, T113_DSP_CTRL_RUNSTALL, 0);

  return 0;
}

/****************************************************************************
 * Name: t113_dsp_halt
 *
 * Description:
 *   Halt the DSP and gate its clock.  Idempotent.
 *
 ****************************************************************************/

void t113_dsp_halt(void)
{
  /* Freeze fetch */

  t113_setbits32(T113_DSP_CTRL_REG0_REG,
           T113_DSP_CTRL_RUNSTALL,
           T113_DSP_CTRL_RUNSTALL);

  up_mdelay(5);

  /* Disable internal clock + assert core reset */

  t113_setbits32(T113_DSP_CTRL_REG0_REG, T113_DSP_CTRL_DSP_CLKEN, 0);

  t113_setbits32(T113_CCU_DSP_BGR_REG,
           T113_CCU_DSP_BGR_DSP0_RST,
           0);

  up_mdelay(5);

  /* Tear down CFG/DBG/CFG_GATING */

  t113_setbits32(T113_CCU_DSP_BGR_REG,
           T113_CCU_DSP_BGR_CFG_GATING |
             T113_CCU_DSP_BGR_CFG_RST   |
             T113_CCU_DSP_BGR_DBG_RST,
           0);
}
