/****************************************************************************
 * arch/arm/src/t113/hardware/t113_dsp.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_DSP_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_DSP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/t113_ccu.h"   /* T113_CCU_BASE = 0x02001000 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DSP CFG block --------------------------------------------------------- */

#define T113_DSP_CFG_BASE              0x01700000

#define T113_DSP_ALT_RESET_VEC_OFFSET  0x0000
#define T113_DSP_CTRL_REG0_OFFSET      0x0004

#define T113_DSP_ALT_RESET_VEC_REG     (T113_DSP_CFG_BASE + T113_DSP_ALT_RESET_VEC_OFFSET)
#define T113_DSP_CTRL_REG0_REG         (T113_DSP_CFG_BASE + T113_DSP_CTRL_REG0_OFFSET)

/* DSP_CTRL_REG0 bits */

#define T113_DSP_CTRL_RUNSTALL         (1 << 0)
#define T113_DSP_CTRL_START_VEC_SEL    (1 << 1)
#define T113_DSP_CTRL_DSP_CLKEN        (1 << 2)

/* CCU DSP clock / bus gating reset --------------------------------------- */

#define T113_CCU_DSP_CLK_OFFSET        0x0c70
#define T113_CCU_DSP_BGR_OFFSET        0x0c7c

#define T113_CCU_DSP_CLK_REG           (T113_CCU_BASE + T113_CCU_DSP_CLK_OFFSET)
#define T113_CCU_DSP_BGR_REG           (T113_CCU_BASE + T113_CCU_DSP_BGR_OFFSET)

/* DSP_CLK_REG fields */

#define T113_CCU_DSP_CLK_GATING        (1u << 31)
#define T113_CCU_DSP_CLK_SRC_SHIFT     24
#define T113_CCU_DSP_CLK_SRC_MASK      (0x7u << T113_CCU_DSP_CLK_SRC_SHIFT)
#define T113_CCU_DSP_CLK_SRC_HOSC      (0u << T113_CCU_DSP_CLK_SRC_SHIFT)
#define T113_CCU_DSP_CLK_SRC_PERI2X    (3u << T113_CCU_DSP_CLK_SRC_SHIFT)
/* Register encodes (M-1); hardware divisor is M.  M must be 1..32. */
#define T113_CCU_DSP_CLK_FACTOR_M(m)   (((m) - 1u) & 0x1fu)

/* DSP_BGR_REG fields */

#define T113_CCU_DSP_BGR_CFG_GATING    (1 << 1)
#define T113_CCU_DSP_BGR_DSP0_RST      (1 << 16)
#define T113_CCU_DSP_BGR_CFG_RST       (1 << 17)
#define T113_CCU_DSP_BGR_DBG_RST       (1 << 18)

/* SRAMC remap -- controls SRAM A1/A2 ownership (AP vs DSP) --------------- */

#define T113_SRAMC_BASE                0x03000000
#define T113_SRAMC_REMAP_OFFSET        0x0008
#define T113_SRAMC_REMAP_REG           (T113_SRAMC_BASE + T113_SRAMC_REMAP_OFFSET)
#define T113_SRAMC_REMAP_ENABLE        (1u << 0)  /* 1=AP owns, 0=DSP owns */

/* UART2 -- assigned to DSP (PE2 TX / PE3 RX, mux 3) --------------------- */

/* UART2 base: 0x02500000 (SP1 section) + 0x800 (UART2 offset).
 * Confirmed by t113_memorymap.h T113_SP1_PSECTION + T113_UART2_OFFSET
 * and sunxi-d1s-t113.dtsi uart2: serial@2500800.
 */

#define T113_UART2_BASE                0x02500800

/* CCU UART_BGR_REG: bit2=UART2 BUS gating, bit18=UART2 reset deassert */

#define T113_CCU_UART_BGR_OFFSET       0x090c
#define T113_CCU_UART_BGR_REG          (T113_CCU_BASE + T113_CCU_UART_BGR_OFFSET)
#define T113_CCU_UART2_GATING          (1 << 2)
#define T113_CCU_UART2_RST             (1 << 18)

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_DSP_H */
