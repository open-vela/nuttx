/****************************************************************************
 * arch/arm/src/t113/hardware/t113_ccu.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_CCU_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_CCU_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define T113_CCU_BASE           0x02001000

/* PLL registers */

#define T113_CCU_PLL_CPUX       (T113_CCU_BASE + 0x000)
#define T113_CCU_PLL_DDR0       (T113_CCU_BASE + 0x010)
#define T113_CCU_PLL_PERIPH0    (T113_CCU_BASE + 0x020)
#define T113_CCU_PLL_VE         (T113_CCU_BASE + 0x058)
#define T113_CCU_PLL_VIDEO0     (T113_CCU_BASE + 0x040)
#define T113_CCU_PLL_VIDEO1     (T113_CCU_BASE + 0x048)
#define T113_CCU_PLL_AUDIO0     (T113_CCU_BASE + 0x078)
#define T113_CCU_PLL_AUDIO1     (T113_CCU_BASE + 0x080)

/* Bus clock source / divider registers */

#define T113_CCU_CPU_AXI_CFG    (T113_CCU_BASE + 0x500)
#define T113_CCU_PSI_CLK        (T113_CCU_BASE + 0x510)
#define T113_CCU_APB0_CLK       (T113_CCU_BASE + 0x520)
#define T113_CCU_APB1_CLK       (T113_CCU_BASE + 0x524)
#define T113_CCU_MBUS_CLK       (T113_CCU_BASE + 0x540)

/* MBUS bandwidth control */

#define T113_CCU_MBUS_MAT       (T113_CCU_BASE + 0x804)

/* MBUS master clock gating value for DMA.
 * Matches boot0 set_mbus() configuration.
 * bit[11]    = DMA master enable
 * bit[10:8]  = DMA master priority
 * bit[7:0]   = DMA master bandwidth limit
 */

#define T113_CCU_MBUS_DMA_GATING  0x00000d87

/* Bus Gating Reset (BGR) registers.
 * Each BGR register controls clock gating and reset for a peripheral bus.
 * bit[N+16] = reset deassert (1 = running, 0 = held in reset)
 * bit[N]    = clock gate     (1 = enabled, 0 = gated off)
 */

#define T113_CCU_PWM_BGR        (T113_CCU_BASE + 0x07ac)
#define T113_CCU_IOMMU_BGR      (T113_CCU_BASE + 0x07bc)
#define T113_CCU_DMA_BGR        (T113_CCU_BASE + 0x070c)
#define T113_CCU_MMC_BGR        (T113_CCU_BASE + 0x084c)
#define T113_CCU_UART_BGR       (T113_CCU_BASE + 0x090c)
#define T113_CCU_TWI_BGR        (T113_CCU_BASE + 0x091c)
#define T113_CCU_SPI_BGR        (T113_CCU_BASE + 0x096c)
#define T113_CCU_CAN_BGR        (T113_CCU_BASE + 0x092c)
#define T113_CCU_EMAC_BGR       (T113_CCU_BASE + 0x097c)
#define T113_CCU_IR_TX_BGR      (T113_CCU_BASE + 0x09cc)
#define T113_CCU_GPADC_BGR      (T113_CCU_BASE + 0x09ec)
#define T113_CCU_THS_BGR        (T113_CCU_BASE + 0x09fc)
#define T113_CCU_I2S_BGR        (T113_CCU_BASE + 0x0a20)
#define T113_CCU_SPDIF_BGR      (T113_CCU_BASE + 0x0a2c)
#define T113_CCU_DMIC_BGR       (T113_CCU_BASE + 0x0a4c)
#define T113_CCU_AUDIO_BGR      (T113_CCU_BASE + 0x0a5c)
#define T113_CCU_USB_BGR        (T113_CCU_BASE + 0x0a8c)
#define T113_CCU_LRADC_BGR      (T113_CCU_BASE + 0x0a9c)
#define T113_CCU_CE_CLK         (T113_CCU_BASE + 0x0680)
#define T113_CCU_CE_BGR         (T113_CCU_BASE + 0x068c)
#define T113_CCU_HSTIMER_BGR    (T113_CCU_BASE + 0x073c)
#define T113_CCU_SPINLOCK_BGR   (T113_CCU_BASE + 0x072c)
#define T113_CCU_TPADC_BGR      (T113_CCU_BASE + 0x0c5c)

/* CE_BGR_REG bit definitions */

#define T113_CCU_CE_GATING      (1 << 0)    /* bit0: clock gate */
#define T113_CCU_CE_RST         (1 << 16)   /* bit16: de-assert reset */

/* MBUS_MAT_CLK_GATING_REG bit for CE (bit2) */

#define T113_CCU_MBUS_CE_EN     (1 << 2)    /* bit2: CE MBUS gate */

/* HSTIMER_BGR_REG bit definitions */

#define T113_CCU_HSTIMER_GATING (1 << 0)    /* bit0: clock gate */
#define T113_CCU_HSTIMER_RST    (1 << 16)   /* bit16: de-assert reset */

/* SPINLOCK_BGR_REG bit definitions (CCU offset 0x072C) */

#define T113_CCU_SPINLOCK_GATING (1 << 0)   /* bit0: clock gate */
#define T113_CCU_SPINLOCK_RST    (1 << 16)  /* bit16: de-assert reset */

/* Peripheral clock source / divider registers */

#define T113_CCU_MMC0_CLK       (T113_CCU_BASE + 0x0830)
#define T113_CCU_MMC1_CLK       (T113_CCU_BASE + 0x0834)
#define T113_CCU_MMC2_CLK       (T113_CCU_BASE + 0x0838)
#define T113_CCU_SPI0_CLK       (T113_CCU_BASE + 0x0940)
#define T113_CCU_SPI1_CLK       (T113_CCU_BASE + 0x0944)
#define T113_CCU_USB0_CLK       (T113_CCU_BASE + 0x0a70)
#define T113_CCU_USB1_CLK       (T113_CCU_BASE + 0x0a74)

/* SPI_CLK_REG bit definitions (SPI0: 0x0940, SPI1: 0x0944)
 * SCLK = Clock_Source / FACTOR_N / (FACTOR_M + 1)
 */

#define T113_CCU_SPI_CLK_GATING         (1 << 31)   /* bit31: clock on/off */
#define T113_CCU_SPI_CLK_SRC_MASK       (0x7 << 24) /* bits26:24 */
#define T113_CCU_SPI_CLK_SRC_HOSC       (0x0 << 24) /* 24 MHz */
#define T113_CCU_SPI_CLK_SRC_PLL_PERI1X (0x1 << 24) /* 600 MHz */
#define T113_CCU_SPI_CLK_SRC_PLL_PERI2X (0x2 << 24) /* 1200 MHz */
#define T113_CCU_SPI_FACTOR_N_MASK      (0x3 << 8)  /* bits9:8: N=1/2/4/8 */
#define T113_CCU_SPI_FACTOR_N(n)        ((n) << 8)
#define T113_CCU_SPI_FACTOR_M_MASK      (0xf << 0)  /* bits3:0: M-1 */
#define T113_CCU_SPI_FACTOR_M(m)        (((m) - 1) << 0)

/* SPI_BGR_REG bit definitions (0x096C) */

#define T113_CCU_SPI0_GATING            (1 << 0)    /* bit0: bus gate */
#define T113_CCU_SPI1_GATING            (1 << 1)    /* bit1: bus gate */
#define T113_CCU_SPI0_RST               (1 << 16)   /* bit16: de-assert reset */
#define T113_CCU_SPI1_RST               (1 << 17)   /* bit17: de-assert reset */

/* CAN_BGR_REG bit definitions (0x092C) */

#define T113_CCU_CAN0_GATING            (1 << 0)    /* bit0: CAN0 bus gate */
#define T113_CCU_CAN1_GATING            (1 << 1)    /* bit1: CAN1 bus gate */
#define T113_CCU_CAN0_RST               (1 << 16)   /* bit16: CAN0 reset deassert */
#define T113_CCU_CAN1_RST               (1 << 17)   /* bit17: CAN1 reset deassert */

/* Audio module clock registers (user manual section 3.3.6.81-3.3.6.85,
 * pages 132-135).  Each consumer (DMIC, audio codec DAC, audio codec ADC)
 * has its own M/N divider register sourced from PLL_AUDIO0(1X) or
 * PLL_AUDIO1.
 */

#define T113_CCU_DMIC_CLK_REG           (T113_CCU_BASE + 0x0a40)
#define T113_CCU_AUDIO_CODEC_DAC_CLK    (T113_CCU_BASE + 0x0a50)
#define T113_CCU_AUDIO_CODEC_ADC_CLK    (T113_CCU_BASE + 0x0a54)

/* Common bit layout for DMIC_CLK_REG / AUDIO_CODEC_{DAC,ADC}_CLK_REG.
 *   bit31    : module clock gating  (0=off, 1=on)
 *   bit26:24 : clock source select  (000=PLL_AUDIO0(1X),
 *                                    001=PLL_AUDIO1(DIV2),
 *                                    010=PLL_AUDIO1(DIV5))
 *   bit9:8   : FACTOR_N              (00=/1, 01=/2, 10=/4, 11=/8)
 *   bit4:0   : FACTOR_M              (M = FACTOR_M + 1, 1..32)
 *   Output frequency = clock source / N / M.
 */

#define T113_CCU_AUDIO_CLK_GATING       (1u << 31)
#define T113_CCU_AUDIO_CLK_SRC_SHIFT    24
#define T113_CCU_AUDIO_CLK_SRC_MASK     (0x7u << 24)
#define T113_CCU_AUDIO_CLK_SRC_PLL_A0   (0x0u << 24)
#define T113_CCU_AUDIO_CLK_SRC_PLL_A1D2 (0x1u << 24)
#define T113_CCU_AUDIO_CLK_SRC_PLL_A1D5 (0x2u << 24)
#define T113_CCU_AUDIO_CLK_FACTOR_N_SHIFT  8
#define T113_CCU_AUDIO_CLK_FACTOR_N_MASK   (0x3u << 8)
#define T113_CCU_AUDIO_CLK_FACTOR_N(n)     (((n) & 0x3u) << 8)
#define T113_CCU_AUDIO_CLK_FACTOR_M_MASK   (0x1fu << 0)
#define T113_CCU_AUDIO_CLK_FACTOR_M(m)     (((m) & 0x1fu) << 0)

/* DMIC_BGR_REG / AUDIO_CODEC_BGR_REG bit definitions
 * (user manual sections 3.3.6.82 / 3.3.6.85, pages 133/135).
 *   bit16 : module reset      (0=assert, 1=de-assert)
 *   bit0  : module bus gating (0=mask,   1=pass)
 */

#define T113_CCU_DMIC_GATING            (1u << 0)
#define T113_CCU_DMIC_RST               (1u << 16)
#define T113_CCU_AUDIO_CODEC_GATING     (1u << 0)
#define T113_CCU_AUDIO_CODEC_RST        (1u << 16)

/* PLL_AUDIO1_CTRL_REG bit definitions (user manual section 3.3.6.8,
 * pages 84-85, register offset 0x0080, default 0x4841_7F00).
 *   PLL_AUDIO1       = 24 MHz * (N+1) / (M+1)
 *   PLL_AUDIO1(DIV2) = PLL_AUDIO1 / (P0+1)
 *   PLL_AUDIO1(DIV5) = PLL_AUDIO1 / (P1+1)
 *
 * The audio clock provider drives DMIC / codec ADC / codec DAC from
 * PLL_AUDIO1(DIV5) which is the same path mainline Linux uses on the
 * sister sun20i-d1 chip (drivers/clk/sunxi-ng/ccu-sun20i-d1.c).  At
 * the reset values N=128, M=1, P1=5 the DIV5 output is 614.4 MHz
 * exactly, and 614.4 / 25 = 24.576 MHz feeds every audio consumer with
 * no fractional-N magic required.
 */

#define T113_CCU_PLL_AUDIO1_EN          (1u << 31)  /* bit31: PLL enable */
#define T113_CCU_PLL_AUDIO1_LDO_EN      (1u << 30)  /* bit30: LDO enable */
#define T113_CCU_PLL_AUDIO1_LOCK_EN     (1u << 29)  /* bit29: lock detect en */
#define T113_CCU_PLL_AUDIO1_LOCK        (1u << 28)  /* bit28: locked (RO) */
#define T113_CCU_PLL_AUDIO1_OUT_EN      (1u << 27)  /* bit27: output gate */
#define T113_CCU_PLL_AUDIO1_SDM_EN      (1u << 24)  /* bit24: sigma-delta */
#define T113_CCU_PLL_AUDIO1_P1_SHIFT    20
#define T113_CCU_PLL_AUDIO1_P1_MASK     (0x7u << 20)    /* P1 = field + 1 */
#define T113_CCU_PLL_AUDIO1_P1(p)       (((p) & 0x7u) << 20)
#define T113_CCU_PLL_AUDIO1_P0_SHIFT    16
#define T113_CCU_PLL_AUDIO1_P0_MASK     (0x7u << 16)    /* P0 = field + 1 */
#define T113_CCU_PLL_AUDIO1_P0(p)       (((p) & 0x7u) << 16)
#define T113_CCU_PLL_AUDIO1_N_SHIFT     8
#define T113_CCU_PLL_AUDIO1_N_MASK      (0xffu << 8)    /* N = field + 1 */
#define T113_CCU_PLL_AUDIO1_N(n)        (((n) & 0xffu) << 8)
#define T113_CCU_PLL_AUDIO1_M           (1u << 1)       /* M = field + 1 */

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_CCU_H */
