/****************************************************************************
 * arch/arm/src/t113/hardware/t113_dsi.h
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

/* T113-S3 MIPI DSI host + DPHY register layout.
 *
 * Host registers are at T113_DSI_BASE (0x05450000); DPHY registers are at
 * T113_DSI_BASE + 0x1000.  Each register is 32-bit, 4-byte aligned.
 *
 * Source: T113-S3 User Manual v1.1 section 5.4 (MIPI DSI), and the GC9503CV
 * X4B panel bring-up sequence verified on R528 HMI EVB4.
 */

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_DSI_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_DSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "t113_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DPHY block sits 4 KB above the host registers within the DSI window. */

#define T113_DPHY_BASE                (T113_DSI_BASE + 0x1000)

/* Register offsets - DSI host block (relative to T113_DSI_BASE) */

/* Global control / interrupt */

#define T113_DSI_GCTL_OFFSET          0x0000  /* Global enable */
#define T113_DSI_GINT0_OFFSET         0x0004  /* IRQ enable [15:0] + flag [31:16] */
#define T113_DSI_GINT1_OFFSET         0x0008  /* Video line interrupt number */
#define T113_DSI_BASIC_CTL_OFFSET     0x000c  /* Burst / trail / back-porch control */

/* Basic control */

#define T113_DSI_BASIC_CTL0_OFFSET    0x0010  /* Inst start, src sel, ECC/CRC/EOTP */
#define T113_DSI_BASIC_CTL1_OFFSET    0x0014  /* Video/cmd mode, start delay */
#define T113_DSI_BASIC_SIZE0_OFFSET   0x0018  /* VSA[11:0], VBP[27:16] */
#define T113_DSI_BASIC_SIZE1_OFFSET   0x001c  /* VACT[11:0], VT[28:16] */

/* Instruction-mode function slots inst_func[0..7] at 0x20 + n*4 */

#define T113_DSI_INST_FUNC_OFFSET(n)  (0x0020 + (n) * 4)   /* n = 0..7 */

/* Instruction sequencer control */

#define T113_DSI_INST_LOOP_SEL_OFFSET 0x0040  /* Loop instruction select */
#define T113_DSI_INST_LOOP_NUM_OFFSET 0x0044  /* Loop count N0[11:0], N1[27:16] */
#define T113_DSI_INST_JUMP_SEL_OFFSET 0x0048  /* Jump target for each slot */
#define T113_DSI_INST_JUMP_CFG0_OFFSET 0x004c /* Jump config 0 */
#define T113_DSI_INST_JUMP_CFG1_OFFSET 0x0050 /* Jump config 1 */
#define T113_DSI_INST_LOOP_NUM2_OFFSET 0x0054 /* Secondary loop count */

/* Transmission timing */

#define T113_DSI_TRANS_START_OFFSET   0x0060  /* Transfer start condition */
#define T113_DSI_TRANS_ZERO_OFFSET    0x0078  /* HS-zero reduce set */
#define T113_DSI_TCON_DRQ_OFFSET      0x007c  /* DRQ threshold / mode */

/* Pixel data path */

#define T113_DSI_PIXEL_CTL0_OFFSET    0x0080  /* Pixel format, endian */
#define T113_DSI_PIXEL_CTL1_OFFSET    0x0084  /* (reserved) */
#define T113_DSI_PIXEL_PH_OFFSET      0x0090  /* Pixel packet header: DT/VC/WC/ECC */
#define T113_DSI_PIXEL_PD_OFFSET      0x0094  /* Pixel padding bytes */
#define T113_DSI_PIXEL_PF0_OFFSET     0x0098  /* CRC force value */
#define T113_DSI_PIXEL_PF1_OFFSET     0x009c  /* CRC init (line0 / lineN) */

/* Sync short packets */

#define T113_DSI_SYNC_HSS_OFFSET      0x00b0  /* Horizontal sync start */
#define T113_DSI_SYNC_HSE_OFFSET      0x00b4  /* Horizontal sync end */
#define T113_DSI_SYNC_VSS_OFFSET      0x00b8  /* Vertical sync start */
#define T113_DSI_SYNC_VSE_OFFSET      0x00bc  /* Vertical sync end */

/* Blanking long packets */

#define T113_DSI_BLK_HSA0_OFFSET      0x00c0  /* HSA packet header */
#define T113_DSI_BLK_HSA1_OFFSET      0x00c4  /* HSA payload/CRC */
#define T113_DSI_BLK_HBP0_OFFSET      0x00c8  /* HBP packet header */
#define T113_DSI_BLK_HBP1_OFFSET      0x00cc  /* HBP payload/CRC */
#define T113_DSI_BLK_HFP0_OFFSET      0x00d0  /* HFP packet header */
#define T113_DSI_BLK_HFP1_OFFSET      0x00d4  /* HFP payload/CRC */
#define T113_DSI_BLK_HBLK0_OFFSET     0x00e0  /* HBLK packet header */
#define T113_DSI_BLK_HBLK1_OFFSET     0x00e4  /* HBLK payload/CRC */
#define T113_DSI_BLK_VBLK0_OFFSET     0x00e8  /* VBLK packet header */
#define T113_DSI_BLK_VBLK1_OFFSET     0x00ec  /* VBLK payload/CRC */

/* Burst timing */

#define T113_DSI_BURST_LINE_OFFSET    0x00f0  /* Burst line count / sync point */
#define T113_DSI_BURST_DRQ_OFFSET     0x00f4  /* Burst DRQ edge 0/1 */

/* LP command path (generic packet interface) */

#define T113_DSI_CMD_CTL_OFFSET       0x0200  /* TX/RX size, status, flags */

/* n = 0..7 */

#define T113_DSI_CMD_RX_OFFSET(n)     (0x0240 + (n) * 4)

/* n = 0..63 */

#define T113_DSI_CMD_TX_OFFSET(n)     (0x0300 + (n) * 4)

/* Debug */

#define T113_DSI_DBG_VIDEO0_OFFSET    0x02e0  /* Current video line */
#define T113_DSI_DBG_VIDEO1_OFFSET    0x02e4  /* Current LP-to-HS count */
#define T113_DSI_DBG_INST_OFFSET      0x02f0  /* Instruction state */
#define T113_DSI_DBG_FIFO_OFFSET      0x02f4  /* FIFO fill level */
#define T113_DSI_DBG_DATA_OFFSET      0x02f8  /* BIST / test data */

/* Register offsets - DPHY block (relative to T113_DPHY_BASE) */

/* DPHY global control */

#define T113_DPHY_GCTL_OFFSET         0x0000  /* Module enable, lane count */
#define T113_DPHY_TX_CTL_OFFSET       0x0004  /* TX force / endian / ULPS / exit */
#define T113_DPHY_RX_CTL_OFFSET       0x0008  /* RX force / endian / sync */

/* DPHY timing */

#define T113_DPHY_TX_TIME0_OFFSET     0x0010  /* LPX / dterm / HS-pre / HS-trail */
#define T113_DPHY_TX_TIME1_OFFSET     0x0014  /* CK-prep / CK-zero / CK-pre / CK-post */
#define T113_DPHY_TX_TIME2_OFFSET     0x0018  /* CK-trail / HS-dly */
#define T113_DPHY_TX_TIME3_OFFSET     0x001c  /* ULPS exit */
#define T113_DPHY_TX_TIME4_OFFSET     0x0020  /* HSTX analog timing */
#define T113_DPHY_RX_TIME0_OFFSET     0x0030  /* RX timeout enables / values */
#define T113_DPHY_RX_TIME1_OFFSET     0x0034  /* ULPS wakeup pulse / RX delay */
#define T113_DPHY_RX_TIME2_OFFSET     0x0038  /* HSRX analog timing */
#define T113_DPHY_RX_TIME3_OFFSET     0x0040  /* Frequency counter / LP reset delay */

/* DPHY analog control */

#define T113_DPHY_ANA0_OFFSET         0x004c  /* Bias / impedance / power */
#define T113_DPHY_ANA1_OFFSET         0x0050  /* TX clock / data delays */
#define T113_DPHY_ANA2_OFFSET         0x0054  /* CPU-controlled enables */
#define T113_DPHY_ANA3_OFFSET         0x0058  /* LP TX/RX enables, LDO */
#define T113_DPHY_ANA4_OFFSET         0x005c  /* Impedance / MIPI mode */

/* DPHY interrupts */

#define T113_DPHY_INT_EN0_OFFSET      0x0060  /* SoT / align error enables */
#define T113_DPHY_INT_EN1_OFFSET      0x0064  /* ULPS / escape / false-ctl enables */
#define T113_DPHY_INT_PD0_OFFSET      0x0070  /* SoT / align pending (W1C) */
#define T113_DPHY_INT_PD1_OFFSET      0x0074  /* ULPS / escape pending (W1C) */

/* DPHY debug */

#define T113_DPHY_DBG0_OFFSET         0x00e0  /* LP TX state per lane */
#define T113_DPHY_DBG1_OFFSET         0x00e4  /* LP TX debug enable / force */

/* DPHY PLL / combo-PHY */

#define T113_DPHY_TX_SKEW0_OFFSET     0x00f8  /* Skew cal sync / trail / zero */
#define T113_DPHY_TX_SKEW1_OFFSET     0x00fc  /* Skew cal init / periodic / sync */
#define T113_DPHY_TX_SKEW2_OFFSET     0x0100  /* Skew cal prepare / enable */
#define T113_DPHY_PLL_REG0_OFFSET     0x0104  /* PLL: M1[3:0], M0[5:4], N[15:8], P[19:16], enables */
#define T113_DPHY_PLL_REG1_OFFSET     0x0108  /* PLL: test, ICP, LPF, Vset, lock-det */
#define T113_DPHY_PLL_REG2_OFFSET     0x010c  /* PLL: frac, SS, SDM enable */
#define T113_DPHY_COMBO_PHY0_OFFSET   0x0110  /* Combo PHY: CP/LDO/LVDS/MIPI enables */
#define T113_DPHY_COMBO_PHY1_OFFSET   0x0114  /* Combo PHY: Vref select */
#define T113_DPHY_COMBO_PHY2_OFFSET   0x0118  /* Combo PHY: HS stop delay */

/* -----------------------------------------------------------------------
 * Bit-field macros - DSI host
 * -----------------------------------------------------------------------
 */

/* T113_DSI_GCTL - global enable */

#define T113_DSI_GCTL_EN              (1 << 0)   /* DSI controller enable */

/* T113_DSI_GINT0 - IRQ enable [15:0] + IRQ flag [31:16] */

#define T113_DSI_GINT0_IRQ_EN_SHIFT   0
#define T113_DSI_GINT0_IRQ_EN_MASK    (0xffff << 0)
#define T113_DSI_GINT0_IRQ_FLAG_SHIFT 16
#define T113_DSI_GINT0_IRQ_FLAG_MASK  (0xffff << 16)

/* GINT0 IRQ source bits - same encoding for enable [15:0] and
 * flag [16+].
 */

#define T113_DSI_GINT0_SRC_INSTR_END   (1u << 0)
#define T113_DSI_GINT0_SRC_INSTR_STEP  (1u << 1)
#define T113_DSI_GINT0_SRC_VIDEO_VBLK  (1u << 2)
#define T113_DSI_GINT0_SRC_VIDEO_LINE  (1u << 3)

/* T113_DSI_BASIC_CTL0 - instruction start, source, ECC/CRC */

#define T113_DSI_BCTL0_INST_ST        (1 << 0)   /* Start instruction sequence */
#define T113_DSI_BCTL0_SRC_SEL_SHIFT  4
#define T113_DSI_BCTL0_SRC_SEL_MASK   (0x3 << 4) /* 0=TCON, 1=DMA */
#define T113_DSI_BCTL0_FIFO_RESET     (1 << 10)  /* Manual FIFO reset */
#define T113_DSI_BCTL0_FIFO_GATE      (1 << 12)  /* FIFO gating */
#define T113_DSI_BCTL0_ECC_EN         (1 << 16)  /* ECC enable */
#define T113_DSI_BCTL0_CRC_EN         (1 << 17)  /* CRC enable */
#define T113_DSI_BCTL0_EOTP_EN        (1 << 18)  /* HS EoTP packet enable */

/* T113_DSI_BASIC_CTL1 - video/command mode select */

#define T113_DSI_BCTL1_DSI_MODE       (1 << 0)   /* 0=video, 1=command */
#define T113_DSI_BCTL1_VFR_START      (1 << 1)   /* Video frame start flag */
#define T113_DSI_BCTL1_VPMA           (1 << 2)   /* Video precision mode align */
#define T113_DSI_BCTL1_VSD_SHIFT      4
#define T113_DSI_BCTL1_VSD_MASK       (0x1fff << 4) /* Video start delay */

/* T113_DSI_PIXEL_PH - pixel packet header */

#define T113_DSI_PPH_DT_SHIFT         0
#define T113_DSI_PPH_DT_MASK          (0x3f << 0)  /* Data type (DCS/generic) */
#define T113_DSI_PPH_VC_SHIFT         6
#define T113_DSI_PPH_VC_MASK          (0x3 << 6)   /* Virtual channel 0..3 */
#define T113_DSI_PPH_WC_SHIFT         8
#define T113_DSI_PPH_WC_MASK          (0xffff << 8) /* Word count (bytes) */

/* T113_DSI_CMD_CTL - LP command TX/RX control */

#define T113_DSI_CMDCTL_TX_SIZE_SHIFT  0
#define T113_DSI_CMDCTL_TX_SIZE_MASK   (0xff << 0)  /* TX payload size in bytes */
#define T113_DSI_CMDCTL_TX_STATUS      (1 << 8)     /* TX in progress */
#define T113_DSI_CMDCTL_TX_FLAG        (1 << 9)     /* TX done flag (W1C) */
#define T113_DSI_CMDCTL_RX_SIZE_SHIFT  16
#define T113_DSI_CMDCTL_RX_SIZE_MASK   (0x1f << 16) /* RX payload size */
#define T113_DSI_CMDCTL_RX_STATUS      (1 << 24)    /* RX in progress */
#define T113_DSI_CMDCTL_RX_FLAG        (1 << 25)    /* RX done flag (W1C) */
#define T113_DSI_CMDCTL_RX_OVERFLOW    (1 << 26)    /* RX overflow */

/* -----------------------------------------------------------------------
 * Bit-field macros - DPHY
 * -----------------------------------------------------------------------
 */

/* T113_DPHY_GCTL - module enable, lane count */

#define T113_DPHY_GCTL_MODULE_EN      (1 << 0)     /* DPHY master enable */
#define T113_DPHY_GCTL_LANE_NUM_SHIFT 4
#define T113_DPHY_GCTL_LANE_NUM_MASK  (0x3 << 4)   /* 0=1-lane,1=2-lane,3=4-lane */

/* T113_DPHY_TX_CTL - TX misc */

#define T113_DPHY_TX_FORCE_LP11       (1 << 12)    /* Force all lanes LP-11 */
#define T113_DPHY_TX_CLK_CONT         (1 << 28)    /* HS clock continuous mode */

/* T113_DPHY_TX_TIME0 - LP/HS prepare/trail counts (each 8b).
 * Counts are in DPHY internal cycles; the count->ns mapping is silicon-
 * specific but the values below match D-PHY 1.1 mins for 372 Mbps lane.
 */

#define T113_DPHY_TX_TIME0_LPX_SHIFT      0
#define T113_DPHY_TX_TIME0_LPX_MASK       (0xff << 0)   /* T_LPX */
#define T113_DPHY_TX_TIME0_DTERM_SHIFT    8
#define T113_DPHY_TX_TIME0_DTERM_MASK     (0xff << 8)   /* DDR-term setup */
#define T113_DPHY_TX_TIME0_HS_PRE_SHIFT   16
#define T113_DPHY_TX_TIME0_HS_PRE_MASK    (0xff << 16)  /* T_HS-PREPARE */
#define T113_DPHY_TX_TIME0_HS_TRAIL_SHIFT 24
#define T113_DPHY_TX_TIME0_HS_TRAIL_MASK  (0xff << 24)  /* T_HS-TRAIL */

/* T113_DPHY_TX_TIME1 - clock-lane prepare/zero/pre/post (each 8b). */

#define T113_DPHY_TX_TIME1_CK_PREP_SHIFT  0
#define T113_DPHY_TX_TIME1_CK_PREP_MASK   (0xff << 0)   /* T_CLK-PREPARE */
#define T113_DPHY_TX_TIME1_CK_ZERO_SHIFT  8
#define T113_DPHY_TX_TIME1_CK_ZERO_MASK   (0xff << 8)   /* T_CLK-ZERO */
#define T113_DPHY_TX_TIME1_CK_PRE_SHIFT   16
#define T113_DPHY_TX_TIME1_CK_PRE_MASK    (0xff << 16)  /* T_CLK-PRE */
#define T113_DPHY_TX_TIME1_CK_POST_SHIFT  24
#define T113_DPHY_TX_TIME1_CK_POST_MASK   (0xff << 24)  /* T_CLK-POST */

/* T113_DPHY_TX_TIME2 - clock trail + HS->LP delay. */

#define T113_DPHY_TX_TIME2_CK_TRAIL_SHIFT 0
#define T113_DPHY_TX_TIME2_CK_TRAIL_MASK  (0xff << 0)   /* T_CLK-TRAIL */
#define T113_DPHY_TX_TIME2_HS_DLY_SHIFT   8
#define T113_DPHY_TX_TIME2_HS_DLY_MASK    (0xffff << 8) /* HS->LP delay cycles */
#define T113_DPHY_TX_TIME2_HS_DLY_MODE    (1u << 28)    /* 0=fixed,1=auto */

/* T113_DPHY_TX_TIME3 - LP TX ULPS exit (20-bit count). */

#define T113_DPHY_TX_TIME3_ULPS_EXIT_SHIFT 0
#define T113_DPHY_TX_TIME3_ULPS_EXIT_MASK  (0xfffff << 0)

/* T113_DPHY_TX_TIME4 - HSTX analog setup. */

#define T113_DPHY_TX_TIME4_HSTX_ANA0_SHIFT 0
#define T113_DPHY_TX_TIME4_HSTX_ANA0_MASK  (0xff << 0)
#define T113_DPHY_TX_TIME4_HSTX_ANA1_SHIFT 8
#define T113_DPHY_TX_TIME4_HSTX_ANA1_MASK  (0xff << 8)

/* T113_DPHY_PLL_REG0 - PLL integer multiplier / divider */

#define T113_DPHY_PLL_M1_SHIFT        0
#define T113_DPHY_PLL_M1_MASK         (0xf << 0)   /* Pre-divider M1 */
#define T113_DPHY_PLL_M0_SHIFT        4
#define T113_DPHY_PLL_M0_MASK         (0x3 << 4)   /* Pre-divider M0 */
#define T113_DPHY_PLL_TDIV            (1 << 6)     /* Test divider */
#define T113_DPHY_PLL_NDET            (1 << 7)     /* N detect enable */
#define T113_DPHY_PLL_N_SHIFT         8
#define T113_DPHY_PLL_N_MASK          (0xff << 8)  /* Feedback multiplier N */
#define T113_DPHY_PLL_P_SHIFT         16
#define T113_DPHY_PLL_P_MASK          (0xf << 16)  /* Post-divider P */
#define T113_DPHY_PLL_EN              (1 << 20)    /* PLL enable */
#define T113_DPHY_PLL_EN_LVS          (1 << 21)    /* LVS path enable */
#define T113_DPHY_PLL_LDO_EN          (1 << 22)    /* PLL LDO enable */
#define T113_DPHY_PLL_CP36_EN         (1 << 23)    /* 3.6 V charge pump enable */

/* T113_DPHY_PLL_REG2 - fractional / spread-spectrum / SDM */

#define T113_DPHY_PLL_FRAC_SHIFT      0
#define T113_DPHY_PLL_FRAC_MASK       (0xfff << 0)  /* Fractional divisor */
#define T113_DPHY_PLL_SS_INT_SHIFT    12
#define T113_DPHY_PLL_SS_INT_MASK     (0xff << 12)  /* SS integer step */
#define T113_DPHY_PLL_SS_FRAC_SHIFT   20
#define T113_DPHY_PLL_SS_FRAC_MASK    (0x1ff << 20) /* SS fractional step (9 bits) */
#define T113_DPHY_PLL_SS_EN           (1 << 29)     /* Spread-spectrum enable */
#define T113_DPHY_PLL_FF_EN           (1 << 30)     /* Frequency-feedback enable */
#define T113_DPHY_PLL_SDM_EN          (1 << 31)     /* SDM enable */

/* T113_DPHY_ANA0 - bias / impedance / power */

#define T113_DPHY_ANA0_SELSCK         (1 << 0)     /* Select SCK source */
#define T113_DPHY_ANA0_RSD            (1 << 1)     /* RSD enable */
#define T113_DPHY_ANA0_SFB_SHIFT      2
#define T113_DPHY_ANA0_SFB_MASK       (0x3 << 2)   /* Sample-feedback select */
#define T113_DPHY_ANA0_PLR_SHIFT      4
#define T113_DPHY_ANA0_PLR_MASK       (0xf << 4)   /* Power level reg */
#define T113_DPHY_ANA0_DEN_SHIFT      8
#define T113_DPHY_ANA0_DEN_MASK       (0xf << 8)   /* Data lane enable mask */

/* T113_DPHY_ANA1 - TX clock / data delays + VTT mode */

#define T113_DPHY_ANA1_VTTMODE        (1u << 31)   /* VTT mode select */

/* T113_DPHY_ANA2 - CPU-controlled enables */

#define T113_DPHY_ANA2_ANA_CPU_EN     (1 << 0)     /* Analog CPU control enable */
#define T113_DPHY_ANA2_ENIB           (1 << 1)     /* Bias current enable */
#define T113_DPHY_ANA2_ENCK_CPU       (1 << 4)     /* Clock-lane CPU enable */
#define T113_DPHY_ANA2_ENTX_CPU_SHIFT 8
#define T113_DPHY_ANA2_ENTX_CPU_MASK  (0xf << 8)   /* Per-lane TX CPU enable */
#define T113_DPHY_ANA2_ENCKRX_CPU     (1 << 14)    /* HS RX clock-lane CPU enable */
#define T113_DPHY_ANA2_ENRX_CPU_SHIFT 20
#define T113_DPHY_ANA2_ENRX_CPU_MASK  (0xf << 20)  /* Per-lane RX CPU enable */
#define T113_DPHY_ANA2_ENP2S_CPU_SHIFT 24
#define T113_DPHY_ANA2_ENP2S_CPU_MASK  (0xf << 24) /* Per-lane parallel-to-serial CPU enable */

/* T113_DPHY_ANA3 - LP TX/RX enables, LDO, termination */

#define T113_DPHY_ANA3_ENLPTX_CPU_SHIFT 0
#define T113_DPHY_ANA3_ENLPTX_CPU_MASK  (0xf << 0)  /* Per-lane LP TX enable */
#define T113_DPHY_ANA3_ENLPRX_CPU_SHIFT 4
#define T113_DPHY_ANA3_ENLPRX_CPU_MASK  (0xf << 4)  /* Per-lane LP RX enable */
#define T113_DPHY_ANA3_ENLPRXC_CPU    (1 << 12)     /* Clock-lane LP RX enable */
#define T113_DPHY_ANA3_ENLDOR         (1 << 18)     /* Reference LDO enable */
#define T113_DPHY_ANA3_ENLDOD         (1 << 24)     /* Data LDO enable */
#define T113_DPHY_ANA3_ENLDOC         (1 << 25)     /* Clock LDO enable */
#define T113_DPHY_ANA3_ENDIV          (1 << 26)     /* PLL post-divider enable */
#define T113_DPHY_ANA3_ENVTTC         (1 << 27)     /* Clock-lane termination enable */
#define T113_DPHY_ANA3_ENVTTD_SHIFT   28
#define T113_DPHY_ANA3_ENVTTD_MASK    (0xfu << 28) /* Per-lane data termination enable */

/* T113_DPHY_ANA4 - impedance / drive / MIPI mode */

#define T113_DPHY_ANA4_TXPUSD_SHIFT   0
#define T113_DPHY_ANA4_TXPUSD_MASK    (0x3 << 0)   /* Pull-up data drive */
#define T113_DPHY_ANA4_TXPUSC_SHIFT   2
#define T113_DPHY_ANA4_TXPUSC_MASK    (0x3 << 2)   /* Pull-up clock drive */
#define T113_DPHY_ANA4_TXDNSD_SHIFT   4
#define T113_DPHY_ANA4_TXDNSD_MASK    (0x3 << 4)   /* Pull-down data drive */
#define T113_DPHY_ANA4_TXDNSC_SHIFT   6
#define T113_DPHY_ANA4_TXDNSC_MASK    (0x3 << 6)   /* Pull-down clock drive */
#define T113_DPHY_ANA4_TMSD_SHIFT     8
#define T113_DPHY_ANA4_TMSD_MASK      (0x3 << 8)   /* Termination match data */
#define T113_DPHY_ANA4_TMSC_SHIFT     10
#define T113_DPHY_ANA4_TMSC_MASK      (0x3 << 10)  /* Termination match clock */
#define T113_DPHY_ANA4_CKDV_SHIFT     12
#define T113_DPHY_ANA4_CKDV_MASK      (0x1f << 12) /* Clock-divider trim (5 bits) */
#define T113_DPHY_ANA4_VTT_SET_SHIFT  17
#define T113_DPHY_ANA4_VTT_SET_MASK   (0x7 << 17)  /* Termination voltage set (3 bits) */
#define T113_DPHY_ANA4_DMPLVD_SHIFT   20
#define T113_DPHY_ANA4_DMPLVD_MASK    (0xf << 20)  /* Data lane LV mismatch protection */
#define T113_DPHY_ANA4_DMPLVC         (1 << 24)    /* Clock lane LV mismatch protection */
#define T113_DPHY_ANA4_IB_SHIFT       25
#define T113_DPHY_ANA4_IB_MASK        (0x3 << 25)  /* Bias current trim */
#define T113_DPHY_ANA4_EN_MIPI        (1u << 31)   /* MIPI mode enable */

/* T113_DPHY_COMBO_PHY0 - combo PHY power */

#define T113_DPHY_COMBO_EN_CP         (1 << 0)     /* Charge pump enable */
#define T113_DPHY_COMBO_EN_LDO        (1 << 1)     /* Combo LDO enable */
#define T113_DPHY_COMBO_EN_MIPI       (1 << 3)     /* MIPI mode select */

/* T113_DPHY_COMBO_PHY2 - HS stop delay */

#define T113_DPHY_COMBO_HS_STOP_DLY_SHIFT 0
#define T113_DPHY_COMBO_HS_STOP_DLY_MASK  (0xff << 0)  /* HS to LP stop delay */

/* T113_DSI_INST_FUNC(n) - instruction-slot configuration */

#define T113_DSI_INST_LANE_DEN_SHIFT     0
#define T113_DSI_INST_LANE_DEN_MASK      (0xf << 0)
#define T113_DSI_INST_LANE_CEN           (1 << 4)
#define T113_DSI_INST_TRANS_START_SHIFT  16
#define T113_DSI_INST_TRANS_START_MASK   (0xf << 16)
#define T113_DSI_INST_TRANS_PACK_SHIFT   20
#define T113_DSI_INST_TRANS_PACK_MASK    (0xf << 20)
#define T113_DSI_INST_ESCAPE_ENTRY_SHIFT 24
#define T113_DSI_INST_ESCAPE_ENTRY_MASK  (0xf << 24)
#define T113_DSI_INST_MODE_SHIFT         28
#define T113_DSI_INST_MODE_MASK          (0xfu << 28)

/* Instruction-slot IDs (used as nibble shifts in JUMP_SEL/LOOP_SEL and as
 * the n in INST_FUNC[n]).
 */

#define T113_DSI_ID_LP11                 0
#define T113_DSI_ID_TBA                  1
#define T113_DSI_ID_HSC                  2
#define T113_DSI_ID_HSD                  3
#define T113_DSI_ID_LPDT                 4
#define T113_DSI_ID_HSCEXIT              5
#define T113_DSI_ID_NOP                  6
#define T113_DSI_ID_DLY                  7
#define T113_DSI_ID_END                  15

/* Instruction-mode encodings (INSTRU_MODE field) */

#define T113_DSI_MODE_STOP               0
#define T113_DSI_MODE_TBA                1
#define T113_DSI_MODE_HS                 2
#define T113_DSI_MODE_ESCAPE             3
#define T113_DSI_MODE_HSCEXIT            4
#define T113_DSI_MODE_NOP                5

/* Trans-packet encodings (TRANS_PACK field) */

#define T113_DSI_PACK_PIXEL              0
#define T113_DSI_PACK_COMMAND            1

/* Escape-entry encodings (ESCAPE_ENTRY field, only for LPDT slot) */

#define T113_DSI_ESCA_LPDT               0
#define T113_DSI_ESCA_ULPS               1
#define T113_DSI_ESCA_RESET              4

/* INST_LOOP_NUM bit layout */

#define T113_DSI_LOOP_N0_SHIFT           0
#define T113_DSI_LOOP_N0_MASK            (0xfff << 0)
#define T113_DSI_LOOP_N1_SHIFT           16
#define T113_DSI_LOOP_N1_MASK            (0xfff << 16)

/* INST_JUMP_CFG0/1 - when LOOP_NUMx[slot] expires, sequencer jumps to
 * JUMP_SEL[slot] for NUM iterations (or always if EN=0), then exits via
 * POINT -> TO.
 */

#define T113_DSI_JUMP_CFG_NUM_SHIFT      0
#define T113_DSI_JUMP_CFG_NUM_MASK       (0xffff << 0)
#define T113_DSI_JUMP_CFG_POINT_SHIFT    16
#define T113_DSI_JUMP_CFG_POINT_MASK     (0xf << 16)
#define T113_DSI_JUMP_CFG_TO_SHIFT       20
#define T113_DSI_JUMP_CFG_TO_MASK        (0xf << 20)
#define T113_DSI_JUMP_CFG_EN             (1 << 28)

/* T113_DSI_BASIC_SIZE0 - vertical sync + back porch (in lines) */

#define T113_DSI_BASIC_SIZE0_VSA_SHIFT   0
#define T113_DSI_BASIC_SIZE0_VSA_MASK    (0xfff << 0)
#define T113_DSI_BASIC_SIZE0_VBP_SHIFT   16
#define T113_DSI_BASIC_SIZE0_VBP_MASK    (0xfff << 16)

/* T113_DSI_BASIC_SIZE1 - vertical active + total (in lines,
 * VT is 13 bits)
 */

#define T113_DSI_BASIC_SIZE1_VACT_SHIFT  0
#define T113_DSI_BASIC_SIZE1_VACT_MASK   (0xfff << 0)
#define T113_DSI_BASIC_SIZE1_VT_SHIFT    16
#define T113_DSI_BASIC_SIZE1_VT_MASK     (0x1fff << 16)

/* T113_DSI_TRANS_START - when in video mode, start the next packet at this
 * count of pixel-clock cycles into the line.
 */

#define T113_DSI_TRANS_START_SET_SHIFT   0
#define T113_DSI_TRANS_START_SET_MASK    (0x1fff << 0)

/* T113_DSI_TRANS_ZERO - HS-zero shrink count. */

#define T113_DSI_TRANS_ZERO_HS_SHIFT     0
#define T113_DSI_TRANS_ZERO_HS_MASK      (0xffff << 0)

/* T113_DSI_TCON_DRQ - DRQ threshold + mode (auto vs. fixed-set) */

#define T113_DSI_TCON_DRQ_SET_SHIFT      0
#define T113_DSI_TCON_DRQ_SET_MASK       (0x3ff << 0)
#define T113_DSI_TCON_DRQ_MODE           (1 << 28)

/* T113_DSI_PIXEL_CTL0 - pixel format / endian */

#define T113_DSI_PIXEL_CTL0_FMT_SHIFT    0
#define T113_DSI_PIXEL_CTL0_FMT_MASK     (0xf << 0)
#define T113_DSI_PIXEL_CTL0_ENDIAN       (1 << 4)
#define T113_DSI_PIXEL_CTL0_PD_PLUG_DIS  (1 << 16)

/* DSI sync-packet header layout (SYNC_HSS/HSE/VSS/VSE).  DT/VC reuse the
 * PPH macros above.  D0/D1 are the two short-packet data bytes (0 in pure
 * sync events) and ECC sits in the high byte.
 */

#define T113_DSI_SHORT_D0_SHIFT          8
#define T113_DSI_SHORT_D0_MASK           (0xff << 8)
#define T113_DSI_SHORT_D1_SHIFT          16
#define T113_DSI_SHORT_D1_MASK           (0xff << 16)
#define T113_DSI_SHORT_ECC_SHIFT         24
#define T113_DSI_SHORT_ECC_MASK          (0xffu << 24)

/* DSI blanking long-packet header (BLK_*0).  DT/VC/WC reuse PPH macros;
 * ECC lives at [31:24].  BLK_*1 carries the padding byte and the CRC of
 * a payload of WC zeros.
 */

#define T113_DSI_BLK_ECC_SHIFT           24
#define T113_DSI_BLK_ECC_MASK            (0xffu << 24)
#define T113_DSI_BLK_PD_SHIFT            0
#define T113_DSI_BLK_PD_MASK             (0xff << 0)
#define T113_DSI_BLK_PF_SHIFT            16
#define T113_DSI_BLK_PF_MASK             (0xffffu << 16)

/* MIPI DSI Data Type encodings used in video sync / blanking / pixel
 * packet headers.  Per DSI spec section 8.7 / section 13.
 */

#define T113_DSI_DT_VSS                  0x01    /* V-sync start (short)    */
#define T113_DSI_DT_VSE                  0x11    /* V-sync end (short)      */
#define T113_DSI_DT_HSS                  0x21    /* H-sync start (short)    */
#define T113_DSI_DT_HSE                  0x31    /* H-sync end (short)      */
#define T113_DSI_DT_BLK                  0x19    /* Blanking long packet    */
#define T113_DSI_DT_PIXEL_RGB888         0x3e    /* Packed pixel RGB888     */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline uint32_t t113_dsi_getreg(uint32_t off)
{
  return getreg32(T113_DSI_BASE + off);
}

static inline void t113_dsi_putreg(uint32_t off, uint32_t val)
{
  putreg32(val, T113_DSI_BASE + off);
}

static inline uint32_t t113_dphy_getreg(uint32_t off)
{
  return getreg32(T113_DPHY_BASE + off);
}

static inline void t113_dphy_putreg(uint32_t off, uint32_t val)
{
  putreg32(val, T113_DPHY_BASE + off);
}

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_DSI_H */
