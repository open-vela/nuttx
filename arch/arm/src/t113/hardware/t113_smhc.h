/****************************************************************************
 * arch/arm/src/t113/hardware/t113_smhc.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_SMHC_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_SMHC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "t113_ccu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SMHC controller base addresses */

#define T113_SMHC0_BASE              0x04020000
#define T113_SMHC1_BASE              0x04021000
#define T113_SMHC2_BASE              0x04022000

/* SMHC register offsets */

#define T113_SMHC_CTRL_OFFSET        0x0000  /* global control     */
#define T113_SMHC_CLKDIV_OFFSET      0x0004  /* clock control      */
#define T113_SMHC_TMOUT_OFFSET       0x0008  /* timeout            */
#define T113_SMHC_CTYPE_OFFSET       0x000c  /* bus width          */
#define T113_SMHC_BLKSIZ_OFFSET      0x0010  /* block size         */
#define T113_SMHC_BYTCNT_OFFSET      0x0014  /* byte count         */
#define T113_SMHC_CMD_OFFSET         0x0018  /* command            */
#define T113_SMHC_CMDARG_OFFSET      0x001c  /* command argument   */
#define T113_SMHC_RESP0_OFFSET       0x0020
#define T113_SMHC_RESP1_OFFSET       0x0024
#define T113_SMHC_RESP2_OFFSET       0x0028
#define T113_SMHC_RESP3_OFFSET       0x002c
#define T113_SMHC_INTMASK_OFFSET     0x0030  /* interrupt mask     */
#define T113_SMHC_MINTSTS_OFFSET     0x0034  /* masked int status  */
#define T113_SMHC_RINTSTS_OFFSET     0x0038  /* raw int status     */
#define T113_SMHC_STATUS_OFFSET      0x003c
#define T113_SMHC_FIFOTH_OFFSET      0x0040  /* fifo water level   */
#define T113_SMHC_FUNS_OFFSET        0x0044
#define T113_SMHC_TCBCNT_OFFSET      0x0048  /* card<->fifo bytes  */
#define T113_SMHC_TBBCNT_OFFSET      0x004c  /* host<->fifo bytes  */
#define T113_SMHC_DBGC_OFFSET        0x0050
#define T113_SMHC_CSDC_OFFSET        0x0054
#define T113_SMHC_A12A_OFFSET        0x0058
#define T113_SMHC_NTSR_OFFSET        0x005c
#define T113_SMHC_HWRST_OFFSET       0x0078
#define T113_SMHC_IDMAC_OFFSET       0x0080  /* idmac control      */
#define T113_SMHC_DLBA_OFFSET        0x0084  /* desc list base     */
#define T113_SMHC_IDST_OFFSET        0x0088  /* idmac status       */
#define T113_SMHC_IDIE_OFFSET        0x008c  /* idmac int enable   */
#define T113_SMHC_THLD_OFFSET        0x0100
#define T113_SMHC_SFC_OFFSET         0x0104
#define T113_SMHC_A23A_OFFSET        0x0108
#define T113_SMHC_EMMC_DDR_SBIT_DET_OFFSET 0x010c
#define T113_SMHC_EXT_CMD_OFFSET     0x0138
#define T113_SMHC_EXT_RESP_OFFSET    0x013c
#define T113_SMHC_DRV_DL_OFFSET      0x0140  /* drive delay control  */
#define T113_SMHC_SAMP_DL_OFFSET     0x0144  /* sample delay control */
#define T113_SMHC_DS_DL_OFFSET       0x0148
#define T113_SMHC_HS400_DL_OFFSET    0x014c
#define T113_SMHC_FIFO_OFFSET        0x0200  /* data fifo port     */

/* SMHC_DRV_DL bits.  Bits 16-17 set the HSSDR output phase
 * (0=90 deg, 1=180 deg).  CMD line is bit 17, DAT line is bit 16.
 */

#define T113_SMHC_DRV_DL_DAT_PH_SH   16
#define T113_SMHC_DRV_DL_CMD_PH_SH   17
#define T113_SMHC_DRV_DL_PHASE_MASK  (0x3u << 16)
#define T113_SMHC_DRV_DL_PHASE_180   ((1u << 16) | (1u << 17))

/* SMHC_SAMP_DL bits */

#define T113_SMHC_SAMP_DL_CFG_MASK   (0x3fu << 0)
#define T113_SMHC_SAMP_DL_ENABLE     (1u << 7)

/* SMHC_CTRL (GCTL) @ 0x0000 bits */

#define T113_SMHC_CTRL_SOFT_RST      (1u << 0)
#define T113_SMHC_CTRL_FIFO_RST      (1u << 1)
#define T113_SMHC_CTRL_DMA_RST       (1u << 2)
#define T113_SMHC_CTRL_INT_ENB       (1u << 4)
#define T113_SMHC_CTRL_DMA_ENB       (1u << 5)
#define T113_SMHC_CTRL_CD_DBC_ENB    (1u << 8)
#define T113_SMHC_CTRL_DDR_MOD_SEL   (1u << 10)
#define T113_SMHC_CTRL_TIME_UNIT_DAT (1u << 11)
#define T113_SMHC_CTRL_TIME_UNIT_CMD (1u << 12)
#define T113_SMHC_CTRL_FIFO_AC_MOD   (1u << 31)

/* SMHC_CLKDIV (CLKCTRL) @ 0x0004 bits */

#define T113_SMHC_CLKDIV_DIV_MASK    (0xffu << 0)
#define T113_SMHC_CLKDIV_DIV(n)      (((n) & 0xffu) << 0)
#define T113_SMHC_CLKDIV_CCLK_ENB    (1u << 16)
#define T113_SMHC_CLKDIV_CCLK_CTRL   (1u << 17)
#define T113_SMHC_CLKDIV_MASK_DATA0  (1u << 31)

/* SMHC_CTYPE @ 0x000c bits */

#define T113_SMHC_CTYPE_1BIT         (0u << 0)
#define T113_SMHC_CTYPE_4BIT         (1u << 0)
#define T113_SMHC_CTYPE_8BIT         (2u << 0)
#define T113_SMHC_CTYPE_MASK         (0x3u << 0)

/* SMHC_BLKSIZ @ 0x0010 bits */

#define T113_SMHC_BLKSIZ_MASK        (0xffffu << 0)

/* SMHC_CMD @ 0x0018 bits */

#define T113_SMHC_CMD_IDX_MASK       (0x3fu << 0)
#define T113_SMHC_CMD_IDX(n)         (((n) & 0x3fu) << 0)
#define T113_SMHC_CMD_RESP_RCV       (1u << 6)
#define T113_SMHC_CMD_LONG_RESP      (1u << 7)
#define T113_SMHC_CMD_CHK_RESP_CRC   (1u << 8)
#define T113_SMHC_CMD_DATA_TRANS     (1u << 9)
#define T113_SMHC_CMD_TRANS_WRITE    (1u << 10)
#define T113_SMHC_CMD_TRANS_STREAM   (1u << 11)

/* Bit 12: STOP_CMD_FLAG -- "Send stop CMD12 automatically at the end of
 * the data transfer".  NuttX always issues CMD12 explicitly via
 * mmcsd_stoptransmission(), so we leave AUTO_STOP off and program
 * SMHC_A12A=0xffff in sendcmd to disable the auto-CMD12 path entirely.
 */

#define T113_SMHC_CMD_AUTO_STOP      (1u << 12)
#define T113_SMHC_CMD_WAIT_PRE_OVER  (1u << 13)

/* Bit 14: STOP_ABT_CMD -- "Send Stop or Abort command to stop the current
 * data transfer in progress (CMD12, CMD52 I/O Abort)".  This is what the
 * NuttX MMCSD_STOPXFR flag maps to.
 */

#define T113_SMHC_CMD_STOP_ABT_CMD   (1u << 14)
#define T113_SMHC_CMD_SEND_INIT_SEQ  (1u << 15)

/* Bit 21: PRG_CLK -- update controller clock without actually issuing
 * a card command.  Used by t113_smhc_clock() to apply a new SDCLK
 * divider before any subsequent data command.
 */

#define T113_SMHC_CMD_PRG_CLK        (1u << 21)
#define T113_SMHC_CMD_BOOT_MOD_SHIFT 24
#define T113_SMHC_CMD_BOOT_MOD_MASK  (0x3u << 24)
#define T113_SMHC_CMD_EXP_BOOT_ACK   (1u << 26)
#define T113_SMHC_CMD_BOOT_ABT       (1u << 27)
#define T113_SMHC_CMD_VOL_SW         (1u << 28)
#define T113_SMHC_CMD_CMD_LOAD       (1u << 31)

/* SMHC_INTMASK / SMHC_RINTSTS / SMHC_MINTSTS bits (shared layout) */

#define T113_SMHC_INT_RE             (1u << 1)   /* response error      */
#define T113_SMHC_INT_CC             (1u << 2)   /* command complete    */
#define T113_SMHC_INT_DTC            (1u << 3)   /* data transfer done  */
#define T113_SMHC_INT_DTR            (1u << 4)   /* data tx request     */
#define T113_SMHC_INT_DRR            (1u << 5)   /* data rx request     */
#define T113_SMHC_INT_RCE            (1u << 6)   /* response crc err    */
#define T113_SMHC_INT_DCE            (1u << 7)   /* data crc error      */
#define T113_SMHC_INT_RTO            (1u << 8)   /* response timeout    */
#define T113_SMHC_INT_DRTO           (1u << 9)   /* data timeout        */
#define T113_SMHC_INT_DSTO           (1u << 10)  /* data starvation to  */
#define T113_SMHC_INT_FU_FO          (1u << 11)  /* fifo under/over     */
#define T113_SMHC_INT_CB_IW          (1u << 12)  /* cmd busy/illegal wr */
#define T113_SMHC_INT_DSE_BC         (1u << 13)  /* start err/busy clr  */
#define T113_SMHC_INT_ACD            (1u << 14)  /* auto cmd12 done     */
#define T113_SMHC_INT_DEE            (1u << 15)  /* data end-bit error  */
#define T113_SMHC_INT_SDIO           (1u << 16)  /* sdio interrupt      */
#define T113_SMHC_INT_CARD_INSERT    (1u << 30)
#define T113_SMHC_INT_CARD_REMOVAL   (1u << 31)

#define T113_SMHC_INT_ERR_MASK       (T113_SMHC_INT_RE   | T113_SMHC_INT_RCE | \
                                      T113_SMHC_INT_DCE  | T113_SMHC_INT_RTO | \
                                      T113_SMHC_INT_DRTO | T113_SMHC_INT_FU_FO | \
                                      T113_SMHC_INT_DEE)

/* SMHC_STATUS @ 0x003c bits */

#define T113_SMHC_STATUS_FIFO_RX_LVL (1u << 0)
#define T113_SMHC_STATUS_FIFO_TX_LVL (1u << 1)
#define T113_SMHC_STATUS_FIFO_EMPTY  (1u << 2)
#define T113_SMHC_STATUS_FIFO_FULL   (1u << 3)
#define T113_SMHC_STATUS_FSM_STA_SH  4
#define T113_SMHC_STATUS_FSM_STA_MASK (0xfu << 4)
#define T113_SMHC_STATUS_CARD_PRESENT (1u << 8)
#define T113_SMHC_STATUS_CARD_BUSY   (1u << 9)
#define T113_SMHC_STATUS_FSM_BUSY    (1u << 10)
#define T113_SMHC_STATUS_RESP_IDX_SH 11
#define T113_SMHC_STATUS_FIFO_LVL_SH 17
#define T113_SMHC_STATUS_FIFO_LVL_MASK (0x1ffu << 17)
#define T113_SMHC_STATUS_DMA_REQ     (1u << 31)

/* SMHC_FIFOTH @ 0x0040 bits */

#define T113_SMHC_FIFOTH_TX_TL_SH    0
#define T113_SMHC_FIFOTH_TX_TL(n)    (((n) & 0xffu) << 0)
#define T113_SMHC_FIFOTH_RX_TL_SH    16
#define T113_SMHC_FIFOTH_RX_TL(n)    (((n) & 0xffu) << 16)
#define T113_SMHC_FIFOTH_BSIZE_SH    28
#define T113_SMHC_FIFOTH_BSIZE_MASK  (0x7u << 28)
#define T113_SMHC_FIFOTH_BSIZE_1     (0u << 28)
#define T113_SMHC_FIFOTH_BSIZE_4     (1u << 28)
#define T113_SMHC_FIFOTH_BSIZE_8     (2u << 28)
#define T113_SMHC_FIFOTH_BSIZE_16    (3u << 28)

/* SMHC_HWRST @ 0x0078 bits */

#define T113_SMHC_HWRST_ACTIVE       (1u << 0)

/* SMHC_IDMAC (DMAC) @ 0x0080 bits */

#define T113_SMHC_IDMAC_SOFT_RST     (1u << 0)
#define T113_SMHC_IDMAC_FIX_BUST_CTL (1u << 1)
#define T113_SMHC_IDMAC_ENB          (1u << 7)
#define T113_SMHC_IDMAC_DES_LOAD_CTL (1u << 31)

/* SMHC_IDST @ 0x0088 bits (W1C where applicable) */

#define T113_SMHC_IDST_TX_INT        (1u << 0)
#define T113_SMHC_IDST_RX_INT        (1u << 1)
#define T113_SMHC_IDST_FATAL_BERR    (1u << 2)
#define T113_SMHC_IDST_DES_UNAVAIL   (1u << 4)
#define T113_SMHC_IDST_ERR_SUM       (1u << 5)
#define T113_SMHC_IDST_NIS           (1u << 8)
#define T113_SMHC_IDST_AIS           (1u << 9)
#define T113_SMHC_IDST_ERR_STA_SH    10
#define T113_SMHC_IDST_ERR_STA_MASK  (0x7u << 10)

/* SMHC_IDIE @ 0x008C bits */

#define T113_SMHC_IDIE_TX_INT_ENB    (1u << 0)
#define T113_SMHC_IDIE_RX_INT_ENB    (1u << 1)
#define T113_SMHC_IDIE_FERR_INT_ENB  (1u << 2)
#define T113_SMHC_IDIE_DES_UNAVL_ENB (1u << 4)
#define T113_SMHC_IDIE_ERR_SUM_ENB   (1u << 5)

/* SMHC_THLD @ 0x0100 bits */

#define T113_SMHC_THLD_RD_ENB        (1u << 0)
#define T113_SMHC_THLD_BCIG          (1u << 1)
#define T113_SMHC_THLD_WR_ENB        (1u << 2)
#define T113_SMHC_THLD_SZ_SH         16

/* SMHC_EXT_CMD @ 0x0138 bits */

#define T113_SMHC_EXT_CMD_AUTO23_EN  (1u << 0)

/* EMMC_DDR_SBIT_DET @ 0x010c bits */

#define T113_SMHC_EMMC_HALF_STARTBIT (1u << 0)
#define T113_SMHC_EMMC_HS400_MD_EN   (1u << 31)

/* SMHC_SAMP_DL @ 0x0144 bits */

#define T113_SMHC_SAMP_DL_SW_MASK    (0x3fu << 0)
#define T113_SMHC_SAMP_DL_SW_EN      (1u << 7)
#define T113_SMHC_SAMP_DL_CAL_DONE   (1u << 14)
#define T113_SMHC_SAMP_DL_CAL_START  (1u << 15)

/* IDMAC descriptor DES0 (ctrl) bits */

#define T113_SMHC_DES0_DIC           (1u << 1)   /* disable int on done */
#define T113_SMHC_DES0_LAST          (1u << 2)
#define T113_SMHC_DES0_FIRST         (1u << 3)
#define T113_SMHC_DES0_CHAIN         (1u << 4)   /* must be 1           */
#define T113_SMHC_DES0_ER            (1u << 5)   /* end of ring         */
#define T113_SMHC_DES0_ERR           (1u << 30)
#define T113_SMHC_DES0_OWN           (1u << 31)

/* IDMAC descriptor DES1 (buf_len).
 *
 * DES1_SIZE_MAX is the largest 4-byte-aligned byte count that fits in
 * the 13-bit DES1.size field.  0x1ffc is the conservative choice that
 * matches the mask exactly and keeps every chunk a multiple of the
 * 32-bit IDMAC bus word (avoids the bit-13 boundary "0 means 8K"
 * semantics that some chips use).
 */

#define T113_SMHC_DES1_SIZE_MASK     (0x1fffu << 0)
#define T113_SMHC_DES1_SIZE_MAX      0x1ffc

/* FIFO depth (recommended programming guidance) */

#define T113_SMHC_FIFO_DEPTH         256

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_SMHC_H */
