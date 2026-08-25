/****************************************************************************
 * arch/risc-v/src/esp32p4/espressif/esp32p4_sdmmc.h
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

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESPRESSIF_ESP32P4_SDMMC_H
#define __ARCH_RISCV_SRC_ESP32P4_ESPRESSIF_ESP32P4_SDMMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sdio.h>

#include <soc/reg_base.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The ESP32-P4 embeds the same Synopsys DesignWare MobileStorage (SDMMC)
 * host IP as the ESP32-S3, so the CIU/BIU register block below has the same
 * offsets and bit layout as the S3 driver.  The differences that this port
 * resolves live outside this block:
 *   1. The peripheral is based at DR_REG_SDMMC_BASE (P4 map, hw_ver3).
 *   2. Bus clock enable, module reset, LS clock source select and the host
 *      clock divider are in the HP_SYS_CLKRST / LP_CLKRST blocks (below),
 *      NOT in the SDMMC 0x800 register as on the S3.
 *
 * NOTE: DR_REG_SDMMC_BASE resolves through <soc/reg_base.h>, which the build
 * selects as hw_ver3 by default (hw_ver1 only when
 * CONFIG_ESP32P4_SELECTS_REV_LESS_V3).  Both revisions place the SDMMC block
 * at the same base, so this driver does not need to parameterize it; if a
 * future hw_ver1-only bring-up needs a different base, override it here.
 */

#define ESP32P4_TXFIFO_DEPTH               32
#define ESP32P4_TXFIFO_WIDTH               4
#define ESP32P4_RXFIFO_DEPTH               32
#define ESP32P4_RXFIFO_WIDTH               4

/* MCI register offsets (with respect to the SDMMC base) ********************/

#define ESP32P4_SDMMC_CTRL_OFFSET          0x0000 /* Control register */
#define ESP32P4_SDMMC_CLKDIV_OFFSET        0x0008 /* Clock-divider */
#define ESP32P4_SDMMC_CLKSRC_OFFSET        0x000c /* Clock-source */
#define ESP32P4_SDMMC_CLKENA_OFFSET        0x0010 /* Clock-enable */
#define ESP32P4_SDMMC_TMOUT_OFFSET         0x0014 /* Time-out */
#define ESP32P4_SDMMC_CTYPE_OFFSET         0x0018 /* Card-type */
#define ESP32P4_SDMMC_BLKSIZ_OFFSET        0x001c /* Block-size */
#define ESP32P4_SDMMC_BYTCNT_OFFSET        0x0020 /* Byte-count */
#define ESP32P4_SDMMC_INTMASK_OFFSET       0x0024 /* Interrupt-mask */
#define ESP32P4_SDMMC_CMDARG_OFFSET        0x0028 /* Command-argument */
#define ESP32P4_SDMMC_CMD_OFFSET           0x002c /* Command */
#define ESP32P4_SDMMC_RESP0_OFFSET         0x0030 /* Response-0 */
#define ESP32P4_SDMMC_RESP1_OFFSET         0x0034 /* Response-1 */
#define ESP32P4_SDMMC_RESP2_OFFSET         0x0038 /* Response-2 */
#define ESP32P4_SDMMC_RESP3_OFFSET         0x003c /* Response-3 */
#define ESP32P4_SDMMC_MINTSTS_OFFSET       0x0040 /* Masked int status */
#define ESP32P4_SDMMC_RINTSTS_OFFSET       0x0044 /* Raw int status */
#define ESP32P4_SDMMC_STATUS_OFFSET        0x0048 /* Status */
#define ESP32P4_SDMMC_FIFOTH_OFFSET        0x004c /* FIFO threshold */
#define ESP32P4_SDMMC_CDETECT_OFFSET       0x0050 /* Card-detect */
#define ESP32P4_SDMMC_WRTPRT_OFFSET        0x0054 /* Write-protect */
#define ESP32P4_SDMMC_TCBCNT_OFFSET        0x005c /* CIU byte count */
#define ESP32P4_SDMMC_TBBCNT_OFFSET        0x0060 /* BIU-FIFO byte count */
#define ESP32P4_SDMMC_DEBNCE_OFFSET        0x0064 /* Debounce count */
#define ESP32P4_SDMMC_RSTN_OFFSET          0x0078 /* Hardware reset */
#define ESP32P4_SDMMC_BMOD_OFFSET          0x0080 /* Bus mode */
#define ESP32P4_SDMMC_PLDMND_OFFSET        0x0084 /* Poll demand */
#define ESP32P4_SDMMC_DBADDR_OFFSET        0x0088 /* Descriptor base addr */
#define ESP32P4_SDMMC_IDSTS_OFFSET         0x008c /* Internal DMAC status */
#define ESP32P4_SDMMC_IDINTEN_OFFSET       0x0090 /* Internal DMAC int en */
#define ESP32P4_SDMMC_DSCADDR_OFFSET       0x0094 /* Current host desc addr */
#define ESP32P4_SDMMC_BUFADDR_OFFSET       0x0098 /* Current buf desc addr */
#define ESP32P4_SDMMC_CARDTHR_OFFSET       0x0100 /* Card read threshold */
#define ESP32P4_SDMMC_DATA_OFFSET          0x0200 /* Data FIFO read/write */
#define ESP32P4_SDMMC_CLKEDGE_OFFSET       0x0800 /* Clock edge/phase sel */

/* MCI register addresses ***************************************************/

#define ESP32P4_SDMMC_CTRL     (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_CTRL_OFFSET)
#define ESP32P4_SDMMC_CLKDIV   (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_CLKDIV_OFFSET)
#define ESP32P4_SDMMC_CLKSRC   (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_CLKSRC_OFFSET)
#define ESP32P4_SDMMC_CLKENA   (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_CLKENA_OFFSET)
#define ESP32P4_SDMMC_TMOUT    (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_TMOUT_OFFSET)
#define ESP32P4_SDMMC_CTYPE    (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_CTYPE_OFFSET)
#define ESP32P4_SDMMC_BLKSIZ   (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_BLKSIZ_OFFSET)
#define ESP32P4_SDMMC_BYTCNT   (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_BYTCNT_OFFSET)
#define ESP32P4_SDMMC_INTMASK  (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_INTMASK_OFFSET)
#define ESP32P4_SDMMC_CMDARG   (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_CMDARG_OFFSET)
#define ESP32P4_SDMMC_CMD      (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_CMD_OFFSET)
#define ESP32P4_SDMMC_RESP0    (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_RESP0_OFFSET)
#define ESP32P4_SDMMC_RESP1    (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_RESP1_OFFSET)
#define ESP32P4_SDMMC_RESP2    (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_RESP2_OFFSET)
#define ESP32P4_SDMMC_RESP3    (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_RESP3_OFFSET)
#define ESP32P4_SDMMC_MINTSTS  (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_MINTSTS_OFFSET)
#define ESP32P4_SDMMC_RINTSTS  (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_RINTSTS_OFFSET)
#define ESP32P4_SDMMC_STATUS   (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_STATUS_OFFSET)
#define ESP32P4_SDMMC_FIFOTH   (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_FIFOTH_OFFSET)
#define ESP32P4_SDMMC_CDETECT  (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_CDETECT_OFFSET)
#define ESP32P4_SDMMC_WRTPRT   (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_WRTPRT_OFFSET)
#define ESP32P4_SDMMC_TCBCNT   (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_TCBCNT_OFFSET)
#define ESP32P4_SDMMC_TBBCNT   (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_TBBCNT_OFFSET)
#define ESP32P4_SDMMC_DEBNCE   (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_DEBNCE_OFFSET)
#define ESP32P4_SDMMC_RSTN     (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_RSTN_OFFSET)
#define ESP32P4_SDMMC_BMOD     (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_BMOD_OFFSET)
#define ESP32P4_SDMMC_PLDMND   (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_PLDMND_OFFSET)
#define ESP32P4_SDMMC_DBADDR   (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_DBADDR_OFFSET)
#define ESP32P4_SDMMC_IDSTS    (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_IDSTS_OFFSET)
#define ESP32P4_SDMMC_IDINTEN  (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_IDINTEN_OFFSET)
#define ESP32P4_SDMMC_DSCADDR  (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_DSCADDR_OFFSET)
#define ESP32P4_SDMMC_BUFADDR  (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_BUFADDR_OFFSET)
#define ESP32P4_SDMMC_CARDTHR  (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_CARDTHR_OFFSET)
#define ESP32P4_SDMMC_DATA     (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_DATA_OFFSET)
#define ESP32P4_SDMMC_CLKEDGE  (DR_REG_SDMMC_BASE + ESP32P4_SDMMC_CLKEDGE_OFFSET)

/* Control register CTRL ****************************************************/

#define SDMMC_CTRL_CNTLRRESET   (1 << 0)  /* Reset module controller */
#define SDMMC_CTRL_FIFORESET    (1 << 1)  /* Reset data FIFO */
#define SDMMC_CTRL_DMARESET     (1 << 2)  /* Reset internal DMA */
#define SDMMC_CTRL_INTENABLE    (1 << 4)  /* Enable interrupts */
#define SDMMC_CTRL_DMAENABLE    (1 << 5)  /* Enable DMA transfer mode */
#define SDMMC_CTRL_READWAIT     (1 << 6)  /* Assert read wait */
#define SDMMC_CTRL_SENDIRQRESP  (1 << 7)  /* Send auto IRQ response */
#define SDMMC_CTRL_ABORTREAD    (1 << 8)  /* Reset data state-machine */
#define SDMMC_CTRL_INTDMA       (1 << 25) /* SD/MMC internal DMA use */

/* Clock divider register CLKDIV ********************************************/

#define SDMMC_CLKDIV_SHIFT(n)   ((n) * 8)
#define SDMMC_CLKDIV_MASK(n)    (255 << SDMMC_CLKDIV_SHIFT(n))
#define SDMMC_CLKDIV(n, div)    ((div) << SDMMC_CLKDIV_SHIFT(n))

/* Clock source register CLKSRC *********************************************/

#define SDMMC_CLKSRC_SHIFT(slot)       ((slot) * 2)
#define SDMMC_CLKSRC_MASK(slot)        (3 << SDMMC_CLKSRC_SHIFT(slot))
#define SDMMC_CLKSRC_CLKDIV(slot, div) ((div) << SDMMC_CLKSRC_SHIFT(slot))

/* Clock enable register CLKENA *********************************************/

#define SDMMC_CLKENA_ENABLE_SHIFT      0
#define SDMMC_CLKENA_ENABLE(slot)      ((1 << (slot)) << \
                                        SDMMC_CLKENA_ENABLE_SHIFT)
#define SDMMC_CLKENA_LOWPOWER_SHIFT    16
#define SDMMC_CLKENA_LOWPOWER(slot)    ((1 << (slot)) << \
                                        SDMMC_CLKENA_LOWPOWER_SHIFT)

/* Timeout register TMOUT ***************************************************/

#define SDMMC_TMOUT_RESPONSE_SHIFT     (0)
#define SDMMC_TMOUT_RESPONSE_MASK      (255 << SDMMC_TMOUT_RESPONSE_SHIFT)
#define SDMMC_TMOUT_DATA_SHIFT         (8)
#define SDMMC_TMOUT_DATA_MASK          (0x00ffffff << SDMMC_TMOUT_DATA_SHIFT)

/* Card type register CTYPE *************************************************/

#define SDMMC_CTYPE_WIDTH4_SHIFT       0
#define SDMMC_CTYPE_WIDTH4_MASK(slot)  ((1 << (slot)) << \
                                        SDMMC_CTYPE_WIDTH4_SHIFT)
#define SDMMC_CTYPE_WIDTH8_SHIFT       (16)
#define SDMMC_CTYPE_WIDTH8_MASK(slot)  ((1 << (slot)) << \
                                        SDMMC_CTYPE_WIDTH8_SHIFT)

/* Blocksize register BLKSIZ ************************************************/

#define SDMMC_BLKSIZ_SHIFT             (0)
#define SDMMC_BLKSIZ_MASK              (0xffff << SDMMC_BLKSIZ_SHIFT)

/* Interrupt registers INTMASK / MINTSTS / RINTSTS **************************/

#define SDMMC_INT_CDET     (1 << 0)  /* Card detect */
#define SDMMC_INT_RE       (1 << 1)  /* Response error */
#define SDMMC_INT_CDONE    (1 << 2)  /* Command done */
#define SDMMC_INT_DTO      (1 << 3)  /* Data transfer over */
#define SDMMC_INT_TXDR     (1 << 4)  /* Transmit FIFO data request */
#define SDMMC_INT_RXDR     (1 << 5)  /* Receive FIFO data request */
#define SDMMC_INT_RCRC     (1 << 6)  /* Response CRC error */
#define SDMMC_INT_DCRC     (1 << 7)  /* Data CRC error */
#define SDMMC_INT_RTO      (1 << 8)  /* Response timeout */
#define SDMMC_INT_DRTO     (1 << 9)  /* Data read timeout */
#define SDMMC_INT_HTO      (1 << 10) /* Data starvation-by-cpu timeout */
#define SDMMC_INT_FRUN     (1 << 11) /* FIFO underrun/overrun error */
#define SDMMC_INT_HLE      (1 << 12) /* Hardware locked write error */
#define SDMMC_INT_SBE      (1 << 13) /* Start-bit error */
#define SDMMC_INT_ACD      (1 << 14) /* Auto command done */
#define SDMMC_INT_EBE      (1 << 15) /* End-bit error / write no CRC */
#define SDMMC_INT_SDIO     (1 << 16) /* SDIO interrupt (card 0) */
#define SDMMC_INT_ALL(slot) ((0xffff) | (1 << ((slot) + 16)))

/* Command register CMD *****************************************************/

#define SDMMC_CMD_CMDINDEX_SHIFT       (0)
#define SDMMC_CMD_CMDINDEX_MASK        (63 << SDMMC_CMD_CMDINDEX_SHIFT)
#define SDMMC_CMD_RESPONSE             (1 << 6)
#define SDMMC_CMD_LONGRESP             (1 << 7)
#define SDMMC_CMD_WAITRESP_SHIFT       (6)
#define SDMMC_CMD_WAITRESP_MASK        (3 << SDMMC_CMD_WAITRESP_SHIFT)
#define SDMMC_CMD_NORESPONSE           (0 << SDMMC_CMD_WAITRESP_SHIFT)
#define SDMMC_CMD_SHORTRESPONSE        (1 << SDMMC_CMD_WAITRESP_SHIFT)
#define SDMMC_CMD_LONGRESPONSE         (3 << SDMMC_CMD_WAITRESP_SHIFT)
#define SDMMC_CMD_RESPCRC              (1 << 8)  /* Check response CRC */
#define SDMMC_CMD_DATAXFREXPTD         (1 << 9)  /* Data transfer expected */
#define SDMMC_CMD_WRITE                (1 << 10) /* Write to card */
#define SDMMC_CMD_XFRMODE              (1 << 11) /* Stream transfer */
#define SDMMC_CMD_AUTOSTOP             (1 << 12) /* Send stop at end */
#define SDMMC_CMD_WAITPREV             (1 << 13) /* Wait previous complete */
#define SDMMC_CMD_STOPABORT            (1 << 14) /* Stop current transfer */
#define SDMMC_CMD_SENDINIT             (1 << 15) /* Send init sequence */
#define SDMMC_CMD_CARD_NUMBER(n)       ((n) << 16)
#define SDMMC_CMD_UPDCLOCK             (1 << 21) /* Update clock register */
#define SDMMC_CMD_USE_HOLE             (1 << 29) /* Use hold register */
#define SDMMC_CMD_STARTCMD             (1 << 31) /* Start command */

/* Status register STATUS ***************************************************/

#define SDMMC_STATUS_FIFOEMPTY         (1 << 2)  /* FIFO is empty */
#define SDMMC_STATUS_FIFOFULL          (1 << 3)  /* FIFO is full */
#define SDMMC_STATUS_DATABUSY          (1 << 9)  /* Card data busy */
#define SDMMC_STATUS_FIFOCOUNT_SHIFT   17
#define SDMMC_STATUS_FIFOCOUNT_MASK    (0x1fffu << 17)
#define SDMMC_STATUS_FIFOCOUNT(s) \
        (((s) & SDMMC_STATUS_FIFOCOUNT_MASK) >> SDMMC_STATUS_FIFOCOUNT_SHIFT)

/* FIFO threshold register FIFOTH *******************************************/

#define SDMMC_FIFOTH_TXWMARK_SHIFT     (0)
#define SDMMC_FIFOTH_TXWMARK_MASK      (0xfff << SDMMC_FIFOTH_TXWMARK_SHIFT)
#define SDMMC_FIFOTH_TXWMARK(n)        ((uint32_t)(n) << \
                                        SDMMC_FIFOTH_TXWMARK_SHIFT)
#define SDMMC_FIFOTH_RXWMARK_SHIFT     (16)
#define SDMMC_FIFOTH_RXWMARK_MASK      (0xfff << SDMMC_FIFOTH_RXWMARK_SHIFT)
#define SDMMC_FIFOTH_RXWMARK(n)        ((uint32_t)(n) << \
                                        SDMMC_FIFOTH_RXWMARK_SHIFT)
#define SDMMC_FIFOTH_DMABURST_SHIFT    (28)
#define SDMMC_FIFOTH_DMABURST_MASK     (7u << SDMMC_FIFOTH_DMABURST_SHIFT)
#define SDMMC_FIFOTH_DMABURST_1XFR     (0u << SDMMC_FIFOTH_DMABURST_SHIFT)
#define SDMMC_FIFOTH_DMABURST_4XFRS    (1u << SDMMC_FIFOTH_DMABURST_SHIFT)
#define SDMMC_FIFOTH_DMABURST_8XFRS    (2u << SDMMC_FIFOTH_DMABURST_SHIFT)

/* Card detect register CDETECT *********************************************/

#define SDMMC_CDETECT_NOTPRESENT(slot) (1 << (slot))

/* Write protect register WRTPRT ********************************************/

#define SDMMC_WRTPRT_PROTECTED(slot)   (1 << (slot))

/* Bus Mode Register BMOD ***************************************************/

#define SDMMC_BMOD_SWR     (1 << 0)  /* Software Reset */
#define SDMMC_BMOD_FB      (1 << 1)  /* Fixed Burst */
#define SDMMC_BMOD_DE      (1 << 7)  /* SD/MMC DMA Enable */

/* Internal DMAC Status Register IDSTS **************************************/

#define SDMMC_IDSTS_TI     (1 << 0)  /* Transmit Interrupt */
#define SDMMC_IDSTS_RI     (1 << 1)  /* Receive Interrupt */
#define SDMMC_IDSTS_FBE    (1 << 2)  /* Fatal Bus Error Interrupt */
#define SDMMC_IDSTS_DU     (1 << 4)  /* Descriptor Unavailable Interrupt */
#define SDMMC_IDSTS_CES    (1 << 5)  /* Card Error Summary */
#define SDMMC_IDSTS_NIS    (1 << 8)  /* Normal Interrupt Summary */
#define SDMMC_IDSTS_AIS    (1 << 9)  /* Abnormal Interrupt Summary */

/* Internal DMAC Interrupt Enable Register IDINTEN **************************/

#define SDMMC_IDINTEN_TI   (1 << 0)  /* Transmit Interrupt */
#define SDMMC_IDINTEN_RI   (1 << 1)  /* Receive Interrupt */
#define SDMMC_IDINTEN_FBE  (1 << 2)  /* Fatal Bus Error Interrupt */
#define SDMMC_IDINTEN_DU   (1 << 4)  /* Descriptor Unavailable Interrupt */
#define SDMMC_IDINTEN_CES  (1 << 5)  /* Card Error Summary */
#define SDMMC_IDINTEN_NIS  (1 << 8)  /* Normal Interrupt Summary */
#define SDMMC_IDINTEN_AIS  (1 << 9)  /* Abnormal Interrupt Summary */
#define SDMMC_IDINTEN_ALL  0x00000333

/* Clock edge/phase select register CLKEDGE (SDMMC 0x800) *******************/

/* On the P4 this register (SDHOST_CLK_EDGE_SEL) holds card-clock phase and
 * an SDIO clock enable at bit 23 (CCLK_EN, default 1) -- it is NOT the
 * host-clock-source select that the S3 kept here.  This port drives the
 * host divider and source from HP_SYS_CLKRST (below) and leaves CCLK_EN set.
 */

#define SDMMC_CLKEDGE_CCLK_EN          (1 << 23) /* SDIO clock enable */

/* HP_SYS_CLKRST / LP_CLKRST -- P4 clock and reset for the SDMMC block ******/

/* Values mirror components/soc/esp32p4/register/hw_ver3/soc/
 * hp_sys_clkrst_reg.h and lp_clkrst_reg.h and the sequence in
 * components/esp_hal_sd/esp32p4/include/hal/sdmmc_ll.h.  These are the KEY
 * S3->P4 clock adaptation and the most likely first bring-up bug: verify on
 * hardware that the LS clock source/divider produce the expected card clock.
 */

#define ESP32P4_HP_SYS_CLKRST_SOC_CLK_CTRL1  (DR_REG_HP_SYS_CLKRST_BASE + 0x18)
#define ESP32P4_HP_SYS_CLKRST_PERI_CLK_CTRL01 \
                                          (DR_REG_HP_SYS_CLKRST_BASE + 0x34)
#define ESP32P4_HP_SYS_CLKRST_PERI_CLK_CTRL02 \
                                          (DR_REG_HP_SYS_CLKRST_BASE + 0x38)
#define ESP32P4_LP_CLKRST_HP_SDMMC_EMAC_RST  (DR_REG_LP_CLKRST_BASE + 0x4c)

/* SOC_CLK_CTRL1: SDMMC AHB (bus) clock enable */

#define HP_SYS_CLKRST_SDMMC_SYS_CLK_EN       (1 << 14)

/* PERI_CLK_CTRL01: LS (low-speed) clock source/enable */

#define HP_SYS_CLKRST_SDIO_HS_MODE           (1 << 22)
#define HP_SYS_CLKRST_SDIO_LS_CLK_SRC_SEL    (1 << 23) /* 0:PLL160M 1:200M */
#define HP_SYS_CLKRST_SDIO_LS_CLK_EN         (1 << 24)

/* PERI_CLK_CTRL02: LS clock edge (host divider) and phase */

#define HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_UPD   (1 << 8)
#define HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_L_S   9
#define HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_L_M   (0xf << 9)
#define HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_H_S   13
#define HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_H_M   (0xf << 13)
#define HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_N_S   17
#define HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_N_M   (0xf << 17)
#define HP_SYS_CLKRST_SDIO_LS_SLF_EDGE_SEL_S 21
#define HP_SYS_CLKRST_SDIO_LS_SLF_EDGE_SEL_M (0x3 << 21)
#define HP_SYS_CLKRST_SDIO_LS_DRV_EDGE_SEL_S 23
#define HP_SYS_CLKRST_SDIO_LS_DRV_EDGE_SEL_M (0x3 << 23)
#define HP_SYS_CLKRST_SDIO_LS_SAM_EDGE_SEL_S 25
#define HP_SYS_CLKRST_SDIO_LS_SAM_EDGE_SEL_M (0x3 << 25)
#define HP_SYS_CLKRST_SDIO_LS_SLF_CLK_EN     (1 << 27)
#define HP_SYS_CLKRST_SDIO_LS_DRV_CLK_EN     (1 << 28)
#define HP_SYS_CLKRST_SDIO_LS_SAM_CLK_EN     (1 << 29)

/* LP_CLKRST HP_SDMMC_EMAC_RST_CTRL: SDMMC module reset */

#define LP_CLKRST_RST_EN_SDMMC               (1 << 28)

/* Source clock feeding the SDMMC LS divider when PLL160M is selected. */

#define ESP32P4_SDMMC_SRC_FREQ_HZ            (160 * 1000 * 1000)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: esp32p4_sdmmc_sdio_initialize
 *
 * Description:
 *   Initialize the SDMMC host for SDIO operation and return the standard
 *   NuttX SDIO interface.
 *
 * Input Parameters:
 *   slotno - The SDMMC slot (0 or 1).  The on-board ESP32-C6 is wired to
 *            slot 1 through the GPIO matrix.
 *
 * Returned Value:
 *   A reference to an SDIO interface structure, or NULL on failure.
 *
 ****************************************************************************/

struct sdio_dev_s *esp32p4_sdmmc_sdio_initialize(int slotno);

/****************************************************************************
 * Name: esp32p4_sdmmc_set_sample_phase
 *
 * Description:
 *   Set the SDMMC LS sample-clock edge (0..3).  Pulses the edge-config
 *   update bit.  Used to find a DAT sampling point that is not the
 *   ESP-IDF default (0).
 *
 ****************************************************************************/

void esp32p4_sdmmc_set_sample_phase(unsigned int phase);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32P4_ESPRESSIF_ESP32P4_SDMMC_H */
