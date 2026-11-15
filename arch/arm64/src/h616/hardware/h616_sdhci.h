/****************************************************************************
 * arch/arm64/src/h616/hardware/h616_sdhci.h
 *
 * H616 (sun50iw9) SD/MMC host controller register definitions.
 *
 * The H616 SD/MMC controller is the Allwinner "sunxi-mmc" SDHCI-compatible
 * block.  This is the same family as A64/A100/H6.  Register layout follows
 * the authoritative Allwinner reference (Tina U-Boot sunxi_mmc.c /
 * linux sunxi-mmc.h, "v4p6x" variant for the eMMC controller sdc2).
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

#ifndef __ARCH_ARM64_SRC_H616_HARDWARE_H616_SDHCI_H
#define __ARCH_ARM64_SRC_H616_HARDWARE_H616_SDHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SD/MMC host controller register offsets (sunxi-mmc) */

#define H616_SDXC_GCTRL          0x00  /* Global control */
#define H616_SDXC_CLKCR          0x04  /* Clock control */
#define H616_SDXC_TMOUT          0x08  /* Timeout */
#define H616_SDXC_WIDTH          0x0c  /* Bus width */
#define H616_SDXC_BLKSZ          0x10  /* Block size */
#define H616_SDXC_BCNTR          0x14  /* Byte count */
#define H616_SDXC_CMDR           0x18  /* Command */
#define H616_SDXC_CARG           0x1c  /* Command argument */
#define H616_SDXC_RESP0          0x20  /* Response 0 */
#define H616_SDXC_RESP1          0x24  /* Response 1 */
#define H616_SDXC_RESP2          0x28  /* Response 2 */
#define H616_SDXC_RESP3          0x2c  /* Response 3 */
#define H616_SDXC_IMASK          0x30  /* Interrupt mask */
#define H616_SDXC_MISTA          0x34  /* Masked interrupt status */
#define H616_SDXC_RINTR          0x38  /* Raw interrupt status */
#define H616_SDXC_STAS           0x3c  /* Status */
#define H616_SDXC_FTRGL          0x40  /* FIFO threshold watermark */
#define H616_SDXC_FUNS           0x44  /* Function select */
#define H616_SDXC_CBCR           0x48  /* CIU byte count */
#define H616_SDXC_BBCR           0x4c  /* BIU byte count */
#define H616_SDXC_DBGC           0x50  /* Debug enable */
#define H616_SDXC_CSDC           0x54  /* Card strobe delay control */
#define H616_SDXC_A12A           0x58  /* Auto CMD12 argument */
#define H616_SDXC_NTSR           0x5c  /* New timing set */
#define H616_SDXC_HWRST          0x78  /* Card hardware reset */
#define H616_SDXC_DMAC           0x80  /* IDMAC control */
#define H616_SDXC_DLBA           0x84  /* IDMAC descriptor list base addr */
#define H616_SDXC_IDST           0x88  /* IDMAC status */
#define H616_SDXC_IDIE           0x8c  /* IDMAC interrupt enable */
#define H616_SDXC_CHDA           0x90  /* Current host descriptor address */
#define H616_SDXC_CBDA           0x94  /* Current buffer descriptor address */
#define H616_SDXC_THLDC          0x100 /* Card threshold control */
#define H616_SDXC_DSBD           0x10c /* eMMC4.5 DDR start bit detect ctrl */
#define H616_SDXC_DRV_DL         0x140 /* Drive delay control (timing mode 4) */
#define H616_SDXC_SAMP_DL        0x144 /* Sample delay control (timing mode 4) */
#define H616_SDXC_DS_DL          0x148 /* Data strobe delay control (HS400) */
#define H616_SDXC_FIFO           0x200 /* Data FIFO */

/* CSDC / DSBD bits.  U-Boot boots the eMMC in HS400 (100MHz, data strobe
 * sampling) and programs CSDC[3:0] = 6 together with DSBD bit31 as its HS400
 * mode switch (mmc_hs400_mode_onoff in sunxi_mmc.c).  Both are *controller*
 * settings that must go back to SDR whenever the card is reset to its
 * default speed mode, or no data command transfers anything.
 */

#define H616_SDXC_CSDC_STROBE_MASK    0x0000000f
#define H616_SDXC_DSBD_HS400          (1U << 31)

/* Sample / strobe delay registers (0x144 / 0x148).  Bit 7 enables the
 * delay, bits[5:0] hold the delay value (Tina U-Boot sunxi_mmc.h:
 * SDXC_EnableDly = 1 << 7, SDXC_CfgDly = 0x3F << 0).  They are what makes
 * HS200/HS400 sampling work; the default values per speed mode and clock
 * live in sunxi_host_mmc.c (timing mode 4 table).
 */

#define H616_SDXC_DLY_VALUE_MASK      0x0000003f
#define H616_SDXC_DLY_ENABLE          (1U << 7)

/* GCTRL bits */

#define H616_SDXC_SOFT_RESET          (1 << 0)
#define H616_SDXC_FIFO_RESET          (1 << 1)
#define H616_SDXC_DMA_RESET           (1 << 2)
#define H616_SDXC_INTERRUPT_ENABLE    (1 << 4)
#define H616_SDXC_DMA_ENABLE          (1 << 5)
#define H616_SDXC_ACCESS_BY_AHB       (1U << 31)
#define H616_SDXC_DEBOUNCE_ENABLE     (1 << 8)
#define H616_SDXC_POSEDGE_LATCH_DATA  (1 << 9)
#define H616_SDXC_DDR_MODE            (1 << 10)
#define H616_SDXC_MEMORY_ACCESS_DONE  (1 << 29)
#define H616_SDXC_ACCESS_DONE_DIRECT  (1 << 30)
#define H616_SDXC_HARDWARE_RESET \
  (H616_SDXC_SOFT_RESET | H616_SDXC_FIFO_RESET | H616_SDXC_DMA_RESET)

/* CLKCR bits */

#define H616_SDXC_CARD_CLOCK_ON       (1 << 16)
#define H616_SDXC_LOW_POWER_ON        (1 << 17)
#define H616_SDXC_MASK_DATA0          (1 << 31)

/* WIDTH */

#define H616_SDXC_WIDTH1              0
#define H616_SDXC_WIDTH4              1
#define H616_SDXC_WIDTH8              2

/* CMDR bits */

#define H616_SDXC_RESP_EXPECT         (1 << 6)
#define H616_SDXC_LONG_RESPONSE       (1 << 7)
#define H616_SDXC_CHECK_RESPONSE_CRC  (1 << 8)
#define H616_SDXC_DATA_EXPECT         (1 << 9)
#define H616_SDXC_WRITE               (1 << 10)
#define H616_SDXC_SEQUENCE_MODE       (1 << 11)
#define H616_SDXC_SEND_AUTO_STOP      (1 << 12)
#define H616_SDXC_WAIT_PRE_OVER       (1 << 13)
#define H616_SDXC_STOP_ABORT_CMD      (1 << 14)
#define H616_SDXC_SEND_INIT_SEQUENCE  (1 << 15)
#define H616_SDXC_UPCLK_ONLY          (1 << 21)
#define H616_SDXC_READ_CEATA_DEV      (1 << 22)
#define H616_SDXC_CCS_EXPECT          (1 << 23)
#define H616_SDXC_ENABLE_BIT_BOOT     (1 << 24)
#define H616_SDXC_ALT_BOOT_OPTIONS    (1 << 25)
#define H616_SDXC_BOOT_ACK_EXPECT     (1 << 26)
#define H616_SDXC_BOOT_ABORT          (1 << 27)
#define H616_SDXC_VOLTAGE_SWITCH      (1 << 28)
#define H616_SDXC_USE_HOLD_REGISTER   (1 << 29)
#define H616_SDXC_START               (1 << 31)

/* Interrupt bits */

#define H616_SDXC_RESP_ERROR          (1 << 1)
#define H616_SDXC_COMMAND_DONE        (1 << 2)
#define H616_SDXC_DATA_OVER           (1 << 3)
#define H616_SDXC_TX_DATA_REQUEST     (1 << 4)
#define H616_SDXC_RX_DATA_REQUEST     (1 << 5)
#define H616_SDXC_RESP_CRC_ERROR      (1 << 6)
#define H616_SDXC_DATA_CRC_ERROR      (1 << 7)
#define H616_SDXC_RESP_TIMEOUT        (1 << 8)
#define H616_SDXC_DATA_TIMEOUT        (1 << 9)
#define H616_SDXC_VOLTAGE_CHANGE_DONE (1 << 10)
#define H616_SDXC_FIFO_RUN_ERROR      (1 << 11)
#define H616_SDXC_HARD_WARE_LOCKED    (1 << 12)
#define H616_SDXC_START_BIT_ERROR     (1 << 13)
#define H616_SDXC_AUTO_COMMAND_DONE   (1 << 14)
#define H616_SDXC_END_BIT_ERROR       (1 << 15)
#define H616_SDXC_SDIO_INTERRUPT      (1 << 16)
#define H616_SDXC_CARD_INSERT         (1 << 30)
#define H616_SDXC_CARD_REMOVE         (1 << 31)

#define H616_SDXC_INTERRUPT_ERROR_BIT \
  (H616_SDXC_RESP_ERROR | H616_SDXC_RESP_CRC_ERROR | H616_SDXC_DATA_CRC_ERROR | \
   H616_SDXC_RESP_TIMEOUT | H616_SDXC_DATA_TIMEOUT | H616_SDXC_FIFO_RUN_ERROR | \
   H616_SDXC_HARD_WARE_LOCKED | H616_SDXC_START_BIT_ERROR | H616_SDXC_END_BIT_ERROR)

#define H616_SDXC_INTERRUPT_DONE_BIT \
  (H616_SDXC_AUTO_COMMAND_DONE | H616_SDXC_DATA_OVER | \
   H616_SDXC_COMMAND_DONE | H616_SDXC_VOLTAGE_CHANGE_DONE)

#define H616_SDXC_INTERRUPT_DDONE_BIT \
  (H616_SDXC_AUTO_COMMAND_DONE | H616_SDXC_DATA_OVER)

/* STAS bits */

#define H616_SDXC_RXWL_FLAG           (1 << 0)
#define H616_SDXC_TXWL_FLAG           (1 << 1)
#define H616_SDXC_FIFO_EMPTY          (1 << 2)
#define H616_SDXC_FIFO_FULL           (1 << 3)
#define H616_SDXC_CARD_PRESENT        (1 << 8)
#define H616_SDXC_CARD_DATA_BUSY      (1 << 9)
#define H616_SDXC_DATA_FSM_BUSY       (1 << 10)
#define H616_SDXC_DMA_REQUEST         (1 << 31)
#define H616_SDXC_FIFO_SIZE           16

/* IDMAC control */

#define H616_SDXC_IDMAC_SOFT_RESET        (1 << 0)
#define H616_SDXC_IDMAC_FIX_BURST         (1 << 1)
#define H616_SDXC_IDMAC_IDMA_ON           (1 << 7)
#define H616_SDXC_IDMAC_REFETCH_DES       (1 << 31)

/* IDMAC status */

#define H616_SDXC_IDMAC_TRANSMIT_INTERRUPT   (1 << 0)
#define H616_SDXC_IDMAC_RECEIVE_INTERRUPT    (1 << 1)
#define H616_SDXC_IDMAC_FATAL_BUS_ERROR      (1 << 2)
#define H616_SDXC_IDMAC_DESTINATION_INVALID  (1 << 4)
#define H616_SDXC_IDMAC_CARD_ERROR_SUM       (1 << 5)
#define H616_SDXC_IDMAC_NORMAL_INTERRUPT_SUM (1 << 8)
#define H616_SDXC_IDMAC_ABNORMAL_INTERRUPT_SUM (1 << 9)
#define H616_SDXC_IDMAC_HOST_ABORT_INTERRUPT (1 << 10)

/* IDMAC descriptor "config" word bits (Linux sunxi-mmc.h) */

#define H616_IDMAC_DES0_DIC   (1 << 1)   /* disable interrupt on completion */
#define H616_IDMAC_DES0_LD    (1 << 2)   /* last descriptor */
#define H616_IDMAC_DES0_FD    (1 << 3)   /* first descriptor */
#define H616_IDMAC_DES0_CH    (1 << 4)   /* chain mode */
#define H616_IDMAC_DES0_ER    (1 << 5)   /* end of ring */
#define H616_IDMAC_DES0_OWN   (1U << 31) /* 1 = IDMA owns it */

/* eMMC reset (HWRST) */

#define H616_SDXC_CARD_RESET          (1 << 0)

/* CCU (0x03001000) MMC clock control registers */

#define H616_CCU_SD0_CLK_CFG          0x830
#define H616_CCU_SD1_CLK_CFG          0x834
#define H616_CCU_SD2_CLK_CFG          0x838
#define H616_CCU_SD_GATE_RESET        0x84c

#define H616_CCM_MMC_CTRL_M(x)        ((x) - 1)
#define H616_CCM_MMC_CTRL_N(x)        ((x) << 8)
#define H616_CCM_MMC_CTRL_OSCM24      (0x0 << 24)
#define H616_CCM_MMC_CTRL_PLL6X2      (0x1 << 24)
#define H616_CCM_MMC_CTRL_PLL_PERIPH2X2 (0x2 << 24)
#define H616_CCM_MMC_CTRL_ENABLE      (0x1 << 31)

/* MMC gate/reset bits in SD_GATE_RESET (H616 / sun50iw9).  Per Tina U-Boot
 * mmc_clk_io_onoff(): gate = bit (0 + n), reset = bit (16 + n).
 */

#define H616_CCM_MMC_GATE(x)          (1 << (0 + (x)))
#define H616_CCM_MMC_RST(x)           (1 << (16 + (x)))

/* Oscillator base clock (H616 OSC24M) */

#define H616_OSC24M                   24000000

/* PLL6 frequency (Hz).  H616 PLL_PERIPH0 = PLL6, 600MHz typical.  The
 * CCU "PLL6X2" source is 2x PLL6.  See Tina clock_sun50iw9.h: CCM_MMC
 * uses PLL6X2 for frequencies > 24MHz.
 */

#define H616_PLL6_HZ                  600000000
#define H616_PLL6X2_HZ                1200000000

#endif /* __ARCH_ARM64_SRC_H616_HARDWARE_H616_SDHCI_H */
