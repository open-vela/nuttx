/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_sdmmc.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_SDMMC_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_SDMMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SDMMC controller count */

#define RK3576_SDMMC_COUNT         3   /* SDMMC0, SDMMC1, eMMC */

/* SDMMC register offsets (DesignWare MSHC) */

#define SDMMC_CTRL                 0x0000   /* Control register */
#define SDMMC_PWREN                0x0004   /* Power enable */
#define SDMMC_CLKDIV               0x0008   /* Clock divider */
#define SDMMC_CLKENA               0x000c   /* Clock enable */
#define SDMMC_CMDCRC               0x0010   /* Command CRC */
#define SDMMC_RESPONSE0            0x0014   /* Response 0 */
#define SDMMC_RESPONSE1            0x0018   /* Response 1 */
#define SDMMC_RESPONSE2            0x001c   /* Response 2 */
#define SDMMC_RESPONSE3            0x0020   /* Response 3 */
#define SDMMC_MINTSTS              0x0024   /* Masked interrupt status */
#define SDMMC_RINTSTS              0x0028   /* Raw interrupt status */
#define SDMMC_INTMASK              0x002c   /* Interrupt mask */
#define SDMMC_CMDARG               0x0030   /* Command argument */
#define SDMMC_CMD                  0x0034   /* Command register */
#define SDMMC_RESP0                0x0038   /* Response (auto) */
#define SDMMC_RESP1                0x003c   /* Response 1 */
#define SDMMC_RESP2                0x0040   /* Response 2 */
#define SDMMC_RESP3                0x0044   /* Response 3 */
#define SDMMC_MINTSTS2             0x0048   /* Masked interrupt status 2 */
#define SDMMC_RINTSTS2             0x004c   /* Raw interrupt status 2 */
#define SDMMC_INTMASK2             0x0050   /* Interrupt mask 2 */
#define SDMMC_CLKDIV2              0x0054   /* Clock divider 2 */
#define SDMMC_CARDDET              0x0080   /* Card detect */
#define SDMMC_CARDCNT              0x0084   /* Card count */
#define SDMMC_CARDENA              0x0088   /* Card enable */
#define SDMMC_FIFOTH               0x0100   /* FIFO threshold */
#define SDMMC_CARD_THRESH          0x0104   /* Card threshold */
#define SDMMC_DBGR                 0x0108   /* Debug register */
#define SDMMC_DBGR2                0x010c   /* Debug register 2 */
#define SDMMC_DATA                 0x0200   /* Data port (FIFO) */

/****************************************************************************
 * SDMMC_CTRL bit definitions
 ****************************************************************************/

#define SDMMC_CTRL_DMA_ENABLE      (1 << 5)
#define SDMMC_CTRL_DMA_RESET       (1 << 2)
#define SDMMC_CTRL_FIFO_RESET      (1 << 1)
#define SDMMC_CTRL_CONTROLLER_RESET (1 << 0)
#define SDMMC_CTRL_INT_ENABLE      (1 << 4)
#define SDMMC_CTRL_SEND_IRQ        (1 << 3)

/****************************************************************************
 * SDMMC_CLKENA bit definitions
 ****************************************************************************/

#define SDMMC_CLKENA_CCLK_EN       (1 << 0)
#define SDMMC_CLKENA_CCLK_LOW_PWR  (1 << 16)

/****************************************************************************
 * SDMMC_CMD bit definitions
 ****************************************************************************/

#define SDMMC_CMD_START            (1 << 31)
#define SDMMC_CMD_USE_HOLD         (1 << 29)
#define SDMMC_CMD_VOLT_SWITCH      (1 << 28)
#define SDMMC_CMD_BOOT_MODE        (1 << 27)
#define SDMMC_CMD_BOOT_EN          (1 << 26)
#define SDMMC_CMD_BOOT_ACK         (1 << 25)
#define SDMMC_CMD_UPDATE_CLK       (1 << 21)
#define SDMMC_CMD_SEND_INIT        (1 << 15)
#define SDMMC_CMD_STOP_ABORT       (1 << 14)
#define SDMMC_CMD_WAIT_PRVDATA     (1 << 13)
#define SDMMC_CMD_SEND_STOP        (1 << 12)
#define SDMMC_CMD_DATA_EXPECT      (1 << 11)
#define SDMMC_CMD_CHECK_RESP_CRC   (1 << 10)
#define SDMMC_CMD_LONG_RESP        (1 << 7)
#define SDMMC_CMD_RESP_EXP         (1 << 6)
#define SDMMC_CMD_INDEX_CHECK      (1 << 5)

#define SDMMC_CMD_RESP_NONE        (0 << 6)
#define SDMMC_CMD_RESP_SHORT       (1 << 6)
#define SDMMC_CMD_RESP_LONG        (3 << 6)

/****************************************************************************
 * SDMMC_RINTSTS bit definitions
 ****************************************************************************/

#define SDMMC_INT_CD               (1 << 0)   /* Card detect */
#define SDMMC_INT_RE               (1 << 2)   /* Response error */
#define SDMMC_INT_CMD_DONE         (1 << 3)   /* Command done */
#define SDMMC_INT_DATA_OVER        (1 << 4)   /* Data transfer over */
#define SDMMC_INT_TXDR             (1 << 8)   /* TX FIFO data request */
#define SDMMC_INT_RXDR             (1 << 9)   /* RX FIFO data request */
#define SDMMC_INT_RCRC             (1 << 5)   /* Response CRC error */
#define SDMMC_INT_DCRC             (1 << 6)   /* Data CRC error */
#define SDMMC_INT_RTO              (1 << 7)   /* Response timeout */
#define SDMMC_INT_DRTO             (1 << 12)  /* Data read timeout */
#define SDMMC_INT_HTO              (1 << 10)  /* Data starvation timeout */
#define SDMMC_INT_FRUN             (1 << 11)  /* FIFO underrun/overrun */
#define SDMMC_INT_HLE              (1 << 13)  /* Hardware locked error */
#define SDMMC_INT_SBE              (1 << 14)  /* Start bit error */
#define SDMMC_INT_EBE              (1 << 15)  /* End bit error */

#define SDMMC_INT_ALL              0xffffffff

/****************************************************************************
 * SDMMC_CARDDET bit definitions
 ****************************************************************************/

#define SDMMC_CARDDET_CARD         (1 << 0)

/****************************************************************************
 * SDMMC_FIFOTH bit definitions
 ****************************************************************************/

#define SDMMC_FIFOTH_DWIDTH_SHIFT  28
#define SDMMC_FIFOTH_DWIDTH_MASK   (3 << SDMMC_FIFOTH_DWIDTH_SHIFT)
#define SDMMC_FIFOTH_DWIDTH_1      (0 << SDMMC_FIFOTH_DWIDTH_SHIFT)
#define SDMMC_FIFOTH_DWIDTH_4      (1 << SDMMC_FIFOTH_DWIDTH_SHIFT)
#define SDMMC_FIFOTH_DWIDTH_8      (2 << SDMMC_FIFOTH_DWIDTH_SHIFT)

/****************************************************************************
 * Clock frequencies
 ****************************************************************************/

#define SDMMC_CLK_400KHZ           400000
#define SDMMC_CLK_25MHZ            25000000
#define SDMMC_CLK_50MHZ            50000000

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

extern const uint32_t g_sdmmc_base[RK3576_SDMMC_COUNT];

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_SDMMC_H */
