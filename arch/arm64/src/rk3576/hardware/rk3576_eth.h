/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_eth.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_ETH_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_ETH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Ethernet controller count */

#define RK3576_ETH_COUNT            2   /* ETH0, ETH1 */

/* MAC Registers (0x0000-0x00FF) */

#define ETH_MAC_CONFIG             0x0000   /* MAC configuration */
#define ETH_MAC_FRAME_FILTER       0x0004   /* Frame filter */
#define ETH_HASH_HIGH              0x0008   /* Hash table high */
#define ETH_HASH_LOW               0x000c   /* Hash table low */
#define ETH_GMII_ADDR              0x0010   /* GMII address (MDIO) */
#define ETH_GMII_DATA              0x0014   /* GMII data */
#define ETH_FLOW_CONTROL           0x0018   /* Flow control */
#define ETH_VLAN_TAG               0x001c   /* VLAN tag */
#define ETH_VERSION                0x0020   /* Version */
#define ETH_DEBUG                  0x0024   /* Debug */
#define ETH_INT_STATUS             0x0038   /* Interrupt status */
#define ETH_INT_MASK               0x003c   /* Interrupt mask */

/* MAC Registers (0x0100-0x01FF) */

#define ETH_HASH_TABLE_REG(n)     (0x0100 + (n) * 4)

/* DMA Registers (0x1000-0x13FF) */

#define ETH_DMA_BUS_MODE           0x1000   /* DMA bus mode */
#define ETH_DMA_TX_POLL_DEMAND     0x1004   /* TX poll demand */
#define ETH_DMA_RX_POLL_DEMAND     0x1008   /* RX poll demand */
#define ETH_DMA_TX_DESC_LIST       0x100c   /* TX descriptor list */
#define ETH_DMA_RX_DESC_LIST       0x1010   /* RX descriptor list */
#define ETH_DMA_STATUS             0x1014   /* DMA status */
#define ETH_DMA_CONTROL            0x1018   /* DMA control */
#define ETH_DMA_MISS_FRAME_CNTR    0x101c   /* Miss frame counter */
#define ETH_DMA_RX_OVERRUN_CNTR    0x1020   /* RX overrun counter */

/* DMA Interrupt Registers */

#define ETH_DMA_INT_STATUS         0x1038   /* DMA interrupt status */
#define ETH_DMA_INT_ENABLE         0x103c   /* DMA interrupt enable */

/* DMA Descriptor Registers */

#define ETH_DMA_TX_DESC_CTRL       0x1044   /* TX descriptor control */
#define ETH_DMA_RX_DESC_CTRL       0x1048   /* RX descriptor control */
#define ETH_DMA_CUR_TX_DESC        0x104c   /* Current TX descriptor */
#define ETH_DMA_CUR_RX_DESC        0x1050   /* Current RX descriptor */

/****************************************************************************
 * ETH_MAC_CONFIG bit definitions
 ****************************************************************************/

#define ETH_MAC_CONFIG_RE          (1 << 2)   /* Receiver Enable */
#define ETH_MAC_CONFIG_TE          (1 << 3)   /* Transmitter Enable */
#define ETH_MAC_CONFIG_PRELEN_SHIFT 6
#define ETH_MAC_CONFIG_PRELEN_MASK (3 << ETH_MAC_CONFIG_PRELEN_SHIFT)
#define ETH_MAC_CONFIG_PRELEN_7    (0 << ETH_MAC_CONFIG_PRELEN_SHIFT)
#define ETH_MAC_CONFIG_MODE_SHIFT  11
#define ETH_MAC_CONFIG_MODE_MASK   (3 << ETH_MAC_CONFIG_MODE_SHIFT)
#define ETH_MAC_CONFIG_MODE_MII    (0 << ETH_MAC_CONFIG_MODE_SHIFT)
#define ETH_MAC_CONFIG_MODE_RMII   (1 << ETH_MAC_CONFIG_MODE_SHIFT)
#define ETH_MAC_CONFIG_MODE_RGMII  (2 << ETH_MAC_CONFIG_MODE_SHIFT)
#define ETH_MAC_CONFIG_FES         (1 << 14)  /* Fast Ethernet Speed */
#define ETH_MAC_CONFIG_DM          (1 << 13)  /* Duplex Mode */

/****************************************************************************
 * ETH_INT_STATUS/INT_MASK bit definitions
 ****************************************************************************/

#define ETH_INT_PMT                (1 << 4)
#define ETH_INT_MMC                (1 << 5)
#define ETH_INT_PCS_ANE            (1 << 6)
#define ETH_INT_LPI                (1 << 11)

/****************************************************************************
 * ETH_DMA_BUS_MODE bit definitions
 ****************************************************************************/

#define ETH_DMA_BUS_MODE_SFT_RESET (1 << 0)   /* Software Reset */
#define ETH_DMA_BUS_MODE_TXPR      (1 << 12)  /* Transmit Priority */
#define ETH_DMA_BUS_MODE_PBL_SHIFT 17
#define ETH_DMA_BUS_MODE_PBL_MASK  (0x3f << ETH_DMA_BUS_MODE_PBL_SHIFT)
#define ETH_DMA_BUS_MODE_PBL_32    (32 << ETH_DMA_BUS_MODE_PBL_SHIFT)

/****************************************************************************
 * ETH_DMA_STATUS bit definitions
 ****************************************************************************/

#define ETH_DMA_STATUS_TI          (1 << 0)   /* Transmit Interrupt */
#define ETH_DMA_STATUS_TU          (1 << 2)   /* Transmit Buffer Unavailable */
#define ETH_DMA_STATUS_RI          (1 << 6)   /* Receive Interrupt */
#define ETH_DMA_STATUS_RBUS        (1 << 7)   /* Receive Buffer Unavailable */
#define ETH_DMA_STATUS_NIS         (1 << 16)  /* Normal Interrupt Summary */
#define ETH_DMA_STATUS_AIS         (1 << 17)  /* Abnormal Interrupt Summary */
#define ETH_DMA_STATUS_FBES        (1 << 13)  /* Fatal Bus Error */

/****************************************************************************
 * ETH_DMA_INT_ENABLE bit definitions
 ****************************************************************************/

#define ETH_DMA_INT_TI             (1 << 0)   /* Transmit Interrupt */
#define ETH_DMA_INT_TU             (1 << 2)   /* Transmit Buffer Unavailable */
#define ETH_DMA_INT_RI             (1 << 6)   /* Receive Interrupt */
#define ETH_DMA_INT_NIS            (1 << 16)  /* Normal Interrupt Summary */

/****************************************************************************
 * ETH_DMA_CONTROL bit definitions
 ****************************************************************************/

#define ETH_DMA_CONTROL_SR         (1 << 1)   /* Start/Stop Receive */
#define ETH_DMA_CONTROL_ST         (1 << 13)  /* Start/Stop Transmission */

/****************************************************************************
 * DMA Descriptor bit definitions (for RX)
 ****************************************************************************/

#define ETH_DESC_OWN               (1 << 31)  /* Ownership (1=DMA) */
#define ETH_DESC_A0                (1 << 30)  /* Address 0 */
#define ETH_DESC_FS                (1 << 9)   /* First Descriptor */
#define ETH_DESC_LS                (1 << 8)   /* Last Descriptor */
#define ETH_DESC_FL_SHIFT          16
#define ETH_DESC_FL_MASK           (0x3fff << ETH_DESC_FL_SHIFT)
#define ETH_DESC_ES                (1 << 15)  /* Error Summary */

/* Descriptor size */

#define ETH_DESC_SIZE              16
#define ETH_DESC_ALIGN             16

/* Buffer sizes */

#define ETH_TX_BUF_SIZE            1536
#define ETH_RX_BUF_SIZE            1536

/* Number of descriptors */

#define ETH_TX_DESC_COUNT          4
#define ETH_RX_DESC_COUNT          4

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

extern const uint32_t g_eth_base[RK3576_ETH_COUNT];

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_ETH_H */
