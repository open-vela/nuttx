/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/n947/n947_enet.h
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

/* DWC EMAC register definitions for MCXN947 ENET0.
 * Register layout is identical to the DesignWare Ethernet MAC used
 * in S32K3xx; only base address and clock/reset registers differ.
 */

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_ENET_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_ENET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/nxxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ENET0 non-secure base address */

#define N947_ENET0_BASE                (0x40100000u)

/* Clock and reset registers (in SYSCON0) */

#define N947_SYSCON0_PRESETCTRL2       (NXXX_SYSCON0_BASE + 0x108)
#define N947_SYSCON0_PRESETCTRLSET2    (NXXX_SYSCON0_BASE + 0x128)
#define N947_SYSCON0_PRESETCTRLCLR2    (NXXX_SYSCON0_BASE + 0x148)
/* ENET reset bit in PRESETCTRL2 */

#define N947_ENET_RST_MASK             (1u << 2)

/* AHB clock gate: bit 2 of AHBCLKCTRL2 */

/* Bit 2 in AHB_CLK_CTRL2 */

#define N947_ENET_AHBCLK_BIT           (1u << 2)

/* ENETRMII clock mux select register (offset 0x5B0 in SYSCON0) */

#define N947_SYSCON0_ENETRMIICLKSEL    (NXXX_SYSCON0_BASE + 0x5B0)
/* External reference clock from the PHY pin or PLL0. */

#define N947_ENETRMII_CLK_NONE         (0u)
#define N947_ENETRMII_CLK_PLL0         (1u)

/* ENET PHY interface type select register (offset 0x5C0 in SYSCON0).
 * Selects MII (0) or RMII (1).  Zephyr: SYSCON_ENET_PHY_INTF_SEL_PHY_SEL(1).
 */

#define N947_SYSCON0_ENETPHYINTFSEL    (NXXX_SYSCON0_BASE + 0x5C0)

/* PHY_SEL is at bit 2.  Select RMII (value 0x4) or MII/GMII. */

#define N947_ENET_PHY_INTF_RMII        (1u << 2)
#define N947_ENET_PHY_INTF_MII         (0u)

/* ENET register offsets ****************************************************/

#define N947_ENET_MAC_CONFIGURATION_OFFSET               (0x0000)
#define N947_ENET_MAC_EXT_CONFIGURATION_OFFSET           (0x0004)
#define N947_ENET_MAC_PACKET_FILTER_OFFSET               (0x0008)
#define N947_ENET_MAC_WATCHDOG_TIMEOUT_OFFSET            (0x000c)
#define N947_ENET_MAC_HASH_TABLE_REG0_OFFSET             (0x0010)
#define N947_ENET_MAC_HASH_TABLE_REG1_OFFSET             (0x0014)
#define N947_ENET_MAC_VLAN_TAG_OFFSET                    (0x0050)
#define N947_ENET_MAC_Q0_TX_FLOW_CTRL_OFFSET             (0x0070)
#define N947_ENET_MAC_RX_FLOW_CTRL_OFFSET                (0x0090)
#define N947_ENET_MAC_RXQ_CTRL0_OFFSET                   (0x00a0)
#define N947_ENET_MAC_RXQ_CTRL1_OFFSET                   (0x00a4)
#define N947_ENET_MAC_RXQ_CTRL2_OFFSET                   (0x00a8)
#define N947_ENET_MAC_INTERRUPT_STATUS_OFFSET            (0x00b0)
#define N947_ENET_MAC_INTERRUPT_ENABLE_OFFSET            (0x00b4)
#define N947_ENET_MAC_RX_TX_STATUS_OFFSET                (0x00b8)
#define N947_ENET_MAC_VERSION_OFFSET                     (0x0110)
#define N947_ENET_MAC_DEBUG_OFFSET                       (0x0114)
#define N947_ENET_MAC_HW_FEATURE0_OFFSET                 (0x011c)
#define N947_ENET_MAC_HW_FEATURE1_OFFSET                 (0x0120)
#define N947_ENET_MAC_HW_FEATURE2_OFFSET                 (0x0124)
#define N947_ENET_MAC_HW_FEATURE3_OFFSET                 (0x0128)
#define N947_ENET_MAC_MDIO_ADDRESS_OFFSET                (0x0200)
#define N947_ENET_MAC_MDIO_DATA_OFFSET                   (0x0204)
#define N947_ENET_MAC_ADDRESS0_HIGH_OFFSET               (0x0300)
#define N947_ENET_MAC_ADDRESS0_LOW_OFFSET                (0x0304)
#define N947_ENET_MMC_CONTROL_OFFSET                     (0x0700)
#define N947_ENET_MMC_RX_INTERRUPT_MASK_OFFSET           (0x070c)
#define N947_ENET_MMC_TX_INTERRUPT_MASK_OFFSET           (0x0710)
#define N947_ENET_MTL_TXQ0_OPERATION_MODE_OFFSET         (0x0d00)
#define N947_ENET_MTL_TXQ0_DEBUG_OFFSET                  (0x0d08)
#define N947_ENET_MTL_TXQ0_QUANTUM_WEIGHT_OFFSET         (0x0d18)
#define N947_ENET_MTL_Q0_INTERRUPT_CTRL_STATUS_OFFSET    (0x0d2c)
#define N947_ENET_MTL_RXQ0_OPERATION_MODE_OFFSET         (0x0d30)
#define N947_ENET_MTL_RXQ0_DEBUG_OFFSET                  (0x0d38)
#define N947_ENET_DMA_MODE_OFFSET                        (0x1000)
#define N947_ENET_DMA_SYSBUS_MODE_OFFSET                 (0x1004)
#define N947_ENET_DMA_INTERRUPT_STATUS_OFFSET            (0x1008)
#define N947_ENET_DMA_DEBUG_STATUS0_OFFSET               (0x100c)
#define N947_ENET_DMA_CH0_CONTROL_OFFSET                 (0x1100)
#define N947_ENET_DMA_CH0_TX_CONTROL_OFFSET              (0x1104)
#define N947_ENET_DMA_CH0_RX_CONTROL_OFFSET              (0x1108)
#define N947_ENET_DMA_CH0_TXDESC_LIST_ADDRESS_OFFSET     (0x1114)
#define N947_ENET_DMA_CH0_RXDESC_LIST_ADDRESS_OFFSET     (0x111c)
#define N947_ENET_DMA_CH0_TXDESC_TAIL_POINTER_OFFSET     (0x1120)
#define N947_ENET_DMA_CH0_RXDESC_TAIL_POINTER_OFFSET     (0x1128)
#define N947_ENET_DMA_CH0_TXDESC_RING_LENGTH_OFFSET      (0x112c)
#define N947_ENET_DMA_CH0_RXDESC_RING_LENGTH_OFFSET      (0x1130)
#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE_OFFSET        (0x1134)
#define N947_ENET_DMA_CH0_RX_INTERRUPT_WD_TIMER_OFFSET   (0x1138)
#define N947_ENET_DMA_CH0_SLOT_FUNC_CTRL_STATUS_OFFSET   (0x113c)
#define N947_ENET_DMA_CH0_CURRENT_APP_TXDESC_OFFSET      (0x1144)
#define N947_ENET_DMA_CH0_CURRENT_APP_RXDESC_OFFSET      (0x114c)
#define N947_ENET_DMA_CH0_CURRENT_APP_TXBUFFER_OFFSET    (0x1154)
#define N947_ENET_DMA_CH0_CURRENT_APP_RXBUFFER_OFFSET    (0x115c)
#define N947_ENET_DMA_CH0_STATUS_OFFSET                  (0x1160)
#define N947_ENET_DMA_CH0_MISS_FRAME_CNT_OFFSET          (0x1164)

/* Register absolute addresses */

#define N947_ENET_MAC_CONFIGURATION      (N947_ENET0_BASE + N947_ENET_MAC_CONFIGURATION_OFFSET)
#define N947_ENET_MAC_EXT_CONFIGURATION  (N947_ENET0_BASE + N947_ENET_MAC_EXT_CONFIGURATION_OFFSET)
#define N947_ENET_MAC_PACKET_FILTER      (N947_ENET0_BASE + N947_ENET_MAC_PACKET_FILTER_OFFSET)
#define N947_ENET_MAC_HASH_TABLE_REG0    (N947_ENET0_BASE + N947_ENET_MAC_HASH_TABLE_REG0_OFFSET)
#define N947_ENET_MAC_HASH_TABLE_REG1    (N947_ENET0_BASE + N947_ENET_MAC_HASH_TABLE_REG1_OFFSET)
#define N947_ENET_MAC_VLAN_TAG           (N947_ENET0_BASE + N947_ENET_MAC_VLAN_TAG_OFFSET)
#define N947_ENET_MAC_Q0_TX_FLOW_CTRL   (N947_ENET0_BASE + N947_ENET_MAC_Q0_TX_FLOW_CTRL_OFFSET)
#define N947_ENET_MAC_RX_FLOW_CTRL      (N947_ENET0_BASE + N947_ENET_MAC_RX_FLOW_CTRL_OFFSET)
#define N947_ENET_MAC_RXQ_CTRL0         (N947_ENET0_BASE + N947_ENET_MAC_RXQ_CTRL0_OFFSET)
#define N947_ENET_MAC_RXQ_CTRL1         (N947_ENET0_BASE + N947_ENET_MAC_RXQ_CTRL1_OFFSET)
#define N947_ENET_MAC_RXQ_CTRL2         (N947_ENET0_BASE + N947_ENET_MAC_RXQ_CTRL2_OFFSET)
#define N947_ENET_MAC_INTERRUPT_STATUS  (N947_ENET0_BASE + N947_ENET_MAC_INTERRUPT_STATUS_OFFSET)
#define N947_ENET_MAC_INTERRUPT_ENABLE  (N947_ENET0_BASE + N947_ENET_MAC_INTERRUPT_ENABLE_OFFSET)
#define N947_ENET_MAC_RX_TX_STATUS      (N947_ENET0_BASE + N947_ENET_MAC_RX_TX_STATUS_OFFSET)
#define N947_ENET_MAC_VERSION           (N947_ENET0_BASE + N947_ENET_MAC_VERSION_OFFSET)
#define N947_ENET_MAC_DEBUG             (N947_ENET0_BASE + N947_ENET_MAC_DEBUG_OFFSET)
#define N947_ENET_MTL_TXQ0_DEBUG        (N947_ENET0_BASE + N947_ENET_MTL_TXQ0_DEBUG_OFFSET)
#define N947_ENET_MTL_RXQ0_DEBUG        (N947_ENET0_BASE + N947_ENET_MTL_RXQ0_DEBUG_OFFSET)
#define N947_ENET_DMA_DEBUG_STATUS0     (N947_ENET0_BASE + N947_ENET_DMA_DEBUG_STATUS0_OFFSET)
#define N947_ENET_MAC_HW_FEATURE0       (N947_ENET0_BASE + N947_ENET_MAC_HW_FEATURE0_OFFSET)
#define N947_ENET_MAC_MDIO_ADDRESS      (N947_ENET0_BASE + N947_ENET_MAC_MDIO_ADDRESS_OFFSET)
#define N947_ENET_MAC_MDIO_DATA         (N947_ENET0_BASE + N947_ENET_MAC_MDIO_DATA_OFFSET)
#define N947_ENET_MAC_ADDRESS0_HIGH     (N947_ENET0_BASE + N947_ENET_MAC_ADDRESS0_HIGH_OFFSET)
#define N947_ENET_MAC_ADDRESS0_LOW      (N947_ENET0_BASE + N947_ENET_MAC_ADDRESS0_LOW_OFFSET)
#define N947_ENET_MMC_CONTROL           (N947_ENET0_BASE + N947_ENET_MMC_CONTROL_OFFSET)
#define N947_ENET_MMC_RX_INTERRUPT_MASK (N947_ENET0_BASE + N947_ENET_MMC_RX_INTERRUPT_MASK_OFFSET)
#define N947_ENET_MMC_TX_INTERRUPT_MASK (N947_ENET0_BASE + N947_ENET_MMC_TX_INTERRUPT_MASK_OFFSET)
#define N947_ENET_MTL_TXQ0_OPERATION_MODE  (N947_ENET0_BASE + N947_ENET_MTL_TXQ0_OPERATION_MODE_OFFSET)
#define N947_ENET_MTL_RXQ0_OPERATION_MODE  (N947_ENET0_BASE + N947_ENET_MTL_RXQ0_OPERATION_MODE_OFFSET)
#define N947_ENET_DMA_MODE              (N947_ENET0_BASE + N947_ENET_DMA_MODE_OFFSET)
#define N947_ENET_DMA_SYSBUS_MODE       (N947_ENET0_BASE + N947_ENET_DMA_SYSBUS_MODE_OFFSET)
#define N947_ENET_DMA_INTERRUPT_STATUS  (N947_ENET0_BASE + N947_ENET_DMA_INTERRUPT_STATUS_OFFSET)
#define N947_ENET_DMA_CH0_CONTROL       (N947_ENET0_BASE + N947_ENET_DMA_CH0_CONTROL_OFFSET)
#define N947_ENET_DMA_CH0_TX_CONTROL    (N947_ENET0_BASE + N947_ENET_DMA_CH0_TX_CONTROL_OFFSET)
#define N947_ENET_DMA_CH0_RX_CONTROL    (N947_ENET0_BASE + N947_ENET_DMA_CH0_RX_CONTROL_OFFSET)
#define N947_ENET_DMA_CH0_TXDESC_LIST_ADDRESS  (N947_ENET0_BASE + N947_ENET_DMA_CH0_TXDESC_LIST_ADDRESS_OFFSET)
#define N947_ENET_DMA_CH0_RXDESC_LIST_ADDRESS  (N947_ENET0_BASE + N947_ENET_DMA_CH0_RXDESC_LIST_ADDRESS_OFFSET)
#define N947_ENET_DMA_CH0_TXDESC_TAIL_POINTER  (N947_ENET0_BASE + N947_ENET_DMA_CH0_TXDESC_TAIL_POINTER_OFFSET)
#define N947_ENET_DMA_CH0_RXDESC_TAIL_POINTER  (N947_ENET0_BASE + N947_ENET_DMA_CH0_RXDESC_TAIL_POINTER_OFFSET)
#define N947_ENET_DMA_CH0_TXDESC_RING_LENGTH   (N947_ENET0_BASE + N947_ENET_DMA_CH0_TXDESC_RING_LENGTH_OFFSET)
#define N947_ENET_DMA_CH0_RXDESC_RING_LENGTH   (N947_ENET0_BASE + N947_ENET_DMA_CH0_RXDESC_RING_LENGTH_OFFSET)
#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE     (N947_ENET0_BASE + N947_ENET_DMA_CH0_INTERRUPT_ENABLE_OFFSET)
#define N947_ENET_DMA_CH0_STATUS        (N947_ENET0_BASE + N947_ENET_DMA_CH0_STATUS_OFFSET)

/* MAC_CONFIGURATION bits */

#define N947_ENET_MAC_CONFIGURATION_RE      (1 << 0)  /* Receiver Enable */
#define N947_ENET_MAC_CONFIGURATION_TE      (1 << 1)  /* Transmitter Enable */
#define N947_ENET_MAC_CONFIGURATION_LM      (1 << 12) /* Loopback Mode */
#define N947_ENET_MAC_CONFIGURATION_DM      (1 << 13) /* Duplex Mode (1=full) */
#define N947_ENET_MAC_CONFIGURATION_FES     (1 << 14) /* Fast Ethernet Speed (1=100M) */
#define N947_ENET_MAC_CONFIGURATION_PS      (1 << 15) /* Port Select (1=10/100M MII/RMII) */
#define N947_ENET_MAC_CONFIGURATION_JD      (1 << 17) /* Jabber Disable */
#define N947_ENET_MAC_CONFIGURATION_WD      (1 << 19) /* Watchdog Disable */
#define N947_ENET_MAC_CONFIGURATION_ACS     (1 << 20) /* Automatic Pad/CRC Stripping */
#define N947_ENET_MAC_CONFIGURATION_CST     (1 << 21) /* CRC Stripping for Type Packets */
#define N947_ENET_MAC_CONFIGURATION_IPC     (1 << 27) /* Checksum Offload */

/* MAC_PACKET_FILTER bits */

#define N947_ENET_MAC_PACKET_FILTER_PR      (1 << 0)  /* Promiscuous Mode */
#define N947_ENET_MAC_PACKET_FILTER_HMC     (1 << 2)  /* Hash Multicast */
#define N947_ENET_MAC_PACKET_FILTER_PM      (1 << 4)  /* Pass All Multicast */
#define N947_ENET_MAC_PACKET_FILTER_DBF     (1 << 5)  /* Disable Broadcast Frames */
#define N947_ENET_MAC_PACKET_FILTER_HPF     (1 << 10) /* Hash or Perfect Filter */
#define N947_ENET_MAC_PACKET_FILTER_RA      (1 << 31) /* Receive All */

/* MAC_MDIO_ADDRESS bits */

#define N947_ENET_MAC_MDIO_ADDRESS_GB       (1 << 0)  /* GMII Busy */
#define N947_ENET_MAC_MDIO_ADDRESS_C45E     (1 << 1)  /* Clause 45 Enable */

/* GOC = 11b (read) or 01b (write) */

#define N947_ENET_MAC_MDIO_ADDRESS_GOC_READ  (0x3 << 2)
#define N947_ENET_MAC_MDIO_ADDRESS_GOC_WRITE (0x1 << 2)
#define N947_ENET_MAC_MDIO_ADDRESS_SKAP     (1 << 4)  /* Skip Address Packet */
#define N947_ENET_MAC_MDIO_ADDRESS_CR_SHIFT (8)
#define N947_ENET_MAC_MDIO_ADDRESS_CR_MASK  (0xf << N947_ENET_MAC_MDIO_ADDRESS_CR_SHIFT)
#define N947_ENET_MAC_MDIO_ADDRESS_CR(n)    (((n) << N947_ENET_MAC_MDIO_ADDRESS_CR_SHIFT) & N947_ENET_MAC_MDIO_ADDRESS_CR_MASK)
#define N947_ENET_MAC_MDIO_ADDRESS_NTC_SHIFT (12)
#define N947_ENET_MAC_MDIO_ADDRESS_NTC_MASK (0x7 << N947_ENET_MAC_MDIO_ADDRESS_NTC_SHIFT)
#define N947_ENET_MAC_MDIO_ADDRESS_RDA_SHIFT (16)      /* Register/Device Address */
#define N947_ENET_MAC_MDIO_ADDRESS_RDA_MASK (0x1f << N947_ENET_MAC_MDIO_ADDRESS_RDA_SHIFT)
#define N947_ENET_MAC_MDIO_ADDRESS_RDA(n)   (((n) << N947_ENET_MAC_MDIO_ADDRESS_RDA_SHIFT) & N947_ENET_MAC_MDIO_ADDRESS_RDA_MASK)
#define N947_ENET_MAC_MDIO_ADDRESS_PA_SHIFT (21)      /* Physical Layer Address */
#define N947_ENET_MAC_MDIO_ADDRESS_PA_MASK  (0x1f << N947_ENET_MAC_MDIO_ADDRESS_PA_SHIFT)
#define N947_ENET_MAC_MDIO_ADDRESS_PA(n)    (((n) << N947_ENET_MAC_MDIO_ADDRESS_PA_SHIFT) & N947_ENET_MAC_MDIO_ADDRESS_PA_MASK)
#define N947_ENET_MAC_MDIO_ADDRESS_BTB      (1 << 26) /* Back to Back transactions */
#define N947_ENET_MAC_MDIO_ADDRESS_PSE      (1 << 27) /* Preamble Suppression Enable */

/* MAC_MDIO_ADDRESS: Clock Range values (CR field)
 * For MDC freq close to 2.5MHz from various input clocks.
 * Use CR=4 for 150-250MHz range (covers most MCU frequencies).
 */

#define N947_ENET_MDIO_CR_60_100MHZ         (0)
#define N947_ENET_MDIO_CR_100_150MHZ        (1)
#define N947_ENET_MDIO_CR_20_35MHZ          (2)
#define N947_ENET_MDIO_CR_35_60MHZ          (3)
#define N947_ENET_MDIO_CR_150_250MHZ        (4)
#define N947_ENET_MDIO_CR_250_300MHZ        (5)

/* MAC_MDIO_DATA bits */

#define N947_ENET_MAC_MDIO_DATA_GD_MASK     (0xffff) /* GMII Data */
#define N947_ENET_MAC_MDIO_DATA_RA_SHIFT    (16)
#define N947_ENET_MAC_MDIO_DATA_RA_MASK     (0xffff << N947_ENET_MAC_MDIO_DATA_RA_SHIFT)

/* MMC_CONTROL bits */

#define N947_ENET_MMC_CONTROL_CNTRST        (1 << 0) /* Counter Reset */
#define N947_ENET_MMC_CONTROL_CNTSTOPRO     (1 << 1) /* Counter Stop Rollover */
#define N947_ENET_MMC_CONTROL_RSTONRD       (1 << 2) /* Reset on Read */
#define N947_ENET_MMC_CONTROL_CNTFREEZ      (1 << 3) /* MMC Counter Freeze */
#define N947_ENET_MMC_CONTROL_CNTPRST       (1 << 4) /* Full-Half Preset */
#define N947_ENET_MMC_CONTROL_CNTPRSTLVL    (1 << 5) /* Preset Level */
#define N947_ENET_MMC_CONTROL_UCDBC         (1 << 8) /* Update MMC Counters for Dropped Broadcast Frames */

/* MTL_TXQ0_OPERATION_MODE bits */

#define N947_ENET_MTL_TXQ0_OPERATION_MODE_FTQ      (1 << 0)  /* Flush Transmit Queue */
#define N947_ENET_MTL_TXQ0_OPERATION_MODE_TSF      (1 << 1)  /* Transmit Store and Forward */
#define N947_ENET_MTL_TXQ0_OPERATION_MODE_TXQEN_SHIFT (2)
#define N947_ENET_MTL_TXQ0_OPERATION_MODE_TXQEN_MASK  (0x3 << N947_ENET_MTL_TXQ0_OPERATION_MODE_TXQEN_SHIFT)
#define N947_ENET_MTL_TXQ0_OPERATION_MODE_TXQEN    (0x2 << N947_ENET_MTL_TXQ0_OPERATION_MODE_TXQEN_SHIFT)
#define N947_ENET_MTL_TXQ0_OPERATION_MODE_TTC_SHIFT (4)
#define N947_ENET_MTL_TXQ0_OPERATION_MODE_TTC_MASK  (0x7 << N947_ENET_MTL_TXQ0_OPERATION_MODE_TTC_SHIFT)
#define N947_ENET_MTL_TXQ0_OPERATION_MODE_TTC_64    (0x0 << N947_ENET_MTL_TXQ0_OPERATION_MODE_TTC_SHIFT)
#define N947_ENET_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT (16)
#define N947_ENET_MTL_TXQ0_OPERATION_MODE_TQS_MASK  (0x1ff << N947_ENET_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT)
#define N947_ENET_MTL_TXQ0_OPERATION_MODE_TQS(n)    (((n) << N947_ENET_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT) & N947_ENET_MTL_TXQ0_OPERATION_MODE_TQS_MASK)

/* MTL_RXQ0_OPERATION_MODE bits */

#define N947_ENET_MTL_RXQ0_OPERATION_MODE_RTC_SHIFT (0)
#define N947_ENET_MTL_RXQ0_OPERATION_MODE_RTC_MASK  (0x3 << N947_ENET_MTL_RXQ0_OPERATION_MODE_RTC_SHIFT)
#define N947_ENET_MTL_RXQ0_OPERATION_MODE_FUP       (1 << 3)  /* Forward Undersized Good Packets */
#define N947_ENET_MTL_RXQ0_OPERATION_MODE_FEP       (1 << 4)  /* Forward Error Packets */
#define N947_ENET_MTL_RXQ0_OPERATION_MODE_RSF       (1 << 5)  /* Receive Store and Forward */
#define N947_ENET_MTL_RXQ0_OPERATION_MODE_DIS_TCP_EF (1 << 6) /* Disable Dropping TCP/IP Checksum Error Frames */
#define N947_ENET_MTL_RXQ0_OPERATION_MODE_EHFC      (1 << 7)  /* Enable Hardware Flow Control */
#define N947_ENET_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT (20)
#define N947_ENET_MTL_RXQ0_OPERATION_MODE_RQS_MASK  (0x7ff << N947_ENET_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT)
#define N947_ENET_MTL_RXQ0_OPERATION_MODE_RQS(n)    (((n) << N947_ENET_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT) & N947_ENET_MTL_RXQ0_OPERATION_MODE_RQS_MASK)

/* MAC_RXQ_CTRL0 bits: enable RX queue 0 for DCB/Generic */

#define N947_ENET_MAC_RXQ_CTRL0_RXQ0EN_SHIFT (0)
#define N947_ENET_MAC_RXQ_CTRL0_RXQ0EN_MASK  (0x3 << N947_ENET_MAC_RXQ_CTRL0_RXQ0EN_SHIFT)
#define N947_ENET_MAC_RXQ_CTRL0_RXQ0EN_DCB   (0x2 << N947_ENET_MAC_RXQ_CTRL0_RXQ0EN_SHIFT)

/* DMA_MODE bits */

#define N947_ENET_DMA_MODE_SWR              (1 << 0) /* Software Reset */
#define N947_ENET_DMA_MODE_DA               (1 << 1) /* DMA Tx/Rx Arbitration */
#define N947_ENET_DMA_MODE_TAA_SHIFT        (2)
#define N947_ENET_DMA_MODE_TAA_MASK         (0x7 << N947_ENET_DMA_MODE_TAA_SHIFT)
#define N947_ENET_DMA_MODE_TXPR             (1 << 11) /* Transmit priority */
#define N947_ENET_DMA_MODE_PR_SHIFT         (12)
#define N947_ENET_DMA_MODE_PR_MASK          (0x7 << N947_ENET_DMA_MODE_PR_SHIFT)
#define N947_ENET_DMA_MODE_INTM_SHIFT       (16)
#define N947_ENET_DMA_MODE_INTM_MASK        (0x3 << N947_ENET_DMA_MODE_INTM_SHIFT)

/* DMA_SYSBUS_MODE bits */

#define N947_ENET_DMA_SYSBUS_MODE_FB        (1 << 0) /* Fixed Burst Length */
#define N947_ENET_DMA_SYSBUS_MODE_AAL      (1 << 12) /* Address-aligned */
#define N947_ENET_DMA_SYSBUS_MODE_ONEKBBE  (1 << 13) /* 1 KB Boundary Crossing Enable for DMA */

/* DMA_CH0_CONTROL bits */

#define N947_ENET_DMA_CH0_CONTROL_PBLx8     (1 << 16) /* PBL 8x */
#define N947_ENET_DMA_CH0_CONTROL_DSL_SHIFT (18)
#define N947_ENET_DMA_CH0_CONTROL_DSL_MASK  (0x7 << N947_ENET_DMA_CH0_CONTROL_DSL_SHIFT)
#define N947_ENET_DMA_CH0_CONTROL_DSL(n)    (((n) << N947_ENET_DMA_CH0_CONTROL_DSL_SHIFT) & N947_ENET_DMA_CH0_CONTROL_DSL_MASK)

/* DMA_CH0_TX_CONTROL bits */

#define N947_ENET_DMA_CH0_TX_CONTROL_ST     (1 << 0)  /* Start/Stop Transmission */
#define N947_ENET_DMA_CH0_TX_CONTROL_OSF    (1 << 4)  /* Operate on Second Frame */
#define N947_ENET_DMA_CH0_TX_CONTROL_TSE    (1 << 12) /* TCP Segmentation Enable */
#define N947_ENET_DMA_CH0_TX_CONTROL_TXPBL_SHIFT (16)
#define N947_ENET_DMA_CH0_TX_CONTROL_TXPBL_MASK  (0x3f << N947_ENET_DMA_CH0_TX_CONTROL_TXPBL_SHIFT)
#define N947_ENET_DMA_CH0_TX_CONTROL_TXPBL(n)    (((n) << N947_ENET_DMA_CH0_TX_CONTROL_TXPBL_SHIFT) & N947_ENET_DMA_CH0_TX_CONTROL_TXPBL_MASK)

/* DMA_CH0_RX_CONTROL bits */

#define N947_ENET_DMA_CH0_RX_CONTROL_SR     (1 << 0)  /* Start/Stop Receive */
#define N947_ENET_DMA_CH0_RX_CONTROL_RBSZ_SHIFT (1)
#define N947_ENET_DMA_CH0_RX_CONTROL_RBSZ_MASK  (0x3fff << N947_ENET_DMA_CH0_RX_CONTROL_RBSZ_SHIFT)
#define N947_ENET_DMA_CH0_RX_CONTROL_RBSZ(n)    (((n) << N947_ENET_DMA_CH0_RX_CONTROL_RBSZ_SHIFT) & N947_ENET_DMA_CH0_RX_CONTROL_RBSZ_MASK)
#define N947_ENET_DMA_CH0_RX_CONTROL_RXPBL_SHIFT (16)
#define N947_ENET_DMA_CH0_RX_CONTROL_RXPBL_MASK  (0x3f << N947_ENET_DMA_CH0_RX_CONTROL_RXPBL_SHIFT)
#define N947_ENET_DMA_CH0_RX_CONTROL_RXPBL(n)    (((n) << N947_ENET_DMA_CH0_RX_CONTROL_RXPBL_SHIFT) & N947_ENET_DMA_CH0_RX_CONTROL_RXPBL_MASK)
#define N947_ENET_DMA_CH0_RX_CONTROL_RPF         (1 << 31) /* Rx Packet Flush */

/* DMA_CH0_INTERRUPT_ENABLE bits */

#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE_TIE    (1 << 0)  /* Transmit Interrupt Enable */
#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE_TXSE   (1 << 1)  /* Transmit Stopped Enable */
#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE_TBUE   (1 << 2)  /* Transmit Buffer Unavailable Enable */
#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE_RIE    (1 << 6)  /* Receive Interrupt Enable */
#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE_RBUE   (1 << 7)  /* Receive Buffer Unavailable Enable */
#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE_RSE    (1 << 8)  /* Receive Stopped Enable */
#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE_RWTE   (1 << 9)  /* Receive Watchdog Timeout Enable */
#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE_ETIE   (1 << 10) /* Early Transmit Interrupt Enable */
#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE_ERIE   (1 << 11) /* Early Receive Interrupt Enable */
#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE_FBEE   (1 << 12) /* Fatal Bus Error Enable */
#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE_CDEE   (1 << 13) /* Context Descriptor Error Enable */
#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE_AIE    (1 << 14) /* Abnormal Interrupt Summary Enable */
#define N947_ENET_DMA_CH0_INTERRUPT_ENABLE_NIE    (1 << 15) /* Normal Interrupt Summary Enable */

/* DMA_CH0_STATUS bits */

#define N947_ENET_DMA_CH0_STATUS_TI     (1 << 0)  /* Transmit Interrupt */
#define N947_ENET_DMA_CH0_STATUS_TPS    (1 << 1)  /* Transmit Process Stopped */
#define N947_ENET_DMA_CH0_STATUS_TBU    (1 << 2)  /* Transmit Buffer Unavailable */
#define N947_ENET_DMA_CH0_STATUS_RI     (1 << 6)  /* Receive Interrupt */
#define N947_ENET_DMA_CH0_STATUS_RBU    (1 << 7)  /* Receive Buffer Unavailable */
#define N947_ENET_DMA_CH0_STATUS_RPS    (1 << 8)  /* Receive Process Stopped */
#define N947_ENET_DMA_CH0_STATUS_RWT    (1 << 9)  /* Receive Watchdog Timeout */
#define N947_ENET_DMA_CH0_STATUS_ETI    (1 << 10) /* Early Transmit Interrupt */
#define N947_ENET_DMA_CH0_STATUS_ERI    (1 << 11) /* Early Receive Interrupt */
#define N947_ENET_DMA_CH0_STATUS_FBE    (1 << 12) /* Fatal Bus Error */
#define N947_ENET_DMA_CH0_STATUS_CDE    (1 << 13) /* Context Descriptor Error */
#define N947_ENET_DMA_CH0_STATUS_AIS    (1 << 14) /* Abnormal Interrupt Summary */
#define N947_ENET_DMA_CH0_STATUS_NIS    (1 << 15) /* Normal Interrupt Summary */

/* TX descriptor word 2 (TDES2) bits */

#define N947_ENET_TDES2_B1L_SHIFT       (0)
#define N947_ENET_TDES2_B1L_MASK        (0x3fff << N947_ENET_TDES2_B1L_SHIFT)
#define N947_ENET_TDES2_B1L(n)          (((n) << N947_ENET_TDES2_B1L_SHIFT) & N947_ENET_TDES2_B1L_MASK)
#define N947_ENET_TDES2_B2L_SHIFT       (16)
#define N947_ENET_TDES2_B2L_MASK        (0x3fff << N947_ENET_TDES2_B2L_SHIFT)
#define N947_ENET_TDES2_B2L(n)          (((n) << N947_ENET_TDES2_B2L_SHIFT) & N947_ENET_TDES2_B2L_MASK)
#define N947_ENET_TDES2_TTSE            (1 << 30) /* Transmit Timestamp Enable */
#define N947_ENET_TDES2_IOC             (1 << 31) /* Interrupt on Completion */

/* TX descriptor word 3 (TDES3) bits */

#define N947_ENET_TDES3_FL_SHIFT        (0)
#define N947_ENET_TDES3_FL_MASK         (0x7fff << N947_ENET_TDES3_FL_SHIFT)
#define N947_ENET_TDES3_FL(n)           (((n) << N947_ENET_TDES3_FL_SHIFT) & N947_ENET_TDES3_FL_MASK)
#define N947_ENET_TDES3_CIC_SHIFT       (16)
#define N947_ENET_TDES3_CIC_MASK        (0x3 << N947_ENET_TDES3_CIC_SHIFT)
#define N947_ENET_TDES3_TSE             (1 << 18) /* TCP Segmentation Enable */
#define N947_ENET_TDES3_SLOT_SHIFT      (19)
#define N947_ENET_TDES3_SLOT_MASK       (0xf << N947_ENET_TDES3_SLOT_SHIFT)
#define N947_ENET_TDES3_SAIC_SHIFT      (23)
#define N947_ENET_TDES3_SAIC_MASK       (0x7 << N947_ENET_TDES3_SAIC_SHIFT)
#define N947_ENET_TDES3_CPC_SHIFT       (26)
#define N947_ENET_TDES3_CPC_MASK        (0x3 << N947_ENET_TDES3_CPC_SHIFT)
#define N947_ENET_TDES3_LD              (1 << 28) /* Last Descriptor */
#define N947_ENET_TDES3_FD              (1 << 29) /* First Descriptor */
#define N947_ENET_TDES3_CTXT            (1 << 30) /* Context Type */
#define N947_ENET_TDES3_OWN             (1 << 31) /* Own bit (1=DMA owns) */

/* RX descriptor word 3 (RDES3) bits (read format) */

#define N947_ENET_RDES3_BUF1V           (1 << 24) /* Buffer 1 Address Valid */
#define N947_ENET_RDES3_BUF2V           (1 << 25) /* Buffer 2 Address Valid */
#define N947_ENET_RDES3_IOC             (1 << 30) /* Interrupt Enabled on Completion */
#define N947_ENET_RDES3_OWN             (1 << 31) /* Own bit (1=DMA owns) */

/* RX descriptor word 3 (RDES3) bits (write-back format) */

#define N947_ENET_RDES3_PL_MASK         (0x7fff)  /* Packet Length */
#define N947_ENET_RDES3_ES              (1 << 15) /* Error Summary */
#define N947_ENET_RDES3_LD              (1 << 28) /* Last Descriptor */
#define N947_ENET_RDES3_FD              (1 << 29) /* First Descriptor */
#define N947_ENET_RDES3_CTXT            (1 << 30) /* Receive Context Descriptor */

/* OWN bit is same position */

/* RX descriptor word 3 error bits */

#define N947_ENET_RDES3_DE              (1 << 19) /* Descriptor Error */
#define N947_ENET_RDES3_RE              (1 << 20) /* Receive Error */
#define N947_ENET_RDES3_OE              (1 << 21) /* Overflow Error */
#define N947_ENET_RDES3_RWT             (1 << 22) /* Receive Watchdog Timeout */
#define N947_ENET_RDES3_GP              (1 << 23) /* Giant Packet */
#define N947_ENET_RDES3_CE              (1 << 24) /* CRC Error */

/* Normal/Abnormal interrupt summary enable bits */

#define N947_ENET_DMAINT_NORMAL    \
  (N947_ENET_DMA_CH0_INTERRUPT_ENABLE_TIE  | \
   N947_ENET_DMA_CH0_INTERRUPT_ENABLE_TBUE | \
   N947_ENET_DMA_CH0_INTERRUPT_ENABLE_RIE  | \
   N947_ENET_DMA_CH0_INTERRUPT_ENABLE_ERIE)

#define N947_ENET_DMAINT_ABNORMAL  \
  (N947_ENET_DMA_CH0_INTERRUPT_ENABLE_TXSE | \
   N947_ENET_DMA_CH0_INTERRUPT_ENABLE_RBUE | \
   N947_ENET_DMA_CH0_INTERRUPT_ENABLE_RSE  | \
   N947_ENET_DMA_CH0_INTERRUPT_ENABLE_RWTE | \
   N947_ENET_DMA_CH0_INTERRUPT_ENABLE_FBEE)

/* Descriptor sizes */

#define N947_ENET_DESC_SIZE            16   /* bytes per descriptor (4 x uint32_t) */

/* ETH frame size */

#define N947_ENET_FRAME_MAX_FRAMELEN   1518u
#define N947_ENET_BUFF_ALIGNMENT       4u

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_ENET_H */
