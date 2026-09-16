/****************************************************************************
 * arch/loongarch/src/ls2k0300/hardware/ls2k0300_gmac.h
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
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_GMAC_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_GMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GMAC Base Addresses */

#define LS2K0300_GMAC0_BASE          0x16020000
#define LS2K0300_GMAC1_BASE          0x16030000

/* GMAC Register Offsets (from MAC base) */

#define GMAC_CONFIG                 0x0000  /* MAC Configuration Register */
#define GMAC_FRAME_FILTER           0x0004  /* MAC Frame Filtering */
#define GMAC_HASH_HIGH              0x0008  /* Multicast Hash Table High */
#define GMAC_HASH_LOW               0x000C  /* Multicast Hash Table Low */
#define GMAC_GMII_ADDR              0x0010  /* GMII Address Register */
#define GMAC_GMII_DATA              0x0014  /* GMII Data Register */
#define GMAC_FLOW_CONTROL           0x0018  /* Flow Control Register */
#define GMAC_VLAN                   0x001C  /* VLAN Tag Register */
#define GMAC_VERSION                0x0020  /* GMAC Core Version */
#define GMAC_WAKEUP_ADDR            0x0028  /* Wake-up Frame Filter Address */
#define GMAC_PMT_CTRL_STATUS        0x002C  /* PMT Control and Status */
#define GMAC_INTERRUPT_STATUS       0x0038  /* MAC Interrupt Status */
#define GMAC_INTERRUPT_MASK         0x003C  /* MAC Interrupt Mask */

/* MAC Address Registers */

#define GMAC_ADDR0_HIGH             0x0040
#define GMAC_ADDR0_LOW              0x0044
#define GMAC_ADDR1_HIGH             0x0048
#define GMAC_ADDR1_LOW              0x004C
#define GMAC_ADDR2_HIGH             0x0050
#define GMAC_ADDR2_LOW              0x0054
#define GMAC_ADDR3_HIGH             0x0058
#define GMAC_ADDR3_LOW              0x005C

/* RGMII/SGMII Status */

#define GMAC_RGSMII_STATUS          0x00D8

/* DMA Register Offsets (from DMA base = MAC base + 0x1000) */

#define DMA_BUS_MODE                0x0000  /* CSR0 - Bus Mode */
#define DMA_TX_POLL_DEMAND          0x0004  /* CSR1 - Transmit Poll Demand */
#define DMA_RX_POLL_DEMAND          0x0008  /* CSR2 - Receive Poll Demand */
#define DMA_RX_BASE_ADDR            0x000C  /* CSR3 - Rx Descriptor List Base */
#define DMA_TX_BASE_ADDR            0x0010  /* CSR4 - Tx Descriptor List Base */
#define DMA_STATUS                  0x0014  /* CSR5 - Status Register */
#define DMA_CONTROL                 0x0018  /* CSR6 - Operation Mode */
#define DMA_INTERRUPT               0x001C  /* CSR7 - Interrupt Enable */
#define DMA_MISSED_FRAME            0x0020  /* CSR8 - Missed Frame Counter */
#define DMA_TX_CURR_DESC            0x0048  /* CSR18 - Current Tx Descriptor */
#define DMA_RX_CURR_DESC            0x004C  /* CSR19 - Current Rx Descriptor */
#define DMA_TX_CURR_ADDR            0x0050  /* CSR20 - Current Tx Buffer */
#define DMA_RX_CURR_ADDR            0x0054  /* CSR21 - Current Rx Buffer */
#define DMA_HW_FEATURE              0x0058  /* CSR22 - HW Feature */

/* GMAC Configuration Register Bits */

#define GMAC_CONFIG_TE              (1 << 3)   /* Transmitter Enable */
#define GMAC_CONFIG_RE              (1 << 2)   /* Receiver Enable */
#define GMAC_CONFIG_DM              (1 << 11)  /* Duplex Mode (1=Full) */
#define GMAC_CONFIG_FES             (1 << 14)  /* Speed (1=100Mbps, 0=10Mbps) */
#define GMAC_CONFIG_JD              (1 << 22)  /* Jabber Disable */
#define GMAC_CONFIG_WD              (1 << 23)  /* Watchdog Disable */

/* GMAC Frame Filter Register Bits */

#define GMAC_FRAME_FILTER_RA        (1 << 31)  /* Receive All */
#define GMAC_FRAME_FILTER_PR        (1 << 0)   /* Promiscuous Mode */
#define GMAC_FRAME_FILTER_PCF       (1 << 6)   /* Pass Control Frames */

/* GMII Address Register Bits */

#define GMAC_GMII_ADDR_PA_MASK      0x0000F800 /* PHY Address Mask */
#define GMAC_GMII_ADDR_PA_SHIFT     11
#define GMAC_GMII_ADDR_GR_MASK      0x000007C0 /* GMII Register Mask */
#define GMAC_GMII_ADDR_GR_SHIFT     6
#define GMAC_GMII_ADDR_CR_MASK      0x0000001C /* CSR Clock Mask */
#define GMAC_GMII_ADDR_CR_150_250   0x00000010 /* 150-250 MHz */
#define GMAC_GMII_ADDR_GW           (1 << 1)   /* Write */
#define GMAC_GMII_ADDR_GB           (1 << 0)   /* Busy */

/* DMA Bus Mode Register Bits */

#define DMA_BUS_MODE_SWR            (1 << 0)   /* Software Reset */
#define DMA_BUS_MODE_DA             (1 << 1)   /* DMA Arbitration (0=RR) */
#define DMA_BUS_MODE_DSL_MASK       0x0000007C /* Descriptor Skip Length */
#define DMA_BUS_MODE_PBL_MASK       0x00003F00 /* Programmable Burst Length */
#define DMA_BUS_MODE_PBL_32         0x00002000
#define DMA_BUS_MODE_FB             (1 << 16)  /* Fixed Burst */
#define DMA_BUS_MODE_MB             (1 << 26)  /* Mixed Burst */

/* DMA Status Register Bits */

#define DMA_STATUS_TI               (1 << 0)   /* Transmit Interrupt */
#define DMA_STATUS_TPS              (1 << 1)   /* Transmit Process Stopped */
#define DMA_STATUS_TU               (1 << 2)   /* Transmit Buffer Unavailable */
#define DMA_STATUS_TJT              (1 << 3)   /* Transmit Jabber Timeout */
#define DMA_STATUS_OVF              (1 << 4)   /* Receive Overflow */
#define DMA_STATUS_UNF              (1 << 5)   /* Transmit Underflow */
#define DMA_STATUS_RI               (1 << 6)   /* Receive Interrupt */
#define DMA_STATUS_RU               (1 << 7)   /* Receive Buffer Unavailable */
#define DMA_STATUS_RPS              (1 << 8)   /* Receive Process Stopped */
#define DMA_STATUS_RWT              (1 << 9)   /* Receive Watchdog Timeout */
#define DMA_STATUS_ETI              (1 << 10)  /* Early Transmit Interrupt */
#define DMA_STATUS_FBI              (1 << 13)  /* Fatal Bus Error */
#define DMA_STATUS_ERI              (1 << 14)  /* Early Receive Interrupt */
#define DMA_STATUS_AIE              (1 << 15)  /* Abnormal Interrupt */
#define DMA_STATUS_NIS              (1 << 16)  /* Normal Interrupt */

/* DMA Interrupt Enable Register Bits */

#define DMA_INT_EN_TIE              DMA_STATUS_TI
#define DMA_INT_EN_TSE              DMA_STATUS_TPS
#define DMA_INT_EN_TUE              DMA_STATUS_TU
#define DMA_INT_EN_TJE              DMA_STATUS_TJT
#define DMA_INT_EN_OVE              DMA_STATUS_OVF
#define DMA_INT_EN_UNE              DMA_STATUS_UNF
#define DMA_INT_EN_RIE              DMA_STATUS_RI
#define DMA_INT_EN_RUE              DMA_STATUS_RU
#define DMA_INT_EN_RSE              DMA_STATUS_RPS
#define DMA_INT_EN_RWE              DMA_STATUS_RWT
#define DMA_INT_EN_ETE              DMA_STATUS_ETI
#define DMA_INT_EN_FBE              DMA_STATUS_FBI
#define DMA_INT_EN_AIE              DMA_STATUS_AIE
#define DMA_INT_EN_NIE              DMA_STATUS_NIS

/* DMA Control Register Bits */

#define DMA_CONTROL_SR              (1 << 1)   /* Start/Stop Receive */
#define DMA_CONTROL_OSF             (1 << 2)   /* Operate on Second Frame */
#define DMA_CONTROL_ST              (1 << 13)  /* Start/Stop Transmit */
#define DMA_CONTROL_TTC_64          (0 << 14)  /* Tx Threshold 64 */
#define DMA_CONTROL_TTC_128         (1 << 14)  /* Tx Threshold 128 */
#define DMA_CONTROL_FTF             (1 << 20)  /* Flush Transmit FIFO */
#define DMA_CONTROL_TSF             (1 << 21)  /* Transmit Store and Forward */
#define DMA_CONTROL_RSF             (1 << 25)  /* Receive Store and Forward */

/* DMA Descriptor Status Bits */

#define DMA_DESC_OWN                (1 << 31)  /* Owned by DMA */
#define DMA_DESC_IC                 (1 << 30)  /* Interrupt on Completion */
#define DMA_DESC_LS                 (1 << 29)  /* Last Segment */
#define DMA_DESC_FS                 (1 << 28)  /* First Segment */
#define DMA_DESC_DC                 (1 << 27)  /* Disable CRC */
#define DMA_DESC_DP                 (1 << 26)  /* Disable Padding */
#define DMA_DESC_TER                (1 << 21)  /* Transmit End of Ring */
#define DMA_DESC_RER                (1 << 15)  /* Receive End of Ring */

/* DMA Descriptor Length Bits */

#define DMA_DESC_TBS1_MASK          0x00001FFF /* Buffer 1 Size Mask */
#define DMA_DESC_TBS2_MASK          0x1FFF0000 /* Buffer 2 Size Mask */
#define DMA_DESC_TBS2_SHIFT         16

/* RGMII Status Register Bits */

#define GMAC_RGSMII_LINK            (1 << 3)   /* Link Status */
#define GMAC_RGSMII_SPEED_MASK      0x00000006 /* Link Speed */
#define GMAC_RGSMII_SPEED_2_5       0x00000000 /* 2.5 MHz (10Mbps) */
#define GMAC_RGSMII_SPEED_25        0x00000002 /* 25 MHz (100Mbps) */
#define GMAC_RGSMII_SPEED_125       0x00000004 /* 125 MHz (1000Mbps) */
#define GMAC_RGSMII_DUPLEX          (1 << 0)   /* Duplex Mode */

/* PHY Registers */

#define PHY_BMCR                    0x00       /* Basic Mode Control */
#define PHY_BMSR                    0x01       /* Basic Mode Status */
#define PHY_ID1                     0x02       /* PHY Identifier 1 */
#define PHY_ID2                     0x03       /* PHY Identifier 2 */
#define PHY_ANAR                    0x04       /* Auto-Negotiation Advertisement */
#define PHY_ANLPAR                  0x05       /* Auto-Negotiation Link Partner */

/* PHY BMCR Bits */

#define PHY_BMCR_RESET              (1 << 15)  /* PHY Reset */
#define PHY_BMCR_LOOPBACK           (1 << 14)  /* Loopback */
#define PHY_BMCR_SPEED_100          (1 << 13)  /* Speed Select (1=100Mbps) */
#define PHY_BMCR_AN_ENABLE          (1 << 12)  /* Auto-Negotiation Enable */
#define PHY_BMCR_POWER_DOWN         (1 << 11)  /* Power Down */
#define PHY_BMCR_ISOLATE            (1 << 10)  /* Isolate */
#define PHY_BMCR_RESTART_AN         (1 << 9)   /* Restart Auto-Negotiation */
#define PHY_BMCR_DUPLEX             (1 << 8)   /* Duplex Mode (1=Full) */

/* PHY BMSR Bits */

#define PHY_BMSR_100BASE_T4         (1 << 9)   /* 100BASE-T4 capable */
#define PHY_BMSR_100BASE_TX_FD      (1 << 8)   /* 100BASE-TX Full Duplex */
#define PHY_BMSR_100BASE_TX_HD      (1 << 7)   /* 100BASE-TX Half Duplex */
#define PHY_BMSR_10BASE_T_FD        (1 << 6)   /* 10BASE-T Full Duplex */
#define PHY_BMSR_10BASE_T_HD        (1 << 5)   /* 10BASE-T Half Duplex */
#define PHY_BMSR_AN_COMPLETE        (1 << 5)   /* Auto-Negotiation Complete */
#define PHY_BMSR_AN_CAPABLE         (1 << 3)   /* Auto-Negotiation Capable */
#define PHY_BMSR_LINK               (1 << 2)   /* Link Status */

/* Descriptor Ring Configuration */

#define LS2K0300_TX_DESC_NUM         16
#define LS2K0300_RX_DESC_NUM         16
#define LS2K0300_TX_BUF_SIZE         2048
#define LS2K0300_RX_BUF_SIZE         2048

/* Default PHY Address */

#define LS2K0300_DEFAULT_PHY_ADDR    0

#endif /* __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_GMAC_H */
