/****************************************************************************
 * arch/arm/src/t113/hardware/t113_spi.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_SPI_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "t113_ccu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI controller base addresses */

#define T113_SPI0_BASE   0x04025000
#define T113_SPI1_BASE   0x04026000

/* SPI register offsets */

#define SPI_VER_REG      0x0000
#define SPI_GCR_REG      0x0004
#define SPI_TCR_REG      0x0008
#define SPI_IER_REG      0x0010
#define SPI_ISR_REG      0x0014
#define SPI_FCR_REG      0x0018
#define SPI_FSR_REG      0x001c
#define SPI_WCR_REG      0x0020
#define SPI_CCR_REG      0x0024
#define SPI_SAMP_DL_REG  0x0028
#define SPI_SAMP_DL_CAL_START (1 << 15)
#define SPI_SAMP_DL_CAL_DONE  (1 << 14)
#define SPI_SAMP_DL_CAL_MASK  (0x3f << 8)
#define SPI_SAMP_DL_SW_EN     (1 << 7)
#define SPI_MBC_REG      0x0030
#define SPI_MTC_REG      0x0034
#define SPI_BCC_REG      0x0038
#define SPI_BATCR_REG    0x0040
#define SPI_BA_CCR_REG   0x0044
#define SPI_TBR_REG      0x0048
#define SPI_RBR_REG      0x004c
#define SPI_NDMA_CTL_REG 0x0088
#define SPI_TXD_REG      0x0200
#define SPI_RXD_REG      0x0300

/* SPI_GCR bits */

#define SPI_GCR_SRST     (1 << 31)
#define SPI_GCR_TP_EN    (1 << 7)
#define SPI_GCR_MODE_SEL (1 << 2)   /* Sample Timing Mode: 0=old, 1=new */
#define SPI_GCR_MASTER   (1 << 1)
#define SPI_GCR_EN       (1 << 0)

/* SPI_TCR bits */

#define SPI_TCR_XCH      (1 << 31)
#define SPI_TCR_SDC1     (1 << 15)  /* Sample Data Control 1: extra half-cycle delay */
#define SPI_TCR_SDDM     (1 << 14)  /* Sending Data Delay Mode */
#define SPI_TCR_SDM      (1 << 13)  /* Sample Data Mode: 1=normal, 0=delay */
#define SPI_TCR_FBS      (1 << 12)
#define SPI_TCR_SDC      (1 << 11)  /* Sample Data Control: 1=delay (high speed) */
#define SPI_TCR_RPSM     (1 << 10)  /* Rapid Mode Select for high speed write */
#define SPI_TCR_DDB      (1 << 9)   /* Dummy Burst Type: 0=zero, 1=one */
#define SPI_TCR_DHB      (1 << 8)   /* Discard Hash Burst: 1=RX only during dummy period */
#define SPI_TCR_SS_LEVEL (1 << 7)
#define SPI_TCR_SS_OWNER (1 << 6)
#define SPI_TCR_SS_SHIFT 4
#define SPI_TCR_SS_MASK  (0x3 << SPI_TCR_SS_SHIFT)
#define SPI_TCR_SS(n)    ((n) << SPI_TCR_SS_SHIFT)
#define SPI_TCR_SSCTL    (1 << 3)
#define SPI_TCR_SPOL     (1 << 2)
#define SPI_TCR_CPOL     (1 << 1)
#define SPI_TCR_CPHA     (1 << 0)

/* SPI_BCC bits */

#define SPI_BCC_QUAD_EN    (1 << 29)  /* Quad mode enable */
#define SPI_BCC_DUAL_EN    (1 << 28)  /* Dual mode enable */
#define SPI_BCC_DBC_SHIFT  24         /* Dummy Burst Counter [27:24] */
#define SPI_BCC_DBC_MASK   (0xf << 24)
#define SPI_BCC_DBC(n)     (((n) & 0xf) << 24)
#define SPI_BCC_STC_MASK   0x00ffffff /* Single Transfer Counter [23:0] */

/* SPI_FCR bits */

#define SPI_FCR_TX_FIFO_RST  (1 << 31)
#define SPI_FCR_TF_DRQ_EN    (1 << 24)
#define SPI_FCR_TX_TRIG_SHIFT 16
#define SPI_FCR_TX_TRIG(n)   ((n) << SPI_FCR_TX_TRIG_SHIFT)
#define SPI_FCR_RF_RST       (1 << 15)
#define SPI_FCR_RF_DRQ_EN    (1 << 8)
#define SPI_FCR_RX_TRIG_SHIFT 0
#define SPI_FCR_RX_TRIG(n)   ((n) << SPI_FCR_RX_TRIG_SHIFT)

/* SPI_FSR bits */

#define SPI_FSR_TB_WR     (1 << 31)
#define SPI_FSR_TF_CNT_SHIFT 16
#define SPI_FSR_TF_CNT_MASK  (0xff << SPI_FSR_TF_CNT_SHIFT)
#define SPI_FSR_RB_WR     (1 << 15)
#define SPI_FSR_RF_CNT_SHIFT 0
#define SPI_FSR_RF_CNT_MASK  (0xff)

/* SPI_IER / SPI_ISR bits */

#define SPI_INT_TC        (1 << 12)
#define SPI_INT_TF_UDF    (1 << 11)
#define SPI_INT_TF_OVF    (1 << 10)
#define SPI_INT_RF_UDF    (1 << 9)
#define SPI_INT_RF_OVF    (1 << 8)
#define SPI_INT_ERR       (SPI_INT_TF_UDF | SPI_INT_TF_OVF | \
                           SPI_INT_RF_UDF | SPI_INT_RF_OVF)

/* SPI FIFO depth */

#define SPI_FIFO_DEPTH    64

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_SPI_H */
