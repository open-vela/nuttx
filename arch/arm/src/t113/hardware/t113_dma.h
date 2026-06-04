/****************************************************************************
 * arch/arm/src/t113/hardware/t113_dma.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_DMA_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "t113_ccu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define T113_DMAC_BASE        0x03002000
#define T113_DMAC_NCHANNELS   16

/* Global registers */

#define T113_DMAC_IRQ_EN0     (T113_DMAC_BASE + 0x0000)
#define T113_DMAC_IRQ_EN1     (T113_DMAC_BASE + 0x0004)
#define T113_DMAC_IRQ_PEND0   (T113_DMAC_BASE + 0x0010)
#define T113_DMAC_IRQ_PEND1   (T113_DMAC_BASE + 0x0014)
#define T113_DMAC_AUTO_GATE   (T113_DMAC_BASE + 0x0028)
#define T113_DMAC_STATUS      (T113_DMAC_BASE + 0x0030)

/* Per-channel registers: base + 0x100 + channel * 0x40 */

#define T113_DMAC_CHAN(n)     (T113_DMAC_BASE + 0x0100 + (n) * 0x0040)
#define T113_DMAC_EN(n)       (T113_DMAC_CHAN(n) + 0x0000)
#define T113_DMAC_PAU(n)      (T113_DMAC_CHAN(n) + 0x0004)
#define T113_DMAC_DESC(n)     (T113_DMAC_CHAN(n) + 0x0008)
#define T113_DMAC_CFG(n)      (T113_DMAC_CHAN(n) + 0x000c)
#define T113_DMAC_SRC(n)      (T113_DMAC_CHAN(n) + 0x0010)
#define T113_DMAC_DST(n)      (T113_DMAC_CHAN(n) + 0x0014)
#define T113_DMAC_CNT(n)      (T113_DMAC_CHAN(n) + 0x0018)
#define T113_DMAC_PARA(n)     (T113_DMAC_CHAN(n) + 0x001c)
#define T113_DMAC_MODE(n)     (T113_DMAC_CHAN(n) + 0x0028)
#define T113_DMAC_FDESC(n)    (T113_DMAC_CHAN(n) + 0x002c)
#define T113_DMAC_PKG(n)      (T113_DMAC_CHAN(n) + 0x0030)

/* DMAC_CFG bit fields */

#define DMAC_CFG_SRC_DRQ_SHIFT    0
#define DMAC_CFG_SRC_DRQ_MASK     (0x3f << DMAC_CFG_SRC_DRQ_SHIFT)
#define DMAC_CFG_SRC_ADDR_SHIFT   8
#define DMAC_CFG_SRC_ADDR_LINEAR  (0 << DMAC_CFG_SRC_ADDR_SHIFT)
#define DMAC_CFG_SRC_ADDR_IO      (1 << DMAC_CFG_SRC_ADDR_SHIFT)
#define DMAC_CFG_SRC_BURST_SHIFT  6
#define DMAC_CFG_SRC_BURST(n)     ((n) << DMAC_CFG_SRC_BURST_SHIFT)
#define DMAC_CFG_SRC_WIDTH_SHIFT  9
#define DMAC_CFG_SRC_WIDTH(n)     ((n) << DMAC_CFG_SRC_WIDTH_SHIFT)
#define DMAC_CFG_DST_DRQ_SHIFT    16
#define DMAC_CFG_DST_DRQ_MASK     (0x3f << DMAC_CFG_DST_DRQ_SHIFT)
#define DMAC_CFG_DST_ADDR_SHIFT   24
#define DMAC_CFG_DST_ADDR_LINEAR  (0 << DMAC_CFG_DST_ADDR_SHIFT)
#define DMAC_CFG_DST_ADDR_IO      (1 << DMAC_CFG_DST_ADDR_SHIFT)
#define DMAC_CFG_DST_BURST_SHIFT  22
#define DMAC_CFG_DST_BURST(n)     ((n) << DMAC_CFG_DST_BURST_SHIFT)
#define DMAC_CFG_DST_WIDTH_SHIFT  25
#define DMAC_CFG_DST_WIDTH(n)     ((n) << DMAC_CFG_DST_WIDTH_SHIFT)
#define DMAC_CFG_BMODE_SEL        (1 << 30)

/* Data width values for DMAC_CFG_SRC_WIDTH / DST_WIDTH */

#define DMAC_WIDTH_8BIT           0
#define DMAC_WIDTH_16BIT          1
#define DMAC_WIDTH_32BIT          2
#define DMAC_WIDTH_64BIT          3

/* Burst length values */

#define DMAC_BURST_1              0
#define DMAC_BURST_4              1
#define DMAC_BURST_8              2
#define DMAC_BURST_16             3

/* IRQ status bits per channel (in PEND0/PEND1 and EN0/EN1) */

#define DMAC_IRQ_HLFDONE(n)       (1 << (((n) & 7) * 4 + 0))
#define DMAC_IRQ_PKGDONE(n)       (1 << (((n) & 7) * 4 + 1))
#define DMAC_IRQ_QDONE(n)         (1 << (((n) & 7) * 4 + 2))

/* DRQ port assignments (T113 manual Table 3-13) */

#define DRQ_SRAM     0
#define DRQ_DRAM     1
#define DRQ_OWA_RX   2
#define DRQ_OWA_TX   2
#define DRQ_I2S1_RX  4
#define DRQ_I2S1_TX  4
#define DRQ_I2S2_RX  5
#define DRQ_I2S2_TX  5
#define DRQ_CODEC    7
#define DRQ_DMIC     8
#define DRQ_UART0_RX 14
#define DRQ_UART0_TX 14
#define DRQ_UART1_RX 15
#define DRQ_UART1_TX 15
#define DRQ_UART2_RX 16
#define DRQ_UART2_TX 16
#define DRQ_UART3_RX 17
#define DRQ_UART3_TX 17
#define DRQ_UART4_RX 18
#define DRQ_UART4_TX 18
#define DRQ_UART5_RX 19
#define DRQ_UART5_TX 19
#define DRQ_SPI0_RX  22
#define DRQ_SPI0_TX  22
#define DRQ_SPI1_RX  23
#define DRQ_SPI1_TX  23
#define DRQ_USB0_EP1 30
#define DRQ_USB0_EP2 31
#define DRQ_USB0_EP3 32
#define DRQ_USB0_EP4 33
#define DRQ_USB0_EP5 34
#define DRQ_TWI0     43
#define DRQ_TWI1     44
#define DRQ_TWI2     45
#define DRQ_TWI3     46

/* Descriptor link: end of chain marker */

#define DMAC_DESC_END          0xfffff800

/* Maximum byte length programmable into DMAC_CNT/desc.len (25 bits) */

#define DMAC_MAX_XFER_LEN      0x1ffffff

/* Descriptor PARA field: normal wait cycles (8 between packets) */

#define DMAC_PARA_NORMAL_WAIT  (8 << 0)

/* DMAC_MODE_REGN bits (per-channel DMA handshake mode) */

#define DMAC_MODE_DST_SHIFT       3
#define DMAC_MODE_DST_WAIT        (0 << DMAC_MODE_DST_SHIFT)
#define DMAC_MODE_DST_HANDSHAKE   (1 << DMAC_MODE_DST_SHIFT)
#define DMAC_MODE_SRC_SHIFT       2
#define DMAC_MODE_SRC_WAIT        (0 << DMAC_MODE_SRC_SHIFT)
#define DMAC_MODE_SRC_HANDSHAKE   (1 << DMAC_MODE_SRC_SHIFT)

/* DMAC_EN_REGN bits */

#define DMAC_CHAN_ENABLE  (1 << 0)

/* DMAC_PAU_REGN bits */

#define DMAC_CHAN_PAUSE   (1 << 0)

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_DMA_H */
