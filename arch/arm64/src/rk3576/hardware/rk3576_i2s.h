/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_i2s.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_I2S_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_I2S_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2S controller count */

#define RK3576_I2S_COUNT            3   /* I2S0 ~ I2S2 */

/* I2S register offsets */

#define I2S_TXCR                    0x00   /* TX config register */
#define I2S_RXCR                    0x04   /* RX config register */
#define I2S_CKR                     0x08   /* Clock configuration */
#define I2S_TXFIFOLR                0x0c   /* TX FIFO level */
#define I2S_RXFIFOLR                0x10   /* RX FIFO level */
#define I2S_SR                      0x14   /* Status register */
#define I2S_IMR                     0x18   /* Interrupt mask */
#define I2S_ISR                     0x1c   /* Interrupt status */
#define I2S_RISR                    0x20   /* Raw interrupt status */
#define I2S_ICR                     0x24   /* Interrupt clear */
#define I2S_DMACR                   0x28   /* DMA control */
#define I2S_TXDR                    0x2c   /* TX data (W) */
#define I2S_RXDR                    0x30   /* RX data (R) */
#define I2S_CLR_SVR                 0x34   /* Channel status clear */
#define I2S_CLR_TX                  0x38   /* TX clear */
#define I2S_CLR_RX                  0x3c   /* RX clear */
#define I2S_TXCH_REG(n)            (0x50 + ((n) * 4))  /* TX channel n */
#define I2S_RXCH_REG(n)            (0x70 + ((n) * 4))  /* RX channel n */

/* I2S_TXCR bit definitions */

#define I2S_TXCR_TXCKS_SHIFT       0
#define I2S_TXCR_TXCKS_MASK        (3 << I2S_TXCR_TXCKS_SHIFT)
#define I2S_TXCR_TXCKS_STEREO      (0 << I2S_TXCR_TXCKS_SHIFT)
#define I2S_TXCR_TXCKS_QUAD        (1 << I2S_TXCR_TXCKS_SHIFT)
#define I2S_TXCR_TXCKS_5_1         (2 << I2S_TXCR_TXCKS_SHIFT)
#define I2S_TXCR_TXCKS_7_1         (3 << I2S_TXCR_TXCKS_SHIFT)

#define I2S_TXCR_VDJ_POL_SHIFT     2
#define I2S_TXCR_VDJ_POL_MASK      (3 << I2S_TXCR_VDJ_POL_SHIFT)

#define I2S_TXCR_FBM_SHIFT         6
#define I2S_TXCR_FBM_MASK          (3 << I2S_TXCR_FBM_SHIFT)

#define I2S_TXCR_TRC_SHIFT         8
#define I2S_TXCR_TRC_MASK          (7 << I2S_TXCR_TRC_SHIFT)
#define I2S_TXCR_TRC_16            (0 << I2S_TXCR_TRC_SHIFT)
#define I2S_TXCR_TRC_20            (1 << I2S_TXCR_TRC_SHIFT)
#define I2S_TXCR_TRC_24            (2 << I2S_TXCR_TRC_SHIFT)
#define I2S_TXCR_TRC_32            (3 << I2S_TXCR_TRC_SHIFT)

#define I2S_TXCR_DFS_SHIFT         12
#define I2S_TXCR_DFS_MASK          (3 << I2S_TXCR_DFS_SHIFT)
#define I2S_TXCR_DFS_16            (0 << I2S_TXCR_DFS_SHIFT)
#define I2S_TXCR_DFS_20            (1 << I2S_TXCR_DFS_SHIFT)
#define I2S_TXCR_DFS_24            (2 << I2S_TXCR_DFS_SHIFT)
#define I2S_TXCR_DFS_32            (3 << I2S_TXCR_DFS_SHIFT)

#define I2S_TXCR_STR_SHIFT         16
#define I2S_TXCR_STR_MASK          (3 << I2S_TXCR_STR_SHIFT)
#define I2S_TXCR_STR_STEREO        (0 << I2S_TXCR_STR_SHIFT)
#define I2S_TXCR_STR_MONO          (1 << I2S_TXCR_STR_SHIFT)

#define I2S_TXCR_TXEN              (1 << 18)
#define I2S_TXCR_TXHALF            (1 << 19)

/* I2S_RXCR bit definitions (mirror of TXCR) */

#define I2S_RXCR_RXCKS_SHIFT       0
#define I2S_RXCR_RXCKS_MASK        (3 << I2S_RXCR_RXCKS_SHIFT)
#define I2S_RXCR_TRC_SHIFT         8
#define I2S_RXCR_TRC_MASK          (7 << I2S_RXCR_TRC_SHIFT)
#define I2S_RXCR_DFS_SHIFT         12
#define I2S_RXCR_DFS_MASK          (3 << I2S_RXCR_DFS_SHIFT)
#define I2S_RXCR_STR_SHIFT         16
#define I2S_RXCR_STR_MASK          (3 << I2S_RXCR_STR_SHIFT)
#define I2S_RXCR_STR_STEREO        (0 << I2S_RXCR_STR_SHIFT)
#define I2S_RXCR_STR_MONO          (1 << I2S_RXCR_STR_SHIFT)
#define I2S_RXCR_RXEN              (1 << 18)
#define I2S_RXCR_RXHALF            (1 << 19)

/* I2S_CKR bit definitions */

#define I2S_CKR_SCLKG_SHIFT        0
#define I2S_CKR_SCLKG_MASK         (3 << I2S_CKR_SCLKG_SHIFT)
#define I2S_CKR_SCLKG_NODELAY      (0 << I2S_CKR_SCLKG_SHIFT)
#define I2S_CKR_SCLKG_128          (1 << I2S_CKR_SCLKG_SHIFT)
#define I2S_CKR_SCLKG_256          (2 << I2S_CKR_SCLKG_SHIFT)
#define I2S_CKR_SCLKG_512          (3 << I2S_CKR_SCLKG_SHIFT)

#define I2S_CKR_RXM_SHIFT          4
#define I2S_CKR_RXM_MASK           (3 << I2S_CKR_RXM_SHIFT)
#define I2S_CKR_RXM_MASTER         (0 << I2S_CKR_RXM_SHIFT)
#define I2S_CKR_RXM_SLAVE          (1 << I2S_CKR_RXM_SHIFT)

#define I2S_CKR_TXM_SHIFT          6
#define I2S_CKR_TXM_MASK           (3 << I2S_CKR_TXM_SHIFT)
#define I2S_CKR_TXM_MASTER         (0 << I2S_CKR_TXM_SHIFT)
#define I2S_CKR_TXM_SLAVE          (1 << I2S_CKR_TXM_SHIFT)

#define I2S_CKR_LRCKP_SHIFT        8
#define I2S_CKR_LRCKP_MASK         (3 << I2S_CKR_LRCKP_SHIFT)

#define I2S_CKR_MSS_SHIFT          12
#define I2S_CKR_MSS_MASK           (3 << I2S_CKR_MSS_SHIFT)
#define I2S_CKR_MSS_MASTER         (0 << I2S_CKR_MSS_SHIFT)
#define I2S_CKR_MSS_SLAVE          (1 << I2S_CKR_MSS_SHIFT)

/* I2S_SR bit definitions */

#define I2S_SR_TXBUSY              (1 << 0)
#define I2S_SR_RXBUSY              (1 << 1)

/* I2S_IMR bit definitions */

#define I2S_IMR_TXFIFOIE           (1 << 0)
#define I2S_IMR_RXFIFOIE           (1 << 1)

/* I2S_ISR bit definitions */

#define I2S_ISR_TXFIFO             (1 << 0)
#define I2S_ISR_RXFIFO             (1 << 1)

/* I2S_DMACR bit definitions */

#define I2S_DMACR_TXDMAE           (1 << 0)
#define I2S_DMACR_RXDMAE           (1 << 1)
#define I2S_DMACR_TXDT_SHIFT       8
#define I2S_DMACR_TXDT_MASK        (0x1f << I2S_DMACR_TXDT_SHIFT)
#define I2S_DMACR_RXDT_SHIFT       16
#define I2S_DMACR_RXDT_MASK        (0x1f << I2S_DMACR_RXDT_SHIFT)

/* I2S TX channel registers */

#define I2S_TXCH_REG_0             0x50
#define I2S_TXCH_REG_1             0x54
#define I2S_TXCH_REG_2             0x58
#define I2S_TXCH_REG_3             0x5c

/* I2S RX channel registers */

#define I2S_RXCH_REG_0             0x70
#define I2S_RXCH_REG_1             0x74
#define I2S_RXCH_REG_2             0x78
#define I2S_RXCH_REG_3             0x7c

/* I2S FIFO depth */

#define I2S_FIFO_DEPTH             64

/* I2S clock frequencies */

#define I2S_FREQ_8KHZ              8000
#define I2S_FREQ_16KHZ             16000
#define I2S_FREQ_44KHZ             44100
#define I2S_FREQ_48KHZ             48000
#define I2S_FREQ_96KHZ             96000

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

extern const uint32_t g_i2s_base[RK3576_I2S_COUNT];

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_I2S_H */
