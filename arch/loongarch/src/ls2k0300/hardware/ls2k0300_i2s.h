/****************************************************************************
 * arch/loongarch/src/ls2k0300/hardware/ls2k0300_i2s.h
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

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_I2S_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_I2S_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/ls2k0300_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2S controller.  Address space 0x16114000 - 0x16117fff (16KB) */

#define LS2K0300_I2S_BASE            PHYS_TO_UNCACHED(0x16114000)

/* Register offsets *********************************************************/

#define IIS_VERSION_OFFSET           0x0000  /* IISVersion */
#define IIS_CONFIG0_OFFSET           0x0004  /* IISConfig0 */
#define IIS_CONTROL_OFFSET           0x0008  /* IISControl */
#define IIS_RXDATA_OFFSET            0x000c  /* IISRxData */
#define IIS_TXDATA_OFFSET            0x0010  /* IISTxData */
#define IIS_CONFIG1_OFFSET           0x0014  /* IISConfig1 */

/* IISVersion ***************************************************************/

#define IIS_VERSION_ADRW_SHIFT       8       /* Address bus width */
#define IIS_VERSION_DATW_SHIFT       4       /* Data bus width */

/* IISConfig0 ***************************************************************/

#define IIS_CONFIG0_LR_LEN_SHIFT     24      /* L/R channel word length */
#define IIS_CONFIG0_TX_DEPTH_SHIFT   16      /* TX sample depth */
#define IIS_CONFIG0_BCLK_RATIO_SHIFT 8       /* BCLK divider */
#define IIS_CONFIG0_RX_DEPTH_SHIFT   0       /* RX sample depth */

/* IISControl ***************************************************************/

#define IIS_CONTROL_MCLK_READY       (1 << 16) /* MCLK output stable (RO) */
#define IIS_CONTROL_MASTER           (1 << 15) /* 1: IIS is master */
#define IIS_CONTROL_MSB_LSB          (1 << 14) /* 1: MSB first */
#define IIS_CONTROL_RX_EN            (1 << 13) /* Receiver enable */
#define IIS_CONTROL_TX_EN            (1 << 12) /* Transmitter enable */
#define IIS_CONTROL_RX_DMA_EN        (1 << 11) /* DMA receive enable */
#define IIS_CONTROL_CLK_READY        (1 << 8)  /* BCLK/LRCLK stable (RO) */
#define IIS_CONTROL_TX_DMA_EN        (1 << 7)  /* DMA transmit enable */
#define IIS_CONTROL_RESETN           (1 << 4)  /* Controller soft reset */
#define IIS_CONTROL_MCLK_EN          (1 << 3)  /* MCLK output enable */
#define IIS_CONTROL_RX_INT_EN        (1 << 1)  /* RX interrupt enable */
#define IIS_CONTROL_TX_INT_EN        (1 << 0)  /* TX interrupt enable */

/* IISConfig1 ***************************************************************/

#define IIS_CONFIG1_MCLK_FRAC_SHIFT  16      /* MCLK divider fraction */
#define IIS_CONFIG1_MCLK_RATIO_SHIFT 0       /* MCLK divider integer part */

#endif /* __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_I2S_H */
