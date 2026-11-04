/****************************************************************************
 * arch/loongarch/src/ls2k0300/hardware/ls2k0300_lsia_dma.h
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

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_LSIA_DMA_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_LSIA_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/ls2k0300_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LSIA DMA controller.  8 channels, register per transfer (no descriptor
 * hardware).  Each channel owns a 0x14 byte register block.
 */

#define LS2K0300_LSIA_DMA_BASE       PHYS_TO_UNCACHED(0x1612c000)

#define LSIA_DMA_MAX_CHANNELS        8
#define LSIA_DMA_CHANNEL_STRIDE      0x14

/* Global registers *********************************************************/

#define LSIA_DMA_ISR                 0x0000  /* DMA interrupt status */
#define LSIA_DMA_IFCR                0x0004  /* DMA interrupt flag clear */

/* Per channel registers ****************************************************/

#define LSIA_DMA_CCR(ch)             (0x0008 + LSIA_DMA_CHANNEL_STRIDE * (ch))
#define LSIA_DMA_CNDTR(ch)           (0x000c + LSIA_DMA_CHANNEL_STRIDE * (ch))
#define LSIA_DMA_CPAR(ch)            (0x0010 + LSIA_DMA_CHANNEL_STRIDE * (ch))
#define LSIA_DMA_CMAR(ch)            (0x0014 + LSIA_DMA_CHANNEL_STRIDE * (ch))

/* DMA_CCRx bits ************************************************************/

#define LSIA_DMA_CCR_MEM2MEM         (1 << 14) /* Memory to memory mode */
#define LSIA_DMA_CCR_PL_MASK         (3 << 12) /* Channel priority */
#define LSIA_DMA_CCR_PL(n)           ((n) << 12)
#define LSIA_DMA_CCR_MSIZE_MASK      (3 << 10) /* Memory data width */
#define LSIA_DMA_CCR_MSIZE(n)        ((n) << 10)
#define LSIA_DMA_CCR_PSIZE_MASK      (3 << 8)  /* Peripheral data width */
#define LSIA_DMA_CCR_PSIZE(n)        ((n) << 8)
#define LSIA_DMA_CCR_MINC            (1 << 7)  /* Memory increment */
#define LSIA_DMA_CCR_PINC            (1 << 6)  /* Peripheral increment */
#define LSIA_DMA_CCR_CIRC            (1 << 5)  /* Circular mode */
#define LSIA_DMA_CCR_DIR             (1 << 4)  /* 0: from peripheral, 1: from memory */
#define LSIA_DMA_CCR_TEIE            (1 << 3)  /* Transfer error IRQ enable */
#define LSIA_DMA_CCR_HTIE            (1 << 2)  /* Half transfer IRQ enable */
#define LSIA_DMA_CCR_TCIE            (1 << 1)  /* Transfer complete IRQ enable */
#define LSIA_DMA_CCR_EN              (1 << 0)  /* Channel enable */

/* Data width encoding (identical for MSIZE and PSIZE) **********************/

#define LSIA_DMA_WIDTH_8BIT          0
#define LSIA_DMA_WIDTH_16BIT         1
#define LSIA_DMA_WIDTH_32BIT         2

/* Interrupt flags.  The DMA_ISR/DMA_IFCR registers carry four status bits
 * per channel, channel x occupying bits 4*x + 3 ... 4*x.
 */

#define LSIA_DMA_TCI                 (1 << 1)  /* Transfer complete */
#define LSIA_DMA_HTI                 (1 << 2)  /* Half transfer */
#define LSIA_DMA_TEI                 (1 << 3)  /* Transfer error */

#endif /* __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_LSIA_DMA_H */
