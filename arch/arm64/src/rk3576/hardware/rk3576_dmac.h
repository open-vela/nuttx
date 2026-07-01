/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_dmac.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_DMAC_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_DMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA channel count */

#define RK3576_DMA_CHANNELS        8

/* DMA register offsets (DesignWare DMA) */

#define DMA_RAW_INT                0x0000   /* Raw interrupt status */
#define DMA_INT                    0x0004   /* Interrupt status */
#define DMA_INT_ENABLE             0x0008   /* Interrupt enable */
#define DMA_INT_CLEAR              0x000c   /* Interrupt clear */

/* Channel registers */

#define DMA_CHEN                   0x0010   /* Channel enable */
#define DMA_CHSUSP                 0x0014   /* Channel suspend */
#define DMA_CHABORT                0x0018   /* Channel abort */
#define DMA_CH_STATUS              0x001c   /* Channel status */

/* Channel configuration registers */

#define DMA_CH_CFG(n)             (0x0040 + (n) * 0x20)
#define DMA_CH_CTL(n)             (0x0044 + (n) * 0x20)
#define DMA_CH_SRC_ADDR(n)        (0x0048 + (n) * 0x20)
#define DMA_CH_DST_ADDR(n)        (0x004c + (n) * 0x20)
#define DMA_CH_CTL2(n)            (0x0050 + (n) * 0x20)
#define DMA_CH_CTL3(n)            (0x0054 + (n) * 0x20)

/****************************************************************************
 * DMA_CHEN bit definitions
 ****************************************************************************/

#define DMA_CHEN_EN(ch)           (1 << (ch))          /* Channel enable */
#define DMA_CHEN_SUSP(ch)         (1 << ((ch) + 8))    /* Channel suspend */
#define DMA_CHEN_ABORT(ch)        (1 << ((ch) + 16))   /* Channel abort */

/****************************************************************************
 * DMA_INT bit definitions
 ****************************************************************************/

#define DMA_INT_TFR               (1 << 0)   /* Transfer complete */
#define DMA_INT_BLOCK             (1 << 1)   /* Block transfer */
#define DMA_INT_SRC_TRAN          (1 << 2)   /* Source transaction */
#define DMA_INT_DST_TRAN          (1 << 3)   /* Destination transaction */
#define DMA_INT_ERROR             (1 << 4)   /* Error */

/****************************************************************************
 * DMA_CH_CFG bit definitions
 ****************************************************************************/

#define DMA_CH_CFG_SRC_INC        (1 << 0)   /* Source increment */
#define DMA_CH_CFG_DST_INC        (1 << 1)   /* Destination increment */
#define DMA_CH_CFG_SRC_FIX        (0 << 0)   /* Source fixed */
#define DMA_CH_CFG_DST_FIX        (0 << 1)   /* Destination fixed */

#define DMA_CH_CFG_SRC_WIDTH_SHIFT 4
#define DMA_CH_CFG_SRC_WIDTH_MASK  (0x7 << DMA_CH_CFG_SRC_WIDTH_SHIFT)
#define DMA_CH_CFG_SRC_WIDTH_8    (0 << DMA_CH_CFG_SRC_WIDTH_SHIFT)
#define DMA_CH_CFG_SRC_WIDTH_16   (1 << DMA_CH_CFG_SRC_WIDTH_SHIFT)
#define DMA_CH_CFG_SRC_WIDTH_32   (2 << DMA_CH_CFG_SRC_WIDTH_SHIFT)

#define DMA_CH_CFG_DST_WIDTH_SHIFT 7
#define DMA_CH_CFG_DST_WIDTH_MASK  (0x7 << DMA_CH_CFG_DST_WIDTH_SHIFT)
#define DMA_CH_CFG_DST_WIDTH_8    (0 << DMA_CH_CFG_DST_WIDTH_SHIFT)
#define DMA_CH_CFG_DST_WIDTH_16   (1 << DMA_CH_CFG_DST_WIDTH_SHIFT)
#define DMA_CH_CFG_DST_WIDTH_32   (2 << DMA_CH_CFG_DST_WIDTH_SHIFT)

#define DMA_CH_CFG_SRC_BURST_SHIFT 10
#define DMA_CH_CFG_SRC_BURST_MASK  (0xf << DMA_CH_CFG_SRC_BURST_SHIFT)
#define DMA_CH_CFG_SRC_BURST_1    (0 << DMA_CH_CFG_SRC_BURST_SHIFT)
#define DMA_CH_CFG_SRC_BURST_4    (3 << DMA_CH_CFG_SRC_BURST_SHIFT)

#define DMA_CH_CFG_DST_BURST_SHIFT 14
#define DMA_CH_CFG_DST_BURST_MASK  (0xf << DMA_CH_CFG_DST_BURST_SHIFT)
#define DMA_CH_CFG_DST_BURST_1    (0 << DMA_CH_CFG_DST_BURST_SHIFT)
#define DMA_CH_CFG_DST_BURST_4    (3 << DMA_CH_CFG_DST_BURST_SHIFT)

#define DMA_CH_CFG_MODE_SHIFT     18
#define DMA_CH_CFG_MODE_MASK      (0x3 << DMA_CH_CFG_MODE_SHIFT)
#define DMA_CH_CFG_MODE_MEM2MEM   (0 << DMA_CH_CFG_MODE_SHIFT)
#define DMA_CH_CFG_MODE_MEM2PER   (1 << DMA_CH_CFG_MODE_SHIFT)
#define DMA_CH_CFG_MODE_PER2MEM   (2 << DMA_CH_CFG_MODE_SHIFT)
#define DMA_CH_CFG_MODE_PER2PER   (3 << DMA_CH_CFG_MODE_SHIFT)

/****************************************************************************
 * DMA_CH_CTL bit definitions
 ****************************************************************************/

#define DMA_CH_CTL_INT_EN         (1 << 0)
#define DMA_CH_CTL_TR_WIDTH_SHIFT 1
#define DMA_CH_CTL_TR_WIDTH_MASK  (0x7 << DMA_CH_CTL_TR_WIDTH_SHIFT)
#define DMA_CH_CTL_TR_WIDTH_8     (0 << DMA_CH_CTL_TR_WIDTH_SHIFT)
#define DMA_CH_CTL_TR_WIDTH_16    (1 << DMA_CH_CTL_TR_WIDTH_SHIFT)
#define DMA_CH_CTL_TR_WIDTH_32    (2 << DMA_CH_CTL_TR_WIDTH_SHIFT)

/****************************************************************************
 * DMA transfer sizes
 ****************************************************************************/

#define DMA_XFER_SIZE_8BIT        1
#define DMA_XFER_SIZE_16BIT       2
#define DMA_XFER_SIZE_32BIT       4

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

extern const uint32_t g_dmac_base;

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_DMAC_H */
