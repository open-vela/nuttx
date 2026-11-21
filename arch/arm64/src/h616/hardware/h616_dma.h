/****************************************************************************
 * arch/arm64/src/h616/hardware/h616_dma.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
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

#ifndef __ARCH_ARM64_SRC_H616_HARDWARE_H616_DMA_H
#define __ARCH_ARM64_SRC_H616_HARDWARE_H616_DMA_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMAC registers (base 0x03002000), sun6i/A100 compatible layout
 * (Linux drivers/dma/sun6i-dma.c, compatible "allwinner,sun50i-h616-dma")
 */

#define H616_DMAC_IRQ_EN0             0x0000  /* IRQ enable, channels 0-7  */
#define H616_DMAC_IRQ_EN1             0x0004  /* IRQ enable, channels 8-15 */
#define H616_DMAC_IRQ_STAT0           0x0010  /* IRQ status, channels 0-7  */
#define H616_DMAC_IRQ_STAT1           0x0014  /* IRQ status, channels 8-15 */

#define H616_DMAC_GATE                0x0028  /* clock auto-gating (H3 style) */
#define H616_DMAC_GATE_ENABLE         0x4

#define H616_DMAC_CH_BASE             0x0100  /* channel register block base */
#define H616_DMAC_CH_STRIDE           0x0040  /* per-channel stride */

#define H616_DMAC_CH_ENABLE           0x0000  /* (offset within block) */
#define H616_DMAC_CH_ENABLE_START     (1u << 0)
#define H616_DMAC_CH_ENABLE_STOP      0

#define H616_DMAC_CH_PAUSE            0x0004
#define H616_DMAC_CH_PAUSE_PAUSE      (1u << 1)
#define H616_DMAC_CH_PAUSE_RESUME     0

#define H616_DMAC_CH_LLI_ADDR         0x0008  /* phys addr of first LLI */
#define H616_DMAC_CH_CUR_CFG          0x000c
#define H616_DMAC_CH_CUR_SRC          0x0010
#define H616_DMAC_CH_CUR_DST          0x0014
#define H616_DMAC_CH_CUR_CNT          0x0018  /* remaining byte count */
#define H616_DMAC_CH_CUR_PARA         0x001c

/* Per-channel IRQ enable/status bits (4 bits per channel) */

#define H616_DMAC_IRQ_HALF            (1u << 0)
#define H616_DMAC_IRQ_PKG             (1u << 1)  /* LLI finished */
#define H616_DMAC_IRQ_QUEUE           (1u << 2)  /* descriptor queue end */

/* CUR_CFG / LLI cfg bit fields (H6/A100 variant) */

#define H616_DMA_CFG_SRC_DRQ(n)       (((n) & 0x3fu) << 0)
#define H616_DMA_CFG_SRC_BURST(n)     (((n) & 0x3u) << 6)
#define H616_DMA_CFG_SRC_MODE(n)      (((n) & 0x1u) << 8)
#define H616_DMA_CFG_SRC_WIDTH(n)     (((n) & 0x3u) << 9)
#define H616_DMA_CFG_DST_DRQ(n)       ((((n) & 0x3fu) << 0) << 16)
#define H616_DMA_CFG_DST_BURST(n)     ((((n) & 0x3u) << 6) << 16)
#define H616_DMA_CFG_DST_MODE(n)      ((((n) & 0x1u) << 8) << 16)
#define H616_DMA_CFG_DST_WIDTH(n)     ((((n) & 0x3u) << 9) << 16)

#define H616_DMA_DRQ_SDRAM            1
#define H616_DMA_MODE_LINEAR          0       /* DRAM side */
#define H616_DMA_MODE_IO              1       /* peripheral side */

/* LLI descriptor (hardware layout, 24 bytes) */

struct h616_dma_lli_s
{
  uint32_t cfg;                         /* transfer configuration */
  uint32_t src;                         /* source physical address */
  uint32_t dst;                         /* destination physical address */
  uint32_t len;                         /* byte count */
  uint32_t para;                        /* wait clock cycles */
  uint32_t p_lli_next;                  /* next LLI phys address */
};

#define H616_DMA_LLI_LAST             0xfffff800u
#define H616_DMA_LLI_NORMAL_WAIT      8

#endif /* __ARCH_ARM64_SRC_H616_HARDWARE_H616_DMA_H */
