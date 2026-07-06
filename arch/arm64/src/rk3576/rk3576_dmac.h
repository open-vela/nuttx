/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_dmac.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_DMAC_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_DMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA transfer mode */

#define RK3576_DMA_MODE_MEM2MEM    0
#define RK3576_DMA_MODE_MEM2PER    1
#define RK3576_DMA_MODE_PER2MEM    2

/* DMA transfer width */

#define RK3576_DMA_WIDTH_8BIT      1
#define RK3576_DMA_WIDTH_16BIT     2
#define RK3576_DMA_WIDTH_32BIT     4

/****************************************************************************
 * DMA transfer descriptor
 ****************************************************************************/

struct rk3576_dma_xfer_s
{
  uint32_t src_addr;            /* Source address */
  uint32_t dst_addr;            /* Destination address */
  uint32_t length;              /* Transfer length in bytes */
  int      mode;                /* Transfer mode */
  int      width;               /* Transfer width (1, 2, or 4 bytes) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialization */

void rk3576_dmac_init(void);

/* Channel management */

int  rk3576_dmac_alloc(void);
void rk3576_dmac_free(int ch);

/* Transfer operations */

int  rk3576_dmac_transfer(int ch, const struct rk3576_dma_xfer_s *xfer);
int  rk3576_dmac_mem2mem(int ch, uint32_t dst, uint32_t src, uint32_t len);
int  rk3576_dmac_per2mem(int ch, uint32_t dst, uint32_t src_per, uint32_t len);
int  rk3576_dmac_mem2per(int ch, uint32_t dst_per, uint32_t src, uint32_t len);

/* Control */

void rk3576_dmac_start(int ch);
void rk3576_dmac_stop(int ch);
void rk3576_dmac_abort(int ch);

/* Status */

int  rk3576_dmac_is_busy(int ch);
void rk3576_dmac_wait(int ch);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_DMAC_H */
