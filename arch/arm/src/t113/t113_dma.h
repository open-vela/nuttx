/****************************************************************************
 * arch/arm/src/t113/t113_dma.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_DMA_H
#define __ARCH_ARM_SRC_T113_T113_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only channels 0-7 are usable from a non-secure ARM context.
 * The T113 DMAC has two IRQ outputs: IRQ 82 (DMAC_NS, chans 0-7)
 * routes to ARM, IRQ 83 (DMAC_S, chans 8-15) routes only to the
 * DSP and never reaches the non-secure CPU.  Without delivered
 * completion interrupts, chan 8-15 cannot drive UART/SPI/etc.
 * reliably (an HPWORK polling work-around exists on legacy
 * branches but caps throughput to ~10 ms / poll period).  Limit
 * the pool to the 8 ARM-reachable channels so that any peripheral
 * that fails to allocate one falls back to a PIO path instead.
 */

#define T113_DMA_NCHANNELS 8

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void *DMA_HANDLE;

/* DMA completion callback.
 *
 * status is a bitmask of the DMAC IRQ PEND bits that fired for this
 * channel, extracted from the channel's 4-bit slot in IRQ_PEND0/1:
 *
 *   bit 0  DMAC_IRQ_HLFDONE  half-transfer reached (circular mode)
 *   bit 1  DMAC_IRQ_PKGDONE  package (buffer) done
 *   bit 2  DMAC_IRQ_QDONE    queue done (last descriptor in chain)
 *
 * Only bits that were enabled via IRQ_EN are delivered.
 */

typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);

/* DMA transfer config passed to t113_dmasetup() */

struct t113_dma_config_s
{
  uint8_t  src_drq;       /* Source DRQ type (DRQ_* from hardware/t113_dma.h) */
  uint8_t  dst_drq;       /* Destination DRQ type */
  uint8_t  src_width;     /* Source data width (DMAC_WIDTH_*) */
  uint8_t  dst_width;     /* Destination data width */
  uint8_t  src_burst;     /* Source burst length (DMAC_BURST_*) */
  uint8_t  dst_burst;     /* Destination burst length */
  bool     src_linear;    /* true=linear (memory), false=IO (peripheral) */
  bool     dst_linear;    /* true=linear (memory), false=IO (peripheral) */
  uint8_t  mode;          /* DMAC_MODE_REGN value (handshake bits) */
  bool     circular;      /* true: self-linking descriptor (loop) */
  bool     bmode;         /* true: BMODE - one block per DRQ assertion */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

void arm_dma_initialize(void);

DMA_HANDLE t113_dmachannel(void);
void       t113_dmafree(DMA_HANDLE handle);

int  t113_dmasetup(DMA_HANDLE handle,
                   uintptr_t src, uintptr_t dst, size_t len,
                   const struct t113_dma_config_s *cfg);

int  t113_dmastart(DMA_HANDLE handle,
                   dma_callback_t callback, void *arg);

void t113_dmapause(DMA_HANDLE handle);
void t113_dmaresume(DMA_HANDLE handle);
void t113_dmastop(DMA_HANDLE handle);

size_t t113_dmaresidual(DMA_HANDLE handle);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_ARM_SRC_T113_T113_DMA_H */
