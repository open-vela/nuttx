/****************************************************************************
 * arch/arm64/src/h616/h616_dma.h
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

#ifndef __ARCH_ARM64_SRC_H616_H616_DMA_H
#define __ARCH_ARM64_SRC_H616_H616_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/dma/dma.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The DMAC interrupt is GIC SPI 42, i.e. NuttX IRQ 74. */

#define H616_DMAC_IRQ               H616_IRQ_DMA

/* Number of DMA channels (H616: 16) */

#define H616_DMAC_NCHANNELS         16

/* Maximum number of LLI descriptors per channel (static allocation) */

#define H616_DMAC_MAXLLI            8

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: h616_dma_initialize
 *
 * Description:
 *   Bring the DMAC out of reset, enable its bus/mbus clocks, clear and
 *   attach its interrupt.  Must be called before any channel is used,
 *   i.e. before the 16550 driver attaches its DMA channels (the serial
 *   console is set up very early in boot).
 *
 ****************************************************************************/

void h616_dma_initialize(void);

/****************************************************************************
 * Name: h616_dmachan_get
 *
 * Description:
 *   Return the dma_chan_s vtable for the given hardware channel
 *   (0..H616_DMAC_NCHANNELS-1).  The channel is marked in-use.
 *
 ****************************************************************************/

FAR struct dma_chan_s *h616_dmachan_get(unsigned int ch);

/****************************************************************************
 * Name: h616_dmachan_setdrq
 *
 * Description:
 *   Record the peripheral DRQ port number (dma-requests value from the
 *   device tree, e.g. 14 for UART0) that requests service on this
 *   channel.  The 16550 DMA client does not pass the DRQ through
 *   struct dma_config_s, so it must be bound at channel-get time.
 *
 ****************************************************************************/

void h616_dmachan_setdrq(FAR struct dma_chan_s *chan, uint8_t drq);

#endif /* __ARCH_ARM64_SRC_H616_H616_DMA_H */
