/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_irqmap.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <arch/t113/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ONLY the boot0 config currently uses this minimal vector table; any
 * other config that enables CONFIG_ARCH_MINIMAL_VECTORTABLE must first
 * extend g_irqmap below to cover every IRQ it attaches.  Without the
 * extension, peripheral IRQs dispatch to irq_unexpected_isr().
 */

#if !defined(CONFIG_T113_BOOT0)
#  error "t113_irqmap.c: extend g_irqmap before enabling \
CONFIG_ARCH_MINIMAL_VECTORTABLE outside boot0"
#endif

/* Secure Physical Timer - PPI1, GIC IRQ 29 */

#define GIC_IRQ_SEC_PHY_TIMER  29

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Minimal IRQ vector table mapping for T113-S3.
 *
 * Only interrupts actually attached via irq_attach() need a valid
 * slot index.  All others map to IRQMAPPED_MAX which causes
 * irq_unexpected_isr() if they ever fire.
 *
 * boot0 config (SMP=n, no DMA):
 *   slot 0: GIC_IRQ_SEC_PHY_TIMER (29) - system tick
 *
 * SMP configs need additional slots for SGI10/SGI11.
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

const irq_mapped_t g_irqmap[NR_IRQS] =
{
  [0 ... 28]             = IRQMAPPED_MAX,
  [29]                   = 0,         /* SEC_PHY_TIMER -> slot 0 */
  [30 ... NR_IRQS - 1]  = IRQMAPPED_MAX,
};
