/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_irq.c
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

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/csr.h>

#include "loongarch_internal.h"
#include "ls2k0300.h"
#include "hardware/ls2k0300_memorymap.h"

/****************************************************************************
 * EIOINTC (Extended I/O Interrupt Controller)
 ****************************************************************************/

#define EIOINTC_VEC_COUNT          128
#define EIOINTC_NODEMAP_COUNT      (EIOINTC_VEC_COUNT / 32)
#define EIOINTC_IPMAP_COUNT        (EIOINTC_VEC_COUNT / 128)
#define EIOINTC_ROUTE_COUNT        (EIOINTC_VEC_COUNT / 4)
#define EIOINTC_BOUNCE_COUNT       (EIOINTC_VEC_COUNT / 32)
#define EIOINTC_ENABLE_COUNT       (EIOINTC_VEC_COUNT / 32)
#define EIOINTC_ISR_COUNT          (EIOINTC_VEC_COUNT / 64)

#define EIOINTC_ENABLE_VEC(vector) \
  (EIOINTC_REG_ENABLE + ((vector >> 5) << 2))

static void eiointc_init(void)
{
  uint32_t data;
  int i;

  uint64_t misc = iocsr_read64(LOONGARCH_IOCSR_MISC_FUNC);
  misc |= IOCSR_MISC_FUNC_EXT_IOI_EN;
  iocsr_write64(misc, LOONGARCH_IOCSR_MISC_FUNC);

  for (i = 0; i < EIOINTC_NODEMAP_COUNT; i++)
    {
      data = (((1 << (i * 2 + 1)) << 16) | (1 << (i * 2)));
      iocsr_write32(data, EIOINTC_REG_NODEMAP + i * 4);
    }

  for (i = 0; i < EIOINTC_IPMAP_COUNT; i++)
    {
      data = (1U << 1) | ((1U << 1) << 8) |
             ((1U << 1) << 16) | ((1U << 1) << 24);
      iocsr_write32(data, EIOINTC_REG_IPMAP + i * 4);
    }

  for (i = 0; i < EIOINTC_ROUTE_COUNT; i++)
    {
      data = 0x1 | (0x1 << 8) | (0x1 << 16) | (0x1 << 24);
      iocsr_write32(data, EIOINTC_REG_ROUTE + i * 4);
    }

  for (i = 0; i < EIOINTC_BOUNCE_COUNT; i++)
    {
      iocsr_write32(0xffffffff, EIOINTC_REG_BOUNCE + i * 4);
      iocsr_write32(0x0, EIOINTC_REG_ENABLE + i * 4);
    }
}

static int eiointc_dispatch(int irq, void *context, void *arg)
{
  uint64_t pending;
  int i;

  for (i = 0; i < EIOINTC_ISR_COUNT; i++)
    {
      pending = iocsr_read64(EIOINTC_REG_ISR + (i << 3));
      if (!pending)
        {
          continue;
        }

      iocsr_write64(pending, EIOINTC_REG_ISR + (i << 3));

      while (pending)
        {
          int bit = __builtin_ffsl(pending) - 1;
          int vector = bit + 64 * i;
          int ext_irq = LS2K0300_IRQ_EIOINTC_START + vector;

          irq_dispatch(ext_irq, (uintreg_t *)context);
          pending &= ~((uint64_t)1 << bit);
        }
    }

  return OK;
}

/****************************************************************************
 * ICU (Traditional I/O Interrupt Controller)
 * Reference: linux-5.10/drivers/irqchip/irq-loongson-2k500-iointc.c
 * DTS: icu@0x16001400, cascades to CPU IP2 (interrupt 4)
 ****************************************************************************/

#define ICU_IRQ_COUNT             64

#define ICU_INTISR_REG(base, i)   ((base) + 0x20 + (((i) > 31) ? 0x40 : 0))
#define ICU_INTEN_REG(base, i)    ((base) + 0x24 + (((i) > 31) ? 0x40 : 0))
#define ICU_INTENSET_REG(base, i) ((base) + 0x28 + (((i) > 31) ? 0x40 : 0))
#define ICU_INTENCLR_REG(base, i) ((base) + 0x2c + (((i) > 31) ? 0x40 : 0))
#define ICU_INTPOL_REG(base, i)   ((base) + 0x30 + (((i) > 31) ? 0x40 : 0))
#define ICU_INTEDGE_REG(base, i)  ((base) + 0x34 + (((i) > 31) ? 0x40 : 0))
#define ICU_INTAUTO_REG(base, i)  ((base) + 0x3c + (((i) > 31) ? 0x40 : 0))
#define ICU_IRQ_ROUTE_REG(base, i) ((base) + 0x00 + (((i) > 31) ? 0x40 : 0) + ((i) % 32))
#define ICU_COREISR_REG(isr, i)   ((isr) + (i) * 8)

static void icu_init(void)
{
  uintptr_t base = LS2K0300_ICU_REG_BASE;
  int i;

  for (i = 0; i < ICU_IRQ_COUNT; i++)
    {
      putreg8(0x41, ICU_IRQ_ROUTE_REG(base, i));
    }

  putreg32(0x0, ICU_INTEN_REG(base, 0));
  putreg32(0x0, ICU_INTEN_REG(base, 32));
  putreg32(0x0, ICU_INTPOL_REG(base, 0));
  putreg32(0x0, ICU_INTPOL_REG(base, 32));
  putreg32(0x0, ICU_INTEDGE_REG(base, 0));
  putreg32(0x0, ICU_INTEDGE_REG(base, 32));
  putreg32(0x0, ICU_INTAUTO_REG(base, 0));
  putreg32(0x0, ICU_INTAUTO_REG(base, 32));
}

static int icu_dispatch(int irq, void *context, void *arg)
{
  uintptr_t base = LS2K0300_ICU_REG_BASE;
  uintptr_t isr_base = LS2K0300_ICU_ISR_BASE;
  uint64_t pending;
  uint32_t reg_val;
  uint32_t inten_lo;
  uint32_t inten_hi;

  pending = getreg32(ICU_COREISR_REG(isr_base, 0));
  reg_val = getreg32(ICU_COREISR_REG(isr_base, 1));
  pending |= (uint64_t)reg_val << 32;

  if (!pending)
    {
      return OK;
    }

  inten_lo = getreg32(ICU_INTEN_REG(base, 0));
  inten_hi = getreg32(ICU_INTEN_REG(base, 32));

  while (pending)
    {
      int bit = __builtin_ffsl(pending) - 1;
      int icu_irq = LS2K0300_IRQ_ICU_START + bit;
      uint32_t inten_reg = (bit < 32) ? inten_lo : inten_hi;
      uint32_t bit_mask = 1 << (bit % 32);

      putreg32(bit_mask, ICU_INTENCLR_REG(base, bit));

      irq_dispatch(icu_irq, (uintreg_t *)context);

      if (inten_reg & bit_mask)
        {
          putreg32(bit_mask, ICU_INTENSET_REG(base, bit));
        }

      pending &= ~((uint64_t)1 << bit);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_irqinitialize(void)
{
  up_irq_save();

  loongarch_exception_attach();

  write_csr_ecfg(ECFGF(ECFGB_TIMER) | ECFGF(ECFGB_IP1) |
                 ECFGF(ECFGB_IP2));

  eiointc_init();
  icu_init();

  irq_attach(LOONGARCH_IRQ_IP1, eiointc_dispatch, NULL);
  irq_attach(LOONGARCH_IRQ_IP2, icu_dispatch, NULL);

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  loongarch_stack_color(g_intstackalloc, intstack_size);
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  loongarch_color_intstack();
  up_irq_enable();
#endif
}

void up_disable_irq(int irq)
{
  unsigned long ecfg;

  if (irq == LOONGARCH_IRQ_TIMER)
    {
      ecfg = csr_readq(LOONGARCH_CSR_ECFG);
      ecfg &= ~ECFGF(ECFGB_TIMER);
      csr_writeq(ecfg, LOONGARCH_CSR_ECFG);
    }
  else if (irq >= LS2K0300_IRQ_ICU_START &&
           irq < LS2K0300_IRQ_ICU_START + LS2K0300_IRQ_ICU_COUNT)
    {
      int hwirq = irq - LS2K0300_IRQ_ICU_START;
      uintptr_t base = LS2K0300_ICU_REG_BASE;
      putreg32(1 << (hwirq % 32), ICU_INTENCLR_REG(base, hwirq));
    }
  else if (irq >= LS2K0300_IRQ_EIOINTC_START &&
           irq < LS2K0300_IRQ_EIOINTC_START + LS2K0300_IRQ_EIOINTC_COUNT)
    {
      int vector = irq - LS2K0300_IRQ_EIOINTC_START;
      uint32_t reg = EIOINTC_ENABLE_VEC(vector);
      uint32_t data = iocsr_read32(reg);
      data &= ~(1U << (vector & 0x1f));
      iocsr_write32(data, reg);
    }
}

void up_enable_irq(int irq)
{
  unsigned long ecfg;

  if (irq == LOONGARCH_IRQ_TIMER)
    {
      ecfg = csr_readq(LOONGARCH_CSR_ECFG);
      ecfg |= ECFGF(ECFGB_TIMER);
      csr_writeq(ecfg, LOONGARCH_CSR_ECFG);
    }
  else if (irq >= LS2K0300_IRQ_ICU_START &&
           irq < LS2K0300_IRQ_ICU_START + LS2K0300_IRQ_ICU_COUNT)
    {
      int hwirq = irq - LS2K0300_IRQ_ICU_START;
      uintptr_t base = LS2K0300_ICU_REG_BASE;
      putreg32(1 << (hwirq % 32), ICU_INTENSET_REG(base, hwirq));
    }
  else if (irq >= LS2K0300_IRQ_EIOINTC_START &&
           irq < LS2K0300_IRQ_EIOINTC_START + LS2K0300_IRQ_EIOINTC_COUNT)
    {
      int vector = irq - LS2K0300_IRQ_EIOINTC_START;
      uint32_t reg = EIOINTC_ENABLE_VEC(vector);
      uint32_t data = iocsr_read32(reg);
      data |= (1U << (vector & 0x1f));
      iocsr_write32(data, reg);
    }
}

void loongarch_ack_irq(int irq)
{
}
