/****************************************************************************
 * arch/arm/src/bk7258/bk7258_irq.c
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

/* Porting reference: arch/arm/src/mps/mps_irq.c (generic armv8-m NVIC). */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "ram_vectors.h"
#include "arm_internal.h"
#include "nvic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BK7258_IRQ_FIRST   BK7258_IRQ_EXTINT

#define NVIC_ENA_OFFSET    (0)
#define NVIC_CLRENA_OFFSET (NVIC_IRQ0_31_CLEAR - NVIC_IRQ0_31_ENABLE)

#define DEFPRIORITY32      (NVIC_SYSH_PRIORITY_DEFAULT << 24 | \
                            NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
                            NVIC_SYSH_PRIORITY_DEFAULT << 8  | \
                            NVIC_SYSH_PRIORITY_DEFAULT)

/* BK7258 SYS-level interrupt aggregator: besides the NVIC, a peripheral
 * interrupt must also be enabled here to reach CPU0's NVIC.  Each bit = the
 * interrupt source number (same as the NVIC line): sources 0-31 at
 * 0x44010080, sources 32-63 at 0x44010084.  (Measured: the UART RX
 * interrupt was asserted on the UART side but never delivered because this
 * step was missing.)
 */

#define BK7258_SYS_CPU0_INT_0_31_EN   0x44010080
#define BK7258_SYS_CPU0_INT_32_63_EN  0x44010084

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_BK7258_DEBUG_FAULT_SPIN

/* SWD debug aid: on a fault, expose the stacked exception frame pointer to
 * a global so OpenOCD/gdb can read it directly.  context is the register
 * frame saved by NuttX, whose REG_PC/REG_LR/REG_XPSR hold the fault site.
 */

volatile void *g_bk7258_fault_regs;

/* bk7258_fault_spin: emit a marker then spin, preserving the fault site for
 * SWD inspection.  Does not call _alert/PANIC (which may fault again or
 * produce no output when the OS is not yet up).
 */

static int bk7258_fault_spin(int irq, void *context, void *arg)
{
#ifdef CONFIG_BK7258_DEBUG_FAULT_SPIN_MARK
  const char *s = "\r\n!!BKFAULT!! irq=";
#endif

  g_bk7258_fault_regs = context;

  up_irq_save();

#ifdef CONFIG_BK7258_DEBUG_FAULT_SPIN_MARK
  while (*s)
    {
      arm_lowputc(*s++);
    }

  /* Print the exception number (single decimal digit is enough:
   * HardFault=3/Mem=4/Bus=5/Usage=6).
   */

  arm_lowputc('0' + (irq % 10));
  arm_lowputc('\r');
  arm_lowputc('\n');
#endif

  /* Spin here: the core stays put and the exception frame on the MSP is
   * preserved for reading PC/CFSR after an SWD halt.
   */

  for (; ; )
    {
    }

  return 0;
}
#endif /* CONFIG_BK7258_DEBUG_FAULT_SPIN */

#ifdef CONFIG_DEBUG_FEATURES
static int bk7258_nmi(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int bk7258_pendsv(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int bk7258_reserved(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

static inline void bk7258_prioritize_syscall(int priority)
{
  uint32_t regval;

  /* SVCALL is system handler 11 */

  regval  = getreg32(NVIC_SYSH8_11_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR11_MASK;
  regval |= (priority << NVIC_SYSH_PRIORITY_PR11_SHIFT);
  putreg32(regval, NVIC_SYSH8_11_PRIORITY);
}

static int bk7258_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
                          uintptr_t offset)
{
  int n;

  DEBUGASSERT(irq >= BK7258_IRQ_NMI && irq < NR_IRQS);

  if (irq >= BK7258_IRQ_FIRST)
    {
      n        = irq - BK7258_IRQ_FIRST;
      *regaddr = NVIC_IRQ_ENABLE(n) + offset;
      *bit     = (uint32_t)0x1 << (n & 0x1f);
    }
  else
    {
      *regaddr = NVIC_SYSHCON;
      if (irq == BK7258_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == BK7258_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == BK7258_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == BK7258_IRQ_SYSTICK)
        {
          *regaddr = NVIC_SYSTICK_CTRL;
          *bit = NVIC_SYSTICK_CTRL_ENABLE;
        }
      else
        {
          return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int up_prioritize_irq(int irq, int priority)
{
  uint32_t regaddr;
  uint32_t regval;
  int shift;

  DEBUGASSERT(irq >= 0 && irq < NR_IRQS &&
              (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq < 16)
    {
      regaddr = NVIC_SYSH_PRIORITY(irq);
      irq    -= 4;
    }
  else
    {
      irq    -= 16;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  regval  = getreg32(regaddr);
  shift   = ((irq & 3) << 3);
  regval &= ~(0xff << shift);
  regval |= (priority << shift);
  putreg32(regval, regaddr);

  return OK;
}

void up_irqinitialize(void)
{
  uint32_t regaddr;
  int num_priority_registers;
  int i;

  /* Disable all interrupts */

  for (i = 0; i < NR_IRQS - BK7258_IRQ_FIRST; i += 32)
    {
      putreg32(0xffffffff, NVIC_IRQ_CLEAR(i));
    }

  putreg32((uint32_t)_vectors, NVIC_VECTAB);

#ifdef CONFIG_ARCH_RAMVECTORS
  arm_ramvec_initialize();
#endif

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

  num_priority_registers = (getreg32(NVIC_ICTR) + 1) * 8;

  regaddr = NVIC_IRQ0_3_PRIORITY;
  while (num_priority_registers--)
    {
      putreg32(DEFPRIORITY32, regaddr);
      regaddr += 4;
    }

  /* Attach the SVCall and Hard Fault exception handlers */

  irq_attach(BK7258_IRQ_SVCALL, arm_svcall, NULL);
  irq_attach(BK7258_IRQ_HARDFAULT, arm_hardfault, NULL);

  up_prioritize_irq(BK7258_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN);
  bk7258_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);

#ifdef CONFIG_ARM_MPU
  irq_attach(BK7258_IRQ_MEMFAULT, arm_memfault, NULL);
  up_enable_irq(BK7258_IRQ_MEMFAULT);
#endif

#ifdef CONFIG_DEBUG_FEATURES
  irq_attach(BK7258_IRQ_NMI, bk7258_nmi, NULL);
#ifndef CONFIG_ARM_MPU
  irq_attach(BK7258_IRQ_MEMFAULT, arm_memfault, NULL);
#endif
  irq_attach(BK7258_IRQ_BUSFAULT, arm_busfault, NULL);
  irq_attach(BK7258_IRQ_USAGEFAULT, arm_usagefault, NULL);
  irq_attach(BK7258_IRQ_PENDSV, bk7258_pendsv, NULL);
  irq_attach(BK7258_IRQ_DBGMONITOR, arm_dbgmonitor, NULL);
  irq_attach(BK7258_IRQ_RESERVED, bk7258_reserved, NULL);
#endif

#ifdef CONFIG_BK7258_DEBUG_FAULT_SPIN
  /* SWD debug: override the default fault handlers above with the spinning
   * handler (attached last, so it wins).  Also enable Mem/Bus/Usage fault
   * individually so they do not escalate to HardFault -> OpenOCD can tell
   * which class of fault it is.
   */

  irq_attach(BK7258_IRQ_HARDFAULT, bk7258_fault_spin, NULL);
  irq_attach(BK7258_IRQ_MEMFAULT, bk7258_fault_spin, NULL);
  irq_attach(BK7258_IRQ_BUSFAULT, bk7258_fault_spin, NULL);
  irq_attach(BK7258_IRQ_USAGEFAULT, bk7258_fault_spin, NULL);
  up_enable_irq(BK7258_IRQ_MEMFAULT);
  up_enable_irq(BK7258_IRQ_BUSFAULT);
  up_enable_irq(BK7258_IRQ_USAGEFAULT);
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  arm_color_intstack();
  up_irq_enable();
#endif
}

static void bk7258_sys_int_set(int irq, bool enable)
{
  uintptr_t reg;
  uint32_t bit;
  int src;

  if (irq < BK7258_IRQ_EXTINT)
    {
      return;
    }

  src = irq - BK7258_IRQ_EXTINT;
  if (src < 32)
    {
      reg = BK7258_SYS_CPU0_INT_0_31_EN;
      bit = 1u << src;
    }
  else
    {
      reg = BK7258_SYS_CPU0_INT_32_63_EN;
      bit = 1u << (src - 32);
    }

  if (enable)
    {
      putreg32(getreg32(reg) | bit, reg);
    }
  else
    {
      putreg32(getreg32(reg) & ~bit, reg);
    }
}

void up_disable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (bk7258_irqinfo(irq, &regaddr, &bit, NVIC_CLRENA_OFFSET) == 0)
    {
      if (irq >= BK7258_IRQ_FIRST)
        {
          putreg32(bit, regaddr);
          bk7258_sys_int_set(irq, false);
        }
      else
        {
          regval  = getreg32(regaddr);
          regval &= ~bit;
          putreg32(regval, regaddr);
        }
    }
}

void up_enable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (bk7258_irqinfo(irq, &regaddr, &bit, NVIC_ENA_OFFSET) == 0)
    {
      if (irq >= BK7258_IRQ_FIRST)
        {
          putreg32(bit, regaddr);
          bk7258_sys_int_set(irq, true);
        }
      else
        {
          regval  = getreg32(regaddr);
          regval |= bit;
          putreg32(regval, regaddr);
        }
    }
}

void arm_ack_irq(int irq)
{
}
