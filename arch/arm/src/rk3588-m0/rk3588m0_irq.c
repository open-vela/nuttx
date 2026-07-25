/****************************************************************************
 * arch/arm/src/rk3588-m0/rk3588m0_irq.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "nvic.h"
#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Four priority fields per IPR register, all set to the default */

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 8  | \
   NVIC_SYSH_PRIORITY_DEFAULT)

/* Is this an external (NVIC) interrupt, as opposed to a processor exception? */

#define RK3588M0_IRQ_ISEXT(irq) \
  ((irq) >= RK3588M0_IRQ_EXTINT && \
   (irq) < RK3588M0_IRQ_EXTINT + ARMV6M_PERIPHERAL_INTERRUPTS)

#define RK3588M0_IRQ_EXTBIT(irq) (1 << ((irq) - RK3588M0_IRQ_EXTINT))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
static int rk3588m0_nmi(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int rk3588m0_pendsv(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int rk3588m0_reserved(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Name: rk3588m0_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.
 *
 ****************************************************************************/

static inline void rk3588m0_clrpend(int irq)
{
  /* Called on every interrupt exit regardless of whether the interrupt can be
   * enabled, so this assertion is necessarily lame.
   */

  DEBUGASSERT((unsigned)irq < NR_IRQS);

  if (RK3588M0_IRQ_ISEXT(irq))
    {
      putreg32(RK3588M0_IRQ_EXTBIT(irq), ARMV6M_NVIC_ICPR);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t regaddr;
  int i;

  /* Disable all interrupts.  u-boot released this core out of reset without
   * touching the NVIC, but the M0 may also have been running earlier firmware
   * before a soft reset, so start from a known state.
   */

  putreg32(0xffffffff, ARMV6M_NVIC_ICER);

  /* Set all exceptions to the default priority */

  putreg32(DEFPRIORITY32, ARMV6M_SYSCON_SHPR2);
  putreg32(DEFPRIORITY32, ARMV6M_SYSCON_SHPR3);

  /* And all of the interrupt lines: 32 IRQs = 8 IPR registers */

  for (i = 0; i < ARMV6M_PERIPHERAL_INTERRUPTS / 4; i++)
    {
      regaddr = ARMV6M_NVIC_IPR(i);
      putreg32(DEFPRIORITY32, regaddr);
    }

  /* Attach the SVCall and Hard Fault exception handlers.  SVCall performs
   * context switches; Hard Fault must also be caught because an SVCall can
   * surface as a Hard Fault under some conditions.
   */

  irq_attach(RK3588M0_IRQ_SVCALL, arm_svcall, NULL);
  irq_attach(RK3588M0_IRQ_HARDFAULT, arm_hardfault, NULL);

  /* Attach the remaining processor exceptions (except reset and SysTick) */

#ifdef CONFIG_DEBUG_FEATURES
  irq_attach(RK3588M0_IRQ_NMI, rk3588m0_nmi, NULL);
  irq_attach(RK3588M0_IRQ_PENDSV, rk3588m0_pendsv, NULL);
  irq_attach(RK3588M0_IRQ_RESERVED, rk3588m0_reserved, NULL);
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  arm_color_intstack();
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  DEBUGASSERT((unsigned)irq < NR_IRQS);

  if (RK3588M0_IRQ_ISEXT(irq))
    {
      putreg32(RK3588M0_IRQ_EXTBIT(irq), ARMV6M_NVIC_ICER);
    }
  else if (irq == RK3588M0_IRQ_SYSTICK)
    {
      /* Of the processor exceptions, only SysTick can be disabled */

      modifyreg32(ARMV6M_SYSTICK_CSR, SYSTICK_CSR_ENABLE, 0);
    }
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  DEBUGASSERT((unsigned)irq < NR_IRQS);

  if (RK3588M0_IRQ_ISEXT(irq))
    {
      putreg32(RK3588M0_IRQ_EXTBIT(irq), ARMV6M_NVIC_ISER);
    }
  else if (irq == RK3588M0_IRQ_SYSTICK)
    {
      modifyreg32(ARMV6M_SYSTICK_CSR, 0, SYSTICK_CSR_ENABLE);
    }
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
  rk3588m0_clrpend(irq);
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.  ARMv6-M implements two bits of priority in
 *   the upper bits of each 8-bit field.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  uint32_t regaddr;
  uint32_t regval;
  int shift;

  DEBUGASSERT(irq == RK3588M0_IRQ_SVCALL ||
              irq == RK3588M0_IRQ_PENDSV ||
              irq == RK3588M0_IRQ_SYSTICK ||
              RK3588M0_IRQ_ISEXT(irq));
  DEBUGASSERT(priority >= NVIC_SYSH_PRIORITY_MAX &&
              priority <= NVIC_SYSH_PRIORITY_MIN);

  if (RK3588M0_IRQ_ISEXT(irq))
    {
      /* ARMV6M_NVIC_IPR() maps IPR0-IPR7, four settings per register */

      regaddr = ARMV6M_NVIC_IPR(irq >> 2);
      shift   = (irq & 3) << 3;
    }

  /* Of the processor exceptions only PendSV and SysTick may be reprioritized
   * here; SVCall is deliberately not modifiable through this interface.
   */

  else if (irq == RK3588M0_IRQ_PENDSV)
    {
      regaddr = ARMV6M_SYSCON_SHPR3;
      shift   = SYSCON_SHPR3_PRI_14_SHIFT;
    }
  else if (irq == RK3588M0_IRQ_SYSTICK)
    {
      regaddr = ARMV6M_SYSCON_SHPR3;
      shift   = SYSCON_SHPR3_PRI_15_SHIFT;
    }
  else
    {
      return ERROR;
    }

  regval  = getreg32(regaddr);
  regval &= ~((uint32_t)0xff << shift);
  regval |= ((uint32_t)priority << shift);
  putreg32(regval, regaddr);

  return OK;
}
#endif
