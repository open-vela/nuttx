/****************************************************************************
 * arch/arm/src/bk7258/bk7258_irq.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/arm_m/nvicpri.h>

#include "arm_internal.h"
#include "nvic.h"

#include "hardware/bk7258_sysctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 8  | \
   NVIC_SYSH_PRIORITY_DEFAULT)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void bk7258_route_irq(int extirq, bool enable)
{
  uintptr_t regaddr;
  uint32_t bit;

  if (extirq < 32)
    {
      regaddr = BK7258_CPU1_IRQ_EN0;
      bit = 1u << extirq;
    }
  else
    {
      regaddr = BK7258_CPU1_IRQ_EN1;
      bit = 1u << (extirq - 32);
    }

  if (enable)
    {
      modifyreg32(regaddr, 0, bit);
    }
  else
    {
      modifyreg32(regaddr, bit, 0);
    }
}

static int bk7258_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
                          uintptr_t offset)
{
  int extirq;

  DEBUGASSERT(irq >= BK7258_IRQ_NMI && irq < NR_IRQS);

  if (irq >= BK7258_IRQ_FIRST)
    {
      extirq = irq - BK7258_IRQ_FIRST;
      *regaddr = NVIC_IRQ_ENABLE(extirq) + offset;
      *bit = 1u << (extirq & 31);
      return OK;
    }

  *regaddr = NVIC_SYSHCON;
  switch (irq)
    {
      case BK7258_IRQ_MEMFAULT:
        *bit = NVIC_SYSHCON_MEMFAULTENA;
        break;
      case BK7258_IRQ_BUSFAULT:
        *bit = NVIC_SYSHCON_BUSFAULTENA;
        break;
      case BK7258_IRQ_USAGEFAULT:
        *bit = NVIC_SYSHCON_USGFAULTENA;
        break;
      case BK7258_IRQ_SECUREFAULT:
        *bit = NVIC_SYSHCON_SECUREFAULTENA;
        break;
      case BK7258_IRQ_SYSTICK:
        *regaddr = NVIC_SYSTICK_CTRL;
        *bit = NVIC_SYSTICK_CTRL_ENABLE;
        break;
      default:
        return ERROR;
    }

  return OK;
}

static inline void bk7258_prioritize_svcall(int priority)
{
  uint32_t regval = getreg32(NVIC_SYSH8_11_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR11_MASK;
  regval |= priority << NVIC_SYSH_PRIORITY_PR11_SHIFT;
  putreg32(regval, NVIC_SYSH8_11_PRIORITY);
}

void up_irqinitialize(void)
{
  uintptr_t regaddr;
  int priority_registers;
  int i;

  for (i = 0; i < BK7258_EXTIRQ_COUNT; i += 32)
    {
      putreg32(0xffffffffu, NVIC_IRQ_CLEAR(i));
    }

  putreg32(0, BK7258_CPU1_IRQ_EN0);
  putreg32(0, BK7258_CPU1_IRQ_EN1);

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

  priority_registers = ((getreg32(NVIC_ICTR) & 0x1f) + 1) * 8;
  regaddr = NVIC_IRQ0_3_PRIORITY;
  while (priority_registers-- > 0)
    {
      putreg32(DEFPRIORITY32, regaddr);
      regaddr += 4;
    }

  irq_attach(BK7258_IRQ_SVCALL, arm_svcall, NULL);
  irq_attach(BK7258_IRQ_HARDFAULT, arm_hardfault, NULL);
  irq_attach(BK7258_IRQ_MEMFAULT, arm_memfault, NULL);
  irq_attach(BK7258_IRQ_BUSFAULT, arm_busfault, NULL);
  irq_attach(BK7258_IRQ_USAGEFAULT, arm_usagefault, NULL);
  irq_attach(BK7258_IRQ_SECUREFAULT, arm_securefault, NULL);
  arm_enable_dbgmonitor();
  irq_attach(BK7258_IRQ_DBGMONITOR, arm_dbgmonitor, NULL);

  bk7258_prioritize_svcall(NVIC_SYSH_SVCALL_PRIORITY);
  up_enable_irq(BK7258_IRQ_MEMFAULT);
  up_enable_irq(BK7258_IRQ_BUSFAULT);
  up_enable_irq(BK7258_IRQ_USAGEFAULT);
  up_enable_irq(BK7258_IRQ_SECUREFAULT);
  up_irq_enable();
}

void up_enable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (bk7258_irqinfo(irq, &regaddr, &bit, 0) == OK)
    {
      if (irq >= BK7258_IRQ_FIRST)
        {
          bk7258_route_irq(irq - BK7258_IRQ_FIRST, true);
          putreg32(bit, regaddr);
        }
      else
        {
          regval = getreg32(regaddr);
          putreg32(regval | bit, regaddr);
        }
    }
}

void up_disable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (bk7258_irqinfo(irq, &regaddr, &bit,
                     NVIC_IRQ0_31_CLEAR - NVIC_IRQ0_31_ENABLE) == OK)
    {
      if (irq >= BK7258_IRQ_FIRST)
        {
          putreg32(bit, regaddr);
          bk7258_route_irq(irq - BK7258_IRQ_FIRST, false);
        }
      else
        {
          regval = getreg32(regaddr);
          putreg32(regval & ~bit, regaddr);
        }
    }
}

void arm_ack_irq(int irq)
{
}

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  uintptr_t regaddr;
  uint32_t regval;
  int shift;

  DEBUGASSERT(irq >= BK7258_IRQ_MEMFAULT && irq < NR_IRQS);
  DEBUGASSERT((unsigned int)priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq < BK7258_IRQ_FIRST)
    {
      regaddr = NVIC_SYSH_PRIORITY(irq);
      irq -= 4;
    }
  else
    {
      irq -= BK7258_IRQ_FIRST;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  shift = (irq & 3) << 3;
  regval = getreg32(regaddr);
  regval &= ~(0xffu << shift);
  regval |= (uint32_t)priority << shift;
  putreg32(regval, regaddr);
  return OK;
}
#endif
