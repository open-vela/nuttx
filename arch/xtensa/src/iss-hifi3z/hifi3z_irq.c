/****************************************************************************
 * arch/xtensa/src/iss-hifi3z/hifi3z_irq.c
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

#include <arch/irq.h>
#include <arch/chip/irq.h>

#include "xtensa.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_cpu_intmap[HIFI3Z_NCPUINTS];

static uint8_t g_irqmap[NR_IRQS];

/* g_intenable[] is a shadow copy of the per-CPU INTENABLE register
 * content.
 */

static uint32_t g_intenable[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Hard code special cases. */

  g_irqmap[HIFI3Z_IRQ_TIMER0] = HIFI3Z_CPUINT_TIMER0;
  g_irqmap[XTENSA_IRQ_SWINT] = HIFI3Z_CPUINT_SOFTWARE0;

  g_cpu_intmap[HIFI3Z_CPUINT_TIMER0] = HIFI3Z_IRQ_TIMER0;
  g_cpu_intmap[HIFI3Z_CPUINT_SOFTWARE0] = XTENSA_IRQ_SWINT;

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts.  Also clears PS.EXCM */

  up_irq_enable();
#endif

  /* Attach the software interrupt */

  irq_attach(XTENSA_IRQ_SYSCALL, xtensa_swint, NULL);
}

/****************************************************************************
 * Name: up_enable_irq
 ****************************************************************************/

void up_enable_irq(int irq)
{
  int cpuint = g_irqmap[irq];

  xtensa_enable_cpuint(&g_intenable[0], 1ul << cpuint);
}

/****************************************************************************
 * Name: up_disable_irq
 ****************************************************************************/

void up_disable_irq(int irq)
{
  int cpuint = g_irqmap[irq];

  xtensa_disable_cpuint(&g_intenable[0], 1ul << cpuint);
}

/****************************************************************************
 * Name: xtensa_int_decode
 ****************************************************************************/

uint32_t *xtensa_int_decode(uint32_t cpuints, uint32_t *regs)
{
  int bit;
  uint32_t mask;

  for (bit = 0; cpuints != 0 && bit < HIFI3Z_NCPUINTS; bit++)
    {
      mask = 1ul << bit;
      if ((cpuints & mask) != 0)
        {
          uint8_t irq = g_cpu_intmap[bit];

          /* Clear software or edge-triggered interrupt */

          xtensa_intclear(mask);

          /* Dispatch the CPU interrupt */

          regs = xtensa_irq_dispatch(irq, regs);

          cpuints &= ~mask;
        }
    }

  return regs;
}
