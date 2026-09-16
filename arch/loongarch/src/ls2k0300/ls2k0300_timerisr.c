/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_timerisr.c
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
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/csr.h>

#include "loongarch_internal.h"
#include "chip.h"
#include "hardware/ls2k0300_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

static uint64_t ls2k0300_const_freq(void)
{
  uint32_t base_freq;
  uint32_t cfg5;
  uint32_t cfm;
  uint32_t cfd;

  base_freq = read_cpucfg(LOONGARCH_CPUCFG4);
  cfg5 = read_cpucfg(LOONGARCH_CPUCFG5);
  cfm = cfg5 & 0xffff;
  cfd = (cfg5 >> 16) & 0xffff;

  if (!base_freq || !cfm || !cfd)
    {
      return LS2K0300_APB_FREQ * 1000000ULL;
    }

  return (uint64_t)base_freq * cfm / cfd;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ls2k0300_timer_interrupt(int irq, void *context, void *arg)
{
  /* Clear timer intr status */

  csr_writeq(CSR_TINTCLR_TI, LOONGARCH_CSR_TINTCLR);

  nxsched_process_timer();

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint64_t tcfg;
  uint64_t tval;
  uint64_t timer_freq;

  timer_freq = ls2k0300_const_freq();
  tval = timer_freq / TICK_PER_SEC;

  tcfg  = tval << CSR_TCFG_VAL_SHIFT;
  tcfg |= CSR_TCFG_PERIOD | CSR_TCFG_EN;

  csr_writeq(tcfg, LOONGARCH_CSR_TCFG);

  irq_attach(LOONGARCH_IRQ_TIMER, ls2k0300_timer_interrupt, NULL);
  up_enable_irq(LOONGARCH_IRQ_TIMER);
}
