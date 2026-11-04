/****************************************************************************
 * arch/loongarch/src/common/loongarch_exception.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "loongarch_internal.h"
#include <arch/csr.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_reasons_str[] =
{
  "INT (Interrupt)",
  "TLB miss on load",
  "TLB miss on store",
  "TLB miss on ifetch",
  "TLB modified fault",
  "TLB Read-Inhibit",
  "TLB Execution-Inhibit",
  "TLB Privilege Error",
  "Address Error",
  "Unalign Access",
  "Out of bounds",
  "System call",
  "Breakpoint",
  "Inst. Not Exist",
  "Inst. Privileged Error",
  "FPU Disabled",
  "LSX Disabled",
  "LASX Disabled",
  "Floating Point Exception",
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: loongarch_exception
 *
 * Description:
 *   This is the exception handler.
 *
 ****************************************************************************/

int loongarch_exception(int excode, void *regs, void *args)
{
  uintreg_t era = csr_readq(LOONGARCH_CSR_ERA);
  uintreg_t badv = csr_readq(LOONGARCH_CSR_BADV);

  if (excode < sizeof(g_reasons_str) / sizeof(g_reasons_str[0]))
    {
      _alert("EXCEPTION: %s. ExCode: %x, ERA: %" PRIxREG
             ", BADV: %" PRIxREG "\n",
             g_reasons_str[excode], excode, era, badv);
    }
  else
    {
      _alert("EXCEPTION: Unknown. ExCode: %x, ERA: %" PRIxREG
             ", BADV: %" PRIxREG "\n",
             excode, era, badv);
    }

  _alert("PANIC!!! Exception = %x\n", excode);
  up_irq_save();
  up_set_interrupt_context(true);
  PANIC_WITH_REGS("panic", regs);

  return 0;
}

/****************************************************************************
 * Name: loongarch_exception_attach
 *
 * Description:
 *   Attach standard exception with suitable handler
 *
 ****************************************************************************/

void loongarch_exception_attach(void)
{
  int i;

  for (i = EXCCODE_TLBL; i < EXCCODE_INT_START; i++)
    {
      if (i != EXCCODE_SYS)
        {
          irq_attach(i, loongarch_exception, NULL);
        }
    }

  irq_attach(EXCCODE_SYS, loongarch_swint, NULL);
}
