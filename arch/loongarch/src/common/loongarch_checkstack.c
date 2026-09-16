/****************************************************************************
 * arch/loongarch/src/common/loongarch_checkstack.c
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

#include <sys/types.h>
#include <stdint.h>
#include <sched.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "loongarch_internal.h"

#ifdef CONFIG_STACK_COLORATION

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_ARCH_INTERRUPTSTACK) && CONFIG_ARCH_INTERRUPTSTACK > 15
void loongarch_color_intstack(void)
{
  uint32_t *ptr = (uint32_t *)g_intstackalloc;
  ssize_t size;

  for (size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
       size > 0;
       size -= sizeof(uint32_t))
    {
      *ptr++ = STACK_COLOR;
    }
}
#endif

size_t loongarch_stack_check(uintptr_t alloc, size_t size)
{
  uintptr_t start;
  uintptr_t end;
  uint32_t *ptr;
  size_t mark;

  if (size == 0)
    {
      return 0;
    }

  start = (alloc + 3) & ~3;
  end   = (alloc + size) & ~3;

  size  = end - start;

  for (ptr = (uint32_t *)start, mark = (size >> 2);
       mark > 0 && *ptr == STACK_COLOR;
       ptr++, mark--);

  return mark << 2;
}

size_t up_check_tcbstack(struct tcb_s *tcb, size_t check_size)
{
  size_t size;

#ifdef CONFIG_ARCH_ADDRENV
  struct addrenv_s *oldenv;

  if (tcb->group->tg_addrenv_own != NULL)
    {
      addrenv_select(tcb->group->tg_addrenv_own, &oldenv);
    }
#endif

  size = loongarch_stack_check((uintptr_t)tcb->stack_base_ptr, check_size);

#ifdef CONFIG_ARCH_ADDRENV
  if (tcb->group->tg_addrenv_own != NULL)
    {
      addrenv_restore(oldenv);
    }
#endif

  return size;
}

#if CONFIG_ARCH_INTERRUPTSTACK > 15
size_t up_check_intstack(int cpu, size_t check_size)
{
  if (check_size == 0)
    {
      check_size = CONFIG_ARCH_INTERRUPTSTACK & ~15;
    }

  return loongarch_stack_check((uintptr_t)g_intstackalloc, check_size);
}
#endif

#endif /* CONFIG_STACK_COLORATION */
