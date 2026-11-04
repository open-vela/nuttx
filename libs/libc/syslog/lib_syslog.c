/****************************************************************************
 * libs/libc/syslog/lib_syslog.c
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

#include <stdarg.h>
#include <syslog.h>

#include <nuttx/tls_task.h>
#include <nuttx/ratelimit.h>
#include <nuttx/syslog/syslog.h>

#include "syslog/syslog.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static DEFINE_RATELIMIT_STATE(g_ratelimit,
  CONFIG_SYSLOG_RATELIMIT_INTERVAL,
  CONFIG_SYSLOG_RATELIMIT_BURST);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vsyslog
 *
 * Description:
 *   The function vsyslog() performs the same task as syslog() with the
 *   difference that it takes a set of arguments which have been obtained
 *   using the stdarg variable argument list macros.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/


/* [V880] task_get_info() cannot be trusted on this board.
 *
 * vsyslog() used to dereference it immediately:
 *
 *     rsr.misc0 a8        thread pointer
 *     l32i    a8, a8, 112 tls_info_s -> tl_task
 *     l32i.n  a8, a8, 0
 *     addmi   a8, a8, 0x100
 *     l8ui    a4, a12, 52 <-- info->ta_syslog_mask, faults
 *
 * and the board died there with EXCCAUSE=001c (vaddr 0x0f20366b) or
 * EXCCAUSE=0003 (vaddr 0x40054d24).  The fault address is always
 * pointer + 52, so the value returned was never a task_info_s.
 *
 * Data in this build only ever lives in two windows, so check those
 * instead of dereferencing blindly. */

unsigned int g_v880_bad  = 0;
int          g_v880_busy = 0;
uintptr_t    g_v880_last = 0;

static int v880_ok(FAR struct task_info_s *info, int priority)
{
  uintptr_t v = (uintptr_t)info;

  g_v880_last = v;

  if (!((v >= 0x3c000000u && v < 0x3d000000u) ||
        (v >= 0x3fc00000u && v < 0x3fd00000u)))
    {
      /* Not a pointer this build can own.  Report the first few and let
       * the message through - dropping it would hide the diagnostics
       * needed to find whatever overwrites tl_task. */

      if (g_v880_bad < 8u && g_v880_busy == 0)
        {
          g_v880_busy = 1;
          g_v880_bad++;
          syslog(LOG_ERR, "[V880] bogus task_info=%p n=%u\n",
                 (FAR void *)info, g_v880_bad);
          g_v880_busy = 0;
        }

      return 1;
    }

  return ((info->ta_syslog_mask & LOG_MASK(priority)) != 0);
}

void vsyslog(int priority, FAR const IPTR char *fmt, va_list ap)
{
  FAR struct task_info_s *info = task_get_info();

  /* Check if this priority is enabled */

  if (info != NULL && v880_ok(info, priority))
    {
      /* Yes.. Perform the nx_vsyslog system call.
       *
       * NOTE:  The va_list parameter is passed by reference.  That is
       * because the va_list is a structure in some compilers and passing
       * of structures in the NuttX syscalls does not work.
       */

#ifdef va_copy
      va_list copy;

      va_copy(copy, ap);
      nx_vsyslog(priority, fmt, &copy);
      va_end(copy);
#else
      nx_vsyslog(priority, fmt, &ap);
#endif
    }
}

/****************************************************************************
 * Name: syslog
 *
 * Description:
 *   syslog() generates a log message. The priority argument is formed by
 *   ORing the facility and the level values (see include/syslog.h). The
 *   remaining arguments are a format, as in printf and any arguments to the
 *   format.
 *
 *   The NuttX implementation does not support any special formatting
 *   characters beyond those supported by printf.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#undef syslog
void syslog(int priority, FAR const IPTR char *fmt, ...)
{
  va_list ap;

  /* Let vsyslog do the work */

  va_start(ap, fmt);
  vsyslog(priority, fmt, ap);
  va_end(ap);
}

/****************************************************************************
 * Name: syslog_ratelimit
 *
 * Description:
 *   The rate-limiting version of syslog. Rate limiting will
 *   be performed using the global rate limiting state.
 *
 ****************************************************************************/

void syslog_ratelimit(int priority,
                      FAR const IPTR char *fmt, ...)
{
  if (!ratelimit_islimited(&g_ratelimit))
    {
      va_list ap;

      va_start(ap, fmt);
      vsyslog(priority, fmt, ap);
      va_end(ap);
    }
}
