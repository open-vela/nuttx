/****************************************************************************
 * arch/x86/src/common/x86_tcbinfo.c
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

#include <nuttx/sched.h>
#include <arch/irq.h>
#include <sys/param.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Name,    Size,  Regnum,   TCB offset,         g/G offset */

static const struct reginfo_s g_reginfo[] =
{
  {"eax",    4,    15,   TCB_REG_OFF(G_EAX),    REGINFO_OFFSET_AUTO},
  {"ecx",    4,    14,   TCB_REG_OFF(G_ECX),    REGINFO_OFFSET_AUTO},
  {"edx",    4,    13,   TCB_REG_OFF(G_EDX),    REGINFO_OFFSET_AUTO},
  {"ebp",    4,    12,   TCB_REG_OFF(G_EBP),    REGINFO_OFFSET_AUTO},
  {"esp",    4,    6,    TCB_REG_OFF(G_ESP),    REGINFO_OFFSET_AUTO},
  {"ebp",    4,    1,    TCB_REG_OFF(G_EBP),    REGINFO_OFFSET_AUTO},
  {"esi",    4,    11,   TCB_REG_OFF(G_ESI),    REGINFO_OFFSET_AUTO},
  {"edi",    4,    10,   TCB_REG_OFF(G_EDI),    REGINFO_OFFSET_AUTO},
  {"eip",    4,    9,    TCB_REG_OFF(G_EIP),    REGINFO_OFFSET_AUTO},
  {"eflags", 4,    8,    TCB_REG_OFF(G_EFLAGS), REGINFO_OFFSET_AUTO},
  {"cs",     4,    0,    TCB_REG_OFF(G_CS),     REGINFO_OFFSET_AUTO},
  {"ss",     4,    2,    TCB_REG_OFF(G_SS),     REGINFO_OFFSET_AUTO},
  {"ds",     4,    3,    TCB_REG_OFF(G_DS),     REGINFO_OFFSET_AUTO},
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct tcbinfo_s g_tcbinfo =
{
  .pid_off        = TCB_PID_OFF,
  .state_off      = TCB_STATE_OFF,
  .pri_off        = TCB_PRI_OFF,
  .name_off       = TCB_NAME_OFF,
  .stack_off      = TCB_STACK_OFF,
  .stack_size_off = TCB_STACK_SIZE_OFF,
  .regs_off       = TCB_REGS_OFF,
  .regs_num       = nitems(g_reginfo),
  {
    .reginfo = g_reginfo,
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
