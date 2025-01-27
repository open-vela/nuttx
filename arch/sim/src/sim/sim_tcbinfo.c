/****************************************************************************
 * arch/sim/src/sim/sim_tcbinfo.c
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

#if defined(CONFIG_HOST_X86_64) && !defined(CONFIG_SIM_M32)
/* Name,  Size, Regnum, TCB offset,             g/G offset */

static const struct reginfo_s g_reginfo[] =
{
  {"rbx", 8,    1,      TCB_REG_OFF(JB_RBX),    REGINFO_OFFSET_INVALID},
  {"rbp", 8,    6,      TCB_REG_OFF(JB_RBP),    REGINFO_OFFSET_INVALID},
  {"rsp", 8,    7,      TCB_REG_OFF(JB_RSP),    REGINFO_OFFSET_INVALID},
  {"r12", 8,    12,     TCB_REG_OFF(JB_R12),    REGINFO_OFFSET_INVALID},
  {"r13", 8,    13,     TCB_REG_OFF(JB_R13),    REGINFO_OFFSET_INVALID},
  {"r14", 8,    14,     TCB_REG_OFF(JB_R14),    REGINFO_OFFSET_INVALID},
  {"r15", 8,    15,     TCB_REG_OFF(JB_R15),    REGINFO_OFFSET_INVALID},
  {"rip", 8,    16,     TCB_REG_OFF(JB_RIP),    REGINFO_OFFSET_INVALID},
};
#elif defined(CONFIG_HOST_X86) || defined(CONFIG_SIM_M32)
static const struct reginfo_s g_reginfo[] =
{
  {"ebx", 4,    1,      TCB_REG_OFF(JB_EBX),    REGINFO_OFFSET_INVALID},
  {"esp", 4,    2,      TCB_REG_OFF(JB_ESP),    REGINFO_OFFSET_INVALID},
  {"ebp", 4,    3,      TCB_REG_OFF(JB_EBP),    REGINFO_OFFSET_INVALID},
  {"esi", 4,    4,      TCB_REG_OFF(JB_ESI),    REGINFO_OFFSET_INVALID},
  {"edi", 4,    5,      TCB_REG_OFF(JB_EDI),    REGINFO_OFFSET_INVALID},
  {"eip", 4,    6,      TCB_REG_OFF(JB_EIP),    REGINFO_OFFSET_INVALID},
};
#elif defined(CONFIG_HOST_ARM64)
static const struct reginfo_s g_reginfo[] =
{
  {"sp", 8,    31,     TCB_REG_OFF(JB_SP),    REGINFO_OFFSET_INVALID},
  {"pc", 8,    32,     TCB_REG_OFF(JB_PC),    REGINFO_OFFSET_INVALID},
};
#elif defined(CONFIG_HOST_ARM)
static const struct reginfo_s g_reginfo[] =
{
  {"",  4,    0,      UINT16_MAX, REGINFO_OFFSET_INVALID},
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct tcbinfo_s g_tcbinfo used_data =
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
    .reginfo       = g_reginfo,
  }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
