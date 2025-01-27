/****************************************************************************
 * arch/arm/src/armv6-m/arm_tcbinfo.c
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

/* Name,  Size, Regnum,   TCB offset,         g/G offset */

static const struct reginfo_s g_reginfo[] =
{
  {"r0",    4,    0,      TCB_REG_OFF(REG_R0),    REGINFO_OFFSET_AUTO},
  {"r1",    4,    1,      TCB_REG_OFF(REG_R1),    REGINFO_OFFSET_AUTO},
  {"r2",    4,    2,      TCB_REG_OFF(REG_R2),    REGINFO_OFFSET_AUTO},
  {"r3",    4,    3,      TCB_REG_OFF(REG_R3),    REGINFO_OFFSET_AUTO},
  {"r4",    4,    4,      TCB_REG_OFF(REG_R4),    REGINFO_OFFSET_AUTO},
  {"r5",    4,    5,      TCB_REG_OFF(REG_R5),    REGINFO_OFFSET_AUTO},
  {"r6",    4,    6,      TCB_REG_OFF(REG_R6),    REGINFO_OFFSET_AUTO},
  {"r7",    4,    7,      TCB_REG_OFF(REG_R7),    REGINFO_OFFSET_AUTO},
  {"r8",    4,    8,      TCB_REG_OFF(REG_R8),    REGINFO_OFFSET_AUTO},
  {"r9",    4,    9,      TCB_REG_OFF(REG_R9),    REGINFO_OFFSET_AUTO},
  {"r10",   4,    10,     TCB_REG_OFF(REG_R10),   REGINFO_OFFSET_AUTO},
  {"r11",   4,    11,     TCB_REG_OFF(REG_R11),   REGINFO_OFFSET_AUTO},
  {"r12",   4,    12,     TCB_REG_OFF(REG_R12),   REGINFO_OFFSET_AUTO},
  {"sp",    4,    13,     TCB_REG_OFF(REG_R13),   REGINFO_OFFSET_AUTO},
  {"lr",    4,    14,     TCB_REG_OFF(REG_R14),   REGINFO_OFFSET_AUTO},
  {"pc",    4,    15,     TCB_REG_OFF(REG_R15),   REGINFO_OFFSET_AUTO},
  {"xpsr",  4,    25,     TCB_REG_OFF(REG_XPSR),  REGINFO_OFFSET_AUTO},
};

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
