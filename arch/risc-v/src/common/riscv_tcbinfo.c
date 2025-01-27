/****************************************************************************
 * arch/risc-v/src/common/riscv_tcbinfo.c
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

/* Name,    Size, Regnum, TCB offset,               g/G offset */

static const struct reginfo_s g_reginfo[] =
{
  {"zero",  4,    0,      TCB_REG_OFF(REG_EPC_NDX), REGINFO_OFFSET_AUTO},
  {"ra",    4,    1,      TCB_REG_OFF(REG_X1_NDX),  REGINFO_OFFSET_AUTO},
  {"sp",    4,    2,      TCB_REG_OFF(REG_X2_NDX),  REGINFO_OFFSET_AUTO},
  {"gp",    4,    3,      TCB_REG_OFF(REG_X3_NDX),  REGINFO_OFFSET_AUTO},
  {"tp",    4,    4,      TCB_REG_OFF(REG_X4_NDX),  REGINFO_OFFSET_AUTO},
  {"t0",    4,    5,      TCB_REG_OFF(REG_X5_NDX),  REGINFO_OFFSET_AUTO},
  {"t1",    4,    6,      TCB_REG_OFF(REG_X6_NDX),  REGINFO_OFFSET_AUTO},
  {"t2",    4,    7,      TCB_REG_OFF(REG_X7_NDX),  REGINFO_OFFSET_AUTO},
  {"fp",    4,    8,      TCB_REG_OFF(REG_X8_NDX),  REGINFO_OFFSET_AUTO},
  {"s1",    4,    9,      TCB_REG_OFF(REG_X9_NDX),  REGINFO_OFFSET_AUTO},
  {"a0",    4,    10,     TCB_REG_OFF(REG_X10_NDX), REGINFO_OFFSET_AUTO},
  {"a1",    4,    11,     TCB_REG_OFF(REG_X11_NDX), REGINFO_OFFSET_AUTO},
  {"a2",    4,    12,     TCB_REG_OFF(REG_X12_NDX), REGINFO_OFFSET_AUTO},
  {"a3",    4,    13,     TCB_REG_OFF(REG_X13_NDX), REGINFO_OFFSET_AUTO},
  {"a4",    4,    14,     TCB_REG_OFF(REG_X14_NDX), REGINFO_OFFSET_AUTO},
  {"a5",    4,    15,     TCB_REG_OFF(REG_X15_NDX), REGINFO_OFFSET_AUTO},
  {"a6",    4,    25,     TCB_REG_OFF(REG_X16_NDX), REGINFO_OFFSET_AUTO},
  {"a7",    4,    25,     TCB_REG_OFF(REG_X17_NDX), REGINFO_OFFSET_AUTO},
  {"s2",    4,    25,     TCB_REG_OFF(REG_X18_NDX), REGINFO_OFFSET_AUTO},
  {"s3",    4,    25,     TCB_REG_OFF(REG_X19_NDX), REGINFO_OFFSET_AUTO},
  {"s4",    4,    25,     TCB_REG_OFF(REG_X20_NDX), REGINFO_OFFSET_AUTO},
  {"s5",    4,    25,     TCB_REG_OFF(REG_X21_NDX), REGINFO_OFFSET_AUTO},
  {"s6",    4,    25,     TCB_REG_OFF(REG_X22_NDX), REGINFO_OFFSET_AUTO},
  {"s7",    4,    25,     TCB_REG_OFF(REG_X23_NDX), REGINFO_OFFSET_AUTO},
  {"s8",    4,    25,     TCB_REG_OFF(REG_X24_NDX), REGINFO_OFFSET_AUTO},
  {"s9",    4,    25,     TCB_REG_OFF(REG_X25_NDX), REGINFO_OFFSET_AUTO},
  {"s10",   4,    25,     TCB_REG_OFF(REG_X26_NDX), REGINFO_OFFSET_AUTO},
  {"s11",   4,    25,     TCB_REG_OFF(REG_X27_NDX), REGINFO_OFFSET_AUTO},
  {"t3",    4,    25,     TCB_REG_OFF(REG_X28_NDX), REGINFO_OFFSET_AUTO},
  {"t4",    4,    25,     TCB_REG_OFF(REG_X29_NDX), REGINFO_OFFSET_AUTO},
  {"t5",    4,    25,     TCB_REG_OFF(REG_X30_NDX), REGINFO_OFFSET_AUTO},
  {"t6",    4,    25,     TCB_REG_OFF(REG_X31_NDX), REGINFO_OFFSET_AUTO},
  {"pc",    4,    25,     TCB_REG_OFF(REG_EPC_NDX), REGINFO_OFFSET_AUTO},
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
    .reginfo      = g_reginfo,
  }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
