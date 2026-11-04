/****************************************************************************
 * arch/loongarch/src/common/loongarch_tcbinfo.c
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

#include <nuttx/sched.h>
#include <arch/irq.h>
#include <sys/param.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct reginfo_s g_reginfo[] =
{
  REGINFO_AUTO("pc",   4, 0,  TCB_REG_OFF(REG_EPC_NDX)),
  REGINFO_AUTO("r1",   4, 1,  TCB_REG_OFF(REG_R1_NDX)),
  REGINFO_AUTO("sp",   4, 2,  TCB_REG_OFF(REG_R3_NDX)),
  REGINFO_AUTO("r4",   4, 3,  TCB_REG_OFF(REG_R4_NDX)),
  REGINFO_AUTO("r5",   4, 4,  TCB_REG_OFF(REG_R5_NDX)),
  REGINFO_AUTO("r6",   4, 5,  TCB_REG_OFF(REG_R6_NDX)),
  REGINFO_AUTO("r7",   4, 6,  TCB_REG_OFF(REG_R7_NDX)),
  REGINFO_AUTO("r8",   4, 7,  TCB_REG_OFF(REG_R8_NDX)),
  REGINFO_AUTO("r9",   4, 8,  TCB_REG_OFF(REG_R9_NDX)),
  REGINFO_AUTO("r10",  4, 9,  TCB_REG_OFF(REG_R10_NDX)),
  REGINFO_AUTO("r11",  4, 10, TCB_REG_OFF(REG_R11_NDX)),
  REGINFO_AUTO("r12",  4, 11, TCB_REG_OFF(REG_R12_NDX)),
  REGINFO_AUTO("r13",  4, 12, TCB_REG_OFF(REG_R13_NDX)),
  REGINFO_AUTO("r14",  4, 13, TCB_REG_OFF(REG_R14_NDX)),
  REGINFO_AUTO("r15",  4, 14, TCB_REG_OFF(REG_R15_NDX)),
  REGINFO_AUTO("r16",  4, 15, TCB_REG_OFF(REG_R16_NDX)),
  REGINFO_AUTO("r17",  4, 16, TCB_REG_OFF(REG_R17_NDX)),
  REGINFO_AUTO("r18",  4, 17, TCB_REG_OFF(REG_R18_NDX)),
  REGINFO_AUTO("r19",  4, 18, TCB_REG_OFF(REG_R19_NDX)),
  REGINFO_AUTO("r20",  4, 19, TCB_REG_OFF(REG_R20_NDX)),
  REGINFO_AUTO("r21",  4, 20, TCB_REG_OFF(REG_R21_NDX)),
  REGINFO_AUTO("r22",  4, 21, TCB_REG_OFF(REG_R22_NDX)),
  REGINFO_AUTO("r23",  4, 22, TCB_REG_OFF(REG_R23_NDX)),
  REGINFO_AUTO("r24",  4, 23, TCB_REG_OFF(REG_R24_NDX)),
  REGINFO_AUTO("r25",  4, 24, TCB_REG_OFF(REG_R25_NDX)),
  REGINFO_AUTO("r26",  4, 25, TCB_REG_OFF(REG_R26_NDX)),
  REGINFO_AUTO("r27",  4, 26, TCB_REG_OFF(REG_R27_NDX)),
  REGINFO_AUTO("r28",  4, 27, TCB_REG_OFF(REG_R28_NDX)),
  REGINFO_AUTO("r29",  4, 28, TCB_REG_OFF(REG_R29_NDX)),
  REGINFO_AUTO("r30",  4, 29, TCB_REG_OFF(REG_R30_NDX)),
  REGINFO_AUTO("r31",  4, 30, TCB_REG_OFF(REG_R31_NDX)),
  REGINFO_AUTO("ctx",  4, 31, TCB_REG_OFF(REG_INT_CTX_NDX)),
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
