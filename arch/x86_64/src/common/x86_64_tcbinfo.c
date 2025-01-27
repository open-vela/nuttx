/****************************************************************************
 * arch/x86_64/src/common/x86_64_tcbinfo.c
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
  {"r15",     8,    15,   TCB_REG_OFF(REG_R15),     120},
  {"r14",     8,    14,   TCB_REG_OFF(REG_R14),     112},
  {"r13",     8,    13,   TCB_REG_OFF(REG_R13),     104},
  {"r12",     8,    12,   TCB_REG_OFF(REG_R12),     96},
  {"rbp",     8,    6,    TCB_REG_OFF(REG_RBP),     48},
  {"rbx",     8,    1,    TCB_REG_OFF(REG_RBX),     8},
  {"r11",     8,    11,   TCB_REG_OFF(REG_R11),     88},
  {"r10",     8,    10,   TCB_REG_OFF(REG_R10),     80},
  {"r9",      8,    9,    TCB_REG_OFF(REG_R9),      72},
  {"r8",      8,    8,    TCB_REG_OFF(REG_R8),      64},
  {"rax",     8,    0,    TCB_REG_OFF(REG_RAX),     0},   /* g/G 0 */
  {"rcx",     8,    2,    TCB_REG_OFF(REG_RCX),     16},
  {"rdx",     8,    3,    TCB_REG_OFF(REG_RDX),     24},
  {"rsi",     8,    4,    TCB_REG_OFF(REG_RSI),     32},
  {"rdi",     8,    5,    TCB_REG_OFF(REG_RDI),     40},
  {"rax",     8,    0,    TCB_REG_OFF(REG_RAX),     REGINFO_OFFSET_INVALID},   /* orig_ax */
  {"rip",     8,    16,   TCB_REG_OFF(REG_RIP),     128},
  {"cs",      4,    18,   TCB_REG_OFF(REG_CS),      140},
  {"eflags",  4,    17,   TCB_REG_OFF(REG_RFLAGS),  136}, /* GDB expects it to be 4bytes, why? */
  {"rsp",     8,    7,    TCB_REG_OFF(REG_RSP),     56},
  {"ss",      4,    19,   TCB_REG_OFF(REG_SS),      144},
  {"fs",      4,    22,   TCB_REG_OFF(REG_FS),      REGINFO_OFFSET_INVALID}, /* fs_base */
  {"gs",      4,    23,   TCB_REG_OFF(REG_GS),      REGINFO_OFFSET_INVALID}, /* gs_base */
  {"ds",      4,    20,   TCB_REG_OFF(REG_DS),      148},
  {"es",      4,    21,   TCB_REG_OFF(REG_ES),      152},
  {"fs",      4,    22,   TCB_REG_OFF(REG_FS),      156},
  {"gs",      4,    23,   TCB_REG_OFF(REG_GS),      160},
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
    .reginfo = g_reginfo,
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
