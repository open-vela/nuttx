/****************************************************************************
 * arch/sparc/src/common/sparc_tcbinfo.c
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
  {"",    4,    0,      REGINFO_OFFSET_INVALID, REGINFO_OFFSET_AUTO},
  {"g1",  4,    1,      TCB_REG_OFF(REG_G1),    REGINFO_OFFSET_AUTO},
  {"g2",  4,    2,      TCB_REG_OFF(REG_G2),    REGINFO_OFFSET_AUTO},
  {"g3",  4,    3,      TCB_REG_OFF(REG_G3),    REGINFO_OFFSET_AUTO},
  {"g4",  4,    4,      TCB_REG_OFF(REG_G4),    REGINFO_OFFSET_AUTO},
  {"g5",  4,    5,      TCB_REG_OFF(REG_G5),    REGINFO_OFFSET_AUTO},
  {"g6",  4,    6,      TCB_REG_OFF(REG_G6),    REGINFO_OFFSET_AUTO},
  {"g7",  4,    7,      TCB_REG_OFF(REG_G7),    REGINFO_OFFSET_AUTO},
  {"o0",  4,    8,      TCB_REG_OFF(REG_O0),    REGINFO_OFFSET_AUTO},
  {"o1",  4,    9,      TCB_REG_OFF(REG_O1),    REGINFO_OFFSET_AUTO},
  {"o2",  4,    10,     TCB_REG_OFF(REG_O2),    REGINFO_OFFSET_AUTO},
  {"o3",  4,    11,     TCB_REG_OFF(REG_O3),    REGINFO_OFFSET_AUTO},
  {"o4",  4,    12,     TCB_REG_OFF(REG_O4),    REGINFO_OFFSET_AUTO},
  {"o5",  4,    13,     TCB_REG_OFF(REG_O5),    REGINFO_OFFSET_AUTO},
  {"o6",  4,    14,     TCB_REG_OFF(REG_O6),    REGINFO_OFFSET_AUTO},
  {"o7",  4,    15,     TCB_REG_OFF(REG_O7),    REGINFO_OFFSET_AUTO},
  {"l0",  4,    16,     TCB_REG_OFF(REG_L0),    REGINFO_OFFSET_AUTO},
  {"l1",  4,    17,     TCB_REG_OFF(REG_L1),    REGINFO_OFFSET_AUTO},
  {"l2",  4,    18,     TCB_REG_OFF(REG_L2),    REGINFO_OFFSET_AUTO},
  {"l3",  4,    19,     TCB_REG_OFF(REG_L3),    REGINFO_OFFSET_AUTO},
  {"l4",  4,    20,     TCB_REG_OFF(REG_L4),    REGINFO_OFFSET_AUTO},
  {"l5",  4,    21,     TCB_REG_OFF(REG_L5),    REGINFO_OFFSET_AUTO},
  {"l6",  4,    22,     TCB_REG_OFF(REG_L6),    REGINFO_OFFSET_AUTO},
  {"l7",  4,    23,     TCB_REG_OFF(REG_L7),    REGINFO_OFFSET_AUTO},
  {"i0",  4,    24,     TCB_REG_OFF(REG_I0),    REGINFO_OFFSET_AUTO},
  {"i1",  4,    25,     TCB_REG_OFF(REG_I1),    REGINFO_OFFSET_AUTO},
  {"i2",  4,    26,     TCB_REG_OFF(REG_I2),    REGINFO_OFFSET_AUTO},
  {"i3",  4,    27,     TCB_REG_OFF(REG_I3),    REGINFO_OFFSET_AUTO},
  {"i4",  4,    28,     TCB_REG_OFF(REG_I4),    REGINFO_OFFSET_AUTO},
  {"i5",  4,    29,     TCB_REG_OFF(REG_I5),    REGINFO_OFFSET_AUTO},
  {"i6",  4,    30,     TCB_REG_OFF(REG_I6),    REGINFO_OFFSET_AUTO},
  {"i7",  4,    31,     TCB_REG_OFF(REG_I7),    REGINFO_OFFSET_AUTO},
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
    .reginfo      = g_reginfo,
  }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
