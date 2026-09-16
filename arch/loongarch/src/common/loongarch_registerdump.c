/****************************************************************************
 * arch/loongarch/src/common/loongarch_registerdump.c
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

#include <stdio.h>
#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "loongarch_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_dump_register(void *dumpregs)
{
  volatile uintreg_t *regs = dumpregs ? dumpregs : running_regs();

  _alert("PC: %" PRIxREG "\n", regs[REG_EPC]);
  _alert("RA: %" PRIxREG " SP: %" PRIxREG " INT_CTX: %" PRIxREG "\n",
         regs[REG_R1], regs[REG_R3], regs[REG_INT_CTX]);
  _alert("A0: %" PRIxREG " A1: %" PRIxREG " A2: %" PRIxREG
         " A3: %" PRIxREG "\n",
         regs[REG_R4], regs[REG_R5], regs[REG_R6], regs[REG_R7]);
  _alert("A4: %" PRIxREG " A5: %" PRIxREG " A6: %" PRIxREG
         " A7: %" PRIxREG "\n",
         regs[REG_R8], regs[REG_R9], regs[REG_R10], regs[REG_R11]);
  _alert("T0: %" PRIxREG " T1: %" PRIxREG " T2: %" PRIxREG
         " T3: %" PRIxREG "\n",
         regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);
  _alert("T4: %" PRIxREG " T5: %" PRIxREG " T6: %" PRIxREG
         " T7: %" PRIxREG "\n",
         regs[REG_R16], regs[REG_R17], regs[REG_R18], regs[REG_R19]);
  _alert("T8: %" PRIxREG "\n", regs[REG_R20]);
  _alert("S0: %" PRIxREG " S1: %" PRIxREG " S2: %" PRIxREG
         " S3: %" PRIxREG "\n",
         regs[REG_R23], regs[REG_R24], regs[REG_R25], regs[REG_R26]);
  _alert("S4: %" PRIxREG " S5: %" PRIxREG " S6: %" PRIxREG
         " S7: %" PRIxREG "\n",
         regs[REG_R27], regs[REG_R28], regs[REG_R29], regs[REG_R30]);
  _alert("S8: %" PRIxREG " FP: %" PRIxREG "\n",
         regs[REG_R22], regs[REG_R21]);
}
