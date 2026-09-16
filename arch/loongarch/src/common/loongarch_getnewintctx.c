/****************************************************************************
 * arch/loongarch/src/common/loongarch_getnewintctx.c
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

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/csr.h>

#include "loongarch_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uintptr_t loongarch_get_newintctx(void)
{
  unsigned long crmd = csr_readq(LOONGARCH_CSR_CRMD);

  crmd &= ~CSR_CRMD_PLV;
  crmd |= (PLV_KERN << CSR_CRMD_PLV_SHIFT);
  crmd |= CSR_CRMD_IE;

  /* Enable FPU */

  unsigned long euen = csr_readq(LOONGARCH_CSR_EUEN);
  euen |= CSR_EUEN_FPEN;
  csr_writeq(euen, LOONGARCH_CSR_EUEN);

  return crmd;
}

void loongarch_set_idleintctx(void)
{
}
