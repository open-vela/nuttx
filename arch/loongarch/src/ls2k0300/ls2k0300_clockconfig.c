/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_clockconfig.c
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

#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"
#include "ls2k0300.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REF_FREQ                100

#define NODE_L1DIV_OUT_SHIFT    24
#define NODE_L1DIV_LOOPC_SHIFT  15
#define NODE_L1DIV_REF_SHIFT    8
#define NODE_L1DIV_OUT_MARK     0x7f
#define NODE_L1DIV_LOOPC_MARK   0x1ff
#define NODE_L1DIV_REF_MARK     0x7f

#define DDR_L1DIV_OUT_SHIFT     24
#define DDR_L1DIV_LOOPC_SHIFT   15
#define DDR_L1DIV_REF_SHIFT     8
#define DDR_L1DIV_OUT_MARK      0x7f
#define DDR_L1DIV_LOOPC_MARK    0x1ff
#define DDR_L1DIV_REF_MARK      0x7f

#define DDR_L2DIV_OUT_DEV_SHIFT  8
#define DDR_L2DIV_OUT_DEV_MARK   0x7f

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_cpu_freq;
static uint32_t g_apb_freq;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ls2k0300_get_clocks(void)
{
  uint32_t refclk = REF_FREQ * 1000;
  uint32_t ctrl;
  uint32_t l1div_out;
  uint32_t l1div_loopc;
  uint32_t l1div_ref;
  uint32_t mult;
  uint32_t div;

  ctrl = getreg32(LS2K0300_NODE_PLL_L);
  l1div_out   = (ctrl >> NODE_L1DIV_OUT_SHIFT) & NODE_L1DIV_OUT_MARK;
  l1div_loopc = (ctrl >> NODE_L1DIV_LOOPC_SHIFT) & NODE_L1DIV_LOOPC_MARK;
  l1div_ref   = (ctrl >> NODE_L1DIV_REF_SHIFT) & NODE_L1DIV_REF_MARK;
  mult = l1div_loopc;
  div  = l1div_ref * l1div_out;
  g_cpu_freq = (refclk * mult / div) * 1000;

  ctrl = getreg32(LS2K0300_DDR_PLL_L);
  l1div_out   = (ctrl >> DDR_L1DIV_OUT_SHIFT) & DDR_L1DIV_OUT_MARK;
  l1div_loopc = (ctrl >> DDR_L1DIV_LOOPC_SHIFT) & DDR_L1DIV_LOOPC_MARK;
  l1div_ref   = (ctrl >> DDR_L1DIV_REF_SHIFT) & DDR_L1DIV_REF_MARK;

  ctrl = getreg32(LS2K0300_DDR_PLL_H);
  uint32_t l2div_out = (ctrl >> DDR_L2DIV_OUT_DEV_SHIFT) &
                       DDR_L2DIV_OUT_DEV_MARK;
  mult = l1div_loopc;
  div  = l1div_ref * l2div_out;
  g_apb_freq = (refclk * mult / div) * 1000;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void ls2k0300_clockconfig(void)
{
  ls2k0300_get_clocks();
}

uint32_t ls2k0300_get_cpuclk(void)
{
  return g_cpu_freq;
}

uint32_t ls2k0300_get_apbclk(void)
{
  return g_apb_freq;
}
