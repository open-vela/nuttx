/****************************************************************************
 * arch/arm/src/t113/t113_dsp.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_DSP_H
#define __ARCH_ARM_SRC_T113_T113_DSP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Release HiFi4 DSP from reset and start it executing at `entry`.
 *
 * Called by the rptun start op after the framework has placed DSP firmware
 * in SRAM/DDR.  `entry` is the DSP-view entry point address.
 *
 * Returns 0 on success.
 */

int t113_dsp_release(uint32_t entry);

/* Halt the DSP and gate its clock.  Idempotent. */

void t113_dsp_halt(void);

/* Configure DSP root clock.  Returns the resulting rate in Hz. */

uint32_t t113_dsp_clk_set(uint32_t freq_hz);

/* Set SRAMC remap bit.  value=1: AP owns; value=0: DSP owns.
 * Exposed for use by board-level code if needed.
 */

void t113_sramc_remap(int value);

#endif /* __ARCH_ARM_SRC_T113_T113_DSP_H */
