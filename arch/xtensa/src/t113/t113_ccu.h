/****************************************************************************
 * arch/xtensa/src/t113/t113_ccu.h
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

#ifndef __ARCH_XTENSA_SRC_T113_T113_CCU_H
#define __ARCH_XTENSA_SRC_T113_T113_CCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DSP-side CCU access.
 *
 * STUB: the AP brings up every clock the DSP currently uses (UART, system
 * timer) before the DSP firmware runs, so peripheral drivers here do not
 * yet touch the CCU.  t113_ccu_modify() is therefore a no-op for now.
 *
 * When the DSP grows drivers for peripherals it owns (I2S/DMIC audio, the
 * DSP's own DMAC channels), it will need to gate those clocks itself.  The
 * CCU registers are shared with the ARM AP, and the SoC hardware spinlock
 * at 0x03005000 is reachable from the DSP (the same lock registers are
 * accessible from both cores).  At that point t113_ccu_modify() should
 * become a real read-modify-write guarded by hwspinlock id 0
 * (T113_HWLOCK_ID_CCU on the AP side) so DSP and AP serialize against each
 * other on the CCU.  Using only a local irq-disable and relying on a static
 * clock-ownership split is a weaker option; matching the AP's hwspinlock id
 * is the more robust choice if the DSP starts touching clocks the AP also
 * manages.
 */

#ifndef __ASSEMBLY__

static inline void t113_ccu_modify(uint32_t reg, uint32_t clr, uint32_t set)
{
  (void)reg;
  (void)clr;
  (void)set;
}

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_XTENSA_SRC_T113_T113_CCU_H */
