/****************************************************************************
 * arch/arm/src/t113/t113_ccu.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_CCU_H
#define __ARCH_ARM_SRC_T113_T113_CCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: t113_ccu_init
 *
 * Description:
 *   One-shot initialization of the shared CCU register-protection layer.
 *
 *   In AMP builds (CONFIG_T113_AMP=y) this resolves the hardware spinlock
 *   instance assigned to T113_HWLOCK_ID_CCU from the hwspinlock framework
 *   and stores it for subsequent t113_ccu_* calls.  Must run after
 *   t113_hwspinlock_initialize().
 *
 *   In non-AMP builds this is an empty no-op; the CCU is protected by a
 *   private software spinlock that needs no runtime setup.
 *
 *   Idempotent: safe to invoke more than once.  Subsequent calls return
 *   without re-resolving the hwspinlock instance.
 *
 ****************************************************************************/

void t113_ccu_init(void);

/****************************************************************************
 * Name: t113_ccu_module_enable
 *
 * Description:
 *   Bring a peripheral out of reset and enable its bus clock gate using
 *   the prevailing T113 BGR sequence:
 *
 *     1. De-assert reset            (set rst_bit  in *bgr_reg)
 *     2. udelay(1)                  (outside the lock)
 *     3. Enable bus clock gate      (set gate_bit in *bgr_reg)
 *
 *   Each register update is a locked read-modify-write so concurrent
 *   bringup of sibling IPs that share the same BGR cannot lose updates.
 *
 *   Reserved for future module-style consumers; current T113 drivers
 *   open-code the BGR sequence with t113_ccu_modify() because individual
 *   IPs (CAN/CE/SPI) order reset-deassert and gate-enable differently.
 *
 * Input Parameters:
 *   bgr_reg  - BGR register absolute address (e.g. T113_CCU_UART_BGR).
 *   rst_bit  - Bit mask to set to de-assert reset.
 *   gate_bit - Bit mask to set to enable the bus clock gate.
 *
 ****************************************************************************/

void t113_ccu_module_enable(uint32_t bgr_reg, uint32_t rst_bit,
                            uint32_t gate_bit);

/****************************************************************************
 * Name: t113_ccu_module_disable
 *
 * Description:
 *   Reverse of t113_ccu_module_enable: clear the clock gate, then assert
 *   reset.  Performed in a single locked critical section.
 *
 ****************************************************************************/

void t113_ccu_module_disable(uint32_t bgr_reg, uint32_t rst_bit,
                             uint32_t gate_bit);

/****************************************************************************
 * Name: t113_ccu_clk_set
 *
 * Description:
 *   Clear the bits in 'mask' and then set the bits in 'val' (already
 *   pre-shifted by the caller, masked by 'mask') in 'clk_reg', as a single
 *   locked read-modify-write.
 *
 ****************************************************************************/

void t113_ccu_clk_set(uint32_t clk_reg, uint32_t mask, uint32_t val);

/****************************************************************************
 * Name: t113_ccu_modify
 *
 * Description:
 *   Generic locked read-modify-write of a CCU register:  *reg &= ~clr ;
 *   *reg |= set.  Use for unusual call sites that do not match
 *   module_enable / module_disable / clk_set.
 *
 ****************************************************************************/

void t113_ccu_modify(uint32_t reg, uint32_t clr, uint32_t set);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_T113_T113_CCU_H */
