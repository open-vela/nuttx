/****************************************************************************
 * arch/arm/src/common/arm_freeze_diag.h
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
 * Temporary freeze diagnostics (QA-04, 2026-10-19)
 *
 * Read-only counters, compiled in only with CONFIG_DEBUG_FREEZE_DIAG.
 * Each update is a single word increment from one context; there is no
 * print, no wait, no allocation and no lock in any of the paths below.
 * The firmware writes them, a debugger only reads them from a halted
 * target, so they stay as plain .bss symbols instead of being optimized
 * into registers.
 *
 * They answer "which contexts keep running after a freeze", not "why".
 * Do not treat a single sample or a missing update as a root cause.
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_COMMON_ARM_FREEZE_DIAG_H
#define __ARCH_ARM_SRC_COMMON_ARM_FREEZE_DIAG_H

#include <nuttx/config.h>

#ifdef CONFIG_DEBUG_FREEZE_DIAG

#include <stdint.h>

struct arm_freeze_diag_s
{
  /* SysTick: vector 15 -> exception_direct -> irq_dispatch ->
   * stm32_timerisr().  A growing entry count with a stalled
   * g_system_ticks would put the problem after the vector.
   */

  volatile uint32_t systick_entries;
  volatile uint32_t systick_exits;
  volatile uint32_t systick_last_tick;    /* g_system_ticks at last exit */

  /* PendSV: vector 14 -> exception_common -> arm_doirq().  The extra
   * fields capture the masking state and the context pointers of the
   * last entry so a freeze sample can be interpreted next to the PC.
   */

  volatile uint32_t pendsv_entries;
  volatile uint32_t pendsv_exits;
  volatile uint32_t pendsv_sigaction;     /* called up_schedule_sigaction */
  volatile uint32_t pendsv_null_regs;     /* entered with context == NULL */
  volatile uint32_t pendsv_last_basepri;
  volatile uint32_t pendsv_last_primask;
  volatile uint32_t pendsv_last_ipsr;
  volatile uint32_t pendsv_last_xpsr;     /* stacked xPSR of interrupted ctxt */
  volatile uint32_t pendsv_last_regs;     /* tcb->xcp.regs at entry */
  volatile uint32_t pendsv_last_saved;    /* tcb->xcp.saved_regs at entry */

  /* USART2.  *recv_before and *recv_after bracket uart_recvchars(); a
   * difference only means one call had not returned when the sample was
   * taken.  up_interrupt() may be entered without reaching that call, so
   * compare against *entries and the hardware ISR snapshot rather than
   * concluding a loop from one pair alone.
   */

  volatile uint32_t usart2_entries;
  volatile uint32_t usart2_exits;
  volatile uint32_t usart2_recv_before;
  volatile uint32_t usart2_recv_after;
  volatile uint32_t usart2_last_isr;      /* ISR at last entry */
  volatile uint32_t usart2_last_ie;       /* priv->ie at last entry */
  volatile uint32_t usart2_last_rxlevel;  /* ISR[3:0] at last entry */
};

extern volatile struct arm_freeze_diag_s g_arm_freeze_diag;

#define ARM_FREEZE_DIAG  (&g_arm_freeze_diag)

#endif /* CONFIG_DEBUG_FREEZE_DIAG */
#endif /* __ARCH_ARM_SRC_COMMON_ARM_FREEZE_DIAG_H */
