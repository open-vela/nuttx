/****************************************************************************
 * arch/xtensa/include/iss-hifi3z/irq.h
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

#ifndef __ARCH_XTENSA_INCLUDE_ISS_HIFI3Z_IRQ_H
#define __ARCH_XTENSA_INCLUDE_ISS_HIFI3Z_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/core-isa.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XTENSA_IRQ_SYSCALL          3  /* User interrupt w/EXCCAUSE=syscall */
#define XTENSA_IRQ_SWINT            4  /* Software interrupt */

#define XTENSA_NIRQ_INTERNAL        5  /* Number of dispatch internal interrupts */
#define XTENSA_IRQ_FIRSTPERIPH      5  /* First peripheral IRQ number */

#define HIFI3Z_IRQ_TIMER0           (XTENSA_IRQ_FIRSTPERIPH + XCHAL_TIMER0_INTERRUPT)
#define HIFI3Z_IRQ_TIMER1           (XTENSA_IRQ_FIRSTPERIPH + XCHAL_TIMER1_INTERRUPT)

#define NR_IRQS                     (XTENSA_NIRQ_INTERNAL + XCHAL_NUM_INTERRUPTS)

#define HIFI3Z_CPUINT_TIMER0        XCHAL_TIMER0_INTERRUPT
#define HIFI3Z_CPUINT_TIMER1        XCHAL_TIMER1_INTERRUPT
#define HIFI3Z_CPUINT_SOFTWARE0     XCHAL_SOFTWARE0_INTERRUPT

#define HIFI3Z_NCPUINTS             XCHAL_NUM_INTERRUPTS

#endif /* __ARCH_XTENSA_INCLUDE_ISS_HIFI3Z_IRQ_H */
