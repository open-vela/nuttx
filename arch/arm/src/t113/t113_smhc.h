/****************************************************************************
 * arch/arm/src/t113/t113_smhc.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_SMHC_H
#define __ARCH_ARM_SRC_T113_T113_SMHC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>
#include <nuttx/sdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum SDIO function index supported by the host (1..7 are valid; index
 * 0 refers to the CCCR / FBR space which the card-side controller exposes
 * but never raises an interrupt for).
 */

#define T113_SMHC_NUM_FUNCS  8

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Function-IRQ callback for SDIO interrupts.  Phase-2 consumers (e.g. an
 * RTL8723DS WiFi adapter) register one callback per active SDIO function
 * via t113_smhc_register_func_irq() and arm dispatch via
 * t113_smhc_enable_func_irq().  Callback runs in IRQ context; keep it
 * short and defer real work to a workqueue.
 */

typedef int (*sdio_func_irq_t)(void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: t113_smhc_initialize
 *
 * Description:
 *   Initialise and return the SDIO host controller for the requested slot.
 *   slotno = 0 -> SMHC0 (microSD), 1 -> SMHC1 (SDIO WiFi), 2 -> SMHC2.
 *
 ****************************************************************************/

struct sdio_dev_s *t113_smhc_initialize(int slotno);

/****************************************************************************
 * Name: t113_smhc_register_func_irq
 *
 * Description:
 *   Register a per-function SDIO IRQ callback.  func must be 1..7 (function
 *   0 is CCCR-only and never raises an IRQ).  Callback is fired from the
 *   SMHC ISR when SMHC_RINTSTS.SDIO_INT (bit 16) latches.
 *
 *   IMPORTANT: the SMHC IP exposes a single SDIO_INT bit (the OR of every
 *   function's Int_Pending) and provides no hardware hook to identify which
 *   function asserted.  This driver therefore fires EVERY registered
 *   callback on EVERY SDIO_INT.  Consumers that register more than one
 *   function callback are responsible for reading CCCR.Int_Pending in
 *   their handler and ignoring spurious wakeups for inactive functions.
 *   The typical single-function combo (RTL8723DS WiFi) registers only
 *   function 1 and so does not encounter the demux issue.
 *
 * Returned Value:
 *   OK on success; -EINVAL on out-of-range func; -EBUSY if a callback is
 *   already registered for that function (use cb=NULL to deregister).
 *
 ****************************************************************************/

int t113_smhc_register_func_irq(struct sdio_dev_s *dev, uint8_t func,
                                sdio_func_irq_t cb, void *arg);

/****************************************************************************
 * Name: t113_smhc_enable_func_irq
 *
 * Description:
 *   Arm or disarm SDIO IO interrupts at the host level.  When enabled, the
 *   SMHC raises an IRQ when the card asserts its DAT[1] interrupt line.
 *   This is independent of waitenable() -- the bit lives outside the
 *   command/data wait surface so a card IRQ can fire while no transfer is
 *   in flight.
 *
 *   Note: the func argument is reserved for future per-function masking;
 *   the T113 SMHC has only a single SDIO-INT enable bit (RINT bit 16) so
 *   any non-zero func enables/disables the host-side dispatch globally.
 *
 ****************************************************************************/

int t113_smhc_enable_func_irq(struct sdio_dev_s *dev, uint8_t func,
                              bool enable);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_ARM_SRC_T113_T113_SMHC_H */
