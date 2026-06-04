/****************************************************************************
 * arch/arm/src/t113/t113_msgbox.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_MSGBOX_H
#define __ARCH_ARM_SRC_T113_T113_MSGBOX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Receive callback: invoked from the MSGBOX RX ISR for each 32-bit word
 * drained from the FIFO.  `arg` is the value passed to t113_msgbox_attach.
 * Runs in interrupt context.
 */

typedef void (*t113_msgbox_rx_t)(void *arg, uint32_t word);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Bring up the ARM<->DSP mailbox: gate+reset the MSGBOX block, clear and
 * enable the RX sub-channel.  Idempotent.  Does NOT enable the GIC IRQ
 * (call t113_msgbox_attach for that).
 */

void t113_msgbox_init(void);

/* Send one 32-bit word ARM -> DSP.  Writes into the DSP-local mailbox MSG
 * FIFO (sub-channel 0).  Spins briefly if the remote FIFO is full.
 */

void t113_msgbox_send(uint32_t word);

/* Attach the RX callback and enable the MSGBOX GIC interrupt.  From this
 * point, words sent DSP -> ARM are drained in ISR context and delivered to
 * `cb(arg, word)`.  Pass cb=NULL to detach and disable the IRQ.
 */

void t113_msgbox_attach(t113_msgbox_rx_t cb, void *arg);

#endif /* __ARCH_ARM_SRC_T113_T113_MSGBOX_H */
