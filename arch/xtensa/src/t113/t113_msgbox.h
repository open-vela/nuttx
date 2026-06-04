/****************************************************************************
 * arch/xtensa/src/t113/t113_msgbox.h
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

#ifndef __ARCH_XTENSA_SRC_T113_T113_MSGBOX_H
#define __ARCH_XTENSA_SRC_T113_T113_MSGBOX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Receive callback (ISR context): one 32-bit word drained from the DSP
 * mailbox FIFO.  `arg` is the value passed to t113_msgbox_attach.
 */

typedef void (*t113_msgbox_rx_t)(void *arg, uint32_t word);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Bring up the DSP-side mailbox RX path: clear and enable the RX
 * sub-channel.  The AP has already gated the MSGBOX clock.  Idempotent.
 */

void t113_msgbox_init(void);

/* Send one 32-bit word DSP -> AP (writes the ARM-local mailbox FIFO). */

void t113_msgbox_send(uint32_t word);

/* Attach the RX callback and enable the MSGBOX interrupt (Xtensa INT 3).
 * cb=NULL detaches.
 */

void t113_msgbox_attach(t113_msgbox_rx_t cb, void *arg);

#endif /* __ARCH_XTENSA_SRC_T113_T113_MSGBOX_H */
