/****************************************************************************
 * arch/xtensa/src/t113/t113_msgbox.c
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

#include <nuttx/irq.h>
#include <arch/irq.h>

#include "xtensa.h"
#include "hardware/t113_msgbox.h"
#include "t113_msgbox.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ARM<->DSP uses direction index n = 0, sub-channel p = 0. */

#define MSGBOX_N            0
#define MSGBOX_P            0

/****************************************************************************
 * Private Data
 ****************************************************************************/

static t113_msgbox_rx_t g_msgbox_cb;
static void            *g_msgbox_arg;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_msgbox_interrupt
 *
 * Description:
 *   MSGBOX RX ISR (Xtensa INT 3).  Drain every queued word from the
 *   DSP-local mailbox FIFO, deliver each to the callback, then ack the
 *   RD_IRQ_STA pending bit so the level-3 line deasserts.
 *
 ****************************************************************************/

static int t113_msgbox_interrupt(int irq, void *context, void *arg)
{
  uintptr_t msgsta = T113_MSGBOX_MSG_STA(T113_MSGBOX_DSP_BASE,
                                         MSGBOX_N, MSGBOX_P);
  uintptr_t msgreg = T113_MSGBOX_MSG(T113_MSGBOX_DSP_BASE,
                                     MSGBOX_N, MSGBOX_P);
  uintptr_t rdsta  = T113_MSGBOX_RD_IRQ_STA(T113_MSGBOX_DSP_BASE, MSGBOX_N);

  UNUSED(irq);
  UNUSED(context);
  UNUSED(arg);

  while (getreg32(msgsta) != 0)
    {
      uint32_t word = getreg32(msgreg);

      if (g_msgbox_cb != NULL)
        {
          g_msgbox_cb(g_msgbox_arg, word);
        }
    }

  putreg32(T113_MSGBOX_RD_BIT(MSGBOX_P), rdsta);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void t113_msgbox_init(void)
{
  uintptr_t rden  = T113_MSGBOX_RD_IRQ_EN(T113_MSGBOX_DSP_BASE, MSGBOX_N);
  uintptr_t rdsta = T113_MSGBOX_RD_IRQ_STA(T113_MSGBOX_DSP_BASE, MSGBOX_N);

  /* Clear any stale pending, enable RX-not-empty for sub-channel 0. */

  putreg32(T113_MSGBOX_RD_BIT(MSGBOX_P), rdsta);
  putreg32(getreg32(rden) | T113_MSGBOX_RD_BIT(MSGBOX_P), rden);
}

void t113_msgbox_send(uint32_t word)
{
  uintptr_t msgsta = T113_MSGBOX_MSG_STA(T113_MSGBOX_ARM_BASE,
                                         MSGBOX_N, MSGBOX_P);
  uintptr_t msgreg = T113_MSGBOX_MSG(T113_MSGBOX_ARM_BASE,
                                     MSGBOX_N, MSGBOX_P);

  /* Write into the ARM-local mailbox FIFO.  Spin while it is full. */

  while (getreg32(msgsta) >= T113_MSGBOX_MAX_QUEUE)
    {
    }

  putreg32(word, msgreg);
}

void t113_msgbox_attach(t113_msgbox_rx_t cb, void *arg)
{
  uint32_t intenable;

  if (cb != NULL)
    {
      g_msgbox_arg = arg;
      g_msgbox_cb  = cb;

      irq_attach(XTENSA_IRQ_MSGBOX, t113_msgbox_interrupt, NULL);

      /* up_enable_irq is a no-op for Xtensa-internal IRQs; unmask the
       * INTENABLE bit directly now that the handler is attached.
       */

      __asm__ volatile ("rsr.intenable %0" : "=a"(intenable));
      intenable |= (1u << T113_MSGBOX_INT);
      __asm__ volatile ("wsr.intenable %0" :: "a"(intenable));
    }
  else
    {
      __asm__ volatile ("rsr.intenable %0" : "=a"(intenable));
      intenable &= ~(1u << T113_MSGBOX_INT);
      __asm__ volatile ("wsr.intenable %0" :: "a"(intenable));
      g_msgbox_cb  = NULL;
      g_msgbox_arg = NULL;
    }
}
