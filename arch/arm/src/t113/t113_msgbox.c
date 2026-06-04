/****************************************************************************
 * arch/arm/src/t113/t113_msgbox.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "hardware/t113_msgbox.h"
#include "t113_ccu.h"
#include "t113_msgbox.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ARM<->DSP uses direction index n = 0 (vendor calculte_n table) and a
 * single sub-channel p = 0 for the rptun doorbell.
 */

#define MSGBOX_N            0
#define MSGBOX_P            0

/* GIC interrupt for the MSGBOX -> CPUX(ARM) line.  IRQ 32 (GIC ID 32 =
 * SPI 0) on the A7 GIC; verified on hardware.
 */

#ifndef CONFIG_T113_MSGBOX_IRQ
#  define CONFIG_T113_MSGBOX_IRQ  32
#endif

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
 *   MSGBOX RX ISR.  Drain every queued word from the ARM-local mailbox
 *   FIFO (sub-channel 0), deliver each to the registered callback, then
 *   write-1-to-clear the RD_IRQ_STA pending bit so the GIC line deasserts.
 *
 ****************************************************************************/

static int t113_msgbox_interrupt(int irq, void *context, void *arg)
{
  uintptr_t msgsta = T113_MSGBOX_MSG_STA(T113_MSGBOX_ARM_BASE,
                                         MSGBOX_N, MSGBOX_P);
  uintptr_t msgreg = T113_MSGBOX_MSG(T113_MSGBOX_ARM_BASE,
                                     MSGBOX_N, MSGBOX_P);
  uintptr_t rdsta  = T113_MSGBOX_RD_IRQ_STA(T113_MSGBOX_ARM_BASE, MSGBOX_N);

  UNUSED(irq);
  UNUSED(context);
  UNUSED(arg);

  /* Drain the FIFO: MSG_STA holds the queued word count. */

  while (getreg32(msgsta) != 0)
    {
      uint32_t word = getreg32(msgreg);

      if (g_msgbox_cb != NULL)
        {
          g_msgbox_cb(g_msgbox_arg, word);
        }
    }

  /* Acknowledge: write-1-to-clear the receive pending bit for sub-channel
   * 0 (bit p*2).
   */

  putreg32(T113_MSGBOX_RD_BIT(MSGBOX_P), rdsta);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void t113_msgbox_init(void)
{
  uintptr_t rden = T113_MSGBOX_RD_IRQ_EN(T113_MSGBOX_ARM_BASE, MSGBOX_N);
  uintptr_t rdsta = T113_MSGBOX_RD_IRQ_STA(T113_MSGBOX_ARM_BASE, MSGBOX_N);

  /* Gate + de-assert reset for both mailbox instances (ARM = MSGBOX0,
   * DSP = MSGBOX1) -- both live in the single BGR register, and the AP
   * must clock the DSP instance too so its FIFO is writable.
   */

  t113_ccu_module_enable(T113_CCU_MSGBOX_BGR,
                         T113_CCU_MSGBOX0_RST | T113_CCU_MSGBOX1_RST,
                         T113_CCU_MSGBOX0_GATING | T113_CCU_MSGBOX1_GATING);

  /* Clear any stale RX pending, then enable the RX-not-empty interrupt for
   * sub-channel 0.
   */

  putreg32(T113_MSGBOX_RD_BIT(MSGBOX_P), rdsta);
  putreg32(getreg32(rden) | T113_MSGBOX_RD_BIT(MSGBOX_P), rden);
}

void t113_msgbox_send(uint32_t word)
{
  uintptr_t msgsta = T113_MSGBOX_MSG_STA(T113_MSGBOX_DSP_BASE,
                                         MSGBOX_N, MSGBOX_P);
  uintptr_t msgreg = T113_MSGBOX_MSG(T113_MSGBOX_DSP_BASE,
                                     MSGBOX_N, MSGBOX_P);

  /* Write into the DSP-local mailbox FIFO.  Spin while it is full. */

  while (getreg32(msgsta) >= T113_MSGBOX_MAX_QUEUE)
    {
    }

  putreg32(word, msgreg);
}

void t113_msgbox_attach(t113_msgbox_rx_t cb, void *arg)
{
  if (cb != NULL)
    {
      g_msgbox_arg = arg;
      g_msgbox_cb  = cb;

      irq_attach(CONFIG_T113_MSGBOX_IRQ, t113_msgbox_interrupt, NULL);
      up_enable_irq(CONFIG_T113_MSGBOX_IRQ);
    }
  else
    {
      up_disable_irq(CONFIG_T113_MSGBOX_IRQ);
      g_msgbox_cb  = NULL;
      g_msgbox_arg = NULL;
    }
}
