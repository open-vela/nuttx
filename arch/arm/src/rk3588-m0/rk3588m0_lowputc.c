/****************************************************************************
 * arch/arm/src/rk3588-m0/rk3588m0_lowputc.c
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

#include "arm_internal.h"
#include "chip.h"
#include "rk3588m0_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The console UART is shared with whoever owns the physical port (u-boot,
 * then Linux's fiq-debugger, plus the NuttX instance on cpu_l3).  This driver
 * is therefore deliberately "borrow-only":
 *
 *   - it polls LSR.THRE and writes THR, nothing else;
 *   - it never programs the baud divisor, line control or FIFOs, so it
 *     inherits whatever the owner configured (1500000 8N1 on this board);
 *   - it never enables a UART interrupt (the fiq-debugger owns the FIQ).
 *
 * This mirrors what the cpu_l3 NuttX port does via CONFIG_SUPPRESS_UART_CONFIG
 * and what the bare-metal M0 demo did, both of which are proven on target.
 */

#define CONSOLE_BASE RK3588M0_UART2_BASE

/* Bound the THRE wait so a wedged or powered-down UART can never hang the
 * whole system in a spin loop.  At 1.5Mbaud a character takes well under a
 * microsecond, so this is a very generous ceiling.
 */

#define TXREADY_TIMEOUT 1000000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3588m0_lowsetup
 *
 * Description:
 *   Called early in the boot sequence.  There is intentionally nothing to do:
 *   the shared UART is already configured by u-boot/Linux and must not be
 *   reprogrammed.  Kept as a hook so the start-up path reads like other ports.
 *
 ****************************************************************************/

void rk3588m0_lowsetup(void)
{
}

/****************************************************************************
 * Name: rk3588m0_lowputc
 *
 * Description:
 *   Output one byte on the shared console UART, polled.
 *
 ****************************************************************************/

void rk3588m0_lowputc(int ch)
{
  uint32_t timeout = TXREADY_TIMEOUT;

  /* Wait until the transmit holding register is empty */

  while ((getreg32(CONSOLE_BASE + RK3588M0_UART_LSR_OFFSET) &
          RK3588M0_UART_LSR_THRE) == 0)
    {
      if (--timeout == 0)
        {
          return;
        }
    }

  putreg32((uint32_t)ch, CONSOLE_BASE + RK3588M0_UART_THR_OFFSET);
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes.  This is
 *   what syslog() ends up calling before (or without) a full serial driver.
 *
 ****************************************************************************/

void up_putc(int ch)
{
  /* Expand \n to \r\n so the shared console stays readable */

  if (ch == '\n')
    {
      rk3588m0_lowputc('\r');
    }

  rk3588m0_lowputc(ch);
}
