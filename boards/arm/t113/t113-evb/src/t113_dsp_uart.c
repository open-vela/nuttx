/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_dsp_uart.c
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

#include "arm_internal.h"
#include "hardware/t113_dsp.h"
#include "hardware/t113_pinmap.h"
#include "t113_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Reserve UART2 for the HiFi4 DSP.
 *
 * UART2 on this board is wired to PE2 (TX) / PE3 (RX) at mux 3, going to
 * the dedicated DSP debug header (J93).  The AP must configure pin mux and
 * enable the UART2 BUS clock and de-assert the UART2 reset, but it does NOT
 * bind a NuttX serial driver -- the DSP firmware programs UART2
 * LCR / baud / FIFO itself once the DSP starts up.
 */

void t113_dsp_uart_prepare(void)
{
  uint32_t v;

  /* Pin mux: PE2 TX, PE3 RX, both mux 3, internal pull-up.
   * The macros encode all four fields (port / pin / mux / pull).
   */

  t113_gpio_config(T113_UART2_TX_2);
  t113_gpio_config(T113_UART2_RX_2);

  /* CCU UART_BGR: enable bus clock gating + de-assert reset for UART2. */

  v = getreg32(T113_CCU_UART_BGR_REG);
  v |= T113_CCU_UART2_GATING | T113_CCU_UART2_RST;
  putreg32(v, T113_CCU_UART_BGR_REG);
}
