/***************************************************************************
 * arch/arm64/src/h616/h616_serial.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/serial/uart_16550.h>
#include "hardware/h616_memorymap.h"
#include "arm64_internal.h"

#ifdef CONFIG_H616_DMA
#include "h616_dma.h"
#endif

/* CCU (real H616 CCU base, see hardware/h616_memorymap.h) */

#define H616_CCU_UART_GATE   (H616_CCU_ADDR + 0x90cu)

/* PIO (pin controller).  Register layout confirmed by the eMMC bring-up
 * (PD bank CFG2 read at PIO+0x48 = 0x0300b000 + 2*0x24 + 0x08, see
 * h616_sdhci.c).  Standard sunxi layout: per-bank stride 0x24,
 * CFGn (8 pins per register, 4 bits per pin: CFG0 = pins 0-7, CFG1 =
 * 8-15, CFG2 = 16-23) at +0x00/+0x04/+0x08, DAT at +0x10, PUL0 (pins
 * 0-15, 2 bits per pin) at +0x1c, PUL1 (pins 16-31) at +0x20.
 * Function codes from the H616 pinctrl table (Linux
 * pinctrl-sun50i-h616.c).  The PIO runs on APB1, which has no separate
 * CCU bus gate (see the dtsi pio node clocks).
 */

#define H616_PIO_BANK(b)     (H616_PIO_ADDR + (b) * 0x24u)

static void h616_pio_mux(int bank, int pin, int func)
{
  uintptr_t reg = H616_PIO_BANK(bank) + (pin >> 3) * 4;
  int       shift = (pin & 7) * 4;
  uint32_t  val = getreg32(reg);

  val = (val & ~(0xfu << shift)) | ((uint32_t)func << shift);
  putreg32(val, reg);
}

/* Pull a pin up (PUL field 01).  RX pins idle high; a floating RX picks
 * up noise and floods the driver with break/error interrupts.
 */

static void h616_pio_pullup(int bank, int pin)
{
  uintptr_t reg = H616_PIO_BANK(bank) + (pin >= 16 ? 0x20u : 0x1cu);
  int       shift = (pin & 15) * 2;
  uint32_t  val = getreg32(reg);

  val = (val & ~(0x3u << shift)) | (1u << shift);
  putreg32(val, reg);
}

/* Bank indices: A=0, C=1, D=2, E=3, F=4, G=5, H=6, I=7 (B does not
 * exist on this package).
 */

#ifdef USE_SERIALDRIVER

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/* Open the APB bus clock gate for UART1..UART5.  UART0 is already ungated
 * by the bootloader (it is the boot console); the others are not, so any
 * register access before ungating would bus-hang.
 */

static void h616_uart_enable_clocks(void)
{
  uintptr_t reg = H616_CCU_UART_GATE;
  uint32_t bits = 0;

#ifdef CONFIG_16550_UART1
  bits |= (1u << 1);
#endif
#ifdef CONFIG_16550_UART2
  bits |= (1u << 2);
#endif
#ifdef CONFIG_16550_UART3
  bits |= (1u << 3);
#endif
#ifdef CONFIG_16550_UART4
  bits |= (1u << 4);
#endif
#ifdef CONFIG_16550_UART5
  bits |= (1u << 5);
#endif

  if (bits != 0)
    {
      putreg32(getreg32(reg) | bits, reg);
    }
}

/* Route the UART pins to their pads.  Without this the controllers
 * answer register access but never reach a physical pin, so /dev/ttyS1-3
 * look alive and are unreachable at the header:
 *   UART1: PG6/PG7  (function 2)      - dedicated pins
 *   UART2: PG15/PG16 (function 2)
 *   UART3: PI9/PI10 (function 3)      - NOTE: conflicts with EMAC0 (PI9/
 *           PI10 default to ETXD1/ETXD0); only claimed when UART3 is on.
 */

static void h616_uart_mux_pins(void)
{
#ifdef CONFIG_16550_UART1
  h616_pio_mux(5, 6, 2);            /* PG6  = UART1 TX */
  h616_pio_mux(5, 7, 2);            /* PG7  = UART1 RX */
  h616_pio_pullup(5, 7);
#endif
#ifdef CONFIG_16550_UART2
  h616_pio_mux(5, 15, 2);           /* PG15 = UART2 TX */
  h616_pio_mux(5, 16, 2);           /* PG16 = UART2 RX */
  h616_pio_pullup(5, 16);
#endif
#ifdef CONFIG_16550_UART3
  h616_pio_mux(7, 9, 3);            /* PI9  = UART3 TX */
  h616_pio_mux(7, 10, 3);           /* PI10 = UART3 RX */
  h616_pio_pullup(7, 10);
#endif
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: arm64_serialinit
 *
 * Description:
 *   see arm64_internal.h
 *
 ***************************************************************************/

void arm64_serialinit(void)
{
  h616_uart_enable_clocks();
  h616_uart_mux_pins();

#ifdef CONFIG_H616_DMA
  /* Initialize the DMAC after the GIC (up_irqinitialize) is up and
   * before any UART write can attach a DMA channel.
   */

  h616_dma_initialize();
#endif
  u16550_serialinit();
}

/***************************************************************************
 * Name: arm64_earlyserialinit
 *
 * Description:
 *   see arm64_internal.h
 *
 ***************************************************************************/

void arm64_earlyserialinit(void)
{
  /* NOTE: do NOT initialize the DMAC here.  This runs before
   * up_irqinitialize() -> arm64_gic_initialize(), so an up_enable_irq()
   * from here would be lost and the DMAC interrupt would never be
   * delivered.
   */

  u16550_earlyserialinit();
}

#endif /* USE_SERIALDRIVER */
