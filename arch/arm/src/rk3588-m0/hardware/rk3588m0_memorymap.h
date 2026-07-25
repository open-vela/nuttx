/****************************************************************************
 * arch/arm/src/rk3588-m0/hardware/rk3588m0_memorymap.h
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

#ifndef __ARCH_ARM_SRC_RK3588_M0_HARDWARE_RK3588M0_MEMORYMAP_H
#define __ARCH_ARM_SRC_RK3588_M0_HARDWARE_RK3588M0_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PMU_M0 address windows, from RK3588 TRM Table 9-3 "PMU_M0 Address Remap".
 *
 * The M0 sees six fixed windows.  Each is relocated onto a physical address by
 * a pmu1_sgrf_soc_conN register (PMU1_SGRF is at physical 0xFD582000).  Only
 * the top 16 bits are programmable, i.e. the relocation granularity is 64KB -
 * which is why u-boot passes (load >> 16) through the SIP call.  The remap
 * does not take effect until the MCU is reset, matching u-boot's order of
 * "program the addresses first, release the reset last".
 *
 *   M0 view       size   memory type   usage             relocated by
 *   0x00000000+   512MB  Normal WT     ROM/flash (code)  soc_con9  <- CODE_START
 *   0x20000000+   512MB  Normal WBWA   on-chip RAM       soc_con10 <- SRAM_START
 *   0x40000000+   512MB  Device XN     peripherals       FIXED -> 0xF0000000
 *   0x60000000+   512MB  Normal WBWA   off-chip DDR      soc_con11
 *   0x80000000+   512MB  Normal WT     off-chip DDR      soc_con11
 *   0xA0000000+     1GB  Device XN     peripherals       soc_con12 <- EXPERI
 *
 * Two consequences this port relies on:
 *
 * 1. Code runs from the 0x00000000 window.  ARMv6-M has no VTOR, so the vector
 *    table must live at address 0 and the hardware maps CODE_START there.  The
 *    window is 512MB, so image size is bounded only by the reserved region
 *    behind it (mcu_reserved, 2MB).  That window is backed by DDR and is
 *    writable, so .data/.bss and the heap live there too - verified on target
 *    by the bare-metal heartbeat, which stores into it.
 *
 * 2. Peripherals need no configuration at all: the 0x40000000 window is fixed
 *    to physical 0xF0000000.  So a peripheral at physical address P (all RK3588
 *    peripherals are above 0xF0000000) is reachable at
 *    P - 0xF0000000 + 0x40000000.
 */

#define RK3588M0_PERIPH_WINDOW    0x40000000 /* M0 view of the fixed window   */
#define RK3588M0_PERIPH_PHYS_BASE 0xf0000000 /* physical address it maps to   */

/* Convert a physical peripheral address to the M0's view of it */

#define RK3588M0_PERIPH(phys) \
  ((phys) - RK3588M0_PERIPH_PHYS_BASE + RK3588M0_PERIPH_WINDOW)

/* UARTs (DW APB, 16550-compatible).  UART2 at physical 0xFEB50000 is the
 * shared debug console: u-boot, Linux (fiq-debugger, ttyFIQ0 @1500000 8N1)
 * and the NuttX instance on cpu_l3 all use it.  This port only ever polls
 * LSR.THRE and writes THR - it never reprograms baud/LCR/FIFO and never
 * enables an interrupt, so it cannot disturb the owner of the port.
 */

#define RK3588_UART0_PHYS         0xfd890000
#define RK3588_UART2_PHYS         0xfeb50000

#define RK3588M0_UART0_BASE       RK3588M0_PERIPH(RK3588_UART0_PHYS)
#define RK3588M0_UART2_BASE       RK3588M0_PERIPH(RK3588_UART2_PHYS)

/* 16550 register offsets used by the low-level console */

#define RK3588M0_UART_THR_OFFSET  0x00 /* Transmit holding register  */
#define RK3588M0_UART_LSR_OFFSET  0x14 /* Line status register       */

#define RK3588M0_UART_LSR_THRE    (1 << 5) /* Transmit holding register empty */

/* Mailbox 0 (physical 0xFEC60000): the doorbell used by the rockchip AMP
 * rpmsg transport.  Listed here for the follow-on rpmsg work.
 */

#define RK3588_MAILBOX0_PHYS      0xfec60000
#define RK3588M0_MAILBOX0_BASE    RK3588M0_PERIPH(RK3588_MAILBOX0_PHYS)

/* PMU1_GRF (physical 0xFD58A000).  SOC_STS at offset 0x0060 reports the M0's
 * own state: halted (bit 7), lockup (bit 8), sleeping (bit 9), deepsleep
 * (bit 10).  Useful from the Linux side to tell a wedged M0 (lockup) from an
 * idle one waiting for an interrupt (sleeping).
 */

#define RK3588_PMU1_GRF_PHYS      0xfd58a000
#define RK3588M0_PMU1_GRF_BASE    RK3588M0_PERIPH(RK3588_PMU1_GRF_PHYS)
#define RK3588M0_PMU1GRF_SOC_STS  0x0060

#endif /* __ARCH_ARM_SRC_RK3588_M0_HARDWARE_RK3588M0_MEMORYMAP_H */
