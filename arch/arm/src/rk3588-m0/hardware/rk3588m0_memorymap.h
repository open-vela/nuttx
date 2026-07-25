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

/* Off-chip DDR: two views of the SAME 512MB region, both relocated by
 * soc_con11, which u-boot programs from the FIT "exsram_start" property (SIP
 * MCU_EXSRAM_START_ADDR).  Read the TRM arithmetic carefully - both rows
 * subtract 0x60000000, not their own base:
 *
 *   0x60000000-0x7FFFFFFF  Normal WBWA  phys = addr - 0x60000000 + base
 *   0x80000000-0x9FFFFFFF  Normal WT    phys = addr - 0x60000000 + base
 *
 * So these are not two independent windows: the write-through view sits
 * 0x20000000 above the write-back view of the same physical byte.  Accessing
 * 0x80000000 does not reach "base", it reaches base + 0x20000000.
 *
 * That cost one hardware round: a store to 0x80000000 with base = 0x07b00000
 * landed at 0x27b00000 instead, so the intended location read back as
 * never-written (and the core did not fault, which is why it looked healthy).
 *
 * Consequently EXSRAM_BASE below is the write-back view, which is the one whose
 * address maps directly onto exsram_start.  Reaching a given physical address
 * through the write-through view would require exsram_start to be 0x20000000
 * lower, which is not generally possible.
 */

#define RK3588M0_EXSRAM_BASE      0x60000000 /* maps 1:1 onto exsram_start   */
#define RK3588M0_EXSRAM_WT_ALIAS  0x80000000 /* same bytes, +0x20000000 skew */
#define RK3588M0_EXSRAM_WT_SKEW   0x20000000

/* rpmsg shared memory, carved out of mcu_reserved (physical 0x07a00000, 2MB) so
 * that no new region has to be reserved from Linux:
 *
 *   0x07a00000  1MB    code, data, bss, heap   (code window, 0x00000000)
 *   0x07b00000  64KB   vring0 + vring1         (exsram window, 0x60000000)
 *   0x07b10000  256KB  rpmsg buffer pool       (exsram window, 0x60010000)
 *
 * exsram_start in the FIT points at 0x07b00000, so the shared area starts at
 * RK3588M0_EXSRAM_BASE from this core's point of view, while Linux addresses the
 * same bytes physically through its own reserved-memory nodes.
 *
 * Sizes follow the Linux side (include/linux/rpmsg/rockchip_rpmsg.h): each vring
 * is RPMSG_VRING_SIZE = 0x8000 with the pair occupying RPMSG_VRING_OVERHEAD, and
 * a direction holds RPMSG_BUF_COUNT = 64 buffers of 512 bytes.
 *
 * IMPORTANT: the heap must stay below the vrings. CONFIG_RAM_SIZE is therefore
 * 1MB, not the full 2MB of the carveout - otherwise the heap grows straight into
 * vring0.
 */

#define RK3588M0_RPMSG_PHYS       0x07b00000 /* == FIT exsram_start          */

/* Physical addresses, i.e. how Linux names this memory. These are what belongs
 * in the resource table: OpenAMP treats those entries as physical and runs them
 * through up_addrenv_pa_to_va() before use.
 */

#define RK3588M0_VRING0_PHYS      (RK3588M0_RPMSG_PHYS + 0x00000)
#define RK3588M0_VRING1_PHYS      (RK3588M0_RPMSG_PHYS + 0x08000)
#define RK3588M0_RPMSG_POOL_PHYS  (RK3588M0_RPMSG_PHYS + 0x10000)

/* The same memory as this core addresses it, for code that dereferences
 * directly rather than going through the translation hooks.
 */

#define RK3588M0_VRING0           (RK3588M0_EXSRAM_BASE + 0x00000)
#define RK3588M0_VRING1           (RK3588M0_EXSRAM_BASE + 0x08000)
#define RK3588M0_VRING_SIZE       0x8000
#define RK3588M0_VRING_ALIGN      0x1000

#define RK3588M0_RPMSG_POOL       (RK3588M0_EXSRAM_BASE + 0x10000)
#define RK3588M0_RPMSG_POOL_SIZE  0x40000

#define RK3588M0_RPMSG_BUF_COUNT  64
#define RK3588M0_RPMSG_BUF_SIZE   512

/* Extent of the shared window, i.e. how much of the carveout past
 * RK3588M0_RPMSG_PHYS is reachable at RK3588M0_EXSRAM_BASE. Used by the address
 * translation hooks to decide whether an address needs rebasing between the
 * physical view Linux writes into the vrings and this core's window view.
 */

#define RK3588M0_RPMSG_WINDOW_SIZE 0x100000  /* 1MB: vrings + buffer pool */

/* Writable copy of the resource table. OpenAMP writes notify ids back into it,
 * so it cannot live in .rodata - that mistake crashed the cpu_l3 port, where
 * .rodata is mapped read-only.
 *
 * It sits at the top of this core's own carveout, just above the heap, rather
 * than past the buffer pool: everything from 0x07b50000 onwards (the end of the
 * dma pool Linux declares) is ordinary memory that Linux is free to allocate,
 * so writing a resource table there would corrupt the kernel. CONFIG_RAM_SIZE
 * is set to stop below this address so the heap cannot reach it either.
 */

#define RK3588M0_RSC_TABLE       0x000f0000  /* phys 0x07af0000 */

/* Observation area, readable from Linux with /dev/mem.
 *
 * This deliberately sits above the heap rather than at the bottom of the
 * carveout. The bare-metal bring-up firmware published its heartbeat at offset
 * 0x800 because the image was 40 bytes long and everything above it was free;
 * carrying that address over to NuttX was a mistake, because a 65KB image puts
 * offset 0x800 in the middle of .text, so every counter write was overwriting
 * instructions. It survived only because the corrupted functions happened not to
 * run again - the diagnostics gave it away by reading back machine code from the
 * miss counter that was never written.
 *
 * Layout of the carveout (offsets are this core's view, physical adds
 * 0x07a00000):
 *
 *   0x00000-0xe0000  code, data, bss, heap   (CONFIG_RAM_SIZE stops here)
 *   0xe0000          this observation area
 *   0xf0000          writable resource table
 */

#define RK3588M0_DIAG_BASE       0x000e0000  /* phys 0x07ae0000 */

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

/* Mailboxes: the doorbell used by the rockchip AMP rpmsg transport.
 *
 * This core shares mailbox0 with the cpu_l3 link but on different channels.
 * mailbox1 looked like the tidier choice and was tried first, but its registers
 * turn out to be inaccessible: the Linux mailbox driver aborted with a
 * synchronous external abort the moment rockchip_mbox_startup() read
 * B2A_INTEN. Note that its probe "version: 0x0100" message proves nothing -
 * for the 1.0.0 controller that value is a driver constant, not a register
 * read, so probe never touched the hardware. mailbox0, by contrast, is proven
 * working by the cpu_l3 link.
 *
 * Channel map on mailbox0:
 *   ch0  cpu_l3 link TX (B2A) - do not touch
 *   ch1  this core's TX (B2A)
 *   ch2  this core's RX (A2B)
 *   ch3  cpu_l3 link RX (A2B) - do not touch
 *
 * The rockchip link-id packs a 4-bit master cpu id and a 4-bit remote id, and
 * the RK3576 AMP dtsi documents the MCU as remote 4, hence link-id 0x04 here.
 */

#define RK3588_MAILBOX0_PHYS      0xfec60000 /* cpu_l3 link - do not touch */
#define RK3588_MAILBOX1_PHYS      0xfec70000 /* this core's link          */
#define RK3588_MAILBOX2_PHYS      0xfece0000 /* unused                    */

#define RK3588M0_MAILBOX0_BASE    RK3588M0_PERIPH(RK3588_MAILBOX0_PHYS)
#define RK3588M0_MAILBOX1_BASE    RK3588M0_PERIPH(RK3588_MAILBOX1_PHYS)

/* Mailbox register layout (rockchip,rk3368-mailbox).  A2B is the direction the
 * A cores write and this core reads; B2A is the reverse.  Writing the CMD
 * register is what latches the doorbell - writing DAT alone does nothing, a
 * detail that cost a debugging round on the cpu_l3 side.
 */

#define RK3588M0_MBOX_A2B_INTEN   0x00
#define RK3588M0_MBOX_A2B_STATUS  0x04
#define RK3588M0_MBOX_A2B_CMD(n)  (0x08 + (n) * 0x08)
#define RK3588M0_MBOX_A2B_DAT(n)  (0x0c + (n) * 0x08)
#define RK3588M0_MBOX_B2A_INTEN   0x28
#define RK3588M0_MBOX_B2A_STATUS  0x2c
#define RK3588M0_MBOX_B2A_CMD(n)  (0x30 + (n) * 0x08)
#define RK3588M0_MBOX_B2A_DAT(n)  (0x34 + (n) * 0x08)

/* Channels 1 and 2 of mailbox0: this core announces on B2A channel 1 (the Linux
 * "rpmsg-rx" mailbox of the M0 link) and listens on A2B channel 2 (its
 * "rpmsg-tx"). Channels 0 and 3 belong to the cpu_l3 link.
 */

#define RK3588M0_MBOX_TX_CHAN     1
#define RK3588M0_MBOX_RX_CHAN     2

/* Handshake value Linux checks in the DAT register before accepting a kick
 * (include/linux/rpmsg/rockchip_rpmsg.h RPMSG_MBOX_MAGIC).
 */

#define RK3588M0_RPMSG_MBOX_MAGIC 0x524d5347 /* "RMSG" */
#define RK3588M0_RPMSG_LINK_ID    0x04       /* master cpu0, remote id 4 */

/* PMU1_GRF (physical 0xFD58A000).  SOC_STS at offset 0x0060 reports the M0's
 * own state: halted (bit 7), lockup (bit 8), sleeping (bit 9), deepsleep
 * (bit 10).  Useful from the Linux side to tell a wedged M0 (lockup) from an
 * idle one waiting for an interrupt (sleeping).
 */

#define RK3588_PMU1_GRF_PHYS      0xfd58a000
#define RK3588M0_PMU1_GRF_BASE    RK3588M0_PERIPH(RK3588_PMU1_GRF_PHYS)
#define RK3588M0_PMU1GRF_SOC_STS  0x0060

#endif /* __ARCH_ARM_SRC_RK3588_M0_HARDWARE_RK3588M0_MEMORYMAP_H */
