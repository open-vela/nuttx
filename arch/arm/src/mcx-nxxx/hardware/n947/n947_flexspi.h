/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/n947/n947_flexspi.h
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

/* MCXN947 FlexSPI0 register definitions.
 * Same Synopsys/NXP FlexSPI IP as i.MXRT - register layout verified
 * identical against SDK PERI_FLEXSPI.h.
 */

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_N947_FLEXSPI_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_N947_FLEXSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/nxxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FlexSPI0 non-secure base and AHB (memory-mapped) base */

#define N947_FLEXSPI0_BASE         (0x400c8000u)

/* AHB flash mapping. */

#define N947_FLEXSPI0_AMBA_BASE    (0x80000000u)

/* Clock/reset in SYSCON0 (base 0x40000000) */

#define N947_SYSCON0_AHBCLKCTRLSET0  (NXXX_SYSCON0_BASE + 0x220) /* set gate */
#define N947_SYSCON0_AHBCLKCTRLCLR0  (NXXX_SYSCON0_BASE + 0x240)
#define N947_SYSCON0_PRESETCTRLSET0  (NXXX_SYSCON0_BASE + 0x120) /* assert  */
#define N947_SYSCON0_PRESETCTRLCLR0  (NXXX_SYSCON0_BASE + 0x140) /* release */

/* AHB_CLK_CTRL0/PRESETCTRL0 bit 11 */

#define N947_FLEXSPI_CLKRST_BIT      (1u << 11)

#define N947_SYSCON0_FLEXSPICLKSEL   (NXXX_SYSCON0_BASE + 0x4a8)
#define N947_SYSCON0_FLEXSPICLKDIV   (NXXX_SYSCON0_BASE + 0x4ac)
#define N947_FLEXSPICLKSEL_PLL0      (1u)          /* SEL=1 -> PLL0 */

/* Register offsets *********************************************************/

#define N947_FLEXSPI_MCR0_OFFSET        (0x000)
#define N947_FLEXSPI_MCR1_OFFSET        (0x004)
#define N947_FLEXSPI_MCR2_OFFSET        (0x008)
#define N947_FLEXSPI_AHBCR_OFFSET       (0x00c)
#define N947_FLEXSPI_INTEN_OFFSET       (0x010)
#define N947_FLEXSPI_INTR_OFFSET        (0x014)
#define N947_FLEXSPI_LUTKEY_OFFSET      (0x018)
#define N947_FLEXSPI_LUTCR_OFFSET       (0x01c)
#define N947_FLEXSPI_AHBRXBUFCR0_OFFSET (0x020)   /* [0..7] step 4 */
#define N947_FLEXSPI_FLSHCR0_OFFSET     (0x060)   /* [0..3] A1/A2/B1/B2 */
#define N947_FLEXSPI_FLSHCR1_OFFSET     (0x070)
#define N947_FLEXSPI_FLSHCR2_OFFSET     (0x080)
#define N947_FLEXSPI_FLSHCR4_OFFSET     (0x094)
#define N947_FLEXSPI_IPCR0_OFFSET       (0x0a0)
#define N947_FLEXSPI_IPCR1_OFFSET       (0x0a4)
#define N947_FLEXSPI_IPCMD_OFFSET       (0x0b0)
#define N947_FLEXSPI_IPRXFCR_OFFSET     (0x0b8)
#define N947_FLEXSPI_IPTXFCR_OFFSET     (0x0bc)
#define N947_FLEXSPI_STS0_OFFSET        (0x0e0)
#define N947_FLEXSPI_STS1_OFFSET        (0x0e4)
#define N947_FLEXSPI_STS2_OFFSET        (0x0e8)
#define N947_FLEXSPI_IPRXFSTS_OFFSET    (0x0f0)
#define N947_FLEXSPI_IPTXFSTS_OFFSET    (0x0f4)
#define N947_FLEXSPI_RFDR_OFFSET        (0x100)   /* [0..31] step 4 */
#define N947_FLEXSPI_TFDR_OFFSET        (0x180)   /* [0..31] step 4 */
#define N947_FLEXSPI_LUT_OFFSET         (0x200)   /* [0..63] step 4 */

/* Absolute addresses *******************************************************/

#define N947_FLEXSPI_MCR0      (N947_FLEXSPI0_BASE + N947_FLEXSPI_MCR0_OFFSET)
#define N947_FLEXSPI_MCR1      (N947_FLEXSPI0_BASE + N947_FLEXSPI_MCR1_OFFSET)
#define N947_FLEXSPI_MCR2      (N947_FLEXSPI0_BASE + N947_FLEXSPI_MCR2_OFFSET)
#define N947_FLEXSPI_AHBCR     (N947_FLEXSPI0_BASE + N947_FLEXSPI_AHBCR_OFFSET)
#define N947_FLEXSPI_INTEN     (N947_FLEXSPI0_BASE + N947_FLEXSPI_INTEN_OFFSET)
#define N947_FLEXSPI_INTR      (N947_FLEXSPI0_BASE + N947_FLEXSPI_INTR_OFFSET)
#define N947_FLEXSPI_LUTKEY    (N947_FLEXSPI0_BASE + N947_FLEXSPI_LUTKEY_OFFSET)
#define N947_FLEXSPI_LUTCR     (N947_FLEXSPI0_BASE + N947_FLEXSPI_LUTCR_OFFSET)
#define N947_FLEXSPI_FLSHCR0(n) (N947_FLEXSPI0_BASE + N947_FLEXSPI_FLSHCR0_OFFSET + ((n) << 2))
#define N947_FLEXSPI_FLSHCR1(n) (N947_FLEXSPI0_BASE + N947_FLEXSPI_FLSHCR1_OFFSET + ((n) << 2))
#define N947_FLEXSPI_FLSHCR2(n) (N947_FLEXSPI0_BASE + N947_FLEXSPI_FLSHCR2_OFFSET + ((n) << 2))
#define N947_FLEXSPI_IPCR0     (N947_FLEXSPI0_BASE + N947_FLEXSPI_IPCR0_OFFSET)
#define N947_FLEXSPI_IPCR1     (N947_FLEXSPI0_BASE + N947_FLEXSPI_IPCR1_OFFSET)
#define N947_FLEXSPI_IPCMD     (N947_FLEXSPI0_BASE + N947_FLEXSPI_IPCMD_OFFSET)
#define N947_FLEXSPI_IPRXFCR   (N947_FLEXSPI0_BASE + N947_FLEXSPI_IPRXFCR_OFFSET)
#define N947_FLEXSPI_IPTXFCR   (N947_FLEXSPI0_BASE + N947_FLEXSPI_IPTXFCR_OFFSET)
#define N947_FLEXSPI_STS0      (N947_FLEXSPI0_BASE + N947_FLEXSPI_STS0_OFFSET)
#define N947_FLEXSPI_STS1      (N947_FLEXSPI0_BASE + N947_FLEXSPI_STS1_OFFSET)
#define N947_FLEXSPI_STS2      (N947_FLEXSPI0_BASE + N947_FLEXSPI_STS2_OFFSET)
#define N947_FLEXSPI_IPRXFSTS  (N947_FLEXSPI0_BASE + N947_FLEXSPI_IPRXFSTS_OFFSET)
#define N947_FLEXSPI_IPTXFSTS  (N947_FLEXSPI0_BASE + N947_FLEXSPI_IPTXFSTS_OFFSET)
#define N947_FLEXSPI_RFDR(n)   (N947_FLEXSPI0_BASE + N947_FLEXSPI_RFDR_OFFSET + ((n) << 2))
#define N947_FLEXSPI_TFDR(n)   (N947_FLEXSPI0_BASE + N947_FLEXSPI_TFDR_OFFSET + ((n) << 2))
#define N947_FLEXSPI_LUT(n)    (N947_FLEXSPI0_BASE + N947_FLEXSPI_LUT_OFFSET + ((n) << 2))

/* Register bit fields ******************************************************/

/* MCR0 */

#define FLEXSPI_MCR0_SWRESET       (1u << 0)
#define FLEXSPI_MCR0_MDIS          (1u << 1)
#define FLEXSPI_MCR0_RXCLKSRC_SHIFT (4)
#define FLEXSPI_MCR0_ARDFEN        (1u << 6)
#define FLEXSPI_MCR0_ATDFEN        (1u << 7)
#define FLEXSPI_MCR0_IPGRANTWAIT_SHIFT (16)
#define FLEXSPI_MCR0_AHBGRANTWAIT_SHIFT (24)

/* MCR2 */

#define FLEXSPI_MCR2_CLRAHBBUFOPT  (1u << 11)
#define FLEXSPI_MCR2_CLRLEARNPHASE (1u << 14)

/* AHBCR */

#define FLEXSPI_AHBCR_PREFETCHEN   (1u << 5)
#define FLEXSPI_AHBCR_BUFFERABLEEN (1u << 4)
#define FLEXSPI_AHBCR_CACHABLEEN   (1u << 3)

/* INTR */

#define FLEXSPI_INTR_IPCMDDONE     (1u << 0)
#define FLEXSPI_INTR_IPCMDGE       (1u << 1)
#define FLEXSPI_INTR_IPCMDERR      (1u << 3)
#define FLEXSPI_INTR_IPRXWA        (1u << 5)
#define FLEXSPI_INTR_IPTXWE        (1u << 6)

/* LUTKEY / LUTCR */

#define FLEXSPI_LUTKEY_VALUE       (0x5af05af0u)
#define FLEXSPI_LUTCR_LOCK         (1u << 0)
#define FLEXSPI_LUTCR_UNLOCK       (1u << 1)

/* IPCR1: data size [15:0], seq idx [31:24], seq num [29:24]? use shifts */

#define FLEXSPI_IPCR1_ISEQID_SHIFT (16)
#define FLEXSPI_IPCR1_ISEQNUM_SHIFT (24)

/* IPCMD */

#define FLEXSPI_IPCMD_TRG          (1u << 0)

/* IPRXFCR / IPTXFCR */

#define FLEXSPI_IPRXFCR_CLRIPRXF   (1u << 0)
#define FLEXSPI_IPTXFCR_CLRIPTXF   (1u << 0)

/* STS0 */

#define FLEXSPI_STS0_SEQIDLE       (1u << 0)
#define FLEXSPI_STS0_ARBIDLE       (1u << 1)

/* LUT instruction encoding (verified against SDK fsl_flexspi.h):
 * each 16-bit instruction = (opcode<<10)|(numpads<<8)|operand,
 * two instructions packed per 32-bit LUT word.
 */

/* opcodes (SDK kFLEXSPI_Command_*) */

#define LUT_CMD_STOP               (0x00)
#define LUT_CMD_SDR                (0x01)   /* transmit command code */
#define LUT_CMD_RADDR_SDR          (0x02)   /* transmit address */
#define LUT_CMD_MODE8_SDR          (0x07)   /* 8-bit mode bits */
#define LUT_CMD_WRITE_SDR          (0x08)   /* transmit program data */
#define LUT_CMD_READ_SDR           (0x09)   /* receive read data */
#define LUT_CMD_DUMMY_SDR          (0x0c)   /* dummy cycles */

/* num pads (SDK kFLEXSPI_xPAD) */

#define LUT_PAD1                   (0x00)
#define LUT_PAD4                   (0x02)

#define LUT_INSTR(opcode, pads, operand) \
  ((uint16_t)(((opcode) << 10) | ((pads) << 8) | (operand)))

#define LUT_SEQ(op0, pad0, opr0, op1, pad1, opr1) \
  (((uint32_t)LUT_INSTR(op1, pad1, opr1) << 16) | LUT_INSTR(op0, pad0, opr0))

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_N947_FLEXSPI_H */
