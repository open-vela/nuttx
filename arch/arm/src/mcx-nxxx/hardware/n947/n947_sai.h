/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/n947/n947_sai.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_SAI_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_SAI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/nxxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets.  MCXN947 SAI1 implements two data channels, each with
 * an eight-word FIFO.  This driver uses data channel 0 for a conventional
 * two-slot I2S stream.
 */

#define N947_SAI_VERID_OFFSET          0x0000
#define N947_SAI_PARAM_OFFSET          0x0004
#define N947_SAI_TCSR_OFFSET           0x0008
#define N947_SAI_TCR1_OFFSET           0x000c
#define N947_SAI_TCR2_OFFSET           0x0010
#define N947_SAI_TCR3_OFFSET           0x0014
#define N947_SAI_TCR4_OFFSET           0x0018
#define N947_SAI_TCR5_OFFSET           0x001c
#define N947_SAI_TDR_OFFSET(n)         (0x0020 + ((n) << 2))
#define N947_SAI_TFR_OFFSET(n)         (0x0040 + ((n) << 2))
#define N947_SAI_TMR_OFFSET            0x0060
#define N947_SAI_RCSR_OFFSET           0x0088
#define N947_SAI_RCR1_OFFSET           0x008c
#define N947_SAI_RCR2_OFFSET           0x0090
#define N947_SAI_RCR3_OFFSET           0x0094
#define N947_SAI_RCR4_OFFSET           0x0098
#define N947_SAI_RCR5_OFFSET           0x009c
#define N947_SAI_RDR_OFFSET(n)         (0x00a0 + ((n) << 2))
#define N947_SAI_RFR_OFFSET(n)         (0x00c0 + ((n) << 2))
#define N947_SAI_RMR_OFFSET            0x00e0
#define N947_SAI_MCR_OFFSET            0x0100

/* Transmit/receive control.  TX and RX use identical bit positions. */

#define SAI_CSR_FRDE                   (1u << 0)
#define SAI_CSR_FWDE                   (1u << 1)
#define SAI_CSR_FRIE                   (1u << 8)
#define SAI_CSR_FWIE                   (1u << 9)
#define SAI_CSR_FEIE                   (1u << 10)
#define SAI_CSR_SEIE                   (1u << 11)
#define SAI_CSR_WSIE                   (1u << 12)
#define SAI_CSR_FRF                    (1u << 16)
#define SAI_CSR_FWF                    (1u << 17)
#define SAI_CSR_FEF                    (1u << 18)
#define SAI_CSR_SEF                    (1u << 19)
#define SAI_CSR_WSF                    (1u << 20)
#define SAI_CSR_SR                     (1u << 24)
#define SAI_CSR_FR                     (1u << 25)
#define SAI_CSR_BCE                    (1u << 28)
#define SAI_CSR_DBGE                   (1u << 29)
#define SAI_CSR_STOPE                  (1u << 30)
#define SAI_TCSR_TE                    (1u << 31)
#define SAI_RCSR_RE                    (1u << 31)

/* Preserve configuration/enable bits while writing one to clear a status
 * flag.  Status bits 16..20 must otherwise be written as zero.
 */

#define SAI_CSR_W1C_MASK               (0x1fu << 16)

/* Configuration 1: FIFO watermark. */

#define SAI_CR1_FW_SHIFT               0
#define SAI_CR1_FW_MASK                (7u << SAI_CR1_FW_SHIFT)
#define SAI_CR1_FW(n)                  (((uint32_t)(n) << SAI_CR1_FW_SHIFT) & \
                                        SAI_CR1_FW_MASK)

/* Configuration 2: bit-clock divider/source and synchronous mode. */

#define SAI_CR2_DIV_SHIFT              0
#define SAI_CR2_DIV_MASK               (0xffu << SAI_CR2_DIV_SHIFT)
#define SAI_CR2_DIV(n)                 (((uint32_t)(n) << SAI_CR2_DIV_SHIFT) & \
                                        SAI_CR2_DIV_MASK)
#define SAI_CR2_BYP                    (1u << 23)
#define SAI_CR2_BCD                    (1u << 24)
#define SAI_CR2_BCP                    (1u << 25)
#define SAI_CR2_MSEL_SHIFT             26
#define SAI_CR2_MSEL_MASK              (3u << SAI_CR2_MSEL_SHIFT)
#define SAI_CR2_MSEL(n)                (((uint32_t)(n) << SAI_CR2_MSEL_SHIFT) & \
                                        SAI_CR2_MSEL_MASK)
#define SAI_CR2_BCI                    (1u << 28)
#define SAI_CR2_BCS                    (1u << 29)
#define SAI_CR2_SYNC_SHIFT             30
#define SAI_CR2_SYNC_MASK              (3u << SAI_CR2_SYNC_SHIFT)
#define SAI_CR2_SYNC(n)                (((uint32_t)(n) << SAI_CR2_SYNC_SHIFT) & \
                                        SAI_CR2_SYNC_MASK)

/* Configuration 3: data-channel enable. */

#define SAI_TCR3_TCE_SHIFT             16
#define SAI_TCR3_TCE_MASK              (3u << SAI_TCR3_TCE_SHIFT)
#define SAI_TCR3_TCE(n)                (((uint32_t)(n) << SAI_TCR3_TCE_SHIFT) & \
                                        SAI_TCR3_TCE_MASK)
#define SAI_RCR3_RCE_SHIFT             16
#define SAI_RCR3_RCE_MASK              (3u << SAI_RCR3_RCE_SHIFT)
#define SAI_RCR3_RCE(n)                (((uint32_t)(n) << SAI_RCR3_RCE_SHIFT) & \
                                        SAI_RCR3_RCE_MASK)

/* Configuration 4: classic I2S frame. */

#define SAI_CR4_FSD                    (1u << 0)
#define SAI_CR4_FSP                    (1u << 1)
#define SAI_CR4_ONDEM                  (1u << 2)
#define SAI_CR4_FSE                    (1u << 3)
#define SAI_CR4_MF                     (1u << 4)
#define SAI_CR4_CHMOD                  (1u << 5)
#define SAI_CR4_SYWD_SHIFT             8
#define SAI_CR4_SYWD_MASK              (0x1fu << SAI_CR4_SYWD_SHIFT)
#define SAI_CR4_SYWD(n)                (((uint32_t)(n) << SAI_CR4_SYWD_SHIFT) & \
                                        SAI_CR4_SYWD_MASK)
#define SAI_CR4_FRSZ_SHIFT             16
#define SAI_CR4_FRSZ_MASK              (0x1fu << SAI_CR4_FRSZ_SHIFT)
#define SAI_CR4_FRSZ(n)                (((uint32_t)(n) << SAI_CR4_FRSZ_SHIFT) & \
                                        SAI_CR4_FRSZ_MASK)
#define SAI_CR4_FPACK_SHIFT            24
#define SAI_CR4_FPACK_MASK             (3u << SAI_CR4_FPACK_SHIFT)
#define SAI_CR4_FCONT                  (1u << 28)

/* Configuration 5: word widths and first shifted bit. */

#define SAI_CR5_FBT_SHIFT              8
#define SAI_CR5_FBT_MASK               (0x1fu << SAI_CR5_FBT_SHIFT)
#define SAI_CR5_FBT(n)                 (((uint32_t)(n) << SAI_CR5_FBT_SHIFT) & \
                                        SAI_CR5_FBT_MASK)
#define SAI_CR5_W0W_SHIFT              16
#define SAI_CR5_W0W_MASK               (0x1fu << SAI_CR5_W0W_SHIFT)
#define SAI_CR5_W0W(n)                 (((uint32_t)(n) << SAI_CR5_W0W_SHIFT) & \
                                        SAI_CR5_W0W_MASK)
#define SAI_CR5_WNW_SHIFT              24
#define SAI_CR5_WNW_MASK               (0x1fu << SAI_CR5_WNW_SHIFT)
#define SAI_CR5_WNW(n)                 (((uint32_t)(n) << SAI_CR5_WNW_SHIFT) & \
                                        SAI_CR5_WNW_MASK)

/* FIFO pointers. */

#define SAI_FR_RFP_SHIFT               0
#define SAI_FR_RFP_MASK                (0xfu << SAI_FR_RFP_SHIFT)
#define SAI_FR_WFP_SHIFT               16
#define SAI_FR_WFP_MASK                (0xfu << SAI_FR_WFP_SHIFT)

/* MCXN947 SYSCON reset aliases for SAI1 (PRESETCTRL2 bit 6). */

#define N947_SYSCON_PRESETCTRLSET2      (NXXX_SYSCON0_BASE + 0x0128)
#define N947_SYSCON_PRESETCTRLCLR2      (NXXX_SYSCON0_BASE + 0x0148)
#define N947_SYSCON_SAI1_RESET          (1u << 6)

/* Hardware stream geometry. */

#define N947_SAI_FIFO_DEPTH             8
#define N947_SAI_DATA_CHANNEL           0
#define N947_SAI_PHYSICAL_SLOTS         2
#define N947_SAI_SLOT_WIDTH             32

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_SAI_H */
