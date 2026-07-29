/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/n947/n947_edma.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_EDMA_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_EDMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/nxxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define N947_EDMA_NCONTROLLERS                 2
#define N947_EDMA_NCHANNELS                    16

/* eDMA management page register offsets */

#define N947_EDMA_MP_CSR_OFFSET                0x0000
#define N947_EDMA_MP_ES_OFFSET                 0x0004
#define N947_EDMA_MP_INT_OFFSET                0x0008
#define N947_EDMA_MP_HRS_OFFSET                0x000c
#define N947_EDMA_CH_GRPRI_OFFSET(n)           (0x0100 + ((n) << 2))

/* Per-channel register offsets.  MCX Nxxx eDMA has one 0x1000-byte page per
 * channel starting at controller_base + 0x1000.
 */

#define N947_EDMA_CH_OFFSET(n)                 (0x1000 + ((n) << 12))

#define N947_EDMA_CH_CSR_OFFSET                0x0000
#define N947_EDMA_CH_ES_OFFSET                 0x0004
#define N947_EDMA_CH_INT_OFFSET                0x0008
#define N947_EDMA_CH_SBR_OFFSET                0x000c
#define N947_EDMA_CH_PRI_OFFSET                0x0010
#define N947_EDMA_CH_MUX_OFFSET                0x0014

#define N947_EDMA_TCD_SADDR_OFFSET             0x0020
#define N947_EDMA_TCD_SOFF_OFFSET              0x0024
#define N947_EDMA_TCD_ATTR_OFFSET              0x0026
#define N947_EDMA_TCD_NBYTES_MLOFFNO_OFFSET    0x0028
#define N947_EDMA_TCD_SLAST_SDA_OFFSET         0x002c
#define N947_EDMA_TCD_DADDR_OFFSET             0x0030
#define N947_EDMA_TCD_DOFF_OFFSET              0x0034
#define N947_EDMA_TCD_CITER_ELINKNO_OFFSET     0x0036
#define N947_EDMA_TCD_DLAST_SGA_OFFSET         0x0038
#define N947_EDMA_TCD_CSR_OFFSET               0x003c
#define N947_EDMA_TCD_BITER_ELINKNO_OFFSET     0x003e

/* eDMA register addresses */

#define N947_EDMA_MP_CSR(b)                    ((b) + N947_EDMA_MP_CSR_OFFSET)
#define N947_EDMA_MP_ES(b)                     ((b) + N947_EDMA_MP_ES_OFFSET)
#define N947_EDMA_MP_INT(b)                    ((b) + N947_EDMA_MP_INT_OFFSET)
#define N947_EDMA_MP_HRS(b)                    ((b) + N947_EDMA_MP_HRS_OFFSET)
#define N947_EDMA_CH_GRPRI(b,n)                ((b) + N947_EDMA_CH_GRPRI_OFFSET(n))

#define N947_EDMA_CH_BASE(b,n)                 ((b) + N947_EDMA_CH_OFFSET(n))
#define N947_EDMA_CH_CSR(b,n)                  (N947_EDMA_CH_BASE(b,n) + N947_EDMA_CH_CSR_OFFSET)
#define N947_EDMA_CH_ES(b,n)                   (N947_EDMA_CH_BASE(b,n) + N947_EDMA_CH_ES_OFFSET)
#define N947_EDMA_CH_INT(b,n)                  (N947_EDMA_CH_BASE(b,n) + N947_EDMA_CH_INT_OFFSET)
#define N947_EDMA_CH_SBR(b,n)                  (N947_EDMA_CH_BASE(b,n) + N947_EDMA_CH_SBR_OFFSET)
#define N947_EDMA_CH_PRI(b,n)                  (N947_EDMA_CH_BASE(b,n) + N947_EDMA_CH_PRI_OFFSET)
#define N947_EDMA_CH_MUX(b,n)                  (N947_EDMA_CH_BASE(b,n) + N947_EDMA_CH_MUX_OFFSET)

#define N947_EDMA_TCD_SADDR(b,n)               (N947_EDMA_CH_BASE(b,n) + N947_EDMA_TCD_SADDR_OFFSET)
#define N947_EDMA_TCD_SOFF(b,n)                (N947_EDMA_CH_BASE(b,n) + N947_EDMA_TCD_SOFF_OFFSET)
#define N947_EDMA_TCD_ATTR(b,n)                (N947_EDMA_CH_BASE(b,n) + N947_EDMA_TCD_ATTR_OFFSET)
#define N947_EDMA_TCD_NBYTES_MLOFFNO(b,n)      (N947_EDMA_CH_BASE(b,n) + N947_EDMA_TCD_NBYTES_MLOFFNO_OFFSET)
#define N947_EDMA_TCD_SLAST_SDA(b,n)           (N947_EDMA_CH_BASE(b,n) + N947_EDMA_TCD_SLAST_SDA_OFFSET)
#define N947_EDMA_TCD_DADDR(b,n)               (N947_EDMA_CH_BASE(b,n) + N947_EDMA_TCD_DADDR_OFFSET)
#define N947_EDMA_TCD_DOFF(b,n)                (N947_EDMA_CH_BASE(b,n) + N947_EDMA_TCD_DOFF_OFFSET)
#define N947_EDMA_TCD_CITER_ELINKNO(b,n)       (N947_EDMA_CH_BASE(b,n) + N947_EDMA_TCD_CITER_ELINKNO_OFFSET)
#define N947_EDMA_TCD_DLAST_SGA(b,n)           (N947_EDMA_CH_BASE(b,n) + N947_EDMA_TCD_DLAST_SGA_OFFSET)
#define N947_EDMA_TCD_CSR(b,n)                 (N947_EDMA_CH_BASE(b,n) + N947_EDMA_TCD_CSR_OFFSET)
#define N947_EDMA_TCD_BITER_ELINKNO(b,n)       (N947_EDMA_CH_BASE(b,n) + N947_EDMA_TCD_BITER_ELINKNO_OFFSET)

/* MP_CSR bit definitions */

#define EDMA_MP_CSR_EDBG                       (1 << 1)
#define EDMA_MP_CSR_ERCA                       (1 << 2)
#define EDMA_MP_CSR_HAE                        (1 << 4)
#define EDMA_MP_CSR_HALT                       (1 << 5)
#define EDMA_MP_CSR_GCLC                       (1 << 6)
#define EDMA_MP_CSR_GMRC                       (1 << 7)
#define EDMA_MP_CSR_ECX                        (1 << 8)
#define EDMA_MP_CSR_CX                         (1 << 9)
#define EDMA_MP_CSR_ACTIVE_ID_SHIFT            24
#define EDMA_MP_CSR_ACTIVE_ID_MASK             (0x0f << EDMA_MP_CSR_ACTIVE_ID_SHIFT)
#define EDMA_MP_CSR_ACTIVE                     (1u << 31)

/* MP_INT bit definitions */

#define EDMA_MP_INT_INT(n)                     (1u << (n))

/* CH_CSR bit definitions */

#define EDMA_CH_CSR_ERQ                        (1 << 0)
#define EDMA_CH_CSR_EARQ                       (1 << 1)
#define EDMA_CH_CSR_EEI                        (1 << 2)
#define EDMA_CH_CSR_EBW                        (1 << 3)
#define EDMA_CH_CSR_DONE                       (1u << 30)
#define EDMA_CH_CSR_ACTIVE                     (1u << 31)

/* CH_ES bit definitions */

#define EDMA_CH_ES_DBE                         (1 << 0)
#define EDMA_CH_ES_SBE                         (1 << 1)
#define EDMA_CH_ES_SGE                         (1 << 2)
#define EDMA_CH_ES_NCE                         (1 << 3)
#define EDMA_CH_ES_DOE                         (1 << 4)
#define EDMA_CH_ES_DAE                         (1 << 5)
#define EDMA_CH_ES_SOE                         (1 << 6)
#define EDMA_CH_ES_SAE                         (1 << 7)
#define EDMA_CH_ES_ERR                         (1u << 31)

/* CH_INT bit definitions */

#define EDMA_CH_INT_INT                        (1 << 0)

/* CH_PRI bit definitions */

#define EDMA_CH_PRI_APL_SHIFT                  0
#define EDMA_CH_PRI_APL_MASK                   (0x07 << EDMA_CH_PRI_APL_SHIFT)
#define EDMA_CH_PRI_APL(n)                     (((uint32_t)(n) << EDMA_CH_PRI_APL_SHIFT) & EDMA_CH_PRI_APL_MASK)
#define EDMA_CH_PRI_DPA                        (1u << 30)
#define EDMA_CH_PRI_ECP                        (1u << 31)

/* CH_MUX bit definitions */

#define EDMA_CH_MUX_SRC_SHIFT                  0
#define EDMA_CH_MUX_SRC_MASK                   (0x7f << EDMA_CH_MUX_SRC_SHIFT)
#define EDMA_CH_MUX_SRC(n)                     (((uint32_t)(n) << EDMA_CH_MUX_SRC_SHIFT) & EDMA_CH_MUX_SRC_MASK)

/* TCD_ATTR bit definitions */

#define EDMA_TCD_ATTR_DSIZE_SHIFT              0
#define EDMA_TCD_ATTR_DSIZE_MASK               (0x07 << EDMA_TCD_ATTR_DSIZE_SHIFT)
#define EDMA_TCD_ATTR_DSIZE(n)                 (((uint16_t)(n) << EDMA_TCD_ATTR_DSIZE_SHIFT) & EDMA_TCD_ATTR_DSIZE_MASK)
#define EDMA_TCD_ATTR_DMOD_SHIFT               3
#define EDMA_TCD_ATTR_DMOD_MASK                (0x1f << EDMA_TCD_ATTR_DMOD_SHIFT)
#define EDMA_TCD_ATTR_DMOD(n)                  (((uint16_t)(n) << EDMA_TCD_ATTR_DMOD_SHIFT) & EDMA_TCD_ATTR_DMOD_MASK)
#define EDMA_TCD_ATTR_SSIZE_SHIFT              8
#define EDMA_TCD_ATTR_SSIZE_MASK               (0x07 << EDMA_TCD_ATTR_SSIZE_SHIFT)
#define EDMA_TCD_ATTR_SSIZE(n)                 (((uint16_t)(n) << EDMA_TCD_ATTR_SSIZE_SHIFT) & EDMA_TCD_ATTR_SSIZE_MASK)
#define EDMA_TCD_ATTR_SMOD_SHIFT               11
#define EDMA_TCD_ATTR_SMOD_MASK                (0x1f << EDMA_TCD_ATTR_SMOD_SHIFT)
#define EDMA_TCD_ATTR_SMOD(n)                  (((uint16_t)(n) << EDMA_TCD_ATTR_SMOD_SHIFT) & EDMA_TCD_ATTR_SMOD_MASK)

/* TCD_NBYTES_MLOFFNO bit definitions */

#define EDMA_TCD_NBYTES_MLOFFNO_NBYTES_MASK    0x3fffffff
#define EDMA_TCD_NBYTES_MLOFFNO_NBYTES(n)      ((uint32_t)(n) & EDMA_TCD_NBYTES_MLOFFNO_NBYTES_MASK)
#define EDMA_TCD_NBYTES_MLOFFNO_DMLOE          (1u << 30)
#define EDMA_TCD_NBYTES_MLOFFNO_SMLOE          (1u << 31)

/* TCD_CITER/TCD_BITER ELINK disabled bit definitions */

#define EDMA_TCD_CITER_ELINKNO_CITER_MASK       0x7fff
#define EDMA_TCD_CITER_ELINKNO_CITER(n)         ((uint16_t)(n) & EDMA_TCD_CITER_ELINKNO_CITER_MASK)
#define EDMA_TCD_CITER_ELINKNO_ELINK            (1u << 15)

#define EDMA_TCD_BITER_ELINKNO_BITER_MASK       0x7fff
#define EDMA_TCD_BITER_ELINKNO_BITER(n)         ((uint16_t)(n) & EDMA_TCD_BITER_ELINKNO_BITER_MASK)
#define EDMA_TCD_BITER_ELINKNO_ELINK            (1u << 15)

/* TCD_CSR bit definitions */

#define EDMA_TCD_CSR_START                      (1 << 0)
#define EDMA_TCD_CSR_INTMAJOR                   (1 << 1)
#define EDMA_TCD_CSR_INTHALF                    (1 << 2)
#define EDMA_TCD_CSR_DREQ                       (1 << 3)
#define EDMA_TCD_CSR_ESG                        (1 << 4)
#define EDMA_TCD_CSR_MAJORELINK                 (1 << 5)
#define EDMA_TCD_CSR_EEOP                       (1 << 6)
#define EDMA_TCD_CSR_ESDA                       (1 << 7)
#define EDMA_TCD_CSR_MAJORLINKCH_SHIFT          8
#define EDMA_TCD_CSR_MAJORLINKCH_MASK           (0x0f << EDMA_TCD_CSR_MAJORLINKCH_SHIFT)
#define EDMA_TCD_CSR_MAJORLINKCH(n)             (((uint16_t)(n) << EDMA_TCD_CSR_MAJORLINKCH_SHIFT) & EDMA_TCD_CSR_MAJORLINKCH_MASK)
#define EDMA_TCD_CSR_BWC_SHIFT                  14
#define EDMA_TCD_CSR_BWC_MASK                   (0x03 << EDMA_TCD_CSR_BWC_SHIFT)
#define EDMA_TCD_CSR_BWC(n)                     (((uint16_t)(n) << EDMA_TCD_CSR_BWC_SHIFT) & EDMA_TCD_CSR_BWC_MASK)

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_EDMA_H */
