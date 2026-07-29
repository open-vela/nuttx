/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/n947/n947_lpspi.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_LPSPI_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_LPSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/nxxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define N947_LPSPI_VERID_OFFSET       0x0000
#define N947_LPSPI_PARAM_OFFSET       0x0004
#define N947_LPSPI_CR_OFFSET          0x0010
#define N947_LPSPI_SR_OFFSET          0x0014
#define N947_LPSPI_IER_OFFSET         0x0018
#define N947_LPSPI_DER_OFFSET         0x001c
#define N947_LPSPI_CFGR0_OFFSET       0x0020
#define N947_LPSPI_CFGR1_OFFSET       0x0024
#define N947_LPSPI_DMR0_OFFSET        0x0030
#define N947_LPSPI_DMR1_OFFSET        0x0034
#define N947_LPSPI_CCR_OFFSET         0x0040
#define N947_LPSPI_FCR_OFFSET         0x0058
#define N947_LPSPI_FSR_OFFSET         0x005c
#define N947_LPSPI_TCR_OFFSET         0x0060
#define N947_LPSPI_TDR_OFFSET         0x0064
#define N947_LPSPI_RSR_OFFSET         0x0070
#define N947_LPSPI_RDR_OFFSET         0x0074

/* Control Register */

#define LPSPI_CR_MEN                  (1 << 0)
#define LPSPI_CR_RST                  (1 << 1)
#define LPSPI_CR_DOZEN                (1 << 2)
#define LPSPI_CR_DBGEN                (1 << 3)
#define LPSPI_CR_RTF                  (1 << 8)
#define LPSPI_CR_RRF                  (1 << 9)

/* Status Register */

#define LPSPI_SR_TDF                  (1 << 0)
#define LPSPI_SR_RDF                  (1 << 1)
#define LPSPI_SR_WCF                  (1 << 8)
#define LPSPI_SR_FCF                  (1 << 9)
#define LPSPI_SR_TCF                  (1 << 10)
#define LPSPI_SR_TEF                  (1 << 11)
#define LPSPI_SR_REF                  (1 << 12)
#define LPSPI_SR_DMF                  (1 << 13)
#define LPSPI_SR_MBF                  (1 << 24)

#define LPSPI_SR_CLEAR                (LPSPI_SR_WCF | LPSPI_SR_FCF | \
                                       LPSPI_SR_TCF | LPSPI_SR_TEF | \
                                       LPSPI_SR_REF | LPSPI_SR_DMF)

/* DMA Enable Register */

#define LPSPI_DER_TDDE                (1 << 0)
#define LPSPI_DER_RDDE                (1 << 1)

/* Configuration Register 1 */

#define LPSPI_CFGR1_MASTER            (1 << 0)
#define LPSPI_CFGR1_SAMPLE            (1 << 1)
#define LPSPI_CFGR1_AUTOPCS           (1 << 2)
#define LPSPI_CFGR1_NOSTALL           (1 << 3)
#define LPSPI_CFGR1_PCSPOL_SHIFT      (8)
#define LPSPI_CFGR1_PCSPOL_MASK       (0xf << LPSPI_CFGR1_PCSPOL_SHIFT)
#  define LPSPI_CFGR1_PCSPOL_LOW      (0 << LPSPI_CFGR1_PCSPOL_SHIFT)
#  define LPSPI_CFGR1_PCSPOL_HIGH(n)  ((1 << (n)) << LPSPI_CFGR1_PCSPOL_SHIFT)
#define LPSPI_CFGR1_PINCFG_SHIFT      (24)
#define LPSPI_CFGR1_PINCFG_MASK       (3 << LPSPI_CFGR1_PINCFG_SHIFT)
#  define LPSPI_CFGR1_PINCFG_SIN_SOUT (0 << LPSPI_CFGR1_PINCFG_SHIFT)
#define LPSPI_CFGR1_OUTCFG            (1 << 26)
#  define LPSPI_CFGR1_OUTCFG_RETAIN   (0 << 26)
#  define LPSPI_CFGR1_OUTCFG_TRISTATE (1 << 26)
#define LPSPI_CFGR1_PCSCFG            (1 << 27)

/* Clock Configuration Register */

#define LPSPI_CCR_SCKDIV_SHIFT        (0)
#define LPSPI_CCR_SCKDIV_MASK         (0xff << LPSPI_CCR_SCKDIV_SHIFT)
#  define LPSPI_CCR_SCKDIV(n)         ((uint32_t)(n) << LPSPI_CCR_SCKDIV_SHIFT)
#define LPSPI_CCR_DBT_SHIFT           (8)
#define LPSPI_CCR_DBT_MASK            (0xff << LPSPI_CCR_DBT_SHIFT)
#  define LPSPI_CCR_DBT(n)            ((uint32_t)(n) << LPSPI_CCR_DBT_SHIFT)
#define LPSPI_CCR_PCSSCK_SHIFT        (16)
#define LPSPI_CCR_PCSSCK_MASK         (0xff << LPSPI_CCR_PCSSCK_SHIFT)
#  define LPSPI_CCR_PCSSCK(n)         ((uint32_t)(n) << LPSPI_CCR_PCSSCK_SHIFT)
#define LPSPI_CCR_SCKPCS_SHIFT        (24)
#define LPSPI_CCR_SCKPCS_MASK         (0xff << LPSPI_CCR_SCKPCS_SHIFT)
#  define LPSPI_CCR_SCKPCS(n)         ((uint32_t)(n) << LPSPI_CCR_SCKPCS_SHIFT)

/* FIFO Control Register */

#define LPSPI_FCR_TXWATER_SHIFT       (0)
#define LPSPI_FCR_TXWATER_MASK        (0xf << LPSPI_FCR_TXWATER_SHIFT)
#  define LPSPI_FCR_TXWATER(n)        ((uint32_t)(n) << LPSPI_FCR_TXWATER_SHIFT)
#define LPSPI_FCR_RXWATER_SHIFT       (8)
#define LPSPI_FCR_RXWATER_MASK        (0xf << LPSPI_FCR_RXWATER_SHIFT)
#  define LPSPI_FCR_RXWATER(n)        ((uint32_t)(n) << LPSPI_FCR_RXWATER_SHIFT)

/* Transmit Command Register */

#define LPSPI_TCR_FRAMESZ_SHIFT       (0)
#define LPSPI_TCR_FRAMESZ_MASK        (0xfff << LPSPI_TCR_FRAMESZ_SHIFT)
#  define LPSPI_TCR_FRAMESZ(n)        ((uint32_t)(n) << LPSPI_TCR_FRAMESZ_SHIFT)
#define LPSPI_TCR_WIDTH_SHIFT         (16)
#define LPSPI_TCR_WIDTH_MASK          (3 << LPSPI_TCR_WIDTH_SHIFT)
#  define LPSPI_TCR_WIDTH_1BIT        (0 << LPSPI_TCR_WIDTH_SHIFT)
#define LPSPI_TCR_TXMSK               (1 << 18)
#define LPSPI_TCR_RXMSK               (1 << 19)
#define LPSPI_TCR_CONTC               (1 << 20)
#define LPSPI_TCR_CONT                (1 << 21)
#define LPSPI_TCR_BYSW                (1 << 22)
#define LPSPI_TCR_LSBF                (1 << 23)
#define LPSPI_TCR_PCS_SHIFT           (24)
#define LPSPI_TCR_PCS_MASK            (3 << LPSPI_TCR_PCS_SHIFT)
#  define LPSPI_TCR_PCS(n)            ((uint32_t)(n) << LPSPI_TCR_PCS_SHIFT)
#define LPSPI_TCR_PRESCALE_SHIFT      (27)
#define LPSPI_TCR_PRESCALE_MASK       (7 << LPSPI_TCR_PRESCALE_SHIFT)
#  define LPSPI_TCR_PRESCALE(n)       ((uint32_t)(n) << LPSPI_TCR_PRESCALE_SHIFT)
#define LPSPI_TCR_CPHA                (1 << 30)
#define LPSPI_TCR_CPOL                (1 << 31)

/* Receive Status Register */

#define LPSPI_RSR_SOF                 (1 << 0)
#define LPSPI_RSR_RXEMPTY             (1 << 1)

/* Receive Data Register */

#define LPSPI_RDR_DATA_MASK           (0xffffffff)

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_LPSPI_H */
