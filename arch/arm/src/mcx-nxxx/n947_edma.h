/****************************************************************************
 * arch/arm/src/mcx-nxxx/n947_edma.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_N947_EDMA_H
#define __ARCH_ARM_SRC_MCX_NXXX_N947_EDMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include "hardware/n947/n947_edma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Transfer configuration flags */

#define EDMA_CONFIG_LINKTYPE_SHIFT       0
#define EDMA_CONFIG_LINKTYPE_MASK        (3 << EDMA_CONFIG_LINKTYPE_SHIFT)
#  define EDMA_CONFIG_LINKTYPE_LINKNONE  (0 << EDMA_CONFIG_LINKTYPE_SHIFT)
#  define EDMA_CONFIG_LINKTYPE_MINORLINK (1 << EDMA_CONFIG_LINKTYPE_SHIFT)
#  define EDMA_CONFIG_LINKTYPE_MAJORLINK (2 << EDMA_CONFIG_LINKTYPE_SHIFT)

#define EDMA_CONFIG_LOOP_SHIFT           2
#define EDMA_CONFIG_LOOP_MASK            (3 << EDMA_CONFIG_LOOP_SHIFT)
#  define EDMA_CONFIG_LOOPNONE           (0 << EDMA_CONFIG_LOOP_SHIFT)
#  define EDMA_CONFIG_LOOPSRC            (1 << EDMA_CONFIG_LOOP_SHIFT)
#  define EDMA_CONFIG_LOOPDEST           (2 << EDMA_CONFIG_LOOP_SHIFT)

#define EDMA_CONFIG_INTHALF              (1 << 4)
#define EDMA_CONFIG_INTMAJOR             (1 << 5)
#define EDMA_CONFIG_SCATTERGATHER         (1 << 6)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void *DMACH_HANDLE;
typedef void (*edma_callback_t)(DMACH_HANDLE handle,
                                void *arg, bool done, int result);

/* eDMA transfer sizes.  These values map directly to TCD_ATTR SSIZE/DSIZE. */

enum n947_edma_sizes_e
{
  EDMA_8BIT    = 0,
  EDMA_16BIT   = 1,
  EDMA_32BIT   = 2,
  EDMA_64BIT   = 3,
  EDMA_16BYTE  = 4,
  EDMA_32BYTE  = 5,
  EDMA_64BYTE  = 6,
};

struct n947_edma_xfrconfig_s
{
  uintptr_t saddr;      /* Source data address */
  uintptr_t daddr;      /* Destination data address */
  int16_t   soff;       /* Source address offset after each minor loop */
  int16_t   doff;       /* Destination address offset after each minor loop */
  uint16_t  iter;       /* Major loop iteration count */
  uint8_t   flags;      /* EDMA_CONFIG_* flags */
  uint8_t   ssize;      /* Source transfer size */
  uint8_t   dsize;      /* Destination transfer size */
  uint32_t  nbytes;     /* Bytes transferred in each minor loop */
#ifdef CONFIG_N947_EDMA_ELINK
  DMACH_HANDLE linkch;  /* Linked channel for LINKTYPE_* */
#endif
};

/* Memory-resident transfer control descriptor.  Its layout is identical to
 * the 32-byte MCX Nxxx eDMA hardware TCD and its address must be 32-byte
 * aligned when used for scatter-gather.
 */

struct n947_edma_tcd_s
{
  uint32_t saddr;
  uint16_t soff;
  uint16_t attr;
  uint32_t nbytes;
  uint32_t slast;
  uint32_t daddr;
  uint16_t doff;
  uint16_t citer;
  uint32_t dlast_sga;
  uint16_t csr;
  uint16_t biter;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

DMACH_HANDLE n947_dmach_alloc(uint32_t reqsrc, uint8_t priority);
void n947_dmach_free(DMACH_HANDLE handle);
int n947_dmach_xfrsetup(DMACH_HANDLE handle,
                        const struct n947_edma_xfrconfig_s *config);
int n947_dmach_sgsetup(DMACH_HANDLE handle,
                       const struct n947_edma_xfrconfig_s *configs,
                       unsigned int count,
                       struct n947_edma_tcd_s *tcds);
int n947_dmach_start(DMACH_HANDLE handle,
                     edma_callback_t callback, void *arg);
void n947_dmach_stop(DMACH_HANDLE handle);
unsigned int n947_dmach_getcount(DMACH_HANDLE handle);
unsigned int n947_dmach_idle(DMACH_HANDLE handle);
void n947_dmach_dump(DMACH_HANDLE handle);

#ifdef CONFIG_N947_EDMA_SELFTEST
int n947_edma_selftest(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_MCX_NXXX_N947_EDMA_H */
