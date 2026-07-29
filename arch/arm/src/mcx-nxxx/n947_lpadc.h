/****************************************************************************
 * arch/arm/src/mcx-nxxx/n947_lpadc.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_N947_LPADC_H
#define __ARCH_ARM_SRC_MCX_NXXX_N947_LPADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

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

/****************************************************************************
 * Name: n947_lpadc_initialize
 *
 * Description:
 *   Initialize one MCX-Nxxx LPADC instance as a NuttX ADC lower-half.
 *
 * Input Parameters:
 *   intf      - LPADC instance number: 0 for ADC0, 1 for ADC1
 *   chanlist  - The list of ADC channel numbers to sample
 *   nchannels - Number of channels in chanlist
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; NULL on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_N947_LPADC
struct adc_dev_s *n947_lpadc_initialize(int intf,
                                        const uint8_t *chanlist,
                                        int nchannels);

/****************************************************************************
 * Name: n947_lpadc_boardinitialize
 *
 * Description:
 *   Optional board hook for configuring analog pins before LPADC use.
 *   The weak default implementation does nothing.
 *
 ****************************************************************************/

int n947_lpadc_boardinitialize(int intf, const uint8_t *chanlist,
                               int nchannels);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_MCX_NXXX_N947_LPADC_H */
