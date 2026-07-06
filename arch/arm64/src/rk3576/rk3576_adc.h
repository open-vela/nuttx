/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_adc.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_ADC_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC reference voltage (mV) */

#define RK3576_ADC_VREF_MV         1800

/* ADC resolution */

#define RK3576_ADC_RESOLUTION      12

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialization */

void rk3576_adc_init(void);

/* Single conversion */

int  rk3576_adc_read(int channel);
int  rk3576_adc_read_mv(int channel);
int  rk3576_adc_read_voltage(int channel, int *mv);

/* Averaged reading */

int  rk3576_adc_read_avg(int channel, int samples);

/* Channel configuration */

int  rk3576_adc_set_channel(int channel);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_ADC_H */
