/****************************************************************************
 * arch/arm/src/t113/hardware/t113_clk.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_CLK_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_CLK_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Layer 1: Oscillator + PLL parameters */

#define T113_HOSC_FREQUENCY       24000000UL

#define T113_PLL_CPUX_N           41
#define T113_PLL_CPUX_FREQUENCY   \
  (T113_HOSC_FREQUENCY * (T113_PLL_CPUX_N + 1))
                                  /* 1008 MHz */

#define T113_PLL_PERI0_N          99
#define T113_PLL_PERI0_2X         \
  (T113_HOSC_FREQUENCY * (T113_PLL_PERI0_N + 1) / 2)
                                  /* 1200 MHz */
#define T113_PLL_PERI0_1X         (T113_PLL_PERI0_2X / 2)
                                  /* 600 MHz */

/* Layer 2: Bus clocks */

#define T113_PSI_DIV_M            2
#define T113_AHB_FREQUENCY        \
  (T113_PLL_PERI0_1X / (T113_PSI_DIV_M + 1))
                                  /* 200 MHz */

#define T113_APB0_DIV_N           1   /* pre: 2^N */
#define T113_APB0_DIV_M           2   /* post: M+1 */
#define T113_APB0_FREQUENCY       \
  (T113_PLL_PERI0_1X / \
   (1 << T113_APB0_DIV_N) / (T113_APB0_DIV_M + 1))
                                  /* 100 MHz */

#define T113_APB1_DIV_N           1
#define T113_APB1_DIV_M           2
#define T113_APB1_FREQUENCY       \
  (T113_PLL_PERI0_1X / \
   (1 << T113_APB1_DIV_N) / (T113_APB1_DIV_M + 1))
                                  /* 100 MHz */

/* Layer 3: Peripheral clocks -- drivers only use these */

#define T113_UART_FREQUENCY       T113_APB1_FREQUENCY
#define T113_I2C_FREQUENCY        T113_APB1_FREQUENCY
#define T113_CAN_FREQUENCY        T113_APB1_FREQUENCY
#define T113_PWM_FREQUENCY        T113_HOSC_FREQUENCY
#define T113_TIMER_FREQUENCY      T113_HOSC_FREQUENCY
#define T113_SPI_SRC_FREQUENCY    T113_PLL_PERI0_1X
#define T113_HSTIMER_FREQUENCY    T113_AHB_FREQUENCY
#define T113_CPUX_FREQUENCY       T113_PLL_CPUX_FREQUENCY

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_CLK_H */
