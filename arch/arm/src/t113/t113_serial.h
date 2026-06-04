/****************************************************************************
 * arch/arm/src/t113/t113_serial.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_SERIAL_H
#define __ARCH_ARM_SRC_T113_T113_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* T113 hardware supports DL=1 (max baud = APB1/16 = 6.25 Mbaud at
 * APB1=100 MHz).  Allow the full hardware range so the upper-half can
 * configure tcsetattr at any T113-supported rate.  At DL<4 the 16x
 * oversample window per bit is tight; users requesting >1.5 Mbaud are
 * expected to verify integrity at their hardware design margin.
 */

#define UART_MINDL 1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifdef CONFIG_SERIAL_RXDMA
void t113_serial_dma_poll(void);
#endif

#endif /* __ARCH_ARM_SRC_T113_T113_SERIAL_H */
