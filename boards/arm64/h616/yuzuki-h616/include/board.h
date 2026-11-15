/****************************************************************************
 * boards/arm64/h616/yuzuki-h616/include/board.h
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

#ifndef __BOARDS_ARM64_H616_YUZUKI_H616_INCLUDE_BOARD_H
#define __BOARDS_ARM64_H616_YUZUKI_H616_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define BOARD_SYSCLK_FREQUENCY 1008000000  /* CPU 1.008GHz (default) */

/* UART0 (console) **********************************************************/

#define BOARD_UART0_BASE   0x05000000  /* H616 UART0, DW 16550 */
#define BOARD_UART0_IRQ    32          /* GIC SPI 0 + 32 */
#define BOARD_UART0_CLOCK  24000000    /* OSC24M */

/* LED definitions **********************************************************/

/* Yuzuki H616 has a power LED; no user LED by default. */

#define LED_STARTED       0
#define LED_HEAPALLOCATE  0
#define LED_IRQSENABLED   0
#define LED_STACKCREATED  0
#define LED_INIRQ         0
#define LED_SIGNAL        0
#define LED_ASSERTION     0
#define LED_PANIC         0

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __BOARDS_ARM64_H616_YUZUKI_H616_INCLUDE_BOARD_H */
