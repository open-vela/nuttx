/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_spi.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_SPI_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI controller count */

#define RK3576_SPI_COUNT            4   /* SPI0 ~ SPI3 */

/* SPI register offsets */

#define SPI_CTRLR0                  0x00   /* Control register 0 */
#define SPI_CTRLR1                  0x04   /* Control register 1 (frame size) */
#define SPI_BAUDR                   0x08   /* Baud rate select */
#define SPI_TXFLR                   0x0c   /* TX FIFO level (R) */
#define SPI_RXFLR                   0x10   /* RX FIFO level (R) */
#define SPI_SR                      0x14   /* Status register (R) */
#define SPI_IMR                     0x18   /* Interrupt mask */
#define SPI_ISR                     0x1c   /* Interrupt status */
#define SPI_RISR                    0x20   /* Raw interrupt status */
#define SPI_ICR                     0x24   /* Interrupt clear (W1C) */
#define SPI_DLR                     0x28   /* Data length register */
#define SPI_TXDR                    0x60   /* TX data register (W) */
#define SPI_RXDR                    0x64   /* RX data register (R) */

/* SPI_CTRLR0 bit definitions */

#define SPI_CTRLR0_DFS_SHIFT        0
#define SPI_CTRLR0_DFS_MASK         (0xf << SPI_CTRLR0_DFS_SHIFT)
#define SPI_CTRLR0_DFS(n)           (((n) - 1) << SPI_CTRLR0_DFS_SHIFT)

#define SPI_CTRLR0_FRF_SHIFT        4
#define SPI_CTRLR0_FRF_MASK         (3 << SPI_CTRLR0_FRF_SHIFT)
#define SPI_CTRLR0_FRF_MOTOROLA     (0 << SPI_CTRLR0_FRF_SHIFT)
#define SPI_CTRLR0_FRF_TI           (1 << SPI_CTRLR0_FRF_SHIFT)
#define SPI_CTRLR0_FRF_MICROWIRE    (2 << SPI_CTRLR0_FRF_SHIFT)

#define SPI_CTRLR0_SCPH             (1 << 6)   /* Clock phase */
#define SPI_CTRLR0_SCPOl            (1 << 7)   /* Clock polarity */

#define SPI_CTRLR0_TMOD_SHIFT       8
#define SPI_CTRLR0_TMOD_MASK        (3 << SPI_CTRLR0_TMOD_SHIFT)
#define SPI_CTRLR0_TMOD_TR          (0 << SPI_CTRLR0_TMOD_SHIFT)
#define SPI_CTRLR0_TMOD_TO          (1 << SPI_CTRLR0_TMOD_SHIFT)
#define SPI_CTRLR0_TMOD_RO          (2 << SPI_CTRLR0_TMOD_SHIFT)
#define SPI_CTRLR0_TMOD_EEPROM      (3 << SPI_CTRLR0_TMOD_SHIFT)

#define SPI_CTRLR0_SRL              (1 << 11)  /* Shift register loop */

/* SPI_SR bit definitions */

#define SPI_SR_BUSY                 (1 << 0)
#define SPI_SR_TFNF                 (1 << 1)   /* TX FIFO not full */
#define SPI_SR_TFE                  (1 << 2)   /* TX FIFO empty */
#define SPI_SR_RFNE                 (1 << 3)   /* RX FIFO not empty */
#define SPI_SR_RFF                  (1 << 4)   /* RX FIFO full */

/* SPI FIFO depth */

#define SPI_FIFO_DEPTH              64

/* SPI clock frequencies */

#define SPI_CLK_1MHZ                1000000
#define SPI_CLK_2MHZ                2000000
#define SPI_CLK_5MHZ                5000000
#define SPI_CLK_10MHZ               10000000
#define SPI_CLK_20MHZ               20000000

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

extern const uint32_t g_spi_base[RK3576_SPI_COUNT];

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_SPI_H */
