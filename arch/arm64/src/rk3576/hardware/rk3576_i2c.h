/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_i2c.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_I2C_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C controller count */

#define RK3576_I2C_COUNT            5   /* I2C0 ~ I2C4 */

/* I2C register offsets */

#define I2C_CON                     0x00   /* Control register */
#define I2C_CLKDIV                  0x04   /* Clock divisor */
#define I2C_MADDR                   0x08   /* Slave address */
#define I2C_DAT                     0x0c   /* Data register */
#define I2C_INTSTS                  0x10   /* Interrupt status */
#define I2C_INTEN                   0x14   /* Interrupt enable */
#define I2C_INTCLR                  0x18   /* Interrupt clear (W1C) */
#define I2C_MSTRT                   0x1c   /* Master start */
#define I2C_MSTOP                   0x20   /* Master stop */
#define I2C_NAKRXDATA               0x24   /* NAK received data */
#define I2C_SR                      0x28   /* Status register */
#define I2C_TFR                     0x2c   /* TX FIFO register */
#define I2C_RFR                     0x30   /* RX FIFO register */

/* I2C_CON bit definitions */

#define I2C_CON_EN                  (1 << 0)   /* I2C enable */
#define I2C_CON_START               (1 << 1)   /* Generate START */
#define I2C_CON_STOP                (1 << 2)   /* Generate STOP */
#define I2C_CON_ACK                 (1 << 3)   /* ACK (0=ACK, 1=NACK) */
#define I2C_CON_MODE_SHIFT          4
#define I2C_CON_MODE_MASK           (1 << I2C_CON_MODE_SHIFT)
#define I2C_CON_MODE_TX             (0 << I2C_CON_MODE_SHIFT)
#define I2C_CON_MODE_RX             (1 << I2C_CON_MODE_SHIFT)
#define I2C_CON_INT_EN              (1 << 5)   /* Interrupt enable */

/* I2C_SR bit definitions */

#define I2C_SR_BUSY                 (1 << 0)   /* Bus busy */
#define I2C_SR_AL                   (1 << 1)   /* Arbitration lost */
#define I2C_SR_RX_FULL              (1 << 2)   /* RX FIFO full */
#define I2C_SR_TX_EMPTY             (1 << 3)   /* TX FIFO empty */

/* I2C_INTSTS bit definitions */

#define I2C_INTSTS_MBTF             (1 << 0)   /* Master TX finished */
#define I2C_INTSTS_MRBF             (1 << 1)   /* Master RX finished */
#define I2C_INTSTS_NAK              (1 << 2)   /* NACK received */
#define I2C_INTSTS_STOP             (1 << 3)   /* STOP detected */
#define I2C_INTSTS_START            (1 << 4)   /* START detected */

/* I2C_INTEN bit definitions */

#define I2C_INTEN_MBTF_EN           (1 << 0)   /* TX finished IRQ */
#define I2C_INTEN_MRBF_EN           (1 << 1)   /* RX finished IRQ */
#define I2C_INTEN_NAK_EN            (1 << 2)   /* NACK IRQ */
#define I2C_INTEN_STOP_EN           (1 << 3)   /* STOP IRQ */

/* I2C clock frequencies */

#define I2C_CLK_100KHZ              100000
#define I2C_CLK_400KHZ              400000
#define I2C_CLK_1MHZ                1000000

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

extern const uint32_t g_i2c_base[RK3576_I2C_COUNT];

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_I2C_H */
