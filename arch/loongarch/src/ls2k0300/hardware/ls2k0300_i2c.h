/****************************************************************************
 * arch/loongarch/src/ls2k0300/hardware/ls2k0300_i2c.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain the copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_I2C_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C Register Offsets */

#define LS2K0300_I2C_CR1          0x00
#define LS2K0300_I2C_CR2          0x04
#define LS2K0300_I2C_DR           0x10
#define LS2K0300_I2C_SR1          0x14
#define LS2K0300_I2C_SR2          0x18
#define LS2K0300_I2C_CCR          0x1c
#define LS2K0300_I2C_TRISE        0x20
#define LS2K0300_I2C_FLTR         0x24

/* CR1 bits */

#define I2C_CR1_POS              (1 << 11)
#define I2C_CR1_ACK              (1 << 10)
#define I2C_CR1_STOP             (1 << 9)
#define I2C_CR1_START            (1 << 8)
#define I2C_CR1_PE               (1 << 0)

/* CR2 bits */

#define I2C_CR2_FREQ_MASK        0x3f
#define I2C_CR2_ITBUFEN          (1 << 10)
#define I2C_CR2_ITEVTEN          (1 << 9)
#define I2C_CR2_ITERREN          (1 << 8)
#define I2C_CR2_IRQ_MASK         (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | \
                                  I2C_CR2_ITERREN)

/* SR1 bits */

#define I2C_SR1_AF               (1 << 10)
#define I2C_SR1_ARLO             (1 << 9)
#define I2C_SR1_BERR             (1 << 8)
#define I2C_SR1_TXE              (1 << 7)
#define I2C_SR1_RXNE             (1 << 6)
#define I2C_SR1_BTF              (1 << 2)
#define I2C_SR1_ADDR             (1 << 1)
#define I2C_SR1_SB               (1 << 0)

/* SR2 bits */

#define I2C_SR2_BUSY             (1 << 1)

/* CCR bits */

#define I2C_CCR_FS               (1 << 15)
#define I2C_CCR_DUTY             (1 << 14)

/* Input reference clock (120 MHz) */

#define LS2K0300_I2C_INPUT_CLK    120000000

/* Timeout for polling operations (in microseconds) */

#define LS2K0300_I2C_TIMEOUT_US   100000

#endif /* __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_I2C_H */
