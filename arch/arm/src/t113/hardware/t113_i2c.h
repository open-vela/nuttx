/****************************************************************************
 * arch/arm/src/t113/hardware/t113_i2c.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_I2C_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "t113_ccu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TWI (I2C) controller base addresses */

#define T113_TWI0_BASE   0x02502000
#define T113_TWI1_BASE   0x02502400
#define T113_TWI2_BASE   0x02502800
#define T113_TWI3_BASE   0x02502c00

/* TWI register offsets */

#define TWI_ADDR_REG     0x0000
#define TWI_XADDR_REG    0x0004
#define TWI_DATA_REG     0x0008
#define TWI_CNTR_REG     0x000c
#define TWI_STAT_REG     0x0010
#define TWI_CCR_REG      0x0014
#define TWI_SRST_REG     0x0018
#define TWI_EFR_REG      0x001c
#define TWI_LCR_REG      0x0020

/* TWI_CNTR bits */

#define TWI_CNTR_INT_EN  (1 << 7)
#define TWI_CNTR_BUS_EN  (1 << 6)
#define TWI_CNTR_START   (1 << 5)
#define TWI_CNTR_STOP    (1 << 4)
#define TWI_CNTR_INT_FL  (1 << 3)
#define TWI_CNTR_A_ACK   (1 << 2)

/* TWI_CCR: Fscl = Fclk_src / (2^N * (M+1) * 10) */

#define TWI_CCR_CLK_N_SHIFT  0
#define TWI_CCR_CLK_N_MASK   (0x7 << TWI_CCR_CLK_N_SHIFT)
#define TWI_CCR_CLK_M_SHIFT  3
#define TWI_CCR_CLK_M_MASK   (0xf << TWI_CCR_CLK_M_SHIFT)

/* TWI_SRST */

#define TWI_SRST_RESET   (1 << 0)

/* TWI status codes (TWI_STAT_REG) */

#define TWI_STAT_BUS_ERR       0x00
#define TWI_STAT_START         0x08
#define TWI_STAT_RSTART        0x10
#define TWI_STAT_ADDR_W_ACK    0x18
#define TWI_STAT_ADDR_W_NAK    0x20
#define TWI_STAT_DATA_T_ACK    0x28
#define TWI_STAT_DATA_T_NAK    0x30
#define TWI_STAT_ARB_LOST      0x38
#define TWI_STAT_ADDR_R_ACK    0x40
#define TWI_STAT_ADDR_R_NAK    0x48
#define TWI_STAT_DATA_R_ACK    0x50
#define TWI_STAT_DATA_R_NAK    0x58
#define TWI_STAT_IDLE          0xf8

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_I2C_H */
