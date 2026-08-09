/****************************************************************************
 * arch/arm/src/bk7258/hardware/bk7258_memorymap.h
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

/* Addresses from ARMINO SDK ap/include/soc/bk7258/reg_base.h.  Secure and
 * Non-secure addresses differ by 0x10000000; the Secure view is used here
 * (SOC_ADDR_OFFSET = 0 when CONFIG_SPE).  Single core CPU0, single
 * security domain.
 */

#ifndef __ARCH_ARM_SRC_BK7258_HARDWARE_BK7258_MEMORYMAP_H
#define __ARCH_ARM_SRC_BK7258_HARDWARE_BK7258_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory regions ***********************************************************/

#define BK7258_ITCM_BASE       0x00000000  /* Per-core private ITCM */
#define BK7258_FLASH_BASE      0x02000000  /* Flash XIP data base, 8MB */
#define BK7258_ROM_BASE        0x06000000  /* Boot ROM */
#define BK7258_DTCM_BASE       0x20000000  /* Per-core private DTCM */
#define BK7258_SRAM_BASE       0x28000000  /* Shared SRAM (SRAM0-5) */
#define BK7258_SRAM_SIZE       0x000a0000  /* 640 KB */
#define BK7258_PSRAM_BASE      0x60000000  /* PSRAM data window (64MB) */
#define BK7258_QSPI0_BASE      0x64000000  /* QSPI0 data window */
#define BK7258_QSPI1_BASE      0x68000000  /* QSPI1 data window */

/* Peripheral register base addresses ***************************************/

#define BK7258_AON_PMU_BASE    0x44000000  /* AON PMU */
#define BK7258_AON_GPIO_BASE   0x44000400  /* AON GPIO */
#define BK7258_AON_RTC_BASE    0x44000200  /* AON RTC */
#define BK7258_AON_WDT_BASE    0x44000600  /* AON WDT */
#define BK7258_SYS_BASE        0x44010000  /* System control */
#define BK7258_FLASH_REG_BASE  0x44030000  /* Flash controller */
#define BK7258_WDT_BASE        0x44800000  /* Watchdog */
#define BK7258_TIMER0_BASE     0x44810000  /* Timer0 */
#define BK7258_UART0_BASE      0x44820000  /* UART0 (CP core log port) */
#define BK7258_SPI0_BASE       0x44870000  /* SPI0 */
#define BK7258_TIMER1_BASE     0x45800000  /* Timer1 */
#define BK7258_UART1_BASE      0x45830000  /* UART1 (AP core console) */
#define BK7258_UART2_BASE      0x45840000  /* UART2 */
#define BK7258_I2C0_BASE       0x45850000  /* I2C0 */
#define BK7258_I2C1_BASE       0x45860000  /* I2C1 */
#define BK7258_SPI1_BASE       0x45880000  /* SPI1 */
#define BK7258_PWM_BASE        0x458a0000  /* PWM */
#define BK7258_SLCD_BASE       0x458e0000  /* SLCD */
#define BK7258_QSPI0_REG_BASE  0x46040000  /* QSPI0 controller */
#define BK7258_QSPI1_REG_BASE  0x46060000  /* QSPI1 controller */
#define BK7258_LCD_DISP_BASE   0x48060000  /* LCD display controller */
#define BK7258_DMA2D_BASE      0x48080000  /* 2D graphics accelerator */

#endif /* __ARCH_ARM_SRC_BK7258_HARDWARE_BK7258_MEMORYMAP_H */
