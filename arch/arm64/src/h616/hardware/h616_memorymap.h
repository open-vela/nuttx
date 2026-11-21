/****************************************************************************
 * arch/arm64/src/h616/hardware/h616_memorymap.h
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

#ifndef __ARCH_ARM64_SRC_H616_HARDWARE_H616_MEMORYMAP_H
#define __ARCH_ARM64_SRC_H616_HARDWARE_H616_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Peripheral Base Addresses (from sun50iw9p1.dtsi, Allwinner H616) */

#define H616_DE_ADDR         0x01000000 /* DE              0x0100:0000-0x012f:ffff 3M */
#define H616_CCU_ADDR        0x03001000 /* CCU (CPUX clk)  0x0300:1000-0x0300:1fff 4K */
#define H616_DMA_ADDR        0x03002000 /* DMA             0x0300:2000 */
#define H616_MSGBOX_ADDR     0x03003000 /* MSGBOX          0x0300:3000 */
#define H616_SID_ADDR        0x03006000 /* SID             0x0300:6000 */
#define H616_SOC_TIMER_ADDR  0x03009000 /* Timer           0x0300:9000 */
#define H616_WDT_ADDR        0x030090a0 /* Watchdog        0x0300:90a0 */
#define H616_PWM_ADDR        0x0300a000 /* PWM             0x0300:a000 */
#define H616_PIO_ADDR        0x0300b000 /* PIO             0x0300:b000 */
#define H616_GIC_ADDR        0x03020000 /* GIC             0x0302:0000 */

#define H616_SDMMC0_ADDR     0x04020000 /* SDMMC0 (SD)     0x0402:0000 */
#define H616_SDMMC1_ADDR     0x04021000 /* SDMMC1 (SDIO)   0x0402:1000 */
#define H616_SDMMC2_ADDR     0x04022000 /* SDMMC2 (eMMC)   0x0402:2000 */

#define H616_UART0_ADDR      0x05000000 /* UART 0          0x0500:0000 (DW 16550) */
#define H616_UART1_ADDR      0x05000400 /* UART 1          0x0500:0400 */
#define H616_UART2_ADDR      0x05000800 /* UART 2          0x0500:0800 */
#define H616_UART3_ADDR      0x05000c00 /* UART 3          0x0500:0c00 */
#define H616_UART4_ADDR      0x05001000 /* UART 4          0x0500:1000 */
#define H616_UART5_ADDR      0x05001400 /* UART 5          0x0500:1400 */

#define H616_TWI0_ADDR       0x05002000 /* TWI 0           0x0500:2000 */
#define H616_TWI1_ADDR       0x05002400 /* TWI 1           0x0500:2400 */
#define H616_TWI2_ADDR       0x05002800 /* TWI 2           0x0500:2800 */
#define H616_TWI3_ADDR       0x05002c00 /* TWI 3           0x0500:2c00 */
#define H616_TWI4_ADDR       0x05003000 /* TWI 4           0x0500:3000 */
#define H616_TWI5_ADDR       0x05003400 /* TWI 5           0x0500:3400 */

#define H616_SPI0_ADDR       0x05010000 /* SPI 0           0x0501:0000 */
#define H616_SPI1_ADDR       0x05011000 /* SPI 1           0x0501:1000 */
#define H616_GMAC_ADDR       0x05020000 /* GMAC (Ethernet) 0x0502:0000 */

#define H616_HDMI_ADDR       0x06000000 /* HDMI            0x0600:0000 */

#define H616_RTC_ADDR        0x07000000 /* RTC             0x0700:0000 */
#define H616_RPIO_ADDR       0x07022000 /* R_PIO           0x0702:2000 */
#define H616_RTWI_ADDR       0x07080000 /* R_TWI           0x0708:0000 */
#define H616_S_UART0_ADDR    0x07080000 /* S_UART0         0x0708:0000 */

#endif /* __ARCH_ARM64_SRC_H616_HARDWARE_H616_MEMORYMAP_H */
