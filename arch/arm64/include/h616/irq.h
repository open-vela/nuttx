/****************************************************************************
 * arch/arm64/include/h616/irq.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied.  See the License for the specific language governing
 * permissions and limitations under the License.
 *
 ****************************************************************************/

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM64_INCLUDE_H616_IRQ_H
#define __ARCH_ARM64_INCLUDE_H616_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allwinner H616 interrupts.
 *
 * The GIC-400 sees 16 SGIs (0-15), 16 PPIs (16-31) and 160 SPIs; SPI n is
 * routed to IRQ n + 32.  Values below are taken from the Linux device tree
 * (arch/arm64/boot/dts/allwinner/sun50i-h616.dtsi).
 */

#define NR_IRQS                 157   /* Total number of interrupts */

/* SGI and PPI (per-CPU) interrupts */

#define H616_IRQ_SGI0            (0)   /* SGI 0-15 (software generated) */
#define H616_IRQ_SGI15           (15)
#define H616_IRQ_PPI0            (16)  /* PPI 0-15 (private peripheral) */
#define H616_IRQ_PPI15           (31)

/* SPI interrupts (shared), listed by IRQ number */

#define H616_IRQ_UART0           (32)  /* serial@5000000, SPI 0  */
#define H616_IRQ_UART1           (33)  /* serial@5000400, SPI 1  */
#define H616_IRQ_UART2           (34)  /* serial@5000800, SPI 2  */
#define H616_IRQ_UART3           (35)  /* serial@5000c00, SPI 3  */
#define H616_IRQ_UART4           (36)  /* serial@5010000, SPI 4  */
#define H616_IRQ_UART5           (37)  /* serial@5014000, SPI 5  */
#define H616_IRQ_I2C0            (38)  /* i2c@5002000,   SPI 6  */
#define H616_IRQ_I2C1            (39)  /* i2c@5002400,   SPI 7  */
#define H616_IRQ_I2C2            (40)  /* i2c@5002800,   SPI 8  */
#define H616_IRQ_I2C3            (41)  /* i2c@5002c00,   SPI 9  */
#define H616_IRQ_I2C4            (42)  /* i2c@5003000,   SPI 10 */
#define H616_IRQ_SPI0            (44)  /* spi@5010000,   SPI 12 */
#define H616_IRQ_SPI1            (45)  /* spi@5011000,   SPI 13 */
#define H616_IRQ_EMAC            (46)  /* ethernet@5020000, SPI 14 */
#define H616_IRQ_TSADC           (50)  /* adc@5070000 (thermal), SPI 18 */
#define H616_IRQ_THS             (51)  /* sensor@5070400, SPI 19 */
#define H616_IRQ_LRADC           (52)  /* lradc@5070800, SPI 20 */
#define H616_IRQ_SPDIF           (53)  /* spdif@5093000, SPI 21 */
#define H616_IRQ_USB_OTG_DEVICE  (57)  /* usb@5100000,   SPI 25 */
#define H616_IRQ_USB_OTG_EHCI    (58)  /* usb@5101000,   SPI 26 */
#define H616_IRQ_USB_OTG_OHCI    (59)  /* usb@5101400,   SPI 27 */
#define H616_IRQ_USB0_EHCI       (60)  /* usb@5200000,   SPI 28 */
#define H616_IRQ_USB0_OHCI       (61)  /* usb@5200400,   SPI 29 */
#define H616_IRQ_USB1_EHCI       (62)  /* usb@5310000,   SPI 30 */
#define H616_IRQ_USB1_OHCI       (63)  /* usb@5310400,   SPI 31 */
#define H616_IRQ_USB2_EHCI       (64)  /* usb@5311000,   SPI 32 */
#define H616_IRQ_USB2_OHCI       (65)  /* usb@5311400,   SPI 33 */
#define H616_IRQ_AUDIO_CODEC     (90)  /* codec@5096000 (analog), SPI 58 */
#define H616_IRQ_IOMMU           (93)  /* iommu@30f0000, SPI 61 */
#define H616_IRQ_VIDEO_CODEC     (125) /* video-codec@1c0e000, SPI 93 */
#define H616_IRQ_NAND            (66)  /* nand-controller@4011000, SPI 34 */
#define H616_IRQ_MMC0            (67)  /* mmc@4020000 (SD),   SPI 35 */
#define H616_IRQ_MMC1            (68)  /* mmc@4021000 (SD),   SPI 36 */
#define H616_IRQ_SD_MMC2         (69)  /* mmc@4022000 (eMMC),  SPI 37 */
#define H616_IRQ_CRYPTO          (123) /* crypto@1904000, SPI 91 */
#define H616_IRQ_HSTIMER         (77)  /* hstimer@3005000, SPI 45 */
#define H616_IRQ_DMA             (74)  /* dma-controller@3002000, SPI 42 */
#define H616_IRQ_TIMER0          (80)  /* timer@3009000, SPI 48 */
#define H616_IRQ_WATCHDOG        (82)  /* watchdog@30090a0, SPI 50 */
#define H616_IRQ_PINCTRL         (83)  /* pinctrl@300b000, SPI 51 */
#define H616_IRQ_GPU             (127) /* gpu@1800000, SPI 95 */
#define H616_IRQ_RTC             (136) /* rtc@7000000, SPI 104 */
#define H616_IRQ_R_I2C           (137) /* i2c@7081400, SPI 105 */
#define H616_IRQ_R_IR            (138) /* ir@7040000, SPI 106 */
#define H616_IRQ_R_RSB           (141) /* rsb@7083000, SPI 109 */
#define H616_IRQ_R_TIMER0        (135) /* r_timer controller@7010320, SPI 103 */

#endif /* __ARCH_ARM64_INCLUDE_H616_IRQ_H */
