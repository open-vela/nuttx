/****************************************************************************
 * arch/arm/include/t113/irq.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * T113-S3 / R528 IRQ definitions (same silicon).
 * Original source: vendor/allwinnertech/chips/r528/include/r528_irq.h
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_T113_IRQ_H
#define __ARCH_ARM_INCLUDE_T113_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* GIC interrupt count */

#define NR_IRQS   160

/* UART interrupt numbers */

#define T113_IRQ_UART0  34
#define T113_IRQ_UART1  35
#define T113_IRQ_UART2  36
#define T113_IRQ_UART3  37
#define T113_IRQ_UART4  38
#define T113_IRQ_UART5  39

/* TWI (I2C) interrupt numbers */

#define T113_IRQ_TWI0   41
#define T113_IRQ_TWI1   42
#define T113_IRQ_TWI2   43
#define T113_IRQ_TWI3   44

/* SPI interrupt numbers */

#define T113_IRQ_SPI0   47
#define T113_IRQ_SPI1   48

/* USB interrupt numbers */

#define T113_IRQ_USB0   61

/* DMA interrupt numbers */

#define T113_IRQ_DMA0   82
#define T113_IRQ_DMA1   83

/* Crypto Engine interrupt numbers */

#define T113_IRQ_CE_NS  84
#define T113_IRQ_CE_S   85

/* CAN interrupt numbers */

#define T113_IRQ_CAN0     53
#define T113_IRQ_CAN1     54

/* HSTimer interrupt numbers */

#define T113_IRQ_HSTIMER0 87
#define T113_IRQ_HSTIMER1 88

/* SMHC interrupt numbers (GIC SPI[40..42] = IRQ 72..74) */

#define T113_IRQ_SMHC0    72
#define T113_IRQ_SMHC1    73
#define T113_IRQ_SMHC2    74

/* GPADC interrupt number (vendor SUNXI_GPADC_IRQ, see
 * drivers/rtos-hal/hal/source/gpadc/platform/gpadc_sun20iw1.h).
 */

#define T113_IRQ_GPADC    89

/* LRADC interrupt number (vendor SUNXI_IRQ_LRADC, see
 * drivers/rtos-hal/hal/source/lradc/platform/lradc_sun8iw20.h).
 */

#define T113_IRQ_LRADC    93

/* PIO Port D External Interrupt (combined, non-secure path).
 * Source: T113-S3 User Manual, GIC Interrupts table, GPIOD_NS = SPI 105.
 */

#define T113_IRQ_PD_EINT  105

/* Display subsystem */

#define T113_IRQ_DE        119
#define T113_IRQ_TCON_LCD0 122
#define T113_IRQ_DSI       124

#endif /* __ARCH_ARM_INCLUDE_T113_IRQ_H */
