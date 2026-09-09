/****************************************************************************
 * arch/arm/src/bk7258/include/irq.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __VENDOR_BEKEN_CHIPS_BK7258_INCLUDE_IRQ_H
#define __VENDOR_BEKEN_CHIPS_BK7258_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/chip/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BK7258_IRQ_RESERVED        0
#define BK7258_IRQ_NMI             2
#define BK7258_IRQ_HARDFAULT       3
#define BK7258_IRQ_MEMFAULT        4
#define BK7258_IRQ_BUSFAULT        5
#define BK7258_IRQ_USAGEFAULT      6
#define BK7258_IRQ_SECUREFAULT     7
#define BK7258_IRQ_SVCALL          11
#define BK7258_IRQ_DBGMONITOR      12
#define BK7258_IRQ_PENDSV          14
#define BK7258_IRQ_SYSTICK         15
#define BK7258_IRQ_FIRST           16

#define BK7258_EXTIRQ_TIMER0      3
#define BK7258_EXTIRQ_UART0       4
#define BK7258_EXTIRQ_DMA         11
#define BK7258_EXTIRQ_TIMER1      13
#define BK7258_EXTIRQ_I2C1        14
#define BK7258_EXTIRQ_UART1       15
#define BK7258_EXTIRQ_UART2       16
#define BK7258_EXTIRQ_SDIO        10

/* Analog audio (AUD) FIFO interrupt.  bk_avdk_smp release/v3.1.1
 * ap/middleware/soc/bk7258_ap/soc/icu_map.h:53 maps INT_SRC_AUDIO to
 * external interrupt line 23 in GROUP0.
 */

#define BK7258_EXTIRQ_AUDIO       23

/* Hardware JPEG encoder interrupt.  bk_avdk_smp release/v3.1.1
 * ap/middleware/soc/bk7258_ap/soc/icu_map.h:55 maps INT_SRC_JPEG_ENC to
 * external interrupt line 25 in GROUP0 (line 26 is INT_SRC_JPEG_DEC --
 * the decoder, a different module).
 */

#define BK7258_EXTIRQ_JPEG_ENC    25
#define BK7258_EXTIRQ_YUVB        58
#define BK7258_EXTIRQ_MAILBOX     63
#define BK7258_EXTIRQ_COUNT       64

#define BK7258_IRQ_TIMER0          (BK7258_IRQ_FIRST + BK7258_EXTIRQ_TIMER0)
#define BK7258_IRQ_UART0           (BK7258_IRQ_FIRST + BK7258_EXTIRQ_UART0)
#define BK7258_IRQ_DMA             (BK7258_IRQ_FIRST + BK7258_EXTIRQ_DMA)
#define BK7258_IRQ_TIMER1          (BK7258_IRQ_FIRST + BK7258_EXTIRQ_TIMER1)
#define BK7258_IRQ_I2C1            (BK7258_IRQ_FIRST + BK7258_EXTIRQ_I2C1)
#define BK7258_IRQ_UART1           (BK7258_IRQ_FIRST + BK7258_EXTIRQ_UART1)
#define BK7258_IRQ_UART2           (BK7258_IRQ_FIRST + BK7258_EXTIRQ_UART2)
#define BK7258_IRQ_SDIO            (BK7258_IRQ_FIRST + BK7258_EXTIRQ_SDIO)
#define BK7258_IRQ_AUDIO           (BK7258_IRQ_FIRST + BK7258_EXTIRQ_AUDIO)
#define BK7258_IRQ_JPEG_ENC        (BK7258_IRQ_FIRST + \
                                    BK7258_EXTIRQ_JPEG_ENC)
#define BK7258_IRQ_YUVB            (BK7258_IRQ_FIRST + BK7258_EXTIRQ_YUVB)
#define BK7258_IRQ_MAILBOX         (BK7258_IRQ_FIRST + BK7258_EXTIRQ_MAILBOX)

#define NR_IRQS                    (BK7258_IRQ_FIRST + BK7258_EXTIRQ_COUNT)

#define NVIC_SYSH_PRIORITY_MIN     0xf0
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80
#define NVIC_SYSH_PRIORITY_MAX     0x00
#define NVIC_SYSH_PRIORITY_STEP    0x10

#endif
