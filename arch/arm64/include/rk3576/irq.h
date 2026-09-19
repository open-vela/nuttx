/****************************************************************************
 * board/contest_board/chip/include/irq.h
 *
 * RK3576 interrupt map for the openvela AMP slave.
 *
 * This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM64_INCLUDE_RK3576_IRQ_H
#define __ARCH_ARM64_INCLUDE_RK3576_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Total number of GIC INTIDs (SGI 0-15, PPI 16-31, SPI 32+).  The RK3576
 * GIC-400 implements SPIs well past 400 (e.g. INTID 421 is used by the
 * Linux fiq debugger), so cover the full useable range.
 */

#define NR_IRQS                  480

/* IRQs owned by this AMP slave core (absolute INTIDs) */

#define RK3576_IRQ_TIMER_NS_EL1  30    /* CNTP: architected timer (PPI) */
#define RK3576_IRQ_UART5_INTID   113   /* UART5 console (GIC_SPI 81)    */
#define RK3576_IRQ_RPMSG_INTID   172   /* rpmsg kick from Linux         */

#endif /* __ARCH_ARM64_INCLUDE_RK3576_IRQ_H */
