/****************************************************************************
 * arch/xtensa/src/t113/hardware/t113_msgbox.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * T113-S3 MSGBOX (hardware mailbox) register definitions, DSP-side view.
 *
 * A core SENDS by writing a 32-bit word into the REMOTE core's mailbox MSG
 * FIFO and RECEIVES from its OWN mailbox.  From the DSP, local = DSP
 * mailbox @ 0x01701000, remote = ARM mailbox @ 0x03003000.  Register block
 * per direction-index n (stride 0x100), sub-channel p (0..7).  On sun8iw20
 * the ARM<->DSP pair uses n = 0 in both directions.  The AP gates the
 * MSGBOX clocks before the DSP runs, so there is no CCU access here.
 *
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_T113_HARDWARE_T113_MSGBOX_H
#define __ARCH_XTENSA_SRC_T113_HARDWARE_T113_MSGBOX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Mailbox instance bases (physical; DSP has no MMU). */

#define T113_MSGBOX_ARM_BASE    0x03003000  /* ARM-local mailbox (remote) */
#define T113_MSGBOX_DSP_BASE    0x01701000  /* DSP-local mailbox (local) */

/* Per-(instance base, direction n, sub-channel p) register addresses. */

#define T113_MSGBOX_VER(b, n)        ((b) + 0x10 + (n) * 0x100)
#define T113_MSGBOX_RD_IRQ_EN(b, n)  ((b) + 0x20 + (n) * 0x100)
#define T113_MSGBOX_RD_IRQ_STA(b, n) ((b) + 0x24 + (n) * 0x100)
#define T113_MSGBOX_WR_IRQ_EN(b, n)  ((b) + 0x30 + (n) * 0x100)
#define T113_MSGBOX_WR_IRQ_STA(b, n) ((b) + 0x34 + (n) * 0x100)
#define T113_MSGBOX_FIFO_STA(b, n, p) ((b) + 0x50 + (n) * 0x100 + (p) * 0x4)
#define T113_MSGBOX_MSG_STA(b, n, p) ((b) + 0x60 + (n) * 0x100 + (p) * 0x4)
#define T113_MSGBOX_MSG(b, n, p)     ((b) + 0x70 + (n) * 0x100 + (p) * 0x4)

/* RD_IRQ_EN / RD_IRQ_STA: receive-not-empty enable/pending bit for
 * sub-channel p is bit (p * 2).
 */

#define T113_MSGBOX_RD_BIT(p)        (1u << ((p) * 2))

#define T113_MSGBOX_MAX_QUEUE        8

#endif /* __ARCH_XTENSA_SRC_T113_HARDWARE_T113_MSGBOX_H */
