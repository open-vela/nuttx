/****************************************************************************
 * arch/arm/src/t113/hardware/t113_msgbox.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * T113-S3 MSGBOX (hardware mailbox) register definitions.
 *
 * The SoC has three mailbox instances (ARM @ 0x03003000, DSP @ 0x01701000,
 * RISC-V @ 0x0601f000).  A core SENDS by writing the 32-bit word into the
 * REMOTE core's mailbox MSG FIFO, and RECEIVES from its OWN mailbox.
 * Register block per direction-index n (stride 0x100), sub-channel p
 * (0..7), from the vendor msgbox-sun8iw20.h.  On sun8iw20 the ARM<->DSP
 * pair uses n = 0 in both directions (vendor calculte_n table).
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_MSGBOX_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_MSGBOX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/t113_ccu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Mailbox instance bases */

#define T113_MSGBOX_ARM_BASE    0x03003000  /* ARM-local mailbox */
#define T113_MSGBOX_DSP_BASE    0x01701000  /* DSP-local mailbox */

/* Per-(instance base, direction n, sub-channel p) register addresses.
 * n selects the direction pair; p (0..7) selects the FIFO sub-channel.
 */

#define T113_MSGBOX_VER(b, n)        ((b) + 0x10 + (n) * 0x100)
#define T113_MSGBOX_RD_IRQ_EN(b, n)  ((b) + 0x20 + (n) * 0x100)
#define T113_MSGBOX_RD_IRQ_STA(b, n) ((b) + 0x24 + (n) * 0x100)
#define T113_MSGBOX_WR_IRQ_EN(b, n)  ((b) + 0x30 + (n) * 0x100)
#define T113_MSGBOX_WR_IRQ_STA(b, n) ((b) + 0x34 + (n) * 0x100)
#define T113_MSGBOX_FIFO_STA(b, n, p) ((b) + 0x50 + (n) * 0x100 + (p) * 0x4)
#define T113_MSGBOX_MSG_STA(b, n, p) ((b) + 0x60 + (n) * 0x100 + (p) * 0x4)
#define T113_MSGBOX_MSG(b, n, p)     ((b) + 0x70 + (n) * 0x100 + (p) * 0x4)

/* RD_IRQ_EN / RD_IRQ_STA: two status bits per sub-channel p; the
 * receive-FIFO-not-empty enable/pending bit is bit (p * 2).
 */

#define T113_MSGBOX_RD_BIT(p)        (1u << ((p) * 2))

/* MSG_STA reads the number of words currently queued (0..8). */

#define T113_MSGBOX_MAX_QUEUE        8

/* CCU gate + reset for the mailboxes: single BGR register at CCU+0x71c.
 * Bit 0/16 = MSGBOX0 (ARM) gate/reset, bit 1/17 = MSGBOX1 (DSP).
 */

#define T113_CCU_MSGBOX_BGR          (T113_CCU_BASE + 0x071c)
#define T113_CCU_MSGBOX0_GATING      (1u << 0)
#define T113_CCU_MSGBOX0_RST         (1u << 16)
#define T113_CCU_MSGBOX1_GATING      (1u << 1)
#define T113_CCU_MSGBOX1_RST         (1u << 17)

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_MSGBOX_H */
