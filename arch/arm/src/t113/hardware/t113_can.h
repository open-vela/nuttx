/****************************************************************************
 * arch/arm/src/t113/hardware/t113_can.h
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

/* T113-S3 CAN Controller (Allwinner sun20i-d1 variant)
 *
 * The T113 CAN IP is NOT a standard SJA1000 - it is an Allwinner custom
 * design with a different register map (not handled by the generic SJA1000
 * driver).
 *
 * Key differences from SJA1000:
 *   - 32-bit registers at word-aligned offsets (not byte-addressed)
 *   - Single 32-bit BTIME register (not two 8-bit BTR0/BTR1)
 *   - Combined error counter register (TX + RX in one 32-bit word)
 *   - Acceptance filter at dedicated offsets 0x28/0x2C (D1 variant)
 *   - No CLOCK_DIVIDER / EXT_MODE register
 *   - Status register includes error capture fields in upper bits
 */

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_CAN_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "t113_clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CAN controller base addresses */

#define T113_CAN0_BASE              0x02504000
#define T113_CAN1_BASE              0x02504400

/* CAN peripheral clock: APB1 bus clock */

#define T113_CAN_CLK_FREQ           T113_CAN_FREQUENCY

/* Register offsets (32-bit word-aligned) */

#define T113_CAN_MSEL_OFFSET        0x0000  /* Mode Select */
#define T113_CAN_CMD_OFFSET         0x0004  /* Command */
#define T113_CAN_STA_OFFSET         0x0008  /* Status */
#define T113_CAN_INT_OFFSET         0x000c  /* Interrupt Flag (read-clear) */
#define T113_CAN_INTEN_OFFSET       0x0010  /* Interrupt Enable */
#define T113_CAN_BTIME_OFFSET       0x0014  /* Bus Timing */
#define T113_CAN_TEWL_OFFSET        0x0018  /* TX Error Warning Limit */
#define T113_CAN_ERRC_OFFSET        0x001c  /* Error Counter */
#define T113_CAN_RMCNT_OFFSET       0x0020  /* RX Message Counter */
#define T113_CAN_RBUFSA_OFFSET      0x0024  /* RX Buffer Start Address */
#define T113_CAN_ACPC_OFFSET        0x0028  /* Acceptance Code (D1 variant) */
#define T113_CAN_ACPM_OFFSET        0x002c  /* Acceptance Mask (D1 variant) */

/* TX/RX Buffer n */

#define T113_CAN_BUF_OFFSET(n)        (0x0040 + (n) * 4)

/* RX Buffer Readback n */

#define T113_CAN_RBUF_RBACK_OFFSET(n) (0x0180 + (n) * 4)
#define T113_CAN_VERSION_OFFSET     0x0300  /* CAN IP Version */

/* Mode Select Register (MSEL) bits */

#define T113_CAN_MSEL_SLEEP         (1 << 4)  /* Sleep mode (write in reset) */
#define T113_CAN_MSEL_FILTER        (1 << 3)  /* Single filter mode */
#define T113_CAN_MSEL_LOOPBACK      (1 << 2)  /* Self-test / loopback mode */
#define T113_CAN_MSEL_LISTEN        (1 << 1)  /* Listen-only mode */
#define T113_CAN_MSEL_RESET         (1 << 0)  /* Reset mode */

/* Command Register (CMD) bits */

#define T113_CAN_CMD_BUSOFF_REQ     (1 << 5)  /* Bus-Off request */
#define T113_CAN_CMD_SELF_REQ       (1 << 4)  /* Self-receive request */
#define T113_CAN_CMD_CLR_OVERRUN    (1 << 3)  /* Clear data overrun */
#define T113_CAN_CMD_RELEASE_BUF    (1 << 2)  /* Release receive buffer */
#define T113_CAN_CMD_ABORT          (1 << 1)  /* Abort transmission */
#define T113_CAN_CMD_TRANS_REQ      (1 << 0)  /* Transmit request */

/* Status Register (STA) bits */

#define T113_CAN_STA_ERR_TYPE_SHIFT 22
#define T113_CAN_STA_ERR_TYPE_MASK  (0x03 << 22)
#define T113_CAN_STA_ERR_DIR        (1 << 21)  /* Error direction: 1=RX */
#define T113_CAN_STA_ERR_SEG_MASK   (0x1f << 16)
#define T113_CAN_STA_BUSOFF         (1 << 7)   /* Bus-off status */
#define T113_CAN_STA_ERR            (1 << 6)   /* Error status */
#define T113_CAN_STA_TRANS_BUSY     (1 << 5)   /* Transmit busy */
#define T113_CAN_STA_RCV_BUSY       (1 << 4)   /* Receive busy */
#define T113_CAN_STA_TRANS_OVER     (1 << 3)   /* Transmission complete */
#define T113_CAN_STA_TBUF_RDY       (1 << 2)   /* TX buffer ready */
#define T113_CAN_STA_DATA_ORUN      (1 << 1)   /* Data overrun */
#define T113_CAN_STA_RBUF_RDY       (1 << 0)   /* RX buffer ready */

/* Interrupt Flag Register (INT) bits - read to clear */

#define T113_CAN_INT_BUSERR         (1 << 7)
#define T113_CAN_INT_ARBLOST        (1 << 6)
#define T113_CAN_INT_ERR_PASSIVE    (1 << 5)
#define T113_CAN_INT_WAKEUP         (1 << 4)
#define T113_CAN_INT_DATA_OVERRUN   (1 << 3)
#define T113_CAN_INT_ERR_WARNING    (1 << 2)
#define T113_CAN_INT_TX_DONE        (1 << 1)
#define T113_CAN_INT_RX_DONE        (1 << 0)

/* Interrupt Enable Register (INTEN) bits - same layout as INT */

#define T113_CAN_INTEN_BUSERR       (1 << 7)
#define T113_CAN_INTEN_ARBLOST      (1 << 6)
#define T113_CAN_INTEN_ERR_PASSIVE  (1 << 5)
#define T113_CAN_INTEN_WAKEUP       (1 << 4)
#define T113_CAN_INTEN_OVERRUN      (1 << 3)
#define T113_CAN_INTEN_ERR_WARNING  (1 << 2)
#define T113_CAN_INTEN_TX           (1 << 1)
#define T113_CAN_INTEN_RX           (1 << 0)

/* Bus Timing Register (BTIME) - single 32-bit register */

#define T113_CAN_BTIME_BRP_SHIFT    0         /* BRP[9:0]: Baud Rate Prescaler */
#define T113_CAN_BTIME_BRP_MASK     (0x3ff << 0)
#define T113_CAN_BTIME_SJW_SHIFT    14        /* SJW[15:14]: Sync Jump Width */
#define T113_CAN_BTIME_SJW_MASK     (0x03 << 14)
#define T113_CAN_BTIME_TSEG1_SHIFT  16        /* TSEG1[19:16]: Time Seg 1 */
#define T113_CAN_BTIME_TSEG1_MASK   (0x0f << 16)
#define T113_CAN_BTIME_TSEG2_SHIFT  20        /* TSEG2[22:20]: Time Seg 2 */
#define T113_CAN_BTIME_TSEG2_MASK   (0x07 << 20)
#define T113_CAN_BTIME_SAM          (1 << 23) /* Triple sampling */

/* Error Counter Register (ERRC) - combined TX and RX */

#define T113_CAN_ERRC_TXERR_MASK    0x000000ff  /* TX error count [7:0] */
#define T113_CAN_ERRC_RXERR_SHIFT   16
#define T113_CAN_ERRC_RXERR_MASK    0x00ff0000  /* RX error count [23:16] */

/* TX/RX Buffer 0 (frame info) flags */

#define T113_CAN_BUF0_EFF           (1 << 7)   /* Extended frame format */
#define T113_CAN_BUF0_RTR           (1 << 6)   /* Remote transmission request */
#define T113_CAN_BUF0_DLC_MASK      0x0f       /* Data length code */

/* CAN BGR/gating/reset bits live in hardware/t113_ccu.h to stay consistent
 * with every other peripheral's CCU-register conventions.
 */

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_CAN_H */
