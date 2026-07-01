/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_can.h
 *
 * CAN controller register definitions for RK3576
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_CAN_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_CAN_H

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/* RK3576 has 2 CAN controllers */

#define RK3576_CAN_COUNT          2

#define RK3576_CAN0_ADDR          0xff280000
#define RK3576_CAN1_ADDR          0xff290000

/* Bosch M_CAN compatible registers */

#define CAN_CREL                  0x0000  /* Core Release */
#define CAN_CDR                   0x0004  /* Core Debug Register */
#define CAN_RIR                   0x0014  /* Receive Info Register */
#define CAN_TXBAR                 0x00d0  /* Transmit Buffer Add Request */
#define CAN_TXBCR                 0x00d4  /* Transmit Buffer Cancellation */
#define CAN_TXBTO                 0x00dc  /* Transmit Buffer Transmission Occurred */
#define CAN_NDAT1                 0x00e0  /* New Data 1 */
#define CAN_NDAT2                 0x00e4  /* New Data 2 */
#define CAN_RXFxA                 0x00e8  /* Rx FIFO Acknowledge */
#define CAN_RXFSA                 0x00ec  /* Rx FIFO Status */
#define CAN_TXBC                  0x00c0  /* Tx Buffer Configuration */
#define CAN_TXESC                 0x00cc  /* Tx Buffer Element Size */
#define CAN_RXESC                 0x00dc  /* Rx Buffer Element Size */
#define CAN bacheca_addr          0x00bc  /* Bittiming Configuration */
#define CAN_CCCR                  0x0018  /* CC Control Register */
#define CAN_BTP1                  0x001c  /* Bit Timing Prescaler */
#define CAN_BTP2                  0x0020  /* Bit Timing Segment */
#define CAN_TSCC                  0x0024  /* Timestamp Counter Configuration */
#define CAN_TOCC                  0x0028  /* Timeout Counter Configuration */
#define CAN_IE                    0x0034  /* Interrupt Enable */
#define CAN_ILS                   0x0038  /* Interrupt Line Select */
#define CAN_ILE                   0x003c  /* Interrupt Line Enable */
#define CAN_IR                    0x0050  /* Interrupt Register */

/* CAN_CCCR bits */

#define CAN_CCCR_INIT             (1 << 0)
#define CAN_CCCR_CCE              (1 << 1)
#define CAN_CCCR_ASM              (1 << 2)
#define CAN_CCCR_CSAR             (1 << 3)
#define CAN_CCCR_CSR              (1 << 4)
#define CAN_CCCR_MON              (1 << 5)
#define CAN_CCCR_DAR              (1 << 6)

/* CAN_IR bits */

#define CAN_IR_RF0N               (1 << 0)
#define CAN_IR_RF0W               (1 << 1)
#define CAN_IR_RF0F               (1 << 2)
#define CAN_IR_RF0L               (1 << 3)
#define CAN_IR_RF1N               (1 << 4)
#define CAN_IR_TFN                (1 << 8)
#define CAN_IR_TFFN               (1 << 9)
#define CAN_IR_TCFN               (1 << 10)
#define CAN_IR_TFHN               (1 << 11)
#define CAN_IR_EW                 (1 << 15)
#define CAN_IR_EPF                (1 << 16)
#define CAN_IR_EWF                (1 << 17)
#define CAN_IR_BO                 (1 << 18)

/* CAN IE bits */

#define CAN_IE_RF0NE              (1 << 0)
#define CAN_IE_TFNE               (1 << 8)
#define CAN_IE_EWE                (1 << 15)
#define CAN_IE_BOE                (1 << 18)

#ifndef __ASSEMBLY__

extern const uint32_t g_can_base[RK3576_CAN_COUNT];

#endif /* __ASSEMBLY__ */

#endif
