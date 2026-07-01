/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_npu.h
 *
 * Neural Processing Unit (NPU) register definitions for RK3576
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_NPU_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_NPU_H

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/* NPU register offsets */

#define NPU_CTRL                 0x0000   /* NPU control register */
#define NPU_STATUS               0x0004   /* NPU status register */
#define NPU_INTEN                0x0008   /* Interrupt enable */
#define NPU_INTSTS               0x000c   /* Interrupt status */
#define NPU_INTCLR               0x0010   /* Interrupt clear */
#define NPU_CLK_EN               0x0014   /* Clock enable */
#define NPU_CLK_DIV              0x0018   /* Clock divider */
#define NPU_AXI_QOS              0x001c   /* AXI QoS */
#define NPU_SRAM_ADDR_MAP       0x0020   /* SRAM address mapping */
#define NPU_QOS_WR              0x0024   /* QoS write */
#define NPU_QOS_RD              0x0028   /* QoS read */
#define NPU_PD_CON               0x002c   /* Power domain control */
#define NPU_PD_STATUS            0x0030   /* Power domain status */

/* NPU_CTRL bits */

#define NPU_CTRL_ENABLE          (1 << 0)
#define NPU_CTRL_RESET           (1 << 1)
#define NPU_CTRL_POWER_DOWN      (1 << 2)

/* NPU_STATUS bits */

#define NPU_STATUS_BUSY          (1 << 0)
#define NPU_STATUS_RESET_DONE    (1 << 1)

/* NPU_INTSTS bits */

#define NPU_INT_DONE             (1 << 0)
#define NPU_INT_ERROR            (1 << 1)

/* NPU_CLK_EN bits */

#define NPU_CLK_EN_CORE         (1 << 0)
#define NPU_CLK_EN_AXI          (1 << 1)
#define NPU_CLK_EN_AHB          (1 << 2)

#endif
