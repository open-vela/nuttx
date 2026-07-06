/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_gpu.h
 *
 * Mali-G51 GPU register definitions for RK3576
 * Based on ARM Mali Midgard architecture
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_GPU_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_GPU_H

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/* GPU register offsets */

#define GPU_JS_CONFIG             0x0000   /* Job slot config */
#define GPU_JS_TAIL_NEXT         0x0004
#define GPU_JS_HEAD_NEXT         0x0008
#define GPU_JS_COMMAND           0x000c
#define GPU_JS_NEXTCMD           0x0010
#define GPU_JS_STATUS0           0x0014
#define GPU_JS_STATUS1           0x0018
#define GPU_JS_STATUS2           0x001c
#define GPU_JS_CONFIG_NEXT       0x0030

#define GPU_CSHWCTL              0x0040   /* Cache hash control */
#define GPU_CSHWPOWERDOWN        0x0044

#define GPU_INT_RAWSTAT          0x004c   /* Raw interrupt status */
#define GPU_INT_CLEAR            0x0050   /* Interrupt clear */
#define GPU_INT_MASK             0x0054   /* Interrupt mask */
#define GPU_INT_STATUS           0x0058   /* Masked interrupt status */

#define GPU_WAIT                0x0060

#define GPU_FEATURES             0x00a0   /* GPU features */
#define GPU_FEATURES2            0x00a4   /* GPU features 2 */
#define GPU_ID                   0x00a8   /* GPU ID */
#define GPU_L2_FEATURES          0x00ac   /* L2 cache features */
#define GPU_CORE_ID              0x0100   /* Core ID */
#define GPU_CORE_VERSION         0x0104   /* Core version */
#define GPU_CORE_FEATURES        0x010c   /* Core features */
#define GPU_CORE_EN              0x0180   /* Core enable */

/* GPU clock registers */

#define GPU_CLK_DIV              0xff3e0200
#define GPU_CLK_GATE             0xff3e0204

/* GPU power registers */

#define GPU_PWRON                0xff3e0240
#define GPU_POWEROFF             0xff3e0244

/* Interrupt bits */

#define GPU_INT_JOB              (1 << 0)
#define GPU_INT_MMU              (1 << 1)
#define GPU_INT_MMU_JOURNAL      (1 << 2)
#define GPU_INT_AS_FAULT         (1 << 3)
#define GPU_INT_RESET_COMPLETED  (1 << 8)

#endif
