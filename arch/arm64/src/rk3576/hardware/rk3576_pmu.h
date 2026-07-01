/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_pmu.h
 *
 * Power Management Unit (PMU) register definitions for RK3576
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_PMU_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_PMU_H

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/* PMU register offsets */

#define PMU_WAKEUP_CFG0         0x0000
#define PMU_WAKEUP_CFG1         0x0004
#define PMU_WAKEUP_CFG2         0x0008
#define PMU_WAKEUP_CFG3         0x000c
#define PMU_WAKEUP_CFG4         0x0010
#define PMU_WAKEUP_CFG5         0x0014
#define PMU_WAKEUP_CFG6         0x0018
#define PMU_WAKEUP_CFG7         0x001c
#define PMU_WAKEUP_CFG8         0x0020
#define PMU_WAKEUP_CFG9         0x0024
#define PMU_WAKEUP_CFG10        0x0028
#define PMU_WAKEUP_CFG11        0x002c

#define PMU_BUS_CON             0x0040
#define PMU_PD_CON              0x0044
#define PMU_SFT_CON             0x0048
#define PMU_DDR_SREF_CON        0x004c
#define PMU_INT_CON             0x0050
#define PMU_INT_STS             0x0054
#define PMU_POWER_STS           0x0058
#define PMU_BASE_ADDR0          0x0060
#define PMU_BASE_ADDR1          0x0064
#define PMU_BASE_ADDR2          0x0068
#define PMU_BASE_ADDR3          0x006c
#define PMU_BASE_ADDR_MAP0      0x0070
#define PMU_BASE_ADDR_MAP1      0x0074
#define PMU_BASE_ADDR_MAP2      0x0078
#define PMU_BASE_ADDR_MAP3      0x007c

#define PMU_OSC_CNT             0x0080
#define PMU_PLL_CON             0x0084
#define PMU_SCD_CON             0x0088
#define PMU_SCRATCH_ST          0x008c
#define PMU_SCRATCH_LEN         0x0090
#define PMU_STOCYCLE_CON        0x0094

/* PMU power domain states */

#define PMU_PD_ON               (0 << 0)
#define PMU_PD_OFF              (1 << 0)
#define PMU_PD_SLP              (2 << 0)

/* PMU_SFT_CON bits */

#define PMU_SFT_WAKEUP          (1 << 0)
#define PMU_SFT_GLOBAL_INT_DIS  (1 << 4)
#define PMU_SFT_DDR_GATING      (1 << 8)
#define PMU_SFT_PLL_GATING      (1 << 9)
#define PMU_SFT_OSC_DIS         (1 << 10)

#endif
