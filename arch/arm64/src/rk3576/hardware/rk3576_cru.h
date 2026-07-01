/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_cru.h
 *
 * Clock and Reset Unit (CRU) register definitions for RK3576
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_CRU_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_CRU_H

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/* PLL registers */

#define CRU_PLL_CON(n)          (0x0000 + (n) * 0x20)   /* PLL n control */
#define CRU_PLL_CON0            0x0000   /* PLL APLL */
#define CRU_PLL_CON1            0x0020   /* PLL DPLL */
#define CRU_PLL_CON2            0x0040   /* PLL CPLL */
#define CRU_PLL_CON3            0x0060   /* PLL GPLL */
#define CRU_PLL_CON4            0x0080   /* PLL NPLL */
#define CRU_PLL_CON5            0x00a0   /* PLL EPLL */

/* PLL_CON0 bits */

#define CRU_PLLCON0_LOCK        (1 << 31)
#define CRU_PLLCON0_BYPASS      (1 << 1)
#define CRU_PLLCON0_POWERDOWN   (1 << 0)
#define CRU_PLLCON0_POSTDIV2_SHIFT 27
#define CRU_PLLCON0_POSTDIV1_SHIFT 24
#define CRU_PLLCON0_FBDIV_SHIFT  0
#define CRU_PLLCON0_REFDIV_SHIFT 12

/* Clock select registers */

#define CRU_CLKSEL_CON(n)      (0x0100 + (n) * 0x04)

/* Clock gate registers */

#define CRU_CLKGATE_CON(n)     (0x0300 + (n) * 0x04)

/* Software reset registers */

#define CRU_SOFTRST_CON(n)     (0x0400 + (n) * 0x04)

/* Bus clock registers */

#define CRU_CLKDIV_CON(n)      (0x0600 + (n) * 0x04)

/* Source clock frequencies */

#define CRU_PLL_APLL_FREQ      1200000000  /* 1.2GHz */
#define CRU_PLL_GPLL_FREQ      1188000000  /* 1.188GHz */
#define CRU_PLL_CPLL_FREQ      1000000000  /* 1GHz */
#define CRU_PLL_NPLL_FREQ      1200000000  /* 1.2GHz */
#define CRU_PLL_EPLL_FREQ      1000000000  /* 1GHz */
#define CRU_XIN_OSC_FREQ       24000000    /* 24MHz oscillator */

/* Common clock dividers */

#define CRU_CLK_DIV(n)         ((n) - 1)

/* Software reset bits */

#define CRU_SOFTRST_CON0       0x0400

#endif
