/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_cru.h
 *
 * Clock and Reset Unit (CRU) driver for RK3576
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_CRU_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_CRU_H

#include <nuttx/config.h>
#include <stdint.h>

/* PLL indices */

#define CRU_PLL_APLL            0
#define CRU_PLL_DPLL            1
#define CRU_PLL_CPLL            2
#define CRU_PLL_GPLL            3
#define CRU_PLL_NPLL            4
#define CRU_PLL_EPLL            5

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

void rk3576_cru_init(void);

/* PLL control */

int  rk3576_cru_pll_enable(int pll);
void rk3576_cru_pll_disable(int pll);
int  rk3576_cru_pll_is_locked(int pll);

/* Clock configuration */

void rk3576_cru_set_clkdiv(int reg, uint32_t div);
uint32_t rk3576_cru_get_clkdiv(int reg);

/* Clock gating */

void rk3576_cru_gate_enable(int reg, int bit);
void rk3576_cru_gate_disable(int reg, int bit);

/* Software reset */

void rk3576_cru_softrst(int reg, int bit);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_CRU_H */
