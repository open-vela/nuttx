/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_cru.c
 *
 * Clock and Reset Unit (CRU) driver for RK3576
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <debug.h>
#include <nuttx/arch.h>
#include "arm64_internal.h"
#include "hardware/rk3576_cru.h"
#include "rk3576_cru.h"

void rk3576_cru_init(void)
{
  /* CRU is typically initialized by bootloader */
  /* Just configure basic clock gating */

  ginfo("CRU: initialized\n");
}

int rk3576_cru_pll_enable(int pll)
{
  uint32_t base = RK3576_CRU_ADDR;
  uint32_t reg = base + CRU_PLL_CON(pll);

  /* Clear powerdown and bypass */

  uint32_t val = getreg32(reg);
  val &= ~(CRU_PLLCON0_POWERDOWN | CRU_PLLCON0_BYPASS);
  putreg32(val, reg);

  /* Wait for lock */

  int timeout = 10000;
  while (timeout--)
    {
      if (getreg32(reg) & CRU_PLLCON0_LOCK)
        {
          return OK;
        }

      up_udelay(10);
    }

  return -ETIMEDOUT;
}

void rk3576_cru_pll_disable(int pll)
{
  uint32_t base = RK3576_CRU_ADDR;
  uint32_t reg = base + CRU_PLL_CON(pll);

  uint32_t val = getreg32(reg);
  val |= CRU_PLLCON0_POWERDOWN;
  putreg32(val, reg);
}

int rk3576_cru_pll_is_locked(int pll)
{
  uint32_t base = RK3576_CRU_ADDR;
  return (getreg32(base + CRU_PLL_CON(pll)) & CRU_PLLCON0_LOCK) ? 1 : 0;
}

void rk3576_cru_set_clkdiv(int reg, uint32_t div)
{
  uint32_t base = RK3576_CRU_ADDR;
  putreg32(div - 1, base + CRU_CLK_DIV(reg));
}

uint32_t rk3576_cru_get_clkdiv(int reg)
{
  uint32_t base = RK3576_CRU_ADDR;
  return (getreg32(base + CRU_CLK_DIV(reg)) & 0xffff) + 1;
}

void rk3576_cru_gate_enable(int reg, int bit)
{
  uint32_t base = RK3576_CRU_ADDR;
  uint32_t val = getreg32(base + CRU_CLKGATE_CON(reg));
  val |= (1 << bit);
  putreg32(val, base + CRU_CLKGATE_CON(reg));
}

void rk3576_cru_gate_disable(int reg, int bit)
{
  uint32_t base = RK3576_CRU_ADDR;
  uint32_t val = getreg32(base + CRU_CLKGATE_CON(reg));
  val &= ~(1 << bit);
  putreg32(val, base + CRU_CLKGATE_CON(reg));
}

void rk3576_cru_softrst(int reg, int bit)
{
  uint32_t base = RK3576_CRU_ADDR;
  putreg32(1 << bit, base + CRU_SOFTRST_CON(reg));
  up_udelay(10);
  putreg32(0, base + CRU_SOFTRST_CON(reg));
}
