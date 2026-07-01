/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_pmu.c
 *
 * Power Management Unit (PMU) driver for RK3576
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <debug.h>
#include <nuttx/arch.h>
#include "arm64_internal.h"
#include "hardware/rk3576_pmu.h"
#include "rk3576_pmu.h"

void rk3576_pmu_init(void)
{
  ginfo("PMU: initialized\n");
}

void rk3576_pmu_pd_enable(int pd)
{
  if (pd < 0 || pd >= 16)
    {
      return;
    }

  uint32_t val = getreg32(RK3576_PMU_ADDR + PMU_PD_CON);
  val &= ~(3 << (pd * 2));
  val |= (PMU_PD_ON << (pd * 2));
  putreg32(val, RK3576_PMU_ADDR + PMU_PD_CON);
}

void rk3576_pmu_pd_disable(int pd)
{
  if (pd < 0 || pd >= 16)
    {
      return;
    }

  uint32_t val = getreg32(RK3576_PMU_ADDR + PMU_PD_CON);
  val &= ~(3 << (pd * 2));
  val |= (PMU_PD_OFF << (pd * 2));
  putreg32(val, RK3576_PMU_ADDR + PMU_PD_CON);
}

void rk3576_pmu_sleep(void)
{
  uint32_t val = getreg32(RK3576_PMU_ADDR + PMU_SFT_CON);
  val |= PMU_SFT_WAKEUP | PMU_SFT_DDR_GATING;
  putreg32(val, RK3576_PMU_ADDR + PMU_SFT_CON);
  ginfo("PMU: entering sleep\n");
}

void rk3576_pmu_wakeup(void)
{
  uint32_t val = getreg32(RK3576_PMU_ADDR + PMU_SFT_CON);
  val &= ~PMU_SFT_WAKEUP;
  putreg32(val, RK3576_PMU_ADDR + PMU_SFT_CON);
}

uint32_t rk3576_pmu_get_power_status(void)
{
  return getreg32(RK3576_PMU_ADDR + PMU_POWER_STS);
}

uint32_t rk3576_pmu_get_int_status(void)
{
  return getreg32(RK3576_PMU_ADDR + PMU_INT_STS);
}

void rk3576_pmu_scratch_write(uint32_t offset, uint32_t value)
{
  putreg32(value, RK3576_PMU_ADDR + PMU_SCRATCH_ST + offset);
}

uint32_t rk3576_pmu_scratch_read(uint32_t offset)
{
  return getreg32(RK3576_PMU_ADDR + PMU_SCRATCH_ST + offset);
}
