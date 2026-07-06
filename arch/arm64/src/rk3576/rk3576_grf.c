/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_grf.c
 *
 * General Register File (GRF) driver for RK3576
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include "arm64_internal.h"
#include "hardware/rk3576_grf.h"
#include "rk3576_grf.h"

void rk3576_grf_init(void)
{
  ginfo("GRF: initialized\n");
}

uint32_t rk3576_grf_read(uint32_t offset)
{
  return getreg32(RK3576_GRF_ADDR + offset);
}

void rk3576_grf_write(uint32_t offset, uint32_t value)
{
  putreg32(value, RK3576_GRF_ADDR + offset);
}

void rk3576_grf_set_bits(uint32_t offset, uint32_t set, uint32_t clr)
{
  irqstate_t flags = enter_critical_section();
  uint32_t val = rk3576_grf_read(offset);
  val = (val & ~clr) | set;
  rk3576_grf_write(offset, val);
  leave_critical_section(flags);
}

uint32_t rk3576_pmugrf_read(uint32_t offset)
{
  return getreg32(RK3576_PMUGRF_ADDR + offset);
}

void rk3576_pmugrf_write(uint32_t offset, uint32_t value)
{
  putreg32(value, RK3576_PMUGRF_ADDR + offset);
}

uint32_t rk3576_sysgrf_read(uint32_t offset)
{
  return getreg32(RK3576_SYSGRF_ADDR + offset);
}

void rk3576_sysgrf_write(uint32_t offset, uint32_t value)
{
  putreg32(value, RK3576_SYSGRF_ADDR + offset);
}
