/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_npu.c
 *
 * Neural Processing Unit (NPU) driver for RK3576
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <nuttx/arch.h>
#include "arm64_internal.h"
#include "hardware/rk3576_npu.h"
#include "rk3576_npu.h"

int rk3576_npu_init(void)
{
  uint32_t base = RK3576_NPU_ADDR;

  /* Disable NPU */

  putreg32(0, base + NPU_CTRL);

  /* Enable clocks */

  putreg32(NPU_CLK_EN_CORE | NPU_CLK_EN_AXI | NPU_CLK_EN_AHB,
           base + NPU_CLK_EN);

  /* Verify clock enable */

  uint32_t clk = getreg32(base + NPU_CLK_EN);
  if ((clk & (NPU_CLK_EN_CORE | NPU_CLK_EN_AXI | NPU_CLK_EN_AHB)) !=
      (NPU_CLK_EN_CORE | NPU_CLK_EN_AXI | NPU_CLK_EN_AHB))
    {
      nerr("NPU: clock enable failed\n");
      return -EIO;
    }

  /* Clear interrupts */

  putreg32(0xffffffff, base + NPU_INTCLR);

  /* Disable interrupts */

  putreg32(0, base + NPU_INTEN);

  ninfo("NPU: initialized\n");
  return OK;
}

void rk3576_npu_enable(void)
{
  putreg32(NPU_CTRL_ENABLE, RK3576_NPU_ADDR + NPU_CTRL);
  ginfo("NPU: enabled\n");
}

void rk3576_npu_disable(void)
{
  putreg32(0, RK3576_NPU_ADDR + NPU_CTRL);
  ginfo("NPU: disabled\n");
}

void rk3576_npu_reset(void)
{
  putreg32(NPU_CTRL_RESET, RK3576_NPU_ADDR + NPU_CTRL);
  up_udelay(100);
  putreg32(0, RK3576_NPU_ADDR + NPU_CTRL);
}

bool rk3576_npu_is_busy(void)
{
  return (getreg32(RK3576_NPU_ADDR + NPU_STATUS) & NPU_STATUS_BUSY) != 0;
}

uint32_t rk3576_npu_get_status(void)
{
  return getreg32(RK3576_NPU_ADDR + NPU_STATUS);
}
