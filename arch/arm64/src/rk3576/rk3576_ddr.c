/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_ddr.c
 *
 * DDR Controller driver for RK3576
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <nuttx/arch.h>
#include "arm64_internal.h"
#include "hardware/rk3576_ddr.h"
#include "rk3576_ddr.h"

void rk3576_ddr_init(void)
{
  /* DDR is typically initialized by bootloader (BL2/BL31) */
  /* This driver provides status query only */

  ginfo("DDR: initialized (by bootloader)\n");
}

bool rk3576_ddr_is_ready(void)
{
  uint32_t status = getreg32(RK3576_DDRPCTL_ADDR + DDR_STATUS);
  return (status & DDR_STATUS_INIT_DONE) != 0;
}

uint32_t rk3576_ddr_get_size(void)
{
  /* DDR size is determined by hardware strapping */
  /* Common configurations: 2GB, 4GB, 8GB */

  if (rk3576_ddr_is_ready())
    {
      /* TODO: Read actual size from DDR controller registers */

      ginfo("DDR: size not queried, using default 4GB\n");
      return 4096;  /* MB */
    }

  return 0;
}
