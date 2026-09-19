/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_soc.c
 *
 * RK3576 CRU (clock) and IOC (pinmux/pull) helpers for the AMP slave.
 *
 * Register layouts mirror the Linux drivers this core has to coexist with:
 *   clocks : drivers/clk/rockchip/clk-rk3576.c
 *   pins   : drivers/pinctrl/pinctrl-rockchip.c (rk3576_pin_banks and the
 *            rk3576_calc_pull_reg_and_bit() offsets)
 *
 * Every register here is a Rockchip "hiword mask" register — the upper 16
 * bits say which of the lower 16 bits the write applies to — so a write
 * touching only our bits cannot corrupt a field Linux owns.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/chip/chip.h>

#include "arm64_internal.h"
#include "rk3576_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOMUX register offsets from RK3576_IOC_BASE, one per pin group (A/B/C/D).
 * Each 32-bit register covers 4 pins (4 bits each) in its low half, so a
 * group of 8 pins spans two consecutive registers.
 */

static const uint32_t g_iomux_offset[5][4] =
{
  { 0x0000, 0x0008, 0x2004, 0x200c },   /* bank0 A B C D */
  { 0x4020, 0x4028, 0x4030, 0x4038 },   /* bank1         */
  { 0x4040, 0x4048, 0x4050, 0x4058 },   /* bank2         */
  { 0x4060, 0x4068, 0x4070, 0x4078 },   /* bank3         */
  { 0x4080, 0x4088, 0xa390, 0xb398 },   /* bank4         */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_pull_reg
 *
 * Description:
 *   Base offset of the pull register covering a pin, mirroring
 *   rk3576_calc_pull_reg_and_bit() in the Linux pinctrl driver.  Pull
 *   fields are 2 bits wide, 8 pins per register.
 *
 ****************************************************************************/

static uint32_t rk3576_pull_reg(unsigned int bank, unsigned int pin)
{
  uint32_t reg;

  switch (bank)
    {
    case 0:
      reg = (pin < 12) ? 0x0020 : (0x2028 - 0x4);
      break;

    case 1:
      reg = 0x6110;
      break;

    case 2:
      reg = 0x6120;
      break;

    case 3:
      reg = 0x6130;
      break;

    case 4:
    default:
      if (pin < 16)
        {
          reg = 0x6140;
        }
      else if (pin < 24)
        {
          reg = 0xa148 - 0x8;
        }
      else
        {
          reg = 0xb14c - 0xc;
        }
      break;
    }

  return reg + (pin / 8) * 4;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void rk3576_clk_gate(unsigned int con, unsigned int bit, bool enable)
{
  /* Gate bits are active low: 0 lets the clock run. */

  uint32_t val = enable ? 0u : (1u << bit);

  putreg32(((1u << bit) << 16) | val, RK3576_CLKGATE_CON(con));
}

void rk3576_clk_set_mux_div(unsigned int con,
                            unsigned int mux_shift, unsigned int mux_width,
                            unsigned int mux,
                            unsigned int div_shift, unsigned int div_width,
                            unsigned int div)
{
  uint32_t mask = 0;
  uint32_t val  = 0;

  if (mux_width != 0)
    {
      uint32_t m = (1u << mux_width) - 1u;

      mask |= m << mux_shift;
      val  |= (mux & m) << mux_shift;
    }

  if (div_width != 0)
    {
      uint32_t m = (1u << div_width) - 1u;

      /* Hardware stores (divider - 1) */

      mask |= m << div_shift;
      val  |= ((div - 1u) & m) << div_shift;
    }

  if (mask != 0)
    {
      putreg32((mask << 16) | val, RK3576_CLKSEL_CON(con));
    }
}

void rk3576_iomux(unsigned int bank, unsigned int pin, unsigned int func)
{
  uint32_t reg;
  uint32_t shift;

  if (bank > 4 || pin > 31)
    {
      return;
    }

  /* Group selects the base offset, the upper nibble of the group selects
   * which of the two registers, and the position inside that register is
   * 4 bits per pin.
   */

  reg   = g_iomux_offset[bank][pin / 8] + (((pin % 8) >= 4) ? 4 : 0);
  shift = (pin % 4) * 4;

  putreg32((0xfu << (shift + 16)) | ((func & 0xfu) << shift),
           RK3576_IOC_BASE + reg);
}

void rk3576_pull(unsigned int bank, unsigned int pin, unsigned int pull)
{
  uint32_t reg;
  uint32_t shift;

  if (bank > 4 || pin > 31)
    {
      return;
    }

  reg   = rk3576_pull_reg(bank, pin);
  shift = (pin % 8) * 2;

  putreg32((0x3u << (shift + 16)) | ((pull & 0x3u) << shift),
           RK3576_IOC_BASE + reg);
}
