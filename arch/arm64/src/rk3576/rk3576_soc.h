/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_soc.h
 *
 * RK3576 low-level SoC helpers for the AMP slave: clock gating / muxing
 * (CRU) and pin multiplexing / pull configuration (IOC).
 *
 * The slave owns only a handful of peripherals, so instead of a full clock
 * framework these helpers just poke the same registers the Linux drivers
 * would.  All Rockchip CRU/IOC registers are "hiword mask" registers: the
 * upper 16 bits select which of the lower 16 bits are written, which makes
 * every write atomic with respect to the other core.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_SOC_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_SOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CRU register blocks (see arch/arm64/include/rk3576/chip.h for the base) */

#define RK3576_CLKSEL_CON(x)    (RK3576_CRU_BASE + 0x300 + (x) * 4)
#define RK3576_CLKGATE_CON(x)   (RK3576_CRU_BASE + 0x800 + (x) * 4)
#define RK3576_SOFTRST_CON(x)   (RK3576_CRU_BASE + 0xa00 + (x) * 4)

/* GPIO banks, as used by the pin helpers below */

#define RK3576_GPIO_BANK0       0
#define RK3576_GPIO_BANK1       1
#define RK3576_GPIO_BANK2       2
#define RK3576_GPIO_BANK3       3
#define RK3576_GPIO_BANK4       4

/* Pin numbers inside a bank: group A/B/C/D x 0..7 */

#define RK3576_PIN(group, idx)  (((group) * 8) + (idx))
#define RK3576_PIN_A(idx)       RK3576_PIN(0, idx)
#define RK3576_PIN_B(idx)       RK3576_PIN(1, idx)
#define RK3576_PIN_C(idx)       RK3576_PIN(2, idx)
#define RK3576_PIN_D(idx)       RK3576_PIN(3, idx)

/* Pull configuration values (RK3576 uses 2 bits per pin).
 *
 * NOT the encoding most Rockchip SoCs use.  Every RK3576 bank is declared
 * PULL_TYPE_IO_1 in pinctrl-rockchip.c, whose rockchip_pull_list[] row is
 * { disable, pull-down, disable, pull-up } — so 1 is a pull-DOWN and a
 * pull-up is 3, not 1.  Getting this backwards is silent: the pin still
 * works whenever something external drives it hard enough.
 */

#define RK3576_PULL_NONE        0
#define RK3576_PULL_DOWN        1
#define RK3576_PULL_UP          3

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Name: rk3576_clk_gate
 *
 * Description:
 *   Enable or disable a clock gate.  CRU gate bits are active-low (0 =
 *   running), and the write is masked so only this bit changes.
 *
 * Input Parameters:
 *   con    - CLKGATE_CON index
 *   bit    - bit number inside that register
 *   enable - true to let the clock run, false to gate it off
 *
 ****************************************************************************/

void rk3576_clk_gate(unsigned int con, unsigned int bit, bool enable);

/****************************************************************************
 * Name: rk3576_clk_set_mux_div
 *
 * Description:
 *   Program the parent mux and divider of a composite clock in one masked
 *   write.  Pass width 0 for a field that should be left alone.
 *
 * Input Parameters:
 *   con        - CLKSEL_CON index
 *   mux_shift  - LSB of the mux field
 *   mux_width  - width of the mux field in bits (0 = do not touch)
 *   mux        - parent index to select
 *   div_shift  - LSB of the divider field
 *   div_width  - width of the divider field in bits (0 = do not touch)
 *   div        - divider value, 1-based (hardware stores div - 1)
 *
 ****************************************************************************/

void rk3576_clk_set_mux_div(unsigned int con,
                            unsigned int mux_shift, unsigned int mux_width,
                            unsigned int mux,
                            unsigned int div_shift, unsigned int div_width,
                            unsigned int div);

/****************************************************************************
 * Name: rk3576_iomux
 *
 * Description:
 *   Select the alternate function of a pin.
 *
 * Input Parameters:
 *   bank - GPIO bank (0..4)
 *   pin  - pin inside the bank (0..31, see RK3576_PIN_x())
 *   func - IOMUX function number (0 = GPIO)
 *
 ****************************************************************************/

void rk3576_iomux(unsigned int bank, unsigned int pin, unsigned int func);

/****************************************************************************
 * Name: rk3576_pull
 *
 * Description:
 *   Configure the pull-up/pull-down of a pin (RK3576_PULL_x).
 *
 ****************************************************************************/

void rk3576_pull(unsigned int bank, unsigned int pin, unsigned int pull);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_SOC_H */
