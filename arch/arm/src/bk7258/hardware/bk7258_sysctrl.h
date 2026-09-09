/****************************************************************************
 * arch/arm/src/bk7258/hardware/bk7258_sysctrl.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_SYSCTRL_H
#define __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_SYSCTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bk7258_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BK7258_SYS_CLKDIV1        (BK7258_SYSCTRL_BASE + 0x20u)
#define BK7258_SYS_DEVCLK_EN      (BK7258_SYSCTRL_BASE + 0x30u)
#define BK7258_CPU1_IRQ_EN0       (BK7258_SYSCTRL_BASE + 0x88u)
#define BK7258_CPU1_IRQ_EN1       (BK7258_SYSCTRL_BASE + 0x8cu)
#define BK7258_GPIO_FUNC0         (BK7258_SYSCTRL_BASE + 0xc0u)

#define BK7258_UART1_CLKDIV_SHIFT 11
#define BK7258_UART1_CLKDIV_MASK  (3u << BK7258_UART1_CLKDIV_SHIFT)
#define BK7258_UART1_CLKSEL       (1u << 13)
#define BK7258_UART1_CLK_EN       (1u << 10)
#define BK7258_UART1_IRQ_ROUTE    (1u << 15)

/* BK7258_SYS_DEVCLK_EN bit[8]: "i2c1_clk enable" per
 * bk_avdk_smp release/v3.1.1
 * ap/middleware/soc/bk7258_ap/hal/sys_types.h's register map comment
 * block (register word index 0xc, i.e. byte offset 0xc*4=0x30 from
 * BK7258_SYSCTRL_BASE -- matching BK7258_SYS_DEVCLK_EN's existing
 * offset, and independently cross-checked by BK7258_UART1_CLK_EN
 * above already using bit[10] for the same register's documented
 * "uart1_clk enable" bit).  This is the *module* clock gate for the
 * hardware I2C1 peripheral (SOC_I2C1_REG_BASE) -- distinct from
 * sm_bus_cfg.freq_div (the I2C *bus* baud-rate divider) and
 * global_ctrl.clk_gate_bypass (an internal I2C1-block clock-gate
 * bypass for reading the ack bit).  Without this bit set, register
 * writes to the I2C1 block still land in mock/real memory (no bus
 * fault), but the state machine never actually runs: START never
 * produces any sm_int, exactly matching this driver's observed board
 * bring-up symptom (int_status stuck at only the start bit set,
 * sm_int never firing) before this bit was added to
 * bk7258_i2c1_init().
 */
#define BK7258_I2C1_MODULE_CLK_EN (1u << 8)

/* BK7258_SYS_DEVCLK_EN bit[20]/bit[21]: "qspi0_clk enable"/"qspi1_clk
 * enable" per the same bk_avdk_smp release/v3.1.1
 * ap/middleware/soc/bk7258_ap/hal/sys_types.h register map comment block
 * used to derive BK7258_I2C1_MODULE_CLK_EN above (source lines:
 * "0xc[20],1:qspi0_clk enable,0,R/W" and "0xc[21],1:qspi1_clk
 * enable,0,R/W").
 * This is the *module* clock gate for the QSPI0/QSPI1 controller blocks --
 * distinct from qspi_hw_t's own glb_ctrl.soft_reset/bps_clkgate fields
 * (which bk7258_qspi.c already touches) and from the QSPI_CLK_320M/480M
 * source-clock-select + divider path (sys_drv_qspi_clk_sel()/
 * sys_drv_qspi_set_src_clk_div() in
 * ap/middleware/driver/qspi/qspi_driver.c, not yet ported to this repo).
 * bk7258_qspi.c's original implementation never set this bit -- discovered
 * while diagnosing GC9D01's persistent "cmd 0xFE timed out" symptom after
 * the QSPI0->QSPI1 pin-routing fix alone did not resolve it, by
 * cross-referencing qspi_id_init_common()'s documented call order ("1. set
 * clock, 2. set gpio as qspi, 3. enable interrupt") against this driver's
 * qspi0_init(), which only did step 2.  Same failure signature as
 * BK7258_I2C1_MODULE_CLK_EN's documented symptom applies here by analogy
 * (register writes accepted, but cmd_start_done never asserts because the
 * controller's internal state machine has no clock) -- not yet confirmed
 * on real hardware as of this comment being written; see
 * docs/superpowers/plans/2026-07-31-gc9d01-qspi1-camera-v4l2-verification.md
 * section 5.1 for the diagnostic trail this was found through.
 */
#define BK7258_QSPI0_MODULE_CLK_EN (1u << 20)
#define BK7258_QSPI1_MODULE_CLK_EN (1u << 21)

/* QSPI1 source-clock select and divider.
 *
 * The QSPI block's own config.clk_rate field is NOT where the vendor
 * driver sets the panel clock: bk_qspi_init()
 * (bk_avdk_smp release/v3.1.1 ap/middleware/driver/qspi/qspi_driver.c)
 * programs the source mux and pre-divider through sysctrl via
 * sys_drv_qspi_clk_sel() / sys_drv_qspi_set_src_clk_div(), and only then
 * calls qspi_hal_set_clk_div(hal, config->clk_div) -- with clk_div == 0
 * for every LCD path, i.e. config.clk_rate stays 0.
 *
 * Register/field citation: ap/middleware/soc/bk7258_ap/soc/sys_reg.h
 *   SYS_CPU_26M_WDT_CLK_DIV_ADDR      = SOC_SYS_REG_BASE + (0xa << 2)
 *   SYS_CPU_26M_WDT_CLK_DIV_CKDIV_QSPI1_POS  = 6,  MASK = 0xf
 *   SYS_CPU_26M_WDT_CLK_DIV_CKSEL_QSPI1_POS  = 10, MASK = 0x1
 * and hal/sys_types.h's { QSPI_CLK_320M = 0, QSPI_CLK_480M = 1 }.
 *
 * SCK = src / (1 + ckdiv) / (clk_rate == 0 ? 1 : 2 * clk_rate)
 * (qspi_clk_div_factor() in qspi_driver.c), so the GC9D01 reference
 * config's LCD_QSPI_60M -- src_clk = QSPI_SCLK_480M, src_clk_div = 7,
 * clk_div = 0 (lcd_spi_driver_init_with_qspi()) -- is 480 / 8 = 60MHz.
 */

#define BK7258_SYS_CPU26M_WDT_CLKDIV (BK7258_SYSCTRL_BASE + 0x28u)

#define BK7258_QSPI1_CKDIV_SHIFT   6
#define BK7258_QSPI1_CKDIV_MASK    (0xfu << BK7258_QSPI1_CKDIV_SHIFT)
#define BK7258_QSPI1_CKSEL_480M    (1u << 10)

/* QSPI0's equivalent fields live in a different sysctrl register:
 *   SYS_CPU_CLK_DIV_MODE2_ADDR             = SOC_SYS_REG_BASE + (0x9 << 2)
 *   SYS_CPU_CLK_DIV_MODE2_CKDIV_QSPI0_POS  = 6,  MASK = 0xf
 *   SYS_CPU_CLK_DIV_MODE2_CKSEL_QSPI0_POS  = 10, MASK = 0x1
 * (same bit positions as QSPI1, different register -- easy to conflate).
 */

#define BK7258_SYS_CPU_CLKDIV_MODE2  (BK7258_SYSCTRL_BASE + 0x24u)

#define BK7258_QSPI0_CKDIV_SHIFT   6
#define BK7258_QSPI0_CKDIV_MASK    (0xfu << BK7258_QSPI0_CKDIV_SHIFT)
#define BK7258_QSPI0_CKSEL_480M    (1u << 10)

#endif
