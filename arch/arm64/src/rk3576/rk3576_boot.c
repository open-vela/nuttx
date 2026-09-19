/****************************************************************************
 * board/contest_board/chip/rk3576_boot.c
 *
 * RK3576 (KickPi K7) AMP-slave chip boot: MMU map + board hook.
 *
 * AMP slave rules honoured here:
 *   - map ONLY what we own: the 8MB private carve-out, the shared-memory
 *     window (Normal Non-cacheable, matching the Linux side), and the
 *     peripheral window.  The rest of DRAM belongs to Linux and stays
 *     unmapped so a stray pointer faults instead of corrupting Linux.
 *   - no PSCI: this core is brought up by U-Boot via PSCI CPU_ON; calling
 *     PSCI from here (cpu on/off, system reset) would affect the whole SoC.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/chip/chip.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_mmu.h"
#include "rk3576_boot.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct arm_mmu_region g_mmu_regions[] =
{
  MMU_REGION_FLAT_ENTRY("DEVICEIO",
                        CONFIG_DEVICEIO_BASEADDR, CONFIG_DEVICEIO_SIZE,
                        MT_DEVICE_NGNRNE | MT_RW | MT_SECURE),

  MMU_REGION_FLAT_ENTRY("DRAM_PRIVATE",
                        CONFIG_RAMBANK1_ADDR, CONFIG_RAMBANK1_SIZE,
                        MT_NORMAL | MT_RW | MT_SECURE),

  MMU_REGION_FLAT_ENTRY("AMP_SHM",
                        RK3576_AMP_SHM_BASE, RK3576_AMP_SHM_SIZE,
                        MT_NORMAL_NC | MT_RW | MT_SECURE),
};

const struct arm_mmu_config g_mmu_config =
{
  .num_regions = nitems(g_mmu_regions),
  .mmu_regions = g_mmu_regions,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm64_el_init(void)
{
  /* Entered from BL31 PSCI CPU_ON; no EL3/EL2 setup required here. */
}

void arm64_chip_boot(void)
{
  /* Map I/O, private DRAM and shared memory, then enable the MMU. */

  arm64_mmu_init(true);

  /* Board-level bring-up (UART5 clock/pinmux bootstrap etc.).  Must run
   * before the early serial init: nothing else on this SoC has programmed
   * UART5 for us at this point (we boot before Linux).
   */

  rk3576_board_initialize();

#ifdef USE_EARLYSERIALINIT
  arm64_earlyserialinit();
#endif
}

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void arm64_netinitialize(void)
{
  /* No networking on the AMP slave. */
}
#endif
