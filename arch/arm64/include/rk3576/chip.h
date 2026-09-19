/****************************************************************************
 * arch/arm64/include/rk3576/chip.h
 *
 * Rockchip RK3576 (KickPi K7) — openvela AMP slave on cpu3 (Cortex-A53).
 *
 * This chip runs as an AMP SLAVE: Linux owns 7 cores and the whole SoC;
 * this image owns only the 8MB carve-out at 0x41800000, UART5, and a few
 * GIC SPIs routed to cpu3 by the Linux device tree (rockchip-amp node).
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM64_INCLUDE_RK3576_CHIP_H
#define __ARCH_ARM64_INCLUDE_RK3576_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define KB(x)           ((x) << 10)
#define MB(x)           (KB(x) << 10)
#define GB(x)           (MB(UINT64_C(x)) << 10)

/* RK3576 GIC-400 (GICv2).  NOTE: with CONFIG_ARM64_GIC_VERSION=2 the
 * arm64_gicv2 driver takes the CPU-interface (GICC) base from the macro
 * named CONFIG_GICR_BASE.
 */

#define CONFIG_GICD_BASE          0x2a701000
#define CONFIG_GICD_SIZE          0x10000
#define CONFIG_GICR_BASE          0x2a702000    /* GICC */
#define CONFIG_GICR_SIZE          0x10000

/* AMP slave private DRAM: U-Boot loads amp.img here; PSCI entry point */

#define CONFIG_RAMBANK1_ADDR      0x41800000
#define CONFIG_RAMBANK1_SIZE      MB(8)
#define CONFIG_LOAD_BASE          0x41800000

/* Peripheral window (IOC 0x26040000 / CRU 0x27200000 / GIC 0x2a70xxxx /
 * UART5 0x2ad80000) — one flat Device mapping.
 */

#define CONFIG_DEVICEIO_BASEADDR  0x26000000
#define CONFIG_DEVICEIO_SIZE      MB(80)

/* Shared memory with Linux (Normal Non-cacheable on both sides):
 *   0x47800000 vring0 / 0x47808000 vring1 (rockchip_rpmsg_softirq contract)
 *   0x47a00000 rpmsg buffer pool (allocated by Linux)
 *   0x47c00000 heartbeat / diagnostics block
 */

#define RK3576_AMP_SHM_BASE       0x47800000
#define RK3576_AMP_SHM_SIZE       MB(8)

#define RK3576_HEARTBEAT_BASE     0x47c00000

/* SoC units used by the board bring-up code */

#define RK3576_CRU_BASE           0x27200000
#define RK3576_IOC_BASE           0x26040000
#define RK3576_UART5_BASE         0x2ad80000

/* GIC SPIs owned by this core (absolute INTID = GIC_SPI + 32) */

#define RK3576_IRQ_UART5          113
#define RK3576_IRQ_RPMSG_RX       172   /* Linux -> us (irq_retrigger) */
#define RK3576_IRQ_RPMSG_TX       173   /* us -> Linux (GICD_ISPENDR)  */

/* AMP interrupt priority, matches the Linux amp-irqs DTS property */

#define RK3576_AMP_IRQ_PRIO_VAL   0xd0

#define MPID_TO_CLUSTER_ID(mpid)  ((mpid) & ~0xff)

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__

/* Single-core AMP slave: we boot on physical cpu3 (MPIDR Aff0 = 3), but as
 * far as this NuttX instance is concerned we ARE cpu0.  Returning the real
 * Aff0 would send arm64_head.S down the secondary-CPU path.
 */

.macro  get_cpu_id xreg0
  mov    \xreg0, xzr
.endm

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_INCLUDE_RK3576_CHIP_H */
