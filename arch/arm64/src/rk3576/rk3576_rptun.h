/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_rptun.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_RPTUN_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_RPTUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Shared-memory layout, must match the Linux rk3576-kickpi-k7-amp.dtsi */

#define RK3576_AMP_VRING0_BASE    0x47800000
#define RK3576_AMP_BUFPOOL_BASE   0x47a00000
#define RK3576_AMP_BUFPOOL_SIZE   0x00200000

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

int rk3576_rptun_init(const char *shmemname, const char *cpuname);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_RPTUN_H */
