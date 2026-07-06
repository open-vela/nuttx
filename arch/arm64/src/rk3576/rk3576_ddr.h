/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_ddr.h
 *
 * DDR Controller driver for RK3576
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_DDR_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_DDR_H

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef __ASSEMBLY__



void rk3576_ddr_init(void);
bool rk3576_ddr_is_ready(void);
uint32_t rk3576_ddr_get_size(void);

#endif /* __ASSEMBLY__ */

#endif
