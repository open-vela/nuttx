/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_grf.h
 *
 * General Register File (GRF) driver for RK3576
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_GRF_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_GRF_H

#include <nuttx/config.h>
#include <stdint.h>

#ifndef __ASSEMBLY__



void rk3576_grf_init(void);

/* Register access */

uint32_t rk3576_grf_read(uint32_t offset);
void rk3576_grf_write(uint32_t offset, uint32_t value);
void rk3576_grf_set_bits(uint32_t offset, uint32_t set, uint32_t clr);

/* PMU GRF access */

uint32_t rk3576_pmugrf_read(uint32_t offset);
void rk3576_pmugrf_write(uint32_t offset, uint32_t value);

/* System GRF access */

uint32_t rk3576_sysgrf_read(uint32_t offset);
void rk3576_sysgrf_write(uint32_t offset, uint32_t value);

#endif /* __ASSEMBLY__ */

#endif
