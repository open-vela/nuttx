/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_pmu.h
 *
 * Power Management Unit (PMU) driver for RK3576
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_PMU_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_PMU_H

#include <nuttx/config.h>
#include <stdint.h>

#ifndef __ASSEMBLY__



void rk3576_pmu_init(void);

/* Power domain control */

void rk3576_pmu_pd_enable(int pd);
void rk3576_pmu_pd_disable(int pd);

/* Sleep control */

void rk3576_pmu_sleep(void);
void rk3576_pmu_wakeup(void);

/* Status */

uint32_t rk3576_pmu_get_power_status(void);
uint32_t rk3576_pmu_get_int_status(void);

/* Scratch memory (for bootloader communication) */

void rk3576_pmu_scratch_write(uint32_t offset, uint32_t value);
uint32_t rk3576_pmu_scratch_read(uint32_t offset);

#endif /* __ASSEMBLY__ */

#endif
