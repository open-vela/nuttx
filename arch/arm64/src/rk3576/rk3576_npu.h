/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_npu.h
 *
 * Neural Processing Unit (NPU) driver for RK3576
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_NPU_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_NPU_H

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef __ASSEMBLY__



int rk3576_npu_init(void);
void rk3576_npu_enable(void);
void rk3576_npu_disable(void);
void rk3576_npu_reset(void);
bool rk3576_npu_is_busy(void);
uint32_t rk3576_npu_get_status(void);

#endif /* __ASSEMBLY__ */

#endif
