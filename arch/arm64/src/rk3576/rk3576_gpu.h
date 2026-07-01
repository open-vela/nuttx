/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_gpu.h
 *
 * Mali-G51 GPU driver for RK3576
 * Supports open-source Lima/Panfrost driver integration
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_GPU_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_GPU_H

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/* GPU types */

#define GPU_TYPE_NONE            0
#define GPU_TYPE_MALI_G51        1
#define GPU_TYPE_SOFTWARE        2

/* GPU power states */

#define GPU_POWER_OFF            0
#define GPU_POWER_ON             1
#define GPU_POWER_RESET          2

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialization */

int  rk3576_gpu_init(void);
void rk3576_gpu_deinit(void);

/* Power management */

int  rk3576_gpu_power_on(void);
int  rk3576_gpu_power_off(void);
int  rk3576_gpu_reset(void);

/* Information */

int  rk3576_gpu_get_type(void);
uint32_t rk3576_gpu_get_id(void);
bool rk3576_gpu_is_available(void);

/* Memory mapping */

int  rk3576_gpu_map_registers(void);
void rk3576_gpu_unmap_registers(void);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_GPU_H */
