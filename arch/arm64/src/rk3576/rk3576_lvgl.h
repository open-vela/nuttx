/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_lvgl.h
 *
 * LVGL integration for RK3576
 *
 ***************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_LVGL_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_LVGL_H

#include <nuttx/config.h>

#ifndef __ASSEMBLY__



#ifdef CONFIG_RK3576_LVGL

int  rk3576_lvgl_init(void);
void rk3576_lvgl_task(void);

#else

#define rk3576_lvgl_init()  (-ENODEV)
#define rk3576_lvgl_task()  ((void)0)

#endif

#endif /* __ASSEMBLY__ */

#endif
