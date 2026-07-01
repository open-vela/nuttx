/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_fb.h
 *
 * Framebuffer driver for RK3576 VOP
 *
 ***************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_FB_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_FB_H

#include <nuttx/config.h>
#include <stdint.h>

#ifndef __ASSEMBLY__



#ifdef CONFIG_RK3576_FB

int      rk3576_fb_init(int vop);
uint8_t *rk3576_fb_getmem(void);

#else

#define rk3576_fb_init(v)     (-ENODEV)
#define rk3576_fb_getmem()    (NULL)

#endif

#endif /* __ASSEMBLY__ */

#endif
