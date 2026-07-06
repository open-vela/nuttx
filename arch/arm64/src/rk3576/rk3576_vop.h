/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_vop.h
 *
 * Video Output Processor (VOP) driver for RK3576
 *
 ***************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_VOP_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_VOP_H

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef __ASSEMBLY__



#define RK3576_VOP0              0
#define RK3576_VOP1              1

/* Window info structure */

struct rk3576_vop_window_info_s
{
  bool enabled;
  int format;
  int width;
  int height;
  int x;
  int y;
  uint32_t addr;
};

/* Alpha blending modes */

#define VOP_ALPHA_PIXEL          0  /* Per-pixel alpha */
#define VOP_ALPHA_PLANE          1  /* Plane alpha */
#define VOP_ALPHA_COMBO          2  /* Combination */

/* Mirror modes */

#define VOP_MIRROR_NONE          0
#define VOP_MIRROR_H             1
#define VOP_MIRROR_V             2
#define VOP_MIRROR_HV            3

/* Core functions */

int rk3576_vop_init(int vop);
int rk3576_vop_enable(int vop);
int rk3576_vop_disable(int vop);

/* Display configuration */

int rk3576_vop_set_resolution(int vop, int w, int h);
int rk3576_vop_set_framebuffer(int vop, uint32_t addr);
int rk3576_vop_set_background(int vop, uint8_t r, uint8_t g, uint8_t b);

/* Window layer management */

int rk3576_vop_enable_window(int vop, int win);
int rk3576_vop_disable_window(int vop, int win);
int rk3576_vop_set_window(int vop, int win, int x, int y,
                            int w, int h, uint32_t addr, int format);
int rk3576_vop_set_window_alpha(int vop, int win, int mode, uint8_t alpha);
int rk3576_vop_set_window_mirror(int vop, int win, int mirror);

/* Scaling */

int rk3576_vop_set_scaler(int vop, int win, int src_w, int src_h,
                            int dst_w, int dst_h);

/* Interrupt management */

int rk3576_vop_enable_irq(int vop, uint32_t mask);
int rk3576_vop_disable_irq(int vop, uint32_t mask);
uint32_t rk3576_vop_get_status(int vop);
int rk3576_vop_clear_status(int vop, uint32_t mask);
int rk3576_vop_wait_vsync(int vop);

/* Status queries */

int rk3576_vop_get_line(int vop);
uint32_t rk3576_vop_get_version(int vop);
int rk3576_vop_get_window_info(int vop, int win,
                                 struct rk3576_vop_window_info_s *info);

#endif /* __ASSEMBLY__ */

#endif
