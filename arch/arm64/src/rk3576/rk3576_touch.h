/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_touch.h - Touch screen driver
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_TOUCH_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_TOUCH_H

#include <nuttx/config.h>
#include <stdint.h>

#ifndef __ASSEMBLY__



struct rk3576_touch_point_s
{
  uint16_t x;
  uint16_t y;
  uint8_t  pressure;
  uint8_t  id;
  bool     touched;
};

void rk3576_touch_init(void);
int  rk3576_touch_read(struct rk3576_touch_point_s *point);
int  rk3576_touch_read_multi(struct rk3576_touch_point_s *points, int max_points);
int  rk3576_touch_get_max_points(void);

#endif /* __ASSEMBLY__ */

#endif
