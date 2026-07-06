/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_csi.h
 *
 * MIPI CSI-2 Camera Interface driver for RK3576
 *
 ***************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_CSI_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_CSI_H

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef __ASSEMBLY__



#define RK3576_CSI0              0
#define RK3576_CSI1              1

/* CSI configuration */

struct rk3576_csi_config_s
{
  int lanes;              /* Number of data lanes (1, 2, or 4) */
  int format;             /* Image format (CSI_FMT_xxx) */
  int width;              /* Image width in pixels */
  int height;             /* Image height in lines */
  int data_type;          /* CSI data type (CSI_DT_xxx) */
};

/* Core functions */

int rk3576_csi_init(int csi);
int rk3576_csi_enable(int csi);
int rk3576_csi_disable(int csi);

/* Configuration */

int rk3576_csi_set_config(int csi, const struct rk3576_csi_config_s *cfg);
int rk3576_csi_set_lanes(int csi, int lanes);
int rk3576_csi_set_format(int csi, int format, int width, int height);

/* Interrupt management */

int rk3576_csi_enable_irq(int csi, uint32_t mask);
int rk3576_csi_disable_irq(int csi, uint32_t mask);
uint32_t rk3576_csi_get_status(int csi);
int rk3576_csi_clear_status(int csi, uint32_t mask);

/* Frame capture */

int rk3576_csi_start_capture(int csi);
int rk3576_csi_stop_capture(int csi);
bool rk3576_csi_is_frame_done(int csi);

#endif /* __ASSEMBLY__ */

#endif
