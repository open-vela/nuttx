/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_dsi.h
 *
 * MIPI DSI Host controller driver for RK3576
 *
 ***************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_DSI_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_DSI_H

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef __ASSEMBLY__



#define RK3576_DSI0              0
#define RK3576_DSI1              1

/* DSI command structure */

struct rk3576_dsi_cmd_s
{
  uint8_t cmd;            /* DCS command byte */
  const uint8_t *params;  /* Parameter data (NULL if no params) */
  int param_len;          /* Number of parameters */
  int delay_ms;           /* Delay after command (0 = none) */
};

/* Panel timing and configuration */

struct rk3576_dsi_panel_s
{
  /* Video timing */

  int hfp;                /* Horizontal front porch */
  int hbp;                /* Horizontal back porch */
  int hsa;                /* Horizontal sync active */
  int vfp;                /* Vertical front porch */
  int vbp;                /* Vertical back porch */
  int vsa;                /* Vertical sync active */
  int hact;               /* Horizontal active pixels */
  int vact;               /* Vertical active lines */

  /* Signal polarity (true = active high) */

  bool hsync_pol;
  bool vsync_pol;
  bool dataen_pol;
  bool dotclk_pol;

  /* DSI configuration */

  int lanes;              /* Number of data lanes (2 or 4) */
  int color_fmt;          /* Color coding (DSI_DPI_xxx) */

  /* Panel initialization sequence */

  const struct rk3576_dsi_cmd_s *init_cmds;
  int init_cmd_count;
};

/* Core functions */

int rk3576_dsi_init(int dsi);
int rk3576_dsi_enable(int dsi);
int rk3576_dsi_disable(int dsi);

/* Configuration */

int rk3576_dsi_set_lanes(int dsi, int lanes);
int rk3576_dsi_set_color(int dsi, int color_fmt);
int rk3576_dsi_set_cmd_mode(int dsi, bool enable);
int rk3576_dsi_set_video_timing(int dsi, int hfp, int hbp, int hsa,
                                  int vfp, int vbp, int vsa, int vact);
int rk3576_dsi_set_polarity(int dsi, bool hsync_pol, bool vsync_pol,
                              bool dataen_pol, bool dotclk_pol);

/* DCS command interface */

int rk3576_dsi_dcs_write(int dsi, uint8_t cmd, const uint8_t *params,
                           int len);
int rk3576_dcs_read(int dsi, uint8_t cmd, uint8_t *data, int len);

/* Panel initialization */

int rk3576_dsi_panel_init(int dsi, const struct rk3576_dsi_panel_s *panel);

#endif /* __ASSEMBLY__ */

#endif
