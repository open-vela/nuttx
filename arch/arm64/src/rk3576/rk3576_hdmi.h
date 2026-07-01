/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_hdmi.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_HDMI_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_HDMI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HDMI output modes */

#define RK3576_HDMI_MODE_HDMI      0
#define RK3576_HDMI_MODE_DVI       1

/* Video formats */

#define RK3576_HDMI_FMT_RGB888     0
#define RK3576_HDMI_FMT_RGB565     1
#define RK3576_HDMI_FMT_YCBCR444  2
#define RK3576_HDMI_FMT_YCBCR422  3

/* Resolution presets */

#define RK3576_HDMI_RES_640x480    0
#define RK3576_HDMI_RES_800x600    1
#define RK3576_HDMI_RES_1024x768   2
#define RK3576_HDMI_RES_1280x720   3
#define RK3576_HDMI_RES_1280x1024  4
#define RK3576_HDMI_RES_1920x1080  5

/****************************************************************************
 * Video timing structure
 ****************************************************************************/

struct rk3576_hdmi_timing_s
{
  uint16_t hactive;             /* Horizontal active pixels */
  uint16_t hsync_start;         /* HSync start */
  uint16_t hsync_end;           /* HSync end */
  uint16_t htotal;              /* Total horizontal pixels */
  uint16_t vactive;             /* Vertical active lines */
  uint16_t vsync_start;         /* VSync start */
  uint16_t vsync_end;           /* VSync end */
  uint16_t vtotal;              /* Total vertical lines */
  uint32_t pixel_clock;         /* Pixel clock in Hz */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialization */

void rk3576_hdmi_init(void);

/* Configuration */

int  rk3576_hdmi_set_resolution(int resolution);
int  rk3576_hdmi_set_format(int format);
int  rk3576_hdmi_set_mode(int mode);

/* Control */

void rk3576_hdmi_enable(void);
void rk3576_hdmi_disable(void);

/* Status */

int  rk3576_hdmi_is_connected(void);
int  rk3576_hdmi_get_resolution(void);

/* EDID */

int  rk3576_hdmi_read_edid(uint8_t *buf, int len);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_HDMI_H */
