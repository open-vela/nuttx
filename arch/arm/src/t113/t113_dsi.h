/****************************************************************************
 * arch/arm/src/t113/t113_dsi.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_T113_T113_DSI_H
#define __ARCH_ARM_SRC_T113_T113_DSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

struct mipi_dsi_host;

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Video timing for the DSI host's HS pixel sequencer.  The 'hbp' and 'vbp'
 * fields follow the panel-vendor convention: each INCLUDES its sync pulse,
 * so back-porch-only = hbp - hsync (resp. vbp - vsync).  This matches the
 * GC9503CV / X4B vendor reference numbers exactly so that values copied
 * out of a panel datasheet drop straight in.
 *
 * All counts are in pixel-clock cycles for horizontal fields and lines for
 * vertical fields.  'format' encodes the wire pixel format:
 *   0 = RGB888 (24 bpp)    - only mode used by the X4B bring-up
 *   1 = RGB666 loose (24)
 *   2 = RGB666 packed (18)
 *   3 = RGB565 (16)
 */

struct t113_dsi_video_s
{
  uint32_t pixel_clk_hz;
  uint16_t hactive;
  uint16_t htotal;
  uint16_t hbp;        /* incl hsync */
  uint16_t hsync;
  uint16_t vactive;
  uint16_t vtotal;
  uint16_t vbp;        /* incl vsync */
  uint16_t vsync;
  uint8_t  lanes;      /* 1, 2 or 4 */
  uint8_t  format;     /* 0=RGB888, 1=RGB666 loose, 2=RGB666, 3=RGB565 */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialize and register the DSI host with the NuttX MIPI DSI framework.
 * Returns 0 on success, negative errno on failure.
 */

int t113_dsi_initialize(void);

/* Get a pointer to the registered mipi_dsi_host (for board glue / panel
 * driver attachment).  Returns NULL before t113_dsi_initialize() succeeds.
 */

struct mipi_dsi_host *t113_dsi_get_host(void);

/* Pulse the HSC chain (LP11 -> HSC -> END) so the DPHY clock lane locks
 * into HS continuous mode.  Must be called AFTER the panel reset edge and
 * BEFORE the first LP DCS command is issued - the GC9503CV (and most
 * Allwinner DSI panels) want the clock lane already running so the byte
 * clock is stable while the data lane goes through LP escape.  Without
 * this the LP DCS bytes are clocked into a clock lane that is still in
 * LP-11 idle and the DDIC sees nothing.  Mirrors vendor dsi_clk_enable()
 * + dsi_start(DSI_START_HSC).  Returns 0 on success, negative errno if
 * the host has not been initialised yet.
 */

int t113_dsi_start_clk(void);

/* Bring the DSI host out of LP-only mode and start streaming HS video to
 * the panel.  Must be called AFTER the panel-side DCS init sequence has
 * completed (the host needs to remain in LP for those commands) and BEFORE
 * pixels are clocked in by the TCON.  Returns 0 on success, negative errno
 * if the supplied timing fails sanity checks.
 */

int t113_dsi_start_video(const struct t113_dsi_video_s *v);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_T113_T113_DSI_H */
