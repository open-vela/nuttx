/****************************************************************************
 * include/nuttx/lcd/gc9503cv_dsi.h
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

#ifndef __INCLUDE_NUTTX_LCD_GC9503CV_DSI_H
#define __INCLUDE_NUTTX_LCD_GC9503CV_DSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mipi_dsi_host;

/* Board-supplied glue.  The panel driver does not know which SoC GPIO is
 * wired to the panel reset pin nor whether reset is active-low or
 * active-high; the board layer owns that knowledge and exposes it through
 * this callback.
 */

struct gc9503cv_config_s
{
  /* Panel reset GPIO control.  Called with active=true to assert (panel
   * held in reset) and active=false to deassert (panel released).  The
   * board layer owns the GPIO line and the inversion semantics.
   */

  CODE int (*reset)(bool active);

  /* Optional hook invoked between the panel hardware reset and the DCS
   * init stream.  May be NULL.  The GC9503CV (and most Allwinner DSI
   * panels) require the DPHY clock lane to already be running in HS
   * continuous mode before the first LP DCS write — otherwise the byte
   * clock is unstable while the data lane goes through LP escape and the
   * DDIC silently drops every command.  On T113 the board layer wires
   * this to t113_dsi_start_clk(); other SoCs are free to leave it NULL.
   */

  CODE int (*pre_init)(void);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: gc9503cv_dsi_register
 *
 * Description:
 *   Register a GC9503CV MIPI DSI panel as a peripheral on the supplied DSI
 *   host.  The panel is configured for 2 data lanes, RGB888 pixel format
 *   and DSI video mode with low-power command transmission.
 *
 *   Only a single panel instance is supported.  The configuration object
 *   pointed to by cfg is retained by the driver and must remain valid for
 *   the lifetime of the panel.
 *
 * Input Parameters:
 *   host - DSI host returned by mipi_dsi_host_get().  Must not be NULL.
 *   cfg  - Board glue providing the reset GPIO callback.  Must not be NULL
 *          and cfg->reset must not be NULL.
 *
 * Returned Value:
 *   OK on success.  A negated errno value on failure:
 *     -EINVAL : invalid arguments
 *     -EBUSY  : a panel instance is already registered
 *     -ENOMEM : DSI device allocation failed
 *     other   : propagated from mipi_dsi_attach()
 *
 ****************************************************************************/

int gc9503cv_dsi_register(FAR struct mipi_dsi_host *host,
                          FAR const struct gc9503cv_config_s *cfg);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_GC9503CV_DSI_H */
