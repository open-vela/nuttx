/****************************************************************************
 * arch/arm/src/t113/t113_tcon.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_TCON_H
#define __ARCH_ARM_SRC_T113_T113_TCON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Output interface type - value passed in t113_tcon_timing_s.if_type */

#define T113_TCON_IF_HV    0    /* Parallel RGB / HV-sync */
#define T113_TCON_IF_CPU   1    /* MCU-bus (8080) */
#define T113_TCON_IF_LVDS  2    /* LVDS */
#define T113_TCON_IF_DSI   3    /* MIPI DSI */

/* Pixel format - value passed in t113_tcon_timing_s.format */

#define T113_TCON_FMT_RGB888  0

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct t113_tcon_timing_s
{
  uint32_t pixel_clk_hz;
  uint16_t hactive;
  uint16_t hbp;
  uint16_t hfp;
  uint16_t hsync;
  uint16_t vactive;
  uint16_t vbp;
  uint16_t vfp;
  uint16_t vsync;
  uint8_t  if_type;       /* T113_TCON_IF_DSI, ... */
  uint8_t  lanes;         /* DSI lane count, ignored for non-DSI */
  uint8_t  format;        /* T113_TCON_FMT_RGB888, ... */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: t113_tcon_initialize
 *
 * Description:
 *   Configure TCON-LCD0 for the given output timing and route its pixel
 *   stream to the appropriate downstream interface (DSI / LVDS / parallel
 *   RGB).  After this call returns successfully the TCON is enabled and
 *   driving sync to the panel.
 *
 * Input Parameters:
 *   timing - Display timing descriptor; must remain valid for the duration
 *            of the call only.
 *
 * Returned Value:
 *   Zero on success, a negated errno value on failure.
 *
 ****************************************************************************/

int t113_tcon_initialize(const struct t113_tcon_timing_s *timing);

#endif /* __ARCH_ARM_SRC_T113_T113_TCON_H */
