/****************************************************************************
 * arch/arm/src/t113/t113_de.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_DE_H
#define __ARCH_ARM_SRC_T113_T113_DE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <nuttx/video/fb.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Display Engine bring-up configuration.  Currently only a single primary
 * UI plane is supported; the framebuffer is allocated by the driver.
 */

struct t113_de_config_s
{
  uint16_t width;       /* Active pixel width  (e.g. 480) */
  uint16_t height;      /* Active pixel height (e.g. 854) */
  uint8_t  bpp;         /* Bits per pixel: 24 = RGB888, 32 = ARGB8888 */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: t113_de_initialize
 *
 * Description:
 *   Initialise the T113 Display Engine 2.0 mixer for a single primary UI
 *   plane sized to the supplied configuration, allocate the backing
 *   framebuffer and arm the mixer pipeline.  After this call returns
 *   successfully the framebuffer is being scanned out continuously.
 *
 *   Must be called after the DE clocks/resets are de-asserted (CCU) and
 *   after the downstream TCON has been configured for the same active
 *   resolution.
 *
 * Input Parameters:
 *   cfg - Display engine configuration; the structure may be discarded
 *         after the call returns.
 *
 * Returned Value:
 *   Zero on success, a negated errno value on failure.
 *
 ****************************************************************************/

int t113_de_initialize(const struct t113_de_config_s *cfg);

/****************************************************************************
 * Name: t113_de_get_fb_vtable
 *
 * Description:
 *   Return the NuttX framebuffer vtable bound to the DE primary plane.  The
 *   vtable is owned by the driver; callers must not free it.
 *
 * Returned Value:
 *   Pointer to a populated fb_vtable_s on success, NULL before
 *   t113_de_initialize() succeeds.
 *
 ****************************************************************************/

struct fb_vtable_s *t113_de_get_fb_vtable(void);

/****************************************************************************
 * Name: t113_de_get_fb_buffer
 *
 * Description:
 *   Return the base address of the primary-plane framebuffer.  Because the
 *   T113 boot path runs without an MMU, the physical and virtual addresses
 *   are identical and may be handed straight to user-space.
 *
 * Returned Value:
 *   Pointer to the framebuffer on success, NULL before
 *   t113_de_initialize() succeeds.
 *
 ****************************************************************************/

uint8_t *t113_de_get_fb_buffer(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_T113_T113_DE_H */
