/****************************************************************************
 * include/nuttx/input/tlsc6x.h
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

#ifndef __INCLUDE_NUTTX_INPUT_TLSC6X_H
#define __INCLUDE_NUTTX_INPUT_TLSC6X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_INPUT_TLSC6X

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Per-board configuration of a TLSC6X touch panel.
 *
 * Transform fields combine with the matching CONFIG_INPUT_TLSC6X_DEFAULT_*
 * Kconfig values:
 *   - bool fields: when the Kconfig default is "y" the transform is
 *     always applied; when the Kconfig default is "n" the board's
 *     bool is honoured.  Bool fields cannot DOWNGRADE a Kconfig
 *     default-on (the driver can't tell "user wrote false" from "field
 *     left zero").  Disabling a default-on transform must be done by
 *     turning the matching Kconfig knob off.
 *   - uint16_t fields: 0 means "use the Kconfig default", any non-zero
 *     value overrides.
 *
 * Boards that route the panel to non-default geometry (rotated panel,
 * mirrored signals, different resolution) just fill the override fields
 * and the driver picks them up at register time.
 */

struct tlsc6x_config_s
{
  uint8_t   address;          /* I2C 7-bit address (0x2e for TLSC6X) */
  uint32_t  frequency;        /* I2C bus frequency, Hz */

  /* Coordinate transform overrides (0/false = use Kconfig default) */

  bool      exchange_xy;      /* Swap X and Y after read */
  bool      revert_x;         /* Mirror X (x = max_x - x) */
  bool      revert_y;         /* Mirror Y (y = max_y - y) */
  uint16_t  max_x;            /* Logical screen width  (0 = Kconfig) */
  uint16_t  max_y;            /* Logical screen height (0 = Kconfig) */

  /* Board callbacks */

  /* attach: Hook the IC's INT line to the given ISR. */

  CODE int  (*attach)(FAR const struct tlsc6x_config_s *cfg,
                      xcpt_t isr, FAR void *arg);

  /* enable: Enable/disable the INT line interrupt. */

  CODE void (*enable)(FAR const struct tlsc6x_config_s *cfg, bool enable);

  /* nreset: Drive the IC's RESET pin (active low).  state=true means
   *         pin is high (out of reset); state=false means pin low
   *         (held in reset).
   */

  CODE void (*nreset)(FAR const struct tlsc6x_config_s *cfg, bool state);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: tlsc6x_register
 *
 * Description:
 *   Register a TLSC6X capacitive touch controller as a touchscreen input
 *   device at the supplied path (typically "/dev/input0").
 *
 * Arguments:
 *   devpath - Device node path, e.g. "/dev/input0"
 *   i2c     - The I2C master that the TLSC6X is wired to
 *   config  - Per-board configuration (address, transform, callbacks)
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure.
 *
 ****************************************************************************/

int tlsc6x_register(FAR const char *devpath,
                    FAR struct i2c_master_s *i2c,
                    FAR const struct tlsc6x_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT_TLSC6X */
#endif /* __INCLUDE_NUTTX_INPUT_TLSC6X_H */
