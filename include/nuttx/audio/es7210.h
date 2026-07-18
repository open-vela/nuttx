/****************************************************************************
 * include/nuttx/audio/es7210.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_ES7210_H
#define __INCLUDE_NUTTX_AUDIO_ES7210_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/irq.h>

#ifdef CONFIG_AUDIO_ES7210

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct es7210_lower_s
{
  /* I2C characterization */

  uint32_t frequency;  /* Initial I2C frequency */
  uint8_t  address;    /* 7-bit I2C address (only bits 0-6 used) */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: es7210_initialize
 *
 * Description:
 *   Initialize the ES7210 ADC codec chip.
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the ES7210
 *   i2s     - An instance of the I2S interface to use for audio data
 *   lower   - Platform-specific lower half configuration
 *
 * Returned Value:
 *   A new lower half audio interface for the ES7210 on success; NULL on
 *   failure.
 *
 ****************************************************************************/

struct i2c_master_s;
struct i2s_dev_s;
struct audio_lowerhalf_s;

FAR struct audio_lowerhalf_s *
  es7210_initialize(FAR struct i2c_master_s *i2c,
                    FAR struct i2s_dev_s *i2s,
                    FAR const struct es7210_lower_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_AUDIO_ES7210 */
#endif /* __INCLUDE_NUTTX_AUDIO_ES7210_H */
