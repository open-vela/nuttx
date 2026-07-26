/****************************************************************************
 * boards/xtensa/esp32s3/common/src/esp32s3_es7210.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/es7210.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/i2c/i2c_master.h>

#include "esp32s3_i2c.h"
#include "esp32s3_i2s.h"

#if defined(CONFIG_ESP32S3_I2S) && defined(CONFIG_AUDIO_ES7210)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct es7210_lower_s g_es7210_lower;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_es7210_initialize
 *
 * Description:
 *   Configure and register the ES7210 4-channel ADC as the audio capture
 *   device /dev/audio/pcm_in0.
 *
 * Input Parameters:
 *   i2c_port - The I2C port used by the device
 *   i2c_addr - The I2C address used by the device
 *   i2c_freq - The I2C frequency used by the device
 *   i2s_port - The I2S port used by the device
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int esp32s3_es7210_initialize(int i2c_port, uint8_t i2c_addr, int i2c_freq,
                              int i2s_port)
{
  struct audio_lowerhalf_s *es7210;
  struct i2s_dev_s *i2s;
  struct i2c_master_s *i2c;
  int ret;

  audinfo("i2c_port %d, i2c_addr %d, i2c_freq %d, i2s_port %d\n",
          i2c_port, i2c_addr, i2c_freq, i2s_port);

  i2s = esp32s3_i2sbus_initialize(i2s_port);
  if (i2s == NULL)
    {
      auderr("ERROR: Failed to initialize I2S%d\n", i2s_port);
      return -ENODEV;
    }

#ifdef CONFIG_AUDIO_I2SCHAR
  ret = i2schar_register(i2s, i2s_port);
  if (ret < 0 && ret != -EEXIST)
    {
      auderr("ERROR: i2schar_register failed: %d\n", ret);
      return ret;
    }
#endif

  i2c = esp32s3_i2cbus_initialize(i2c_port);
  if (i2c == NULL)
    {
      auderr("ERROR: Failed to initialize I2C%d\n", i2c_port);
      return -ENODEV;
    }

  g_es7210_lower.address = i2c_addr;
  g_es7210_lower.frequency = i2c_freq;

  es7210 = es7210_initialize(i2c, i2s, &g_es7210_lower);
  if (es7210 == NULL)
    {
      auderr("ERROR: Failed to initialize the ES7210\n");
      return -ENODEV;
    }

  ret = audio_register("pcm_in0", es7210);
  if (ret < 0)
    {
      auderr("ERROR: Failed to register /dev/audio/pcm_in0: %d\n", ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_ESP32S3_I2S && CONFIG_AUDIO_ES7210 */
