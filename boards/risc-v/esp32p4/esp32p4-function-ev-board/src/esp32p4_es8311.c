/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_es8311.c
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

/* ES8311 audio codec + I2S0 wiring for the ESP32-P4-Function-EV-Board.
 *
 * Hardware (per Espressif BSP esp32_p4_function_ev_board):
 *   I2S0:  BCLK=GPIO12  MCLK=GPIO13  WS=GPIO10  DOUT=GPIO9  DIN=GPIO11
 *   I2C0:  SCL=GPIO8    SDA=GPIO7   (shared bus, ES8311 @ 0x18)
 *   PA:    GPIO53       (power amplifier enable, active high)
 *
 * Registers /dev/audio/pcm0 (playback, WAV/PCM) and /dev/audio/pcm_in0
 * (recording) once CONFIG_AUDIO + CONFIG_AUDIO_ES8311 are enabled.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/pcm.h>
#include <nuttx/audio/es8311.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "espressif/esp_i2c.h"
#include "espressif/esp_i2s.h"
#include "espressif/esp_gpio.h"

#include "esp32p4-function-ev-board.h"

#if defined(CONFIG_ESPRESSIF_I2S0) && defined(CONFIG_AUDIO_ES8311)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Two lower-half descriptors: [0] = playback (DAC), [1] = record (ADC) */

static struct es8311_lower_s g_es8311_lower[2];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: funev_pa_enable
 *
 * Description:
 *   Drive the power amplifier enable pin (GPIO53, active high).
 *
 ****************************************************************************/

static int funev_pa_enable(bool on)
{
  int ret;

  ret = esp_configgpio(FUNEV_GPIO_PA_EN, OUTPUT);
  if (ret < 0)
    {
      auderr("ERROR: PA GPIO config failed: %d\n", ret);
      return ret;
    }

  esp_gpiowrite(FUNEV_GPIO_PA_EN, on);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32p4_es8311_initialize
 *
 * Description:
 *   Initialize and register the ES8311 codec + I2S0 audio driver.
 *
 * Input Parameters:
 *   i2c_port  - The I2C port used for the codec control (0)
 *   i2c_addr  - The I2C address of the ES8311 (0x18)
 *   i2c_freq  - The I2C frequency (400000)
 *   i2s_port  - The I2S port used for audio data (0)
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int esp32p4_es8311_initialize(int i2c_port, uint8_t i2c_addr, int i2c_freq,
                              int i2s_port)
{
  struct audio_lowerhalf_s *es8311;
#ifndef CONFIG_SYSTEM_NXLOOPER
  struct audio_lowerhalf_s *pcm;
#endif
  struct i2s_dev_s *i2s;
  struct i2c_master_s *i2c;
  static bool initialized = false;
  int ret;

  audinfo("i2c_port %d, i2c_addr %02x, i2c_freq %d, i2s_port %d\n",
          i2c_port, i2c_addr, i2c_freq, i2s_port);

  /* Prevent multiple initializations */

  if (!initialized)
    {
      /* Enable the power amplifier before bringing up the codec */

      ret = funev_pa_enable(true);
      if (ret < 0)
        {
          auderr("ERROR: PA enable failed: %d\n", ret);
          goto errout;
        }

      /* Get an instance of the I2S interface for the ES8311 data channel */

      i2s = esp_i2sbus_initialize(i2s_port);
      if (i2s == NULL)
        {
          auderr("ERROR: Failed to initialize I2S%d\n", i2s_port);
          ret = -ENODEV;
          goto errout;
        }

      /* Get an instance of the I2C interface for the ES8311 control */

      i2c = esp_i2cbus_initialize(i2c_port);
      if (i2c == NULL)
        {
          auderr("ERROR: Failed to initialize I2C%d\n", i2c_port);
          ret = -ENODEV;
          goto errout;
        }

      /* Optionally register the I2S character driver for raw I2S testing */

#ifdef CONFIG_AUDIO_I2SCHAR
      ret = i2schar_register(i2s, 0);
      if (ret < 0)
        {
          auderr("ERROR: i2schar_register failed: %d\n", ret);
          goto errout;
        }
#endif

      /* Initialize the ES8311 output (DAC / playback) path */

      g_es8311_lower[0].address   = i2c_addr;
      g_es8311_lower[0].frequency = i2c_freq;

      es8311 = es8311_initialize(i2c, i2s, &g_es8311_lower[0]);
      if (es8311 == NULL)
        {
          auderr("ERROR: Failed to initialize the ES8311 (playback)\n");
          ret = -ENODEV;
          goto errout;
        }

#ifdef CONFIG_SYSTEM_NXLOOPER
      /* nxlooper bypasses PCM decode, register the raw codec directly */

      ret = audio_register("pcm0", es8311);
#else
      /* Embed the ES8311/I2S conglomerate into a PCM decoder so we have
       * a WAV/PCM front end for playback.
       */

      pcm = pcm_decode_initialize(es8311);
      if (pcm == NULL)
        {
          auderr("ERROR: Failed to create the PCM decoder\n");
          ret = -ENODEV;
          goto errout;
        }

      ret = audio_register("pcm0", pcm);
#endif
      if (ret < 0)
        {
          auderr("ERROR: Failed to register /dev/audio/pcm0: %d\n", ret);
          goto errout;
        }

      /* Initialize the ES8311 input (ADC / recording) path */

      g_es8311_lower[1].address   = i2c_addr;
      g_es8311_lower[1].frequency = i2c_freq;

      es8311 = es8311_initialize(i2c, i2s, &g_es8311_lower[1]);
      if (es8311 == NULL)
        {
          auderr("ERROR: Failed to initialize the ES8311 (record)\n");
          ret = -ENODEV;
          goto errout;
        }

      ret = audio_register("pcm_in0", es8311);
      if (ret < 0)
        {
          auderr("ERROR: Failed to register /dev/audio/pcm_in0: %d\n", ret);
          goto errout;
        }

      initialized = true;
    }

  return OK;

errout:
  return ret;
}

#endif /* CONFIG_ESPRESSIF_I2S0 && CONFIG_AUDIO_ES8311 */
