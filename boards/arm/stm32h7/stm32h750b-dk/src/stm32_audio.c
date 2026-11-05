/****************************************************************************
 * boards/arm/stm32h7/stm32h750b-dk/src/stm32_audio.c
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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/pcm.h>
#include <nuttx/audio/wm8994.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"

#include "stm32h750b-dk.h"
#include "stm32_i2c.h"
#include "stm32_sai.h"

#ifdef CONFIG_AUDIO_WM8994

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The codec hangs off I2C4 together with the touch controller.  Both are
 * plain 7-bit addresses on the same bus: the WM8994 answers at 0x1a (the
 * schematic records it as the 8-bit form 0x34, which is the same address
 * shifted left) and the FT5x06 at 0x38.  Nothing here probes for either;
 * the bus is opened and the codec is addressed directly, so a wrong address
 * fails as a read of the ID register rather than as a write to an unknown
 * device.
 */

#define WM8994_I2C_ADDRESS  (0x34 >> 1)
#define WM8994_I2C_BUS      4
#define WM8994_SAI_BUS      SAI2_BLOCK_A

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_audioinfo_s
{
  struct wm8994_lower_s lower;
  bool enabled;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int wm8994_attach(const struct wm8994_lower_s *lower,
                         wm8994_handler_t isr, void *arg)
{
  /* The driver only uses this hook for its FLL-lock interrupt, which is
   * disabled (WM8994_USE_FFLOCK_INT is not defined).  The board Audio_Int
   * net is PG7; this playback path does not configure that optional
   * interrupt.
   */

  audinfo("WM8994 interrupt not used on this board\n");
  return 0;
}

static bool wm8994_enable(const struct wm8994_lower_s *lower, bool enable)
{
  struct stm32_audioinfo_s *info =
    (struct stm32_audioinfo_s *)
      ((uintptr_t)lower - offsetof(struct stm32_audioinfo_s, lower));

  audinfo("enable=%d\n", enable);
  info->enabled = enable;
  return true;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct i2s_dev_s *g_audio_i2s;

static int wm8994_mclk_enable(const struct wm8994_lower_s *lower,
                               bool enable)
{
  return stm32_sai_mclk_enable(g_audio_i2s, enable);
}

static struct stm32_audioinfo_s g_wm8994 =
{
  .lower =
    {
      .address   = WM8994_I2C_ADDRESS,
      .frequency = I2C_SPEED_FAST,
      .mclk      = 4096000,
      .attach    = wm8994_attach,
      .enable    = wm8994_enable,
      .mclk_enable = wm8994_mclk_enable,
    }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_audio_initialize
 *
 * Description:
 *   Register the WM8994 as /dev/audio/pcm0p.
 *
 *   The playback chain is PCM decoder -> WM8994 -> SAI2 block A -> HPOUT1,
 *   which connects directly to the green CN9 jack.  C113/C114 are shunt
 *   branches through R182/R183, not series coupling capacitors.  The
 *   separate speaker headers are unused; the codec speaker drivers stay off.
 *
 *   SAI2 runs as master transmitter and must therefore also drive MCLK: the
 *   codec takes AIF1CLK from its MCLK1 pin, which the schematic routes to
 *   SAI2_MCLKA (PI4).  That is handled inside the SAI driver.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_audio_initialize(void)
{
  FAR struct audio_lowerhalf_s *wm8994;
  FAR struct audio_lowerhalf_s *pcm;
  FAR struct i2c_master_s *i2c;
  FAR struct i2s_dev_s *i2s;
  static bool initialized = false;
  int ret;

  if (initialized)
    {
      return OK;
    }

  /* The I2C bus is shared with the touch controller, which bring-up has
   * already initialized.  stm32_i2cbus_initialize() is reference counted,
   * so taking another reference here neither resets the bus nor disturbs the
   * device that is already using it.
   */

  i2c = stm32_i2cbus_initialize(WM8994_I2C_BUS);
  if (i2c == NULL)
    {
      auderr("ERROR: stm32_i2cbus_initialize(%d) failed\n", WM8994_I2C_BUS);
      return -ENODEV;
    }

  i2s = stm32_sai_initialize(WM8994_SAI_BUS, NULL);
  if (i2s == NULL)
    {
      auderr("ERROR: stm32_sai_initialize(%d) failed\n", WM8994_SAI_BUS);
      return -ENODEV;
    }

  g_audio_i2s = i2s;
  wm8994 = wm8994_initialize(i2c, i2s, &g_wm8994.lower);
  if (wm8994 == NULL)
    {
      auderr("ERROR: wm8994_initialize failed\n");
      return -ENODEV;
    }

  /* Wrap the codec in the PCM decoder so that WAV headers are handled above
   * the codec.  Without it the player would have to hand over raw samples.
   */

  pcm = pcm_decode_initialize(wm8994);
  if (pcm == NULL)
    {
      auderr("ERROR: pcm_decode_initialize failed\n");
      return -ENODEV;
    }

  ret = audio_register("pcm0p", pcm);
  if (ret < 0)
    {
      auderr("ERROR: audio_register failed: %d\n", ret);
      return ret;
    }

  g_audio_i2s = i2s;
  initialized = true;
  return OK;
}

/****************************************************************************
 * Name: stm32_audio_codec_readreg
 *
 * Description:
 *   Read one WM8994 register.  See board.h for the contract.
 *
 ****************************************************************************/

int stm32_audio_codec_readreg(uint16_t offset, FAR uint16_t *val)
{
  FAR struct i2c_master_s *i2c;
  struct i2c_msg_s msg[2];
  uint8_t buf[2];
  uint16_t subaddr;
  int ret;

  if (val == NULL)
    {
      return -EINVAL;
    }

  i2c = stm32_i2cbus_initialize(WM8994_I2C_BUS);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* The WM8994 takes a 16-bit register subaddress, not an 8-bit one.  The
   * driver in drivers/audio/wm8994.c builds it by byte-swapping the offset,
   * because the codec reads the subaddress most significant byte first while
   * this CPU is little endian.  Sending a single byte instead puts the codec
   * out of step: it waits for the second half of the address, the read then
   * returns whatever the shift register holds, and the transaction still
   * reports success.  That is why an 8-bit subaddress reads back as 0x0000
   * for every register rather than as an error.
   */

  subaddr = (uint16_t)(((offset >> 8) & 0xff) | ((offset << 8) & 0xff00));

  msg[0].frequency = I2C_SPEED_FAST;
  msg[0].addr      = WM8994_I2C_ADDRESS;
  msg[0].flags     = 0;
  msg[0].buffer    = (FAR uint8_t *)&subaddr;
  msg[0].length    = 2;

  msg[1].frequency = I2C_SPEED_FAST;
  msg[1].addr      = WM8994_I2C_ADDRESS;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buf;
  msg[1].length    = 2;

  ret = I2C_TRANSFER(i2c, msg, 2);
  stm32_i2cbus_uninitialize(i2c);
  if (ret < 0)
    {
      return ret;
    }

  *val = ((uint16_t)buf[0] << 8) | buf[1];
  return OK;
}

/****************************************************************************
 * Name: stm32_audio_sai_state
 *
 * Description:
 *   Report the SAI2 block A registers that decide whether playback works.
 *
 ****************************************************************************/

static void audio_rejected(FAR struct i2s_dev_s *dev,
                           FAR struct ap_buffer_s *apb, FAR void *arg,
                           int result)
{
  /* Invalid requests must return synchronously, never complete later. */

  *(volatile bool *)arg = true;
}

int stm32_audio_tx_check(FAR int *results, int count)
{
  struct ap_buffer_s apb;
  uint16_t samples[32];
  volatile bool callback = false;
  int i;

  if (g_audio_i2s == NULL || results == NULL || count < 5)
    {
      return -EINVAL;
    }

  memset(&apb, 0, sizeof(apb));
  apb.samp = (FAR uint8_t *)samples;
  apb.nmaxbytes = sizeof(samples);
  results[0] = stm32_sai_initialize(-1, NULL) == NULL ? 0 : -EIO;
  for (i = 0; i < 20; i++)
    {
      apb.curbyte = 0;
      apb.nbytes = 0;
      results[1] = I2S_SEND(g_audio_i2s, &apb, audio_rejected,
                            (FAR void *)&callback, 1);
      apb.nbytes = 3;
      results[2] = I2S_SEND(g_audio_i2s, &apb, audio_rejected,
                            (FAR void *)&callback, 1);
      apb.nbytes = sizeof(samples) + 2;
      results[3] = I2S_SEND(g_audio_i2s, &apb, audio_rejected,
                            (FAR void *)&callback, 1);
      apb.nbytes = 2;
      apb.curbyte = 4;
      results[4] = I2S_SEND(g_audio_i2s, &apb, audio_rejected,
                            (FAR void *)&callback, 1);
      if (callback || results[0] != 0 || results[1] != -EINVAL ||
          results[2] != -EINVAL || results[3] != -EINVAL ||
          results[4] != -EINVAL)
        {
          return -EIO;
        }
    }

  return OK;
}

int stm32_audio_tx_record(FAR uint32_t *regs, int nregs)
{
  if (g_audio_i2s == NULL)
    {
      return -ENODEV;
    }

  return stm32_sai_staterecord(g_audio_i2s, regs, nregs);
}

int stm32_audio_sai_state(FAR uint32_t *cr1, FAR uint32_t *gcr)
{
  if (cr1 == NULL || gcr == NULL)
    {
      return -EINVAL;
    }

  *cr1 = getreg32(STM32H7_SAI2_A_BASE + STM32H7_SAI_CR1_OFFSET);
  *gcr = getreg32(STM32H7_SAI2_GCR);
  return OK;
}

#endif /* CONFIG_AUDIO_WM8994 */
