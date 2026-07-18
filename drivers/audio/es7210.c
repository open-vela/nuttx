/****************************************************************************
 * drivers/audio/es7210.c
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
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/spinlock.h>

#include "es7210.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MCLK/LRCK ratio.  The ES7210 datasheet lists the supported ratios as
 * 256, 384, 512, 768, 1024, and 1536.  This driver uses the common 256 x FS
 * setting; for example, 48 kHz uses a 12.288 MHz master clock.
 */

#define ES7210_MCLK_MULTIPLE    256

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  es7210_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                           FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es7210_configure(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session,
                             FAR const struct audio_caps_s *caps);
#else
static int  es7210_configure(FAR struct audio_lowerhalf_s *dev,
                             FAR const struct audio_caps_s *caps);
#endif
static int  es7210_shutdown(FAR struct audio_lowerhalf_s *dev);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es7210_start(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session);
#else
static int  es7210_start(FAR struct audio_lowerhalf_s *dev);
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es7210_stop(FAR struct audio_lowerhalf_s *dev,
                        FAR void *session);
#  else
static int  es7210_stop(FAR struct audio_lowerhalf_s *dev);
#  endif
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es7210_pause(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session);
static int  es7210_resume(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session);
#  else
static int  es7210_pause(FAR struct audio_lowerhalf_s *dev);
static int  es7210_resume(FAR struct audio_lowerhalf_s *dev);
#  endif
#endif

static int  es7210_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb);
static int  es7210_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb);
static int  es7210_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                         unsigned long arg);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es7210_reserve(FAR struct audio_lowerhalf_s *dev,
                           FAR void **session);
static int  es7210_release(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session);
#else
static int  es7210_reserve(FAR struct audio_lowerhalf_s *dev);
static int  es7210_release(FAR struct audio_lowerhalf_s *dev);
#endif

static void  es7210_processbegin(FAR struct es7210_dev_s *priv);
static void  es7210_processdone(FAR struct i2s_dev_s *i2s,
                                FAR struct ap_buffer_s *apb,
                                FAR void *arg, int result);
static FAR struct ap_buffer_s *
  es7210_dequeuebuffer(FAR struct es7210_dev_s *priv);
static void  es7210_returnbuffers(FAR struct es7210_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audioops =
{
  es7210_getcaps,       /* getcaps        */
  es7210_configure,     /* configure      */
  es7210_shutdown,      /* shutdown       */
  es7210_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  es7210_stop,          /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  es7210_pause,         /* pause          */
  es7210_resume,        /* resume         */
#endif
  NULL,                 /* allocbuffer    */
  NULL,                 /* freebuffer     */
  es7210_enqueuebuffer, /* enqueue_buffer */
  es7210_cancelbuffer,  /* cancel_buffer  */
  es7210_ioctl,         /* ioctl          */
  NULL,                 /* read           */
  NULL,                 /* write          */
  es7210_reserve,       /* reserve        */
  es7210_release        /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: es7210_writereg
 ****************************************************************************/

static int es7210_writereg(FAR struct es7210_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_msg_s msg;
  uint8_t txbuffer[2];
  int ret;

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg.frequency = priv->lower.frequency;
  msg.addr      = priv->lower.address;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = 2;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      auderr("ERROR: I2C_TRANSFER reg=0x%02x failed: %d\n", regaddr, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: es7210_setmicgain
 ****************************************************************************/

static int es7210_setmicgain(FAR struct es7210_dev_s *priv, uint8_t gain)
{
  /* Enable all four analog front ends.  The ES7210 is configured for four
   * TDM slots below, so every slot must have a corresponding powered ADC.
   */

  es7210_writereg(priv, ES7210_ADC1_GAIN_REG43,
                  gain | ES7210_ADC_PGA_POWER_ON);
  es7210_writereg(priv, ES7210_ADC2_GAIN_REG44,
                  gain | ES7210_ADC_PGA_POWER_ON);
  es7210_writereg(priv, ES7210_ADC3_GAIN_REG45,
                  gain | ES7210_ADC_PGA_POWER_ON);
  es7210_writereg(priv, ES7210_ADC4_GAIN_REG46,
                  gain | ES7210_ADC_PGA_POWER_ON);
  return OK;
}

/****************************************************************************
 * Name: es7210_reset
 ****************************************************************************/

static int es7210_reset(FAR struct es7210_dev_s *priv)
{
  uint16_t lrck_div = ES7210_MCLK_MULTIPLE;

  audinfo("ES7210 reset\n");

  /* Software reset */

  es7210_writereg(priv, ES7210_RESET_REG00, ES7210_RESET_CMD);
  up_mdelay(10);  /* 10ms for ES7210 to complete reset */
  es7210_writereg(priv, ES7210_RESET_REG00, ES7210_RESET_NORMAL);
  es7210_writereg(priv, ES7210_CLK_ON_REG01, 0x3f);

  /* Set the initialization time when device powers up */

  es7210_writereg(priv, ES7210_TCT0_CHPINI_REG09, 0x30);
  es7210_writereg(priv, ES7210_TCT1_CHPINI_REG0A, 0x30);

  /* Configure HPF for ADC1-4 */

  es7210_writereg(priv, ES7210_ADC4_HPF_REG23, 0x2a);
  es7210_writereg(priv, ES7210_ADC3_HPF_REG22, 0x0a);
  es7210_writereg(priv, ES7210_ADC2_HPF_REG21, 0x2a);
  es7210_writereg(priv, ES7210_ADC1_HPF_REG20, 0x0a);

  /* Set SDP: I2S format, 16-bit, four-channel TDM.
   * REG11=0x60: I2S format, 16-bit word length
   * REG12=0x02: TDM mode for ADC1-4.  The ESP32-S3 I2S lower half
   * configures four active TDM slots when four channels are selected.
   */

  es7210_writereg(priv, ES7210_SDP_CFG1_REG11, ES7210_SDP_I2S_16BIT);
  es7210_writereg(priv, ES7210_SDP_CFG2_REG12, ES7210_SDP_TDM);

  /* Configure analog power: VMID voltage selection */

  es7210_writereg(priv, ES7210_ANALOG_SYS_REG40, ES7210_VMID_SELECT);

  /* Set MIC bias to 2.87V */

  es7210_writereg(priv, ES7210_MICBIAS12_REG41, ES7210_MICBIAS_2V87);
  es7210_writereg(priv, ES7210_MICBIAS34_REG42, ES7210_MICBIAS_2V87);

  /* Set MIC gain (30dB) */

  es7210_setmicgain(priv, ES7210_MIC_GAIN_30DB);

  /* Power on the four individual microphone channels.  Registers 0x47
   * through 0x4a are per-channel power controls, not mute registers.
   */

  es7210_writereg(priv, ES7210_MIC1_POWER_REG47, ES7210_MIC_POWER_ON);
  es7210_writereg(priv, ES7210_MIC2_POWER_REG48, ES7210_MIC_POWER_ON);
  es7210_writereg(priv, ES7210_MIC3_POWER_REG49, ES7210_MIC_POWER_ON);
  es7210_writereg(priv, ES7210_MIC4_POWER_REG4A, ES7210_MIC_POWER_ON);

  /* Set sample rate: LRCK divider = MCLK / FS = MCLK_MULTIPLE.
   * With MCLK = ES7210_MCLK_MULTIPLE * FS, the divider is constant
   * regardless of the actual sample rate, so compute it from the
   * defined multiple rather than hard-coding 48 kHz values.
   */

  es7210_writereg(priv, ES7210_ADC_OSR_REG07, 0x20);
  es7210_writereg(priv, ES7210_MCLK_CTL_REG02, ES7210_MCLK_ADC_DIV1_DLL);
  es7210_writereg(priv, ES7210_MST_LRCDIVH_REG04,
                 (uint8_t)((lrck_div >> 8) & 0xff));
  es7210_writereg(priv, ES7210_MST_LRCDIVL_REG05,
                 (uint8_t)(lrck_div & 0xff));

  /* Power on the DLL and ADC digital blocks */

  es7210_writereg(priv, ES7210_DIGITAL_PDN_REG06,
                  ES7210_DIGITAL_POWER_ON);

  /* Power on the MIC1/2 and MIC3/4 analog groups.  This matches the
   * four-microphone selection used by the Korvo-2 reference firmware.
   */

  es7210_writereg(priv, ES7210_MIC12_POWER_REG4B, ES7210_MIC_ADC_PGA_ON);
  es7210_writereg(priv, ES7210_MIC34_POWER_REG4C, ES7210_MIC_ADC_PGA_ON);

  /* Enable clocks and device */

  es7210_writereg(priv, ES7210_CLK_ON_REG01, ES7210_CLOCK_ENABLE_ALL);
  es7210_writereg(priv, ES7210_RESET_REG00, ES7210_DEVICE_ON);

  audinfo("ES7210 reset done\n");
  return OK;
}

/****************************************************************************
 * Name: es7210_getcaps
 ****************************************************************************/

static int es7210_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                          FAR struct audio_caps_s *caps)
{
  /* Validate the structure */

  DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));
  audinfo("type=%d ac_type=%d\n", type, caps->ac_type);

  /* Fill in the caller's structure based on requested info */

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_QUERY:
        caps->ac_channels = 4;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:
              caps->ac_controls.b[0] = AUDIO_TYPE_INPUT;
              caps->ac_format.hw     = (1 << (AUDIO_FMT_PCM - 1));
              break;

            default:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;
          }
        break;

      case AUDIO_TYPE_INPUT:
        caps->ac_channels = 4;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* Report supported sample rates */

              caps->ac_controls.hw[0] = AUDIO_SAMP_RATE_8K  |
                                        AUDIO_SAMP_RATE_16K |
                                        AUDIO_SAMP_RATE_32K |
                                        AUDIO_SAMP_RATE_48K;
              break;

            default:
              break;
          }
        break;

      case AUDIO_TYPE_PROCESSING:
        break;

      default:
        break;
    }

  return caps->ac_len;
}

/****************************************************************************
 * Name: es7210_configure
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es7210_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session,
                            FAR const struct audio_caps_s *caps)
#else
static int es7210_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct es7210_dev_s *priv = (FAR struct es7210_dev_s *)dev;
  int ret = OK;

  DEBUGASSERT(priv != NULL && caps != NULL);
  audinfo("ac_type: %d\n", caps->ac_type);

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_INPUT:
        {
          audinfo("  AUDIO_TYPE_INPUT:\n");
          audinfo("    Number of channels: %u\n", caps->ac_channels);
          audinfo("    Sample rate:        %u\n", caps->ac_controls.hw[0]);
          audinfo("    Sample width:       %u\n", caps->ac_controls.b[2]);

          /* The codec is configured for four 16-bit TDM slots.  Accepting
           * another channel count or sample width would make the ESP32-S3
           * bit clock disagree with the ES7210 stream format.
           */

          if (caps->ac_channels != ES7210_DEFAULT_NCHANNELS ||
              caps->ac_controls.b[2] != ES7210_DEFAULT_BPSAMP)
            {
              return -EINVAL;
            }

          switch (caps->ac_controls.hw[0])
            {
              case 8000:
              case 16000:
              case 32000:
              case 48000:
                break;

              default:
                return -EINVAL;
            }

          /* Save the stream configuration.  The actual I2S setup is
           * deferred to es7210_start() so RX is not started without DMA
           * buffers.
           */

          priv->nchannels = caps->ac_channels;
          priv->samprate  = caps->ac_controls.hw[0];
          priv->bpsamp    = caps->ac_controls.b[2];
        }
        break;

      case AUDIO_TYPE_PROCESSING:
        break;

      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: es7210_shutdown
 ****************************************************************************/

static int es7210_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("ES7210 shutdown\n");

  /* Do nothing here. The nxrecorder "device" command triggers
   * open, shutdown, and close just to probe capabilities.  Any I2C
   * writes here could leave the chip in a state that causes
   * problems during the subsequent es7210_reset() in start().
   * The full reset in es7210_start() handles all initialization.
   */

  return OK;
}

/****************************************************************************
 * Name: es7210_dequeuebuffer
 ****************************************************************************/

static FAR struct ap_buffer_s *
  es7210_dequeuebuffer(FAR struct es7210_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;

  flags = enter_critical_section();
  apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq);
  leave_critical_section(flags);

  return apb;
}

/****************************************************************************
 * Name: es7210_processbegin
 ****************************************************************************/

static void es7210_processbegin(FAR struct es7210_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  int ret;

  /* Do not start a new transfer if we are no longer running.
   * es7210_start may call us after es7210_stop has already run
   * (race with the 50 ms codec stabilization sleep).
   */

  if (!priv->running)
    {
      return;
    }

  /* Submit ALL pending buffers to the I2S layer at once.  The I2S
   * driver queues them internally: the first buffer starts DMA,
   * subsequent buffers are placed on the pend queue and picked up
   * automatically when the previous transfer completes (in
   * i2s_rx_worker).  This avoids the costly I2S stop/start cycle
   * between every single buffer that was causing device freezes.
   */

  while (priv->running && !priv->paused &&
         (apb = es7210_dequeuebuffer(priv)) != NULL)
    {
      ret = I2S_RECEIVE(priv->i2s, apb, es7210_processdone, priv, 0);
      if (ret < 0)
        {
#ifdef CONFIG_AUDIO_MULTI_SESSION
          priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                          apb, ret, NULL);
#else
          priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                          apb, ret);
#endif
        }
    }
}

/****************************************************************************
 * Name: es7210_processdone
 ****************************************************************************/

static void es7210_processdone(FAR struct i2s_dev_s *i2s,
                               FAR struct ap_buffer_s *apb,
                               FAR void *arg, int result)
{
  FAR struct es7210_dev_s *priv = (FAR struct es7210_dev_s *)arg;

  DEBUGASSERT(priv && apb);

  /* If we are no longer running, return buffer directly to upper half.
   * We cannot just put it on pendq because es7210_stop/returnbuffers
   * may have already run.  Otherwise, the buffer would be leaked and
   * nxrecorder would hang waiting for it.
   */

  if (!priv->running)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                      apb, -ENODATA, NULL);
#else
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                      apb, -ENODATA);
#endif
      return;
    }

  /* I2S invokes this callback from HPWORK, so return the buffer directly to
   * the audio upper half.  Deferring it through LPWORK lets all four DMA
   * buffers drain before nxrecorder can requeue one, creating a capture gap
   * after every group of buffers.
   */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                  apb, result, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                  apb, result);
#endif
}

/****************************************************************************
 * Name: es7210_start
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es7210_start(FAR struct audio_lowerhalf_s *dev,
                        FAR void *session)
#else
static int es7210_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct es7210_dev_s *priv = (FAR struct es7210_dev_s *)dev;

  audinfo("ES7210 start\n");

  priv->paused  = false;
  priv->running = true;

  /* Reset the ES7210 hardware */

  es7210_reset(priv);

  /* Configure the I2S lower-half with the stream parameters.
   * Note: I2S_RXSAMPLERATE starts the RX channel, so we must
   * call processbegin immediately after to submit DMA buffers.
   * If done in configure(), the channel runs without buffers
   * causing an interrupt storm.
   */

  audinfo("ES7210 start: ch=%d bps=%d rate=%lu pendq_empty=%d\n",
          priv->nchannels, priv->bpsamp, (unsigned long)priv->samprate,
          dq_empty(&priv->pendq));

  I2S_IOCTL(priv->i2s, AUDIOIOC_START, 0);
  I2S_RXCHANNELS(priv->i2s, priv->nchannels);
  I2S_RXDATAWIDTH(priv->i2s, priv->bpsamp);
  I2S_SETMCLKFREQUENCY(priv->i2s,
                       priv->samprate * ES7210_MCLK_MULTIPLE);
  I2S_RXSAMPLERATE(priv->i2s, priv->samprate);

  /* Wait for the ES7210 codec to start outputting valid I2S data.
   * After reset + I2S channel start, the codec needs time to stabilize
   * its clock and begin transmitting. Without this delay, the first
   * DMA transfer may never complete because there is no data on DIN.
   * This must come BEFORE processbegin so DMA is not started before
   * the codec is ready.
   *
   * Note: use up_mdelay instead of nxsig_usleep because the audio
   * framework calls start() with a semaphore held, which prevents
   * signal-based sleep from completing.
   */

  up_mdelay(50);

  /* es7210_stop may have been called while we were sleeping.
   * processbegin guards on running, but check here too so we
   * avoid the I2S_RECEIVE call entirely.
   */

  if (!priv->running)
    {
      return OK;
    }

  /* Start initial receive operations */

  es7210_processbegin(priv);

  return OK;
}

#ifndef CONFIG_AUDIO_EXCLUDE_STOP

/****************************************************************************
 * Name: es7210_stop
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es7210_stop(FAR struct audio_lowerhalf_s *dev,
                       FAR void *session)
#else
static int es7210_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct es7210_dev_s *priv = (FAR struct es7210_dev_s *)dev;

  audinfo("ES7210 stop\n");

  priv->running = false;
  priv->paused  = false;

  /* Tell I2S layer to stop streaming so DMA won't keep running */

  I2S_IOCTL(priv->i2s, AUDIOIOC_STOP, 0);

  /* Mute and power down */

  es7210_writereg(priv, ES7210_ADC12_MUTE_REG15, ES7210_ADC_MUTE);
  es7210_writereg(priv, ES7210_ADC34_MUTE_REG14, ES7210_ADC_MUTE);
  es7210_writereg(priv, ES7210_MIC1_POWER_REG47,
                  ES7210_MIC_POWER_DOWN);
  es7210_writereg(priv, ES7210_MIC2_POWER_REG48,
                  ES7210_MIC_POWER_DOWN);
  es7210_writereg(priv, ES7210_MIC3_POWER_REG49,
                  ES7210_MIC_POWER_DOWN);
  es7210_writereg(priv, ES7210_MIC4_POWER_REG4A,
                  ES7210_MIC_POWER_DOWN);
  es7210_writereg(priv, ES7210_MIC12_POWER_REG4B,
                  ES7210_MIC_POWER_DOWN);
  es7210_writereg(priv, ES7210_MIC34_POWER_REG4C,
                  ES7210_MIC_POWER_DOWN);

  /* Return any pending/done buffers */

  es7210_returnbuffers(priv);

  /* Notify upper half that we are done */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  return OK;
}

#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME

/****************************************************************************
 * Name: es7210_pause
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es7210_pause(FAR struct audio_lowerhalf_s *dev,
                        FAR void *session)
#else
static int es7210_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct es7210_dev_s *priv = (FAR struct es7210_dev_s *)dev;

  audinfo("ES7210 pause\n");
  priv->paused = true;

  /* Mute ADCs */

  es7210_writereg(priv, ES7210_ADC12_MUTE_REG15, ES7210_ADC_MUTE);
  es7210_writereg(priv, ES7210_ADC34_MUTE_REG14, ES7210_ADC_MUTE);

  return OK;
}

/****************************************************************************
 * Name: es7210_resume
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es7210_resume(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session)
#else
static int es7210_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct es7210_dev_s *priv = (FAR struct es7210_dev_s *)dev;

  audinfo("ES7210 resume\n");
  priv->paused = false;

  /* Unmute ADCs */

  es7210_writereg(priv, ES7210_ADC12_MUTE_REG15, ES7210_ADC_UNMUTE);
  es7210_writereg(priv, ES7210_ADC34_MUTE_REG14, ES7210_ADC_UNMUTE);

  es7210_processbegin(priv);

  return OK;
}

#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: es7210_enqueuebuffer
 ****************************************************************************/

static int es7210_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb)
{
  FAR struct es7210_dev_s *priv = (FAR struct es7210_dev_s *)dev;
  irqstate_t flags;

  DEBUGASSERT(priv && apb);

  flags = enter_critical_section();
  dq_addlast((FAR dq_entry_t *)apb, &priv->pendq);
  leave_critical_section(flags);

  /* If we are running, submit pending buffers to the I2S layer.  The I2S
   * driver keeps active and pending DMA buffers in its own queues.
   */

  if (priv->running && !priv->paused)
    {
      es7210_processbegin(priv);
    }

  return OK;
}

/****************************************************************************
 * Name: es7210_cancelbuffer
 ****************************************************************************/

static int es7210_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                               FAR struct ap_buffer_s *apb)
{
  audinfo("apb=%p\n", apb);
  return OK;
}

/****************************************************************************
 * Name: es7210_ioctl
 ****************************************************************************/

static int es7210_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                        unsigned long arg)
{
  FAR struct es7210_dev_s *priv = (FAR struct es7210_dev_s *)dev;
  int ret = OK;

  switch (cmd)
    {
      case AUDIOIOC_HWRESET:
        ret = es7210_reset(priv);
        break;

      case AUDIOIOC_GETBUFFERINFO:
        {
          FAR struct ap_buffer_info_s *bufinfo =
            (FAR struct ap_buffer_info_s *)arg;
          bufinfo->buffer_size = CONFIG_ES7210_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_ES7210_NUM_BUFFERS;
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: es7210_reserve
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es7210_reserve(FAR struct audio_lowerhalf_s *dev,
                          FAR void **session)
#else
static int es7210_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct es7210_dev_s *priv = (FAR struct es7210_dev_s *)dev;
  int ret = OK;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->reserved)
    {
      ret = -EBUSY;
    }
  else
    {
      priv->reserved = true;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: es7210_release
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es7210_release(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session)
#else
static int es7210_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct es7210_dev_s *priv = (FAR struct es7210_dev_s *)dev;

  nxmutex_lock(&priv->devlock);
  priv->reserved = false;
  nxmutex_unlock(&priv->devlock);

  return OK;
}

/****************************************************************************
 * Name: es7210_returnbuffers
 *
 * Description:
 *   Return all pending and done buffers to the upper half.
 *
 ****************************************************************************/

static void es7210_returnbuffers(FAR struct es7210_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;

  for (; ; )
    {
      apb = es7210_dequeuebuffer(priv);

      if (apb == NULL)
        {
          break;
        }

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                      apb, -ENODATA, NULL);
#else
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                      apb, -ENODATA);
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: es7210_initialize
 ****************************************************************************/

FAR struct audio_lowerhalf_s *
  es7210_initialize(FAR struct i2c_master_s *i2c,
                    FAR struct i2s_dev_s *i2s,
                    FAR const struct es7210_lower_s *lower)
{
  FAR struct es7210_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL && i2s != NULL && lower != NULL);

  audinfo("ES7210 initialize: addr=0x%02x freq=%lu\n",
          lower->address, (unsigned long)lower->frequency);

  /* Allocate an ES7210 device structure */

  priv = kmm_zalloc(sizeof(struct es7210_dev_s));
  if (priv == NULL)
    {
      auderr("ERROR: Failed to allocate ES7210 device structure\n");
      return NULL;
    }

  /* Initialize the ES7210 device structure */

  priv->dev.ops   = &g_audioops;
  priv->i2c       = i2c;
  priv->i2s       = i2s;
  priv->lower     = *lower;
  priv->samprate  = ES7210_DEFAULT_SAMPRATE;
  priv->nchannels = ES7210_DEFAULT_NCHANNELS;
  priv->bpsamp    = ES7210_DEFAULT_BPSAMP;
  priv->running   = false;
  priv->paused    = false;

  nxmutex_init(&priv->devlock);
  dq_init(&priv->pendq);

  /* Do not access I2C here.  The ES7210 chip may not be powered yet at
   * board bringup time, which would cause I2C to hang and block the entire
   * boot process.  Hardware initialization is deferred until recording
   * starts.
   */

  return &priv->dev;
}
