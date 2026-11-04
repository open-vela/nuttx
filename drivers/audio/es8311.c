/****************************************************************************
 * drivers/audio/es8311.c
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

#include <sys/ioctl.h>
#include <sys/param.h>
#include <sys/types.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <fixedmath.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/audio/audio.h>

/* [AUDIO-FIX-R36] CONFIG_DEBUG_AUDIO_ERROR/INFO are not set in this build, so
 * auderr()/audinfo() expand to nothing and the driver cannot explain itself.
 * The adapter logs through syslog(LOG_ERR) and netinit drains /dev/kmsg to the
 * console, so that is the channel that actually reaches us. */

#include <syslog.h>
#include <nuttx/audio/es8311.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/sched.h>
#include <nuttx/signal.h>   /* [V751] nxsig_usleep()              */
#include <nuttx/mqueue.h>   /* [V751] file_mq_timedreceive/send() */
#include <time.h>           /* [V751] clock_gettime()             */

/* [V649] ROM console printf, declared before first use (see v645:
 * an implicit declaration followed by an explicit one is a hard
 * "conflicting types" error).  Used by the teardown probes below,
 * which must not go through syslog().
 */

/* [V94] Counters published by arch/xtensa/src/esp32s3/esp32s3_i2s.c. */

extern volatile unsigned int g_v94_recv;
extern volatile int          g_v94_setupret;
extern volatile unsigned int g_v94_start;
extern volatile unsigned int g_v94_isr;
extern volatile unsigned int g_v94_eof;
extern volatile unsigned int g_v94_pb;
extern volatile int          g_v94_i2sret;
extern volatile unsigned int g_v94_done;
extern volatile int          g_v94_rxonly;

/* [V93] Declared here rather than in a header: this is the only board driver
 * that needs it and the arch does not export the symbol.
 */

extern void esp32s3_i2s_set_rxonly(FAR struct i2s_dev_s *dev, bool on);
#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/mqueue.h>
#include <nuttx/queue.h>
#include <nuttx/wqueue.h>
#include <nuttx/spinlock.h>

#include "es8311.h"

/* [V1024] Probe gate.
 *
 * This build carries ~60 probe lines/s during a reply, and a few
 * of them ([V810], [V1012]) dump raw PCM straight at the console.
 * Console throughput here is ~4300 chars/s with an 8 KB ring, so a
 * printf blocks hard once it fills.  At 60 lines/s the audio thread
 * spends most of its budget inside printf, which is itself a source
 * of the stutter we are trying to measure.
 *
 * g_v1024_quiet == 1 suppresses every [Vnnn] printf except the
 * handful of load-bearing ones (see KEEP in the patch script).
 * Flip it to 0 at runtime from NSH to bring them all back.
 */
#ifdef __cplusplus
extern "C" int g_v1024_quiet;
#else
extern int g_v1024_quiet;
#endif

#ifndef V1024_QUIET
#define V1024_QUIET 1
#endif
#define V1024P(...) do { if (V1024_QUIET == 0) printf(__VA_ARGS__); } while (0)
#define V1024S(lvl, ...) do { if (V1024_QUIET == 0) syslog(lvl, __VA_ARGS__); } while (0)


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if !defined(CONFIG_ES8311_REGDUMP)
static
#endif /* !CONFIG_ES8311_REGDUMP */
uint8_t     es8311_readreg(FAR struct es8311_dev_s *priv, uint8_t regaddr);
static int  es8311_writereg(FAR struct es8311_dev_s *priv,
                            uint8_t regaddr,
                            uint16_t regval);
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static int  es8311_setvolume(FAR struct es8311_dev_s *priv,
                             es_module_e module,
                             uint16_t volume);
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */
static int  es8311_getcoeff(FAR struct es8311_dev_s *priv,
                            uint32_t samplerate);
#ifndef CONFIG_AUDIO_EXCLUDE_MUTE
static int  es8311_setmute(FAR struct es8311_dev_s *priv,
                           es_module_e module,
                           bool enable);
#endif /* CONFIG_AUDIO_EXCLUDE_MUTE */
static int  es8311_setmicgain(FAR struct es8311_dev_s *priv, uint32_t gain);
/* [V660] Forward declaration: es8311_start() calls this before its
 * definition further down the file. */
static void es8311_v660_dump(FAR struct es8311_dev_s *priv,
                             const char *tag);
static int  es8311_setbitspersample(FAR struct es8311_dev_s *priv);
static int  es8311_setsamplerate(FAR struct es8311_dev_s *priv);
static int  es8311_getcaps(FAR struct audio_lowerhalf_s *dev,
                           int type,
                           FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8311_configure(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session,
                             FAR const struct audio_caps_s *caps);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int  es8311_configure(FAR struct audio_lowerhalf_s *dev,
                             FAR const struct audio_caps_s *caps);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
static int  es8311_shutdown(FAR struct audio_lowerhalf_s *dev);
static void es8311_processdone(FAR struct i2s_dev_s *i2s,
                                  FAR struct ap_buffer_s *apb,
                                  FAR void *arg,
                                  int result);
static void es8311_returnbuffers(FAR struct es8311_dev_s *priv);
static int  es8311_processbegin(FAR struct es8311_dev_s *priv);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8311_start(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int  es8311_start(FAR struct audio_lowerhalf_s *dev);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8311_stop(FAR struct audio_lowerhalf_s *dev,
                        FAR void *session);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int  es8311_stop(FAR struct audio_lowerhalf_s *dev);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8311_pause(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session);
static int  es8311_resume(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int  es8311_pause(FAR struct audio_lowerhalf_s *dev);
static int  es8311_resume(FAR struct audio_lowerhalf_s *dev);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */
static int  es8311_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb);
static int  es8311_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb);
static int  es8311_ioctl(FAR struct audio_lowerhalf_s *dev,
                         int cmd,
                         unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
/* [V755] Arch-layer hook: give back every buffer container stalled in
 * the I2S TX queues.  Defined in
 * arch/xtensa/src/esp32s3/esp32s3_i2s.c.  Returns immediately when
 * there is nothing to reclaim. */

extern void esp32s3_i2s_tx_flush(void);

static int  es8311_reserve(FAR struct audio_lowerhalf_s *dev,
                           FAR void **session);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int  es8311_reserve(FAR struct audio_lowerhalf_s *dev);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8311_release(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int  es8311_release(FAR struct audio_lowerhalf_s *dev);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
static void *es8311_workerthread(pthread_addr_t pvarg);
static void es8311_audio_output(FAR struct es8311_dev_s *priv);
static void es8311_audio_input(FAR struct es8311_dev_s *priv);
static void es8311_reset(FAR struct es8311_dev_s *priv);
static int  es8311_get_mclk_src(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* [V685] Microphone register sweep instrument.
 *
 * /tmp/es8311sweep.txt holds up to 16 "AA BB" pairs.  Every
 * ES8311_V685_CALLS_PER_STEP completed capture buffers the sweep applies
 * the next pair and reports the peak that the previous step saw, so one
 * BOOT press maps every candidate register to a measured signal level.
 *
 * processdone() runs in DMA callback context, so it only accumulates into
 * these counters; all logging happens in processbegin() (task context).
 */

#define ES8311_V685_MAX_PAIRS      16
/* [V703] Calibration sweep parked.  v701/v702 used this knob to rotate
 * the candidate table every 10 buffers, which meant the codec kept being
 * reconfigured while the user was speaking.  100000 buffers per step
 * leaves the table in place (it stays as documentation of the two walks)
 * but applies it at most once per session. */
#define ES8311_V685_CALLS_PER_STEP 100000

static int32_t  g_v685_step_max;    /* max |sample| seen this step */
static uint32_t g_v685_step_bytes;  /* bytes completed this step */


/* [V889] Clean single-variable experiment: pin ONE es8311 worker stack
 * into internal DRAM at the SAME SIZE as the v887 baseline (16384 bytes).
 *
 * v888 and v888b moved the stack into internal DRAM but also shrank it
 * from 16384 to 8192, so two variables changed at once and the result
 * could not be attributed.  Both still crashed with the identical
 * signature (_window_overflow12 / _xtensa_context_save, EXCCAUSE 29
 * StoreProhibited), and dump_task confirmed the stack really was in
 * internal DRAM (0x3fcc1070 / 0x3fcbd830) - so "stack in PSRAM" is NOT the
 * root cause.  v889 exists only to remove the size variable and confirm
 * that conclusion on a like-for-like basis: same 16384 bytes, different
 * location.
 *
 * Only one buffer is provided, so only one worker at a time may use it.
 * g_v889_stack_taken guards it and g_v889_stack_owner records which
 * device instance owns it.  [V891] That used to hold a generation
 * number; see the note at the definition below for why that was wrong.  es8311_start() can abandon a previous worker
 * (es8311_wait_worker_exit(priv, 3000) times out and sets worker_stuck =
 * 1) and start a new one right away, so two workers can be alive at the
 * same time and must never share this buffer; if the slot is busy,
 * es8311_start() falls back to the v887 path, pthread_attr_setstacksize(
 * &tattr, 16384), which leaves the stack on the heap (PSRAM) - exactly the
 * baseline behaviour.
 *
 * Budget: the original internal DRAM headroom was 15120 bytes (_sheap
 * 0x3fccc4f0 against a dram0_0_seg end of 0x3fcd0000); halving the LVGL
 * line buffer s_v824_buf (SCREEN_W * 40 -> * 20 rows) in xiaozhi_ui_main.c
 * released another 14400, giving 29520.  One 16384-byte buffer leaves
 * 29520 - 16384 = 13136 bytes.  Two would need 32768 and does not fit,
 * and the remaining candidates (IOB, g_v786_ui_stack, RAMLOG) are
 * explicitly out of scope.
 *
 * Safety: up_use_stack() does not set TCB_FLAG_FREE_STACK (only
 * up_create_stack() does), so up_release_stack() never tries to free a
 * pre-allocated stack - a static buffer is safe for a detached worker.
 */

#define ES8311_V889_STACK_SIZE  16384

/* 16-byte aligned: up_use_stack() asserts STACKFRAME_ALIGN_MASK on the
 * base address.
 */

static uint8_t g_v889_es8311_stack[ES8311_V889_STACK_SIZE]
  __attribute__((aligned(16)));

/* Owner bookkeeping for the single pinned stack.  g_v889_stack_taken ==
 * false means the slot is free; g_v889_stack_owner identifies which device
 * instance owns it.
 *
 * [V891] g_v889_stack_owner used to hold a generation number.  This board
 * has two es8311 instances - pcm0 and pcm_in0, registered in
 * arch/xtensa/src/esp32s3/esp32s3_es8311.c - and each carries its own
 * priv->worker_gen counter, so a worker of one instance can hold exactly
 * the same generation number as a worker of the other.  The old test
 * against that number was therefore satisfied by the wrong instance and
 * released a stack another worker was still running on, leaving two
 * threads sharing one 16 KB buffer with the newer one writing over the
 * older one's frames.  The priv pointer is unique per instance, so key
 * the ownership on that.
 *
 * [V891] The layout makes this bug reach further than usual: the stack
 * base sits at 0x3fcbeff0 and these two flag words at 0x3fcbefe4 and
 * 0x3fcbefe8, i.e. within the 12 bytes immediately below it, so a
 * small underflow of the stack corrupts precisely this bookkeeping.
 */

static bool     g_v889_stack_taken;
/* [V891] Ownership of the single pinned stack is a (priv, generation)
 * pair, and BOTH halves must match before the slot is released.
 *
 *   priv - this board has two es8311 instances, pcm0 and pcm_in0,
 *          registered in arch/xtensa/src/esp32s3/esp32s3_es8311.c.
 *          Each has its own priv->worker_gen counter, so a worker of
 *          one instance can carry exactly the same generation number
 *          as a worker of the other.  Comparing priv keeps instance A
 *          from releasing a slot that instance B is running on.
 *
 *   gen  - within one instance, an abandoned worker (es8311_start()
 *          times out after 3000 ms and starts a new one anyway) is
 *          still alive with an older generation number.  Comparing the
 *          generation keeps that late worker from releasing the slot
 *          that a newer worker of the SAME instance already owns.
 *
 * [V891] These two words also sit directly below the stack in .bss
 * (stack base 0x3fcbeff0, words at 0x3fcbefe4/0x3fcbefe8), so a small
 * underflow of the stack corrupts precisely this bookkeeping.
 */

static void     *g_v889_stack_owner;  /* [V891B] priv of owning device */
static uint32_t  g_v889_stack_gen;    /* [V891B] generation within it  */

static const struct audio_ops_s g_audioops =
{
  es8311_getcaps,       /* getcaps        */
  es8311_configure,     /* configure      */
  es8311_shutdown,      /* shutdown       */
  es8311_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  es8311_stop,          /* stop           */
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  es8311_pause,         /* pause          */
  es8311_resume,        /* resume         */
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */
  NULL,                 /* allocbuffer    */
  NULL,                 /* freebuffer     */
  es8311_enqueuebuffer, /* enqueue_buffer */
  es8311_cancelbuffer,  /* cancel_buffer  */
  es8311_ioctl,         /* ioctl          */
  NULL,                 /* read           */
  NULL,                 /* write          */
  es8311_reserve,       /* reserve        */
  es8311_release        /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: es8311_get_mclk_src
 *
 * Description:
 *    Get the MCLK source.
 *
 * Returned Value:
 *   ES8311_MCLK_FROM_MCLK_PIN or ES8311_MCLK_FROM_SCLK_PIN.
 *
 ****************************************************************************/

static int es8311_get_mclk_src(void)
{
#ifdef CONFIG_ES8311_SRC_MCLK
  return ES8311_MCLK_FROM_MCLK_PIN;
#else
  return ES8311_MCLK_FROM_SCLK_PIN;
#endif
}

/****************************************************************************
 * Name: es8311_readreg
 *
 * Description:
 *   Read the specified 8-bit register from the ES8311 device through I2C.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *   regaddr - Address of the register to be read.
 *
 * Returned Value:
 *   On success, the byte stored in the register.
 *   On failure, 0.
 *
 ****************************************************************************/

#if !defined(CONFIG_ES8311_REGDUMP)
static
#endif /* !CONFIG_ES8311_REGDUMP */
uint8_t es8311_readreg(FAR struct es8311_dev_s *priv, uint8_t regaddr)
{
  int retries;

  /* Try up to three times to read the register */

  for (retries = 0; retries < 3; retries++)
    {
      struct i2c_msg_s msg[2];
      uint8_t data;
      int ret;

      /* Set up to write the address */

      msg[0].frequency = priv->lower->frequency;
      msg[0].addr      = priv->lower->address;
      msg[0].flags     = 0;
      msg[0].buffer    = &regaddr;
      msg[0].length    = 1;

      /* Followed by the read data */

      msg[1].frequency = priv->lower->frequency;
      msg[1].addr      = priv->lower->address;
      msg[1].flags     = I2C_M_READ;
      msg[1].buffer    = &data;
      msg[1].length    = 12;

      /* Read the register data. The returned value is the number messages
       * completed.
       */

      ret = I2C_TRANSFER(priv->i2c, msg, 2);
      if (ret < 0)
        {
#ifdef CONFIG_I2C_RESET
          /* Perhaps the I2C bus is locked up? Try to shake the bus free */

          audwarn("WARNING: I2C_TRANSFER failed: %d ... Resetting\n", ret);

          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              auderr("I2C_RESET failed: %d\n", ret);
              break;
            }
#else /* !CONFIG_I2C_RESET */
          auderr("I2C_TRANSFER failed: %d\n", ret);
#endif /* !CONFIG_I2C_RESET */
        }
      else
        {
          /* The I2C transfer was successful... break out of the loop and
           * return the value read.
           */

          audinfo("Read: %02x -> %02x\n", regaddr, data);
          return data;
        }

      audinfo("retries=%d regaddr=%02x\n", retries, regaddr);
    }

  /* No error indication is returned on a failure... just return zero */

  return 0;
}

/****************************************************************************
 * Name: es8311_writereg
 *
 * Description:
 *   Write the specified 8-bit register to the ES8311 device through I2C.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *   regaddr - Address of the register to be written.
 *   regval - Value to be written.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8311_writereg(FAR struct es8311_dev_s *priv,
                           uint8_t regaddr,
                           uint16_t regval)
{
  struct i2c_config_s config;
  int retries;

  /* Setup up the I2C configuration */

  config.frequency = priv->lower->frequency;
  config.address   = priv->lower->address;
  config.addrlen   = 7;

  /* Try up to three times to write the register */

  for (retries = 0; retries < 3; retries++)
    {
      uint8_t data[2];
      int ret;

      /* Set up the data to write */

      data[0] = regaddr;
      data[1] = regval;

      /* Read the register data. The returned value is the number messages
       * completed.
       */

      ret = i2c_write(priv->i2c, &config, data, 2);
      if (ret < 0)
        {
#ifdef CONFIG_I2C_RESET
          /* Perhaps the I2C bus is locked up? Try to shake the bus free */

          audwarn("WARNING: i2c_write failed: %d ... Resetting\n", ret);

          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              auderr("I2C_RESET failed: %d\n", ret);
              break;
            }
#else /* !CONFIG_I2C_RESET */
          auderr("I2C_TRANSFER failed: %d\n", ret);
#endif /* !CONFIG_I2C_RESET */
        }
      else
        {
          /* The I2C transfer was successful... break out of the loop and
           * return.
           */

          audinfo("Write: %02x <- %02x\n", regaddr, regval);
          return OK;
        }

      audinfo("retries=%d regaddr=%02x\n", retries, regaddr);
    }

  return -EIO;
}

/****************************************************************************
 * Name: es8311_setmicgain
 *
 * Description:
 *   Set the microphone gain.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *   gain - The microphone gain to be set in the codec (0..42).
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8311_setmicgain(FAR struct es8311_dev_s *priv, uint32_t gain)
{
  int ret;
  static const es8311_mic_gain_e gain_map[] =
  {
    ES8311_MIC_GAIN_0DB,
    ES8311_MIC_GAIN_6DB,
    ES8311_MIC_GAIN_12DB,
    ES8311_MIC_GAIN_18DB,
    ES8311_MIC_GAIN_24DB,
    ES8311_MIC_GAIN_30DB,
    ES8311_MIC_GAIN_36DB,
    ES8311_MIC_GAIN_42DB
  };

  /* Our range is from 0 to 42 dB with steps of 6 dB. We map this to the
   * ES8311 gain values.
   */

  priv->mic_gain = gain_map[MIN(gain, 42) / 6];

  /* [V730] *** REG16 is SCALE in bits[2:0]; bits[7:4] are INV. ***
   *
   * The [V700] comment above was wrong, and it is the reason the
   * microphone has been quiet since the port started.  The
   * authoritative map (Linux sound/soc/codecs/es8311.h) is
   *
   *   ES8311_ADC2 (0x16):  INV   bits[7:4]      (invert)
   *                        SCALE bits[2:0]      (gain, max 0x07)
   *   ES8311_SYS10 (0x14): PGAGAIN bits[3:0]    (analog PGA, max 0x0A)
   *   ES8311_ADC3 (0x17):  volume 0..0xFF
   *
   * and the chip proves it: writing 0x24 / 0x44 / 0x84 / 0xc4 to 0x16
   * reads back as 0x04 every time (bits[7:4] are not writable), while
   * bits[2:0] take effect at once.  Measured with the v729 `ac=` /
   * `ms=` probe (_work\\_v729_scale.log), quiet-room median:
   *
   *   0x16=0x00 SCALE 0   ac 250   rms  57
   *   0x16=0x04 SCALE 4   ac 322   rms  68   <- what every build shipped
   *   0x16=0x06 SCALE 6   ac 1448  rms 398
   *   0x16=0x07 SCALE 7   ac 1985  rms 438   <- +15 dB, the documented max
   *
   * The eight-sample peak-to-peak grows by the same factor, so this is
   * signal, not DC.  Controls: 0x01=0x00 silences the capture outright
   * (the data really is the ES8311), 0x17 has no headroom left (0xbf is
   * already 0 dB), 0x14 already carries PGAGAIN 0x0A, and 0x18=0x00
   * means ALC is not compensating.
   *
   * One gain_map[] ordinal is one SCALE step. */

  ret = es8311_writereg(priv, ES8311_ADC_REG16,
                        (uint8_t)((uint32_t)priv->mic_gain & 0x07u));

  if (ret < 0)
    {
      auderr("Failed to set mic gain: %d\n", ret);
    }
  else
    {
      audinfo("Set mic gain: %d\n", priv->mic_gain);
    }

  return ret;
}

/****************************************************************************
 * Name: es8311_setvolume
 *
 * Description:
 *   Set the right and left volume values in the ES8311 device based on the
 *   desired volume settings.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure;
 *   module - The module to change the volume for;
 *   volume - The volume to be set in the codec (0..1000).
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static int es8311_setvolume(FAR struct es8311_dev_s *priv,
                            es_module_e module,
                            uint16_t volume)
{
  /* Convert from (0..1000) to (0..200) ln curve to avoid distortion */

  uint16_t dblvl = (int16_t) (volume ? (29 * logf(volume)) : 0);
  int ret = 0;

  /* Set the volume */

  if (module == ES_MODULE_DAC || module == ES_MODULE_ADC_DAC)
    {
      ret |= es8311_writereg(priv, ES8311_DAC_REG32, dblvl);
      priv->volume_out = volume;
    }

  if (module == ES_MODULE_ADC || module == ES_MODULE_ADC_DAC)
    {
      ret |= es8311_writereg(priv, ES8311_ADC_REG17, dblvl);
      priv->volume_in = volume;
    }

  if (ret < 0)
    {
      auderr("Failed to set volume.\n");
      return -EIO;
    }
  else
    {
      audinfo("Volume set to %d\n", volume);
      return OK;
    }
}
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

/****************************************************************************
 * Name: es8311_getcoeff
 *
 * Description:
 *   Get the coefficient values for the current MCLK and given sample rate.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *   samplerate - The sample rate to be set in the codec.
 *
 * Returned Value:
 *   The index of the divider in the es8311_coeff_div array or -EINVAL if
 *   the combination of MCLK and sample rate is not supported.
 *
 ****************************************************************************/

static int es8311_getcoeff(FAR struct es8311_dev_s *priv,
                           uint32_t samplerate)
{
  int i;

  for (i = 0; i < nitems(es8311_coeff_div); i++)
    {
      if (priv->mclk == es8311_coeff_div[i].mclk &&
          samplerate == es8311_coeff_div[i].rate)
        {
          audinfo("MCLK: %d, samplerate: %d\n", priv->mclk, samplerate);
          return i;
        }
    }

  auderr("MCLK = %d and samplerate = %d not supported.\n",
         priv->mclk, samplerate);

  return -EINVAL;
}

/****************************************************************************
 * Name: es8311_setmute
 *
 * Description:
 *   Mute/unmute the ADC or DAC of the codec based on the current settings.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *   enable - Boolean to enable or disable the mute function.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_MUTE
static int es8311_setmute(FAR struct es8311_dev_s *priv,
                          es_module_e module,
                          bool enable)
{
  uint8_t reg = 0;
  int ret = 0;

  audinfo("module=%d, mute=%d\n", module, (int)enable);

  priv->mute = enable;

  if (module == ES_MODULE_DAC || module == ES_MODULE_ADC_DAC)
    {
      reg = es8311_readreg(priv, ES8311_DAC_REG31) & 0x9f;
      if (enable)
        {
          ret |= es8311_writereg(priv, ES8311_SYSTEM_REG12, 0x02);
          ret |= es8311_writereg(priv, ES8311_DAC_REG31,    reg | 0x60);
          ret |= es8311_writereg(priv, ES8311_DAC_REG32,    0x00);
          ret |= es8311_writereg(priv, ES8311_DAC_REG37,    0x08);
        }
      else
        {
          ret |= es8311_writereg(priv, ES8311_DAC_REG31,    reg);
          ret |= es8311_writereg(priv, ES8311_SYSTEM_REG12, 0x00);
        }
    }

  if (module == ES_MODULE_ADC || module == ES_MODULE_ADC_DAC)
    {
      reg = es8311_readreg(priv, ES8311_SDPOUT_REG0A) & 0xbf;
      ret |= es8311_writereg(priv, ES8311_SDPOUT_REG0A,
                             reg | ((int) enable << 6));
    }

  if (ret < 0)
    {
      auderr("Failed to set mute.\n");
      return -EIO;
    }
  else
    {
      audinfo("Set mute to: %d\n", (int)enable);
      return OK;
    }
}
#endif

/****************************************************************************
 * Name: es8311_setbitspersample
 *
 * Description:
 *   Set the number of bits per sample used by the I2S driver and the codec.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

/* [V682] Hex nibble for the /tmp/es8311.txt register override. */

static int es8311_v682_hex(int c)
{
  if (c >= '0' && c <= '9')
    {
      return c - '0';
    }

  if (c >= 'a' && c <= 'f')
    {
      return c - 'a' + 10;
    }

  if (c >= 'A' && c <= 'F')
    {
      return c - 'A' + 10;
    }

  return -1;
}

static int es8311_setbitspersample(FAR struct es8311_dev_s *priv)
{
  uint8_t reg;
  uint8_t bit_config;
  int ret;

  DEBUGASSERT(priv && priv->lower);

  switch (priv->bpsamp)
    {
      case 16:
        bit_config = ES_WORD_LENGTH_16BITS;
        break;

      case 24:
        bit_config = ES_WORD_LENGTH_24BITS;
        break;

      case 32:
        bit_config = ES_WORD_LENGTH_32BITS;
        break;

      default:
        audwarn("Data length not supported.\n");
        return -EINVAL;
    }

  ret = I2S_RXDATAWIDTH(priv->i2s, priv->bpsamp);
  if (ret < 0)
    {
      auderr("I2S_RXDATAWIDTH failed.\n");
      return ret;
    }

  ret = I2S_TXDATAWIDTH(priv->i2s, priv->bpsamp);
  if (ret < 0)
    {
      auderr("I2S_TXDATAWIDTH failed.\n");
      return ret;
    }

  ret = 0;
  if (priv->audio_mode == ES_MODULE_ADC ||
      priv->audio_mode == ES_MODULE_ADC_DAC)
    {
      reg = es8311_readreg(priv, ES8311_SDPOUT_REG0A) & 0xe3;
      ret |= es8311_writereg(priv, ES8311_SDPOUT_REG0A,
                             reg | (bit_config << 2));
    }

  if (priv->audio_mode == ES_MODULE_DAC ||
      priv->audio_mode == ES_MODULE_ADC_DAC)
    {
      reg = es8311_readreg(priv, ES8311_SDPIN_REG09) & 0xe3;
      ret |= es8311_writereg(priv, ES8311_SDPIN_REG09,
                             reg | (bit_config << 2));
    }

  if (ret < 0)
    {
      auderr("Failed to set bits per sample\n");
      return -EIO;
    }
  else
    {
      audinfo("Bits per sample set to: %d\n", priv->bpsamp);
      return OK;
    }
}

/****************************************************************************
 * Name: es8311_setsamplerate
 *
 * Description:
 *  Sets the sample frequency for the codec ADC/DAC and the I2S driver.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8311_setsamplerate(FAR struct es8311_dev_s *priv)
{
  DEBUGASSERT(priv && priv->lower);

  uint8_t regconfig;
  uint8_t datmp;
  int coeff_index;
  int ret;

  /* [AUDIO-FIX-R36] Program the clock for THIS rate, then apply the rate, and
   * only then look the divider up.
   *
   * The original order read MCLK first, but i2s_getmclkfrequency() returns the
   * clock that was programmed last - i2s_configure() is the only thing that
   * primes it and nothing re-primes it per stream - so the value read belonged
   * to whatever rate came before.  es8311_getcoeff() matches the exact
   * (MCLK, rate) pair, so that mismatch failed the lookup and CONFIGURE came
   * back -EINVAL for settings this codec fully supports.
   *
   * rate x 256 is the multiple the ES8311 tables use for 16-bit samples (the
   * I2S upper half picks the same one for anything that is not 24-bit), and
   * the table carries an entry for every standard rate: 24000 -> 6144000,
   * 16000 -> 4096000.
   */

  I2S_SETMCLKFREQUENCY(priv->i2s,
                       priv->samprate * (priv->bpsamp == 24 ? 384 : 256));

  ret = I2S_RXSAMPLERATE(priv->i2s, priv->samprate);
  syslog(LOG_ERR, "[R36] I2S RX rate %lu -> %d\n",
         (unsigned long)priv->samprate, ret);

  ret = I2S_TXSAMPLERATE(priv->i2s, priv->samprate);
  syslog(LOG_ERR, "[R36] I2S TX rate %lu -> %d\n",
         (unsigned long)priv->samprate, ret);

  priv->mclk = I2S_GETMCLKFREQUENCY(priv->i2s);
  coeff_index = es8311_getcoeff(priv, priv->samprate);

  syslog(LOG_ERR, "[R36] setsamplerate rate=%lu mclk=%lu coeff=%d\n",
         (unsigned long)priv->samprate, (unsigned long)priv->mclk,
         coeff_index);

  if (coeff_index < 0)
    {
      auderr("Failed to set sample rate: %d\n", -EINVAL);
      return -EINVAL;
    }

  ret = 0;
  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG02) & 0x07;
  regconfig |= (es8311_coeff_div[coeff_index].pre_div - 1) << 5;

  datmp = 0;
  switch (es8311_coeff_div[coeff_index].pre_multi)
    {
      case 1:
        datmp = 0;
        break;
      case 2:
        datmp = 1;
        break;
      case 4:
        datmp = 2;
        break;
      case 8:
        datmp = 3;
        break;
      default:
        break;
    }

  if (es8311_get_mclk_src() == ES8311_MCLK_FROM_SCLK_PIN)
    {
      datmp = 3;     /* DIG_MCLK = LRCK * 256 = BCLK * 8 */
    }

  regconfig |= (datmp) << 3;
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG02, regconfig);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG05) & 0x00;
  regconfig |= (es8311_coeff_div[coeff_index].adc_div - 1) << 4;
  regconfig |= (es8311_coeff_div[coeff_index].dac_div - 1) << 0;
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG05, regconfig);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG03) & 0x80;
  regconfig |= es8311_coeff_div[coeff_index].fs_mode << 6;
  regconfig |= es8311_coeff_div[coeff_index].adc_osr << 0;
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG03, regconfig);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG04) & 0x80;
  regconfig |= es8311_coeff_div[coeff_index].dac_osr << 0;
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG04, regconfig);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG07) & 0xc0;
  regconfig |= es8311_coeff_div[coeff_index].lrck_h << 0;
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG07, regconfig);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG08) & 0x00;
  regconfig |= es8311_coeff_div[coeff_index].lrck_l << 0;
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG08, regconfig);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG06) & 0xe0;
  if (es8311_coeff_div[coeff_index].bclk_div < 19)
    {
      regconfig |= (es8311_coeff_div[coeff_index].bclk_div - 1) << 0;
    }
  else
    {
      regconfig |= (es8311_coeff_div[coeff_index].bclk_div) << 0;
    }

  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG06, regconfig);

  if (ret < 0)
    {
      auderr("Failed to set sample rate.\n");
      return -EIO;
    }
  else
    {
      audinfo("Sample rate set to: %d.\n", priv->samprate);
      return OK;
    }
}

/****************************************************************************
 * Name: es8311_getcaps
 *
 * Description:
 *   Get the audio device capabilities.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   type - The type of query.
 *   caps - A reference to an audio caps structure.
 *
 * Returned Value:
 *   Length of the caps structure.
 *
 ****************************************************************************/

static int es8311_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                          FAR struct audio_caps_s *caps)
{
  /* Validate the structure */

  DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));
  audinfo("getcaps: type=%d ac_type=%d\n", type, caps->ac_type);

  /* Fill in the caller's structure based on requested info */

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      /* Caller is querying for the types of units we support */

      case AUDIO_TYPE_QUERY:

        /* Provide our overall capabilities. The interfacing software
         * must then call us back for specific info for each capability.
         */

        caps->ac_channels = 1;       /* Mono output */

        if (caps->ac_subtype == AUDIO_TYPE_QUERY)
          {
              /* The types of audio units we implement */

              caps->ac_controls.b[0] = AUDIO_TYPE_INPUT |
                                       AUDIO_TYPE_OUTPUT |
                                       AUDIO_TYPE_FEATURE;
              break;
          }

        caps->ac_controls.b[0] = AUDIO_SUBFMT_END;

        break;

      /* Provide capabilities of our OUTPUT unit */

      case AUDIO_TYPE_OUTPUT:

        caps->ac_channels = 1;

        if (caps->ac_subtype == AUDIO_TYPE_QUERY)
          {
              /* Report the Sample rates we support */

              /* 8kHz is hardware dependent */

              caps->ac_controls.hw[0] =
                AUDIO_SAMP_RATE_11K | AUDIO_SAMP_RATE_16K |
                AUDIO_SAMP_RATE_22K | AUDIO_SAMP_RATE_32K |
                AUDIO_SAMP_RATE_44K | AUDIO_SAMP_RATE_48K;
              break;
         }

        break;

      case AUDIO_TYPE_INPUT:

        caps->ac_channels = 1;

        if (caps->ac_subtype == AUDIO_TYPE_QUERY)
          {
              /* Report supported input sample rates */

              caps->ac_controls.hw[0] =
                AUDIO_SAMP_RATE_11K | AUDIO_SAMP_RATE_16K |
                AUDIO_SAMP_RATE_22K | AUDIO_SAMP_RATE_32K |
                AUDIO_SAMP_RATE_44K | AUDIO_SAMP_RATE_48K;
              break;
         }

        break;

      /* Provide capabilities of our FEATURE units */

      case AUDIO_TYPE_FEATURE:

        /* If the sub-type is UNDEF, then report the Feature Units we
         * support.
         */

        if (caps->ac_subtype == AUDIO_FU_UNDEF)
          {
            /* Fill in the ac_controls section with the Feature Units we
             * have.
             */

            caps->ac_controls.b[0] = AUDIO_FU_VOLUME | AUDIO_FU_MUTE;
            caps->ac_controls.b[1] = (AUDIO_FU_INP_GAIN) >> 8;
          }

        break;

      /* All others we don't support */

      default:

        /* Zero out the fields to indicate no support */

        caps->ac_subtype = 0;
        caps->ac_channels = 0;

        break;
    }

  /* Return the length of the audio_caps_s struct for validation of
   * proper Audio device type.
   */

  return caps->ac_len;
}

/****************************************************************************
 * Name: es8311_configure
 *
 * Description:
 *   Configure the audio device for the specified mode of operation.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *   caps - A reference to an audio caps structure.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8311_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session,
                            FAR const struct audio_caps_s *caps)
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int es8311_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR const struct audio_caps_s *caps)
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;
  int ret = OK;

  DEBUGASSERT(priv != NULL && caps != NULL);
  audinfo("configure: ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
    case AUDIO_TYPE_FEATURE:
      audinfo("  AUDIO_TYPE_FEATURE\n");

      /* Process based on Feature Unit */

      switch (caps->ac_format.hw)
        {
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
        case AUDIO_FU_VOLUME:
          {
            /* Set the volume */

            uint16_t volume = caps->ac_controls.hw[0];
            audinfo("    Volume: %d\n", volume);

            if (volume >= 0 && volume <= 1000)
              {
                es8311_setvolume(priv, priv->audio_mode, volume);
                break;
              }

            ret = -EDOM;
          }
          break;
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

#ifndef CONFIG_AUDIO_EXCLUDE_MUTE
        case AUDIO_FU_MUTE:
          {
            /* Mute/Unmute */

            bool mute = (bool)caps->ac_controls.hw[0];
            audinfo("    Mute: %d\n", mute);

            es8311_setmute(priv, ES_MODULE_DAC, mute);
          }
          break;
#endif /* CONFIG_AUDIO_EXCLUDE_MUTE */

        case AUDIO_FU_INP_GAIN:
          {
            /* Set the mic gain */

            uint32_t mic_gain = caps->ac_controls.hw[0];

            /* [V1234] *** the client silently zeroes the mic ***
             *
             * native_capture_open() hands us an audio_caps_s whose
             * ac_controls.hw[0] is 0 for the input feature unit, so every
             * capture session ended by writing REG16 SCALE = 0, i.e. 0 dB.
             * The v730 sweep measured exactly that point:
             *
             *   0x16 SCALE 0 -> ac 250   (this is what we measure now:
             *                              mn=-64 mx=63, peak 127)
             *   0x16 SCALE 6 -> ac 1448
             *   0x16 SCALE 7 -> ac 1985
             *
             * 42 dB clipped on speech (v979), 36 dB did not.  Treat any
             * request below 30 dB as "client did not mean it" and hold the
             * front end at 36 dB instead of dropping the mic 36 dB below
             * where it belongs.
             */

            if (mic_gain < 30)
              {
                mic_gain = 36;
              }

            audinfo("    Mic gain: %" PRIu32 "\n", mic_gain);

            es8311_setmicgain(priv, mic_gain);
          }
          break;

        default:
          auderr("    Unrecognized feature unit\n");
          ret = -ENOTTY;
          break;
        }
        break;

    case AUDIO_TYPE_OUTPUT:
      {
        audinfo("  AUDIO_TYPE_OUTPUT:\n");
        audinfo("    Number of channels: %u\n", caps->ac_channels);
        audinfo("    Sample rate:        %u\n", caps->ac_controls.hw[0]);
        audinfo("    Sample width:       %u\n", caps->ac_controls.b[2]);

        /* Verify that all of the requested values are supported */

        ret = -ERANGE;

        /* The codec can take stereo audio and play only one channel */

        if (caps->ac_channels != 1 && caps->ac_channels != 2)
          {
            auderr("Unsupported number of channels: %d\n",
                   caps->ac_channels);
            break;
          }

        if (caps->ac_controls.b[2] != 16 &&
            caps->ac_controls.b[2] != 24 &&
            caps->ac_controls.b[2] != 32)
          {
            auderr("Unsupported bits per sample: %d\n",
                   caps->ac_controls.b[2]);
            break;
          }

        es8311_audio_output(priv);
        es8311_reset(priv);

        /* Save the current stream configuration */

        priv->samprate  = caps->ac_controls.hw[0];
        priv->bpsamp    = caps->ac_controls.b[2];

        /* [AUDIO-FIX-R36] The ternary was inverted.
         *
         * ret still holds the -ERANGE sentinel assigned just above, and
         * "setter() == -ENOTTY ? OK : ret" keeps that sentinel whenever the
         * setter actually worked - so CONFIGURE reported -ERANGE (-34) and the
         * stream was refused.  That is what silenced TTS and the native
         * capture path.  Treat only a real NOTTY as "method absent". */

        syslog(LOG_ERR, "[R36] configure OUT rate=%lu ch=%u bps=%u\n",
               (unsigned long)caps->ac_controls.hw[0],
               (unsigned int)caps->ac_channels,
               (unsigned int)caps->ac_controls.b[2]);

        ret = es8311_setsamplerate(priv);
        if (ret == -ENOTTY)
          {
            ret = OK;
          }

        if (ret < 0)
          {
            break;
          }

        ret = es8311_setbitspersample(priv);
        if (ret == -ENOTTY)
          {
            ret = OK;
          }
      }
      break;

    case AUDIO_TYPE_INPUT:
      {
        audinfo("  AUDIO_TYPE_INPUT:\n");
        audinfo("    Number of channels: %u\n", caps->ac_channels);
        audinfo("    Sample rate:        %u\n", caps->ac_controls.hw[0]);
        audinfo("    Sample width:       %u\n", caps->ac_controls.b[2]);

        /* Verify that all of the requested values are supported */

        ret = -ERANGE;

        /* The codec can take stereo audio and play only one channel */

        if (caps->ac_channels != 1 && caps->ac_channels != 2)
          {
            auderr("Unsupported number of channels: %d\n",
                   caps->ac_channels);
            break;
          }

        if (caps->ac_controls.b[2] != 16 &&
            caps->ac_controls.b[2] != 24 &&
            caps->ac_controls.b[2] != 32)
          {
            auderr("Unsupported bits per sample: %d\n",
                   caps->ac_controls.b[2]);
            break;
          }

        es8311_audio_input(priv);
        es8311_reset(priv);

        /* Save the current stream configuration */

        priv->samprate  = caps->ac_controls.hw[0];
        priv->bpsamp    = caps->ac_controls.b[2];

        /* [AUDIO-FIX-R36] Inverted ternary, see the AUDIO_TYPE_OUTPUT branch. */

        syslog(LOG_ERR, "[R36] configure IN  rate=%lu ch=%u bps=%u\n",
               (unsigned long)caps->ac_controls.hw[0],
               (unsigned int)caps->ac_channels,
               (unsigned int)caps->ac_controls.b[2]);

        ret = es8311_setsamplerate(priv);
        if (ret == -ENOTTY)
          {
            ret = OK;
          }

        if (ret != OK)
          {
            break;
          }

        ret = es8311_setbitspersample(priv);
        if (ret == -ENOTTY)
          {
            ret = OK;
          }

        /* [V661] Bring the ADC up here, not in es8311_start().
         *
         * es8311_start() is the only place that switches the analog ADC
         * path on, and on this build it never runs for the capture
         * direction: audio.c only calls lower->ops->start() while
         * status->state is PREPARED/XRUN, and pcm0p/pcm0c share one upper
         * half (the [V647] audio_register fix), so the playback side has
         * already driven that state machine to RUNNING.
         *
         * The register set below is exactly the unconditional block from
         * es8311_start().  ES8311_GPIO_REG44 = 0x50 is the internal
         * reference (ADCL + DACR); with it missing the ADC has no
         * reference and puts out a constant small value -- which is what
         * the v658 field run measured as a constant peak=6 on every
         * capture chunk.
         */

        /* [V681] Full alignment with esp_codec_dev's es8311 start sequence.
         *
         * The v661 block only copied part of es8311_start().  Comparing
         * against the official driver (the one that demonstrably records on
         * this very board) showed three missing pieces, all of which matter
         * for the ADC:
         *
         *   0x0A (SDPOUT, "adc serial digital port"): the official
         *        set_fs() writes (& 0xBF) | 0x0C -- bit6 cleared (ADC serial
         *        output enabled) AND bits[3:2] = 11 (16-bit words) with
         *        bits[1:0] = 00 (I2S).  We only cleared bit6, so the ADC
         *        transmitted with whatever word length the reset default
         *        gave it.
         *   0x44 (GPIO, "dac2adc for test"): official writes 0x58 = the
         *        internal reference (ADCL + DACR).  We wrote 0x50, which
         *        lacks bit3 -- the mic (ADCL) side therefore had no
         *        reference and the ADC converted silence (the constant
         *        0xFFFF / peak=1 signature measured in [V380] ph=2).
         *   0x16 (ADC mic gain): official calls set_in_gain(37.5) -> the
         *        36 dB step, a lot hotter than the default.
         *
         * 0x00/0x01 are re-written too so the codec's clock scheme matches
         * a fresh open() (0x80 = slave mode / use MCLK, 0x3F = MCLK on).
         */

        {
          uint8_t v681;

          /* [V906] Software reset FIRST, word length SECOND.
           *
           * The reset used to sit below, right after the REG0A/REG09
           * writes, which meant the 16-bit / I2S word length we had just
           * programmed was written into a chip that was about to drop back
           * to its reset defaults -- and nothing after step 7 ever writes
           * REG0A's word-length field again (es8311_start() only clears
           * bit6, and per [V661] it does not run for the capture
           * direction at all).
           *
           * Hoisting the REG00 write above the SDP block keeps the reset
           * (the official driver issues it at the top of init too) and
           * lets the word length stick.  REG01 and everything below are
           * untouched.
           */
          es8311_writereg(priv, ES8311_RESET_REG00,       0x80);

          /* --- SDP: 16-bit, I2S, ADC serial output enabled --- */
          v681 = (uint8_t)((es8311_readreg(priv, ES8311_SDPOUT_REG0A) & 0xbf)
                           | 0x0c);
          es8311_writereg(priv, ES8311_SDPOUT_REG0A, v681);
          v681 = es8311_readreg(priv, ES8311_SDPIN_REG09) & 0xbf;
          es8311_writereg(priv, ES8311_SDPIN_REG09, v681);

          /* --- clock scheme --- */
          es8311_writereg(priv, ES8311_CLK_MANAGER_REG01, 0x3f);

          /* --- ADC power-up train (identical to es8311_start()) --- */
          es8311_writereg(priv, ES8311_ADC_REG17,    0xbf);
          es8311_writereg(priv, ES8311_SYSTEM_REG0E, 0x02);
          es8311_writereg(priv, ES8311_SYSTEM_REG12, 0x00);
          es8311_writereg(priv, ES8311_SYSTEM_REG14, 0x1a);
          es8311_writereg(priv, ES8311_SYSTEM_REG0D, 0x01);
          es8311_writereg(priv, ES8311_ADC_REG15,    0x40);
          es8311_writereg(priv, ES8311_GP_REG45,     0x00);

          /* [V681] *** the missing reference ***
           * 0x44 = 0x58 == internal reference signal ADCL + DACR.  With
           * 0x50 the analog ADC input had no reference at all. */
          es8311_writereg(priv, ES8311_GPIO_REG44,   0x58);

          /* [V707] Was 6, which encodes to 0x24 (step 2).  The v706 hook
           * experiments settled this -- see patch_v707.py for the table:
           *
           *   "17 00" (ADC digital gain -95 dB) changed the waveform and
           *      stopped the peak from being a constant 32768
           *      => the samples really do come out of the ES8311 ADC.
           *
           *   "16 04" (MIC PGA step 2 -> step 0) turned
           *      "8000 8000 7fff 7fff" (full-scale clip) into
           *      "fffa ff94 fef8 ff28" with peak 660..1170
           *      => step 2 clips the analog front end, and the square
           *         wave it produces is why the ASR answers "no speech"
           *         on one press and hallucinates on the next.
           *
           * v701's sweep missed this because es8311_setmicgain() encodes
           * the value as ((mic_gain * 2) << 4) | 0x04 -- the raw sweep
           * values were mangled by that shift before reaching the chip.
           *
           * [V730] That decision was taken from a "16 04" experiment run
           * while RX was still broken (RX_SLAVE_MOD = 0), so it measured
           * an arbitrary bit shift, not clipping.  With the receiver
           * fixed, the measurement in patch_v730.py shows SCALE is the
           * gain field and step 7 is worth ~15 dB without clipping.
           * Ask for the top step (42 dB -> ordinal 7 -> REG16 = 0x07). */
          /* [V979] *** 30 dB, not 42 dB: the input was clipping. ***
           *
           * Measured on v978 (capture `_v978_cap.txt`), [V718] peak while the
           * user said "现在几点":
           *
           *   193.9 s and earlier  peak = 1100 .. 8145      (room noise)
           *   194.06               peak = 32768             <- hard clip
           *   195.38               peak = 32768             <- hard clip
           *   196.38               peak = 31776             <- near clip
           *
           * 32768 is positive full scale for int16, so the waveform was being
           * flattened.  A clipped sine turns into a square wave and its
           * harmonics swamp the formants the recogniser needs; that is why the
           * same firmware transcribed "嗯" correctly but returned "是际6。" for
           * "现在几点" and "几得了。" on the next try, with perfectly healthy
           * mic levels.
           *
           * V730 asked for 42 dB (ordinal 7) off a measurement that claimed
           * step 7 did not clip.  It does.  One ordinal is 6 dB, so 30 dB is
           * exactly 12 dB down: speech that was pinned at full scale lands near
           * half scale, and the room noise floor (2000..8000) drops to
           * 500..2000, which is still well above the quantiser.
           *
           * The uplink resampler is NOT the cause -- [V978] reports
           * req=960 out=1440 ratio_x1000=1500 on every single frame. */
          es8311_setmicgain(priv, 30);

          /* [V682] Live register override, swept from NSH with no
           * reflash.  /tmp/es8311.txt holds up to 16 lines of the form
           *     AA BB
           * (register, value, both hexadecimal, two digits each); blank
           * lines and anything that is not a hex pair are skipped.  The
           * pairs are applied after the configuration above, which is the
           * same thing the official driver would be told to do.
           */
          {
            FILE *v682f = fopen("/tmp/es8311.txt", "r");

            if (v682f != NULL)
              {
                char  v682l[64];
                int   v682n = 0;

                while (v682n < 16 &&
                       fgets(v682l, sizeof(v682l), v682f) != NULL)
                  {
                    int v682a0 = es8311_v682_hex(v682l[0]);
                    int v682a1 = es8311_v682_hex(v682l[1]);
                    int v682v0 = es8311_v682_hex(v682l[3]);
                    int v682v1 = es8311_v682_hex(v682l[4]);

                    if (v682a0 < 0 || v682a1 < 0 ||
                        v682v0 < 0 || v682v1 < 0)
                      {
                        continue;
                      }

                    es8311_writereg(priv,
                                    (uint8_t)((v682a0 << 4) | v682a1),
                                    (uint8_t)((v682v0 << 4) | v682v1));
                    v682n++;
                  }

                fclose(v682f);

                if (v682n > 0)
                  {
                    V1024S(LOG_ERR, "[V682]] applied %d override pair(s) from "
                           "/tmp/es8311.txt\n", v682n);
                  }
              }
          }

          /* [V682] Dump the WHOLE configuration, not just the eight
           * registers v661/v681 checked.  The clock manager (0x00..0x08)
           * and the DAC/ramp block (0x31..0x45) are what a rate or slot
           * mismatch would show up in.
           */
          {
            char v682b[320];
            int  v682p = 0;
            int  v682i;

            for (v682i = 0; v682i <= 0x1f && v682p < 240; v682i++)
              {
                v682p += snprintf(v682b + v682p,
                                  sizeof(v682b) - (size_t)v682p,
                                  "%02x=%02x ", v682i,
                                  (unsigned)(es8311_readreg(priv,
                                             (uint8_t)v682i) & 0xff));
              }

            V1024S(LOG_ERR, "[V682]] es8311 00..1f: %s\n", v682b);

          /* [V1236] Straight-to-console snapshot of the whole analogue and
           * serial-port configuration.  [V682] goes through syslog, whose
           * channel interleaves with six other threads on this board, so its
           * multi-hundred-byte lines never survive intact; printf with the
           * console lock held does.
           */
          {
            char v1236b[320];
            int  v1236p = 0;
            int  v1236i;

            v1236p += snprintf(v1236b + v1236p,
                               sizeof(v1236b) - (size_t)v1236p,
                               "[V1236] mode=%d rate=%lu mclk=%lu gain=%d:",
                               (int)priv->audio_mode,
                               (unsigned long)priv->samprate,
                               (unsigned long)priv->mclk,
                               (int)priv->mic_gain);

            for (v1236i = 0x00; v1236i <= 0x2f &&
                 v1236p < 280; v1236i++)
              {
                if ((v1236i % 8) == 0 && v1236i != 0)
                  {
                    v1236p += snprintf(v1236b + v1236p,
                                       sizeof(v1236b) - (size_t)v1236p,
                                       "\n[V1236] %02x:", v1236i);
                  }

                v1236p += snprintf(v1236b + v1236p,
                                   sizeof(v1236b) - (size_t)v1236p,
                                   " %02x",
                                   (unsigned)(es8311_readreg(priv,
                                              (uint8_t)v1236i) & 0xff));
              }

            v1236p += snprintf(v1236b + v1236p,
                               sizeof(v1236b) - (size_t)v1236p,
                               " | 31=%02x 32=%02x 37=%02x 44=%02x 45=%02x",
                               (unsigned)(es8311_readreg(priv, 0x31) & 0xff),
                               (unsigned)(es8311_readreg(priv, 0x32) & 0xff),
                               (unsigned)(es8311_readreg(priv, 0x37) & 0xff),
                               (unsigned)(es8311_readreg(priv, 0x44) & 0xff),
                               (unsigned)(es8311_readreg(priv, 0x45) & 0xff));

            printf("%s\n", v1236b);
          }

            V1024S(LOG_ERR, "[V682]] es8311 31=%02x 32=%02x 37=%02x 44=%02x 45=%02x "
                   "mclk=%lu rate=%lu bps=%d gain=%d mode=%d\n",
                   (unsigned)(es8311_readreg(priv, 0x31) & 0xff),
                   (unsigned)(es8311_readreg(priv, 0x32) & 0xff),
                   (unsigned)(es8311_readreg(priv, 0x37) & 0xff),
                   (unsigned)(es8311_readreg(priv, 0x44) & 0xff),
                   (unsigned)(es8311_readreg(priv, 0x45) & 0xff),
                   (unsigned long)priv->mclk,
                   (unsigned long)priv->samprate,
                   (int)priv->bpsamp, (int)priv->mic_gain,
                   (int)priv->audio_mode);
          }
        }
      }
      break;

    case AUDIO_TYPE_PROCESSING:
      break;
    }

  /* [V660] Show the post-configuration codec state in /dev/kmsg.
   * es8311_dump_registers() already exists, but it prints through audinfo()
   * and CONFIG_AUDIO_INFO is not set, so it compiles away -- which is why we
   * have never actually seen the codec's ADC, PGA, mute and clock registers.
   * es8311_configure() is used instead of es8311_start() because it is the
   * one that runs for BOTH directions ([R36] "configure IN/OUT" is in every
   * boot log).
   */

  {
    static int s_v660_dump;

    if (s_v660_dump < 4)
      {
        char tb[24];

        s_v660_dump++;
        snprintf(tb, sizeof(tb), "es8311 mode=%d", (int)priv->audio_mode);
        es8311_v660_dump(priv, tb);
      }
  }

  /* [V898] One-shot full register snapshot (0x00 .. 0x22), read back
   * from the chip so it shows what the ES8311 actually kept rather
   * than what the driver believes it wrote.
   *
   * The [V682] dump only reaches 0x1f and goes through syslog(); this
   * one goes straight to printf() and adds 0x20..0x22.  Audit only --
   * nothing else is touched.
   */

  {
    static int s_v898_dumped;
    uint8_t    v898r;

    if (s_v898_dumped == 0)
      {
        s_v898_dumped = 1;

        V1024P("[V898]] es8311 dump:");

        for (v898r = 0x00; v898r <= 0x22; v898r++)
          {
            printf(" %02x=%02x", (unsigned)v898r,
                   (unsigned)(es8311_readreg(priv, v898r) & 0xff));
          }

        printf(" mclk=%lu rate=%lu bps=%d gain=%d mode=%d\n",
               (unsigned long)priv->mclk,
               (unsigned long)priv->samprate,
               (int)priv->bpsamp, (int)priv->mic_gain,
               (int)priv->audio_mode);
      }
  }

  return ret;
}

/****************************************************************************
 * Name: es8311_shutdown
 *
 * Description:
 *   Shutdown the ES8311 chip and reset it.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *
 * Returned Value:
 *   Returns OK.
 *
 ****************************************************************************/

static int es8311_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;

  DEBUGASSERT(priv);

  audinfo("Shutdown triggered\n");

  /* Now issue a software reset. This puts all ES8311 registers back in
   * their default state.
   */

  es8311_reset(priv);
  return OK;
}

/****************************************************************************
 * Name: es8311_processdone
 *
 * Description:
 *   This is the I2S callback function that is invoked when the transfer
 *   completes.
 *
 * Input Parameters:
 *   i2s - A reference to the I2S interface.
 *   apb - A reference to the audio pipeline buffer.
 *   arg - A void reference to the driver state structure.
 *   result - The result of the last transfer.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void es8311_processdone(FAR struct i2s_dev_s *i2s,
                               FAR struct ap_buffer_s *apb,
                               FAR void *arg,
                               int result)
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)arg;
  struct audio_msg_s msg;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(i2s && priv && priv->running && apb);

  DEBUGASSERT(i2s && priv && priv->running && apb);

  /* [V685] Peak accumulator for the sweep.  This callback may run in
   * interrupt context, so it only touches memory -- no syslog here.
   */

  if (result == OK && priv->audio_mode != ES_MODULE_DAC && apb->nbytes >= 2)
    {
      FAR const int16_t *v685s = (FAR const int16_t *)apb->samp;
      uint32_t v685n = (uint32_t)apb->nbytes / 2u;
      uint32_t v685i;

      for (v685i = 0; v685i < v685n; v685i++)
        {
          int32_t v685v = v685s[v685i];
          int32_t v685a = v685v < 0 ? -v685v : v685v;

          if (v685a > g_v685_step_max)
            {
              g_v685_step_max = v685a;
            }
        }

      g_v685_step_bytes += (uint32_t)apb->nbytes;
    }


  /* [V659] Rate-limited.  This fired on every DMA completion -- roughly 190
   * times per capture session, each with its own [begin] and [xfer] -- and
   * 32 KB of kmsg ring cannot hold ~600 such lines, so the ring wrapped and
   * flushed the real diagnostics.  That is why a capture that failed with
   * EIO left no trace of the reason.  First four, then every 256th. */

  {
    static unsigned s_v659_done;

    if (s_v659_done < 4u || (s_v659_done % 256u) == 0u)
      {
        syslog(LOG_INFO, "[AUDIO-R42] done apb=%p result=%d inflight=%d\n",
               apb, result, priv->inflight);
      }

    s_v659_done++;
  }

  audinfo("Transfer done: apb=%p inflight=%d result=%d\n",
          apb, priv->inflight, result);

  /* We do not place any restriction on the context in which this function
   * is called. It may be called from an interrupt handler. Therefore, the
   * doneq and in-flight values might be accessed from the interrupt level.
   * Not the best design. But we will use interrupt controls to protect
   * against that possibility.
   */

  flags = enter_critical_section();

  /* Add the completed buffer to the end of our doneq. We do not yet
   * decrement the reference count.
   */

  dq_addlast((FAR dq_entry_t *)apb, &priv->doneq);

  /* And decrement the number of buffers in-flight */

  DEBUGASSERT(priv->inflight > 0);
  priv->inflight--;

  /* Save the result of the transfer */

  priv->result = result;
  leave_critical_section(flags);

  /* Now send a message to the worker thread, informing it that there are
   * buffers in the done queue that need to be cleaned up.
   */

  msg.msg_id = AUDIO_MSG_COMPLETE;

  /* [V649-A] This is the I2S completion callback: it is asynchronous
   * with respect to the client, so it can fire after the stream has
   * been torn down.  es8311_workerthread() closes priv->mq when it
   * exits (file_mq_close + file_mq_unlink at the end of the worker),
   * and file_mq_send() then dereferences the now NULL f_inode:
   *   4216a874  l32i a8, a2, 12   ; mq->f_inode
   *   4216a877  l32i a7, a8, 28   ; f_inode->i_private  <- vaddr=0x1c
   * That is EXCCAUSE=001c, task recording_thread.  Use the same guard
   * es8311_enqueuebuffer() already has ("If mq is NULL, then the
   * playing has not yet started").
   */

  if (priv->mq.f_inode != NULL)
    {
      ret = file_mq_send(&priv->mq, (FAR const char *)&msg, sizeof(msg),
                         CONFIG_ES8311_MSG_PRIO);
      if (ret < 0)
        {
          auderr("file_mq_send failed: %d\n", ret);
        }
    }
  else
    {
      ret = OK;

      /* [V649-C] mq already gone: the worker thread has exited. */
      
    }
}

/****************************************************************************
 * Name: es8311_returnbuffers
 *
 * Description:
 *   This function is called after the completion of one or more data
 *   transfers. This function will empty the done queue and release our
 *   reference to each buffer.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void es8311_returnbuffers(FAR struct es8311_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;

  /* The doneq and in-flight values might be accessed from the interrupt
   * level in some implementations. Not the best design. But we will
   * use interrupt controls to protect against that possibility.
   */

  flags = enter_critical_section();
  while (dq_peek(&priv->doneq) != NULL)
    {
      /* Take the next buffer from the queue of completed transfers */

      apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->doneq);
      leave_critical_section(flags);

      audinfo("Returning: apb=%p curbyte=%d nbytes=%d flags=%04x\n",
              apb, apb->curbyte, apb->nbytes, apb->flags);

      /* Are we returning the final buffer in the stream? */

      if ((apb->flags & AUDIO_APB_FINAL) != 0)
        {
          /* [V698] After the v698 fix this must never fire for capture.
           * If it does, something is still flagging live RX buffers as the
           * end of the stream, and the worker is about to exit underneath
           * the capture session.
           */

          V1024S(LOG_INFO, "[V698]] FINAL latch apb=%p inflight=%d\n",
                 apb, priv->inflight);

          /* Both the pending and the done queues should be empty and there
           * should be no buffers in-flight.
           */

          DEBUGASSERT(dq_empty(&priv->doneq) && dq_empty(&priv->pendq) &&
                      priv->inflight == 0);

          /* Set the terminating flag. This will, eventually, cause the
           * worker thread to exit (if it is not already terminating).
           */

          audinfo("Terminating\n");
          priv->terminating = true;
        }

      /* Release our reference to the audio buffer */

      apb_free(apb);

      /* Send the buffer back up to the previous level. */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
      flags = enter_critical_section();
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: es8311_processbegin
 *
 * Description:
 *   Start the transfer an audio buffer to the ES8311 via I2S. This
 *   will not wait for the transfer to complete but will return immediately.
 *   the es8311_processdone called will be invoked when the transfer
 *   completes, stimulating the worker thread to call this function again.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8311_processbegin(FAR struct es8311_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;
  uint32_t timeout;
  int ret;

  /* [V754] Stop feeding the DMA as soon as a stop has been requested.
   *
   * The old code kept submitting buffers while priv->terminating was set,
   * so priv->inflight was continuously replenished and never fell to 0.
   * The worker's own (and only safe) exit test
   *     `priv->terminating && priv->inflight <= 0`
   * therefore never fired and every stop fell through to the 2000 ms
   * abandon fallback -- measured on v752 as 22 abandons per 8 minute run,
   * i.e. roughly one detached thread left behind per two sessions.
   *
   * Not feeding is also strictly safe: buffers still sitting in pendq are
   * returned by the worker teardown, and buffers still owned by the DMA
   * keep their inflight accounting intact (see H1 above).
   */

  if (priv->terminating)
    {
      return OK;
    }

  /* Loop while there are audio buffers to be sent and we have few than
   * CONFIG_ES8311_INFLIGHT then "in-flight"
   *
   * The 'inflight' value might be modified from the interrupt level in some
   * implementations. We will use interrupt controls to protect against
   * that possibility.
   *
   * The 'pendq', on the other hand, is protected via a mutex. Let's
   * hold the mutex while we are busy here and disable the interrupts
   * only while accessing 'inflight'.
   */

  ret = nxmutex_lock(&priv->pendlock);
  if (ret < 0)
    {
      return ret;
    }

  while (priv->inflight < CONFIG_ES8311_INFLIGHT &&
         dq_peek(&priv->pendq) != NULL && !priv->paused)
    {
      /* Take next buffer from the queue of pending transfers */

      apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq);
      audinfo("Transferring apb=%p, size=%d inflight=%d\n",
              apb, apb->nbytes, priv->inflight);
      
      

      /* Increment the number of buffers in-flight before sending in order
       * to avoid a possible race condition.
       */

      flags = enter_critical_section();
      priv->inflight++;
      leave_critical_section(flags);

      /* Send the entire audio buffer via I2S. What is a reasonable timeout
       * to use? This would depend on the bit rate and size of the buffer.
       *
       * Samples in the buffer (samples):
       *   = buffer_size * 8 / bpsamp
       * Sample rate (samples/second):
       *   = samplerate * nchannels
       * Expected transfer time (seconds):
       *   = (buffer_size * 8) / bpsamp / samplerate
       *
       * We will set the timeout about twice that.
       *
       * NOTES:
       * - The multiplier of 8 becomes 16000 for 2x and units of
       *   milliseconds.
       * - 16000 is a approximately 16384 (1 << 14).
       *   So this can be simplifies to (milliseconds):
       *
       *   = (buffer_size << 14) / bpsamp / samplerate
       */

      timeout = MSEC2TICK(((uint32_t)(apb->nbytes - apb->curbyte) << 14) /
                          (uint32_t)priv->samprate / (uint32_t)priv->bpsamp);
      /* [V685] Live register sweep -- see the g_v685_* comment above.
       * /tmp/es8311sweep.txt is re-read every 32 buffers, so the host can
       * change the candidate list between captures (or even mid-session)
       * without a reflash.
       */

      if (priv->audio_mode != ES_MODULE_DAC)
        {
          static unsigned s_v685_call;
          static int      s_v685_npair;
          static int      s_v685_applied = -1;
          static uint8_t  s_v685_reg[ES8311_V685_MAX_PAIRS];
          static uint8_t  s_v685_val[ES8311_V685_MAX_PAIRS];

          if ((s_v685_call % 32u) == 0u)
            {
              FILE *v685f = fopen("/tmp/es8311sweep.txt", "r");

              s_v685_npair = 0;

              if (v685f != NULL)
                {
                  char v685l[64];

                  while (s_v685_npair < ES8311_V685_MAX_PAIRS &&
                         fgets(v685l, sizeof(v685l), v685f) != NULL)
                    {
                      int v685a0 = es8311_v682_hex(v685l[0]);
                      int v685a1 = es8311_v682_hex(v685l[1]);
                      int v685v0 = es8311_v682_hex(v685l[3]);
                      int v685v1 = es8311_v682_hex(v685l[4]);

                      if (v685a0 < 0 || v685a1 < 0 ||
                          v685v0 < 0 || v685v1 < 0)
                        {
                          continue;
                        }

                      s_v685_reg[s_v685_npair] =
                        (uint8_t)((v685a0 << 4) | v685a1);
                      s_v685_val[s_v685_npair] =
                        (uint8_t)((v685v0 << 4) | v685v1);
                      s_v685_npair++;
                    }

                  fclose(v685f);
                }

              if (s_v685_npair == 0)
                {
                  /* [V685] Built-in default candidate list, used when
                   * /tmp/es8311sweep.txt is absent or unparsable.  After a
                   * reflash /tmp is empty, so the sweep runs on the very
                   * first capture with no host-side setup at all.
                   * Choosing a value == baseline is deliberate: those steps
                   * are the controls.
                   */

                  static const uint8_t v685dft[][2] =
                  {
                    /* [V702] 0x44 walk.  v700 measured prevmax=1498 with
                     * 0x44=0x00 and 32767/32768 with 0x08/0x58, and v701
                     * showed 0x16 has no effect at all, so 0x44 -- the
                     * register the V681 comment calls "internal reference
                     * signal ADCL + DACR" -- is what rails the ADC.
                     * Two passes over the plausible bit patterns. */

                    {0x44, 0x58},   /* current baseline */
                    {0x44, 0x00},   /* no reference at all */
                    {0x44, 0x08},   /* ADCL only */
                    {0x44, 0x18},
                    {0x44, 0x10},
                    {0x44, 0x40},
                    {0x44, 0x50},
                    {0x44, 0x60},
                    {0x44, 0x58},   /* pass 2 */
                    {0x44, 0x00},
                    {0x44, 0x08},
                    {0x44, 0x18},
                    {0x44, 0x10},
                    {0x44, 0x40},
                    {0x44, 0x50},
                    {0x44, 0x60}
                  };

                  int v685d;

                  for (v685d = 0; v685d < ES8311_V685_MAX_PAIRS; v685d++)
                    {
                      s_v685_reg[v685d] = v685dft[v685d][0];
                      s_v685_val[v685d] = v685dft[v685d][1];
                    }

                  s_v685_npair = ES8311_V685_MAX_PAIRS;
                }
            }

          if (s_v685_npair > 0)
            {
              int v685idx = (int)((s_v685_call / ES8311_V685_CALLS_PER_STEP)
                                  % (unsigned)s_v685_npair);

              if (v685idx != s_v685_applied)
                {
                  if (s_v685_applied >= 0)
                    {
                      V1024S(LOG_ERR, "[V685]] step %d/%d reg=%02x val=%02x | "
                             "prevmax=%d bytes=%lu\n",
                             s_v685_applied, s_v685_npair,
                             (unsigned)s_v685_reg[s_v685_applied],
                             (unsigned)s_v685_val[s_v685_applied],
                             (int)g_v685_step_max,
                             (unsigned long)g_v685_step_bytes);
                    }

                  g_v685_step_max   = 0;
                  g_v685_step_bytes = 0;
                  es8311_writereg(priv, s_v685_reg[v685idx],
                                  s_v685_val[v685idx]);
                  s_v685_applied = v685idx;

                  V1024S(LOG_ERR, "[V685]] apply %d/%d reg=%02x val=%02x\n",
                         v685idx, s_v685_npair,
                         (unsigned)s_v685_reg[v685idx],
                         (unsigned)s_v685_val[v685idx]);
                }
            }

          s_v685_call++;
        }


      /* [V659] Rate-limited, see es8311_processdone(). */

      {
        static unsigned s_v659_begin;

        if (s_v659_begin < 4u || (s_v659_begin % 256u) == 0u)
          {
            syslog(LOG_INFO, "[AUDIO-R42] begin mode=%d nmax=%d nb=%d cb=%d "
                   "to=%" PRIu32 " inflight=%d\n",
                   (int)priv->audio_mode, (int)apb->nmaxbytes,
                   (int)apb->nbytes, (int)apb->curbyte,
                   (uint32_t)timeout, (int)priv->inflight);
          }

        s_v659_begin++;
      }

      /* [V93] Hand the I2S port to whichever direction is about to run.
       * Without this the receiver waits for a transmit clock that never
       * arrives during capture.
       */

      esp32s3_i2s_set_rxonly(priv->i2s,
                             priv->audio_mode != ES_MODULE_DAC);
      g_v94_pb++;
      g_v94_rxonly = (priv->audio_mode != ES_MODULE_DAC) ? 1 : 0;

      if (priv->audio_mode == ES_MODULE_DAC)
        {
          ret = I2S_SEND(priv->i2s, apb, es8311_processdone,
                         priv, timeout);
        }
      else
        {
          ret = I2S_RECEIVE(priv->i2s, apb, es8311_processdone,
                            priv, timeout);
          g_v94_i2sret = ret;
        }

      /* [V659] Only report failures -- ret == 0 on every healthy transfer. */

      if (ret != 0)
        {
          syslog(LOG_ERR, "[AUDIO-R42] xfer ret=%d\n", ret);
        }

      if (ret < 0)
        {
          auderr("I2S transfer failed: %d\n", ret);
          break;
        }
    }

  nxmutex_unlock(&priv->pendlock);
  return ret;
}

/****************************************************************************
 * Name: es8311_wait_worker_exit
 *
 * Description:
 *   [V751] Wait, with a hard bound, for the worker thread to return.
 *
 *   The worker is created detached (see es8311_start), so pthread_join()
 *   is neither possible nor safe here.  The worker cannot leave its loop
 *   while priv->inflight > 0, and when the I2S DMA fails to deliver a
 *   completion (observed as "I2S TX completion TIMEOUT") inflight never
 *   reaches zero and a join would never return.  Before v751 that join
 *   hung es8311_stop() -> voice_channel_stop() -> the button command
 *   worker -> s_ptt_hold, which is why the UI froze on "thinking" and
 *   BOOT stopped responding.
 *
 * Input Parameters:
 *   priv       - ES8311 device state
 *   timeout_ms - Hard bound; the wait never exceeds this.
 *
 * Returned Value:
 *   OK         - the worker returned; priv->threadid has been cleared
 *   -ETIMEDOUT - the worker did not return; it was abandoned (detached).
 *                The worker notices via worker_gen / worker_stuck and
 *                leaves the message queue and the hardware alone.
 *
 ****************************************************************************/

static int es8311_wait_worker_exit(FAR struct es8311_dev_s *priv,
                                   int timeout_ms)
{
  int waited = 0;

  if (priv->threadid == 0)
    {
      return OK;
    }

  while (priv->worker_done == 0)
    {
      if (waited >= timeout_ms)
        {
          priv->worker_stuck = 1;
          priv->threadid     = 0;

          V1024S(LOG_ERR, "[V751]] es8311 worker did not exit in %d ms "
                 "(inflight=%d running=%d) -- abandoned, left detached\n",
                 waited, (int)priv->inflight, (int)priv->running);

          return -ETIMEDOUT;
        }

      nxsig_usleep(2000);           /* 2 ms, yields the CPU */
      waited += 2;
    }

  priv->worker_stuck = 0;
  priv->threadid     = 0;
  return OK;
}

/****************************************************************************
 * Name: es8311_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8311_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int es8311_start(FAR struct audio_lowerhalf_s *dev)
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;
  struct sched_param sparam;
  struct mq_attr attr;
  pthread_attr_t tattr;
  FAR void *value;
  int ret = OK;
  uint8_t dac_iface;
  uint8_t adc_iface;

  audinfo("ES8311 Start\n");

  if (priv->audio_mode == ES_MODULE_LINE)
    {
      auderr("The codec ES8311 doesn't support ES_MODULE_LINE mode");
      return -EINVAL;
    }

  if (priv->audio_mode == ES_MODULE_ADC ||
      priv->audio_mode == ES_MODULE_ADC_DAC)
    {
      adc_iface = es8311_readreg(priv, ES8311_SDPOUT_REG0A) & 0xbf;
      ret |= es8311_writereg(priv, ES8311_SDPOUT_REG0A, adc_iface);
    }

  if (priv->audio_mode == ES_MODULE_DAC ||
      priv->audio_mode == ES_MODULE_ADC_DAC)
    {
      dac_iface = es8311_readreg(priv, ES8311_SDPIN_REG09) & 0xbf;
      ret |= es8311_writereg(priv, ES8311_SDPIN_REG09,  dac_iface);
    }

  ret |= es8311_writereg(priv, ES8311_ADC_REG17,    0xbf);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG0E, 0x02);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG12, 0x00);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG14, 0x1a);

  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG0D, 0x01);
  ret |= es8311_writereg(priv, ES8311_ADC_REG15,    0x40);
  ret |= es8311_writereg(priv, ES8311_DAC_REG37,    0x08);  /* [V685] 0x48 -> 0x08, matches esp_codec_dev es8311_start() */
  ret |= es8311_writereg(priv, ES8311_GP_REG45,     0x00);

  /* Set internal reference signal (ADCL + DACR) */

  ret |= es8311_writereg(priv, ES8311_GPIO_REG44, 0x58);  /* [V681] ADCL+DACR ref */

  if (ret < 0)
    {
      auderr("Failed to start operation.\n");
      return -EIO;
    }
  else
    {
      audinfo("Operation has been started.\n");
    }

  /* [V660] Show the post-configuration codec state (ADC/DAC enable, MIC PGA,
   * mute, clock dividers) in the serial log.  mode tells us which direction
   * this instance believes it is in. */

  {
    static int s_v660_dump;

    if (s_v660_dump < 2)
      {
        char tb[24];

        s_v660_dump++;
        snprintf(tb, sizeof(tb), "es8311 mode=%d", (int)priv->audio_mode);
        es8311_v660_dump(priv, tb);
      }
  }

  /* Create a message queue for the worker thread */

  snprintf(priv->mqname, sizeof(priv->mqname), "/regconfig/%" PRIXPTR,
           (uintptr_t)priv);

  attr.mq_maxmsg  = 16;
  attr.mq_msgsize = sizeof(struct audio_msg_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  ret = file_mq_open(&priv->mq, priv->mqname,
                     O_RDWR | O_CREAT, 0644, &attr);
  if (ret < 0)
    {
      /* Error creating message queue! */

      auderr("Couldn't allocate message queue\n");
      return ret;
    }

  /* [V751] Reap any old worker thread we had created to prevent a memory
   * leak -- but never block on it.  A worker stuck on an I2S DMA that
   * never completes can no longer hang us here.
   */

  (void)value;

  if (priv->threadid != 0)
    {
      audinfo("Reaping old thread\n");
      es8311_wait_worker_exit(priv, 3000);   /* [V886] was 300 */
    }

  /* Start our thread for processing device data */

  pthread_attr_init(&tattr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 3;
  pthread_attr_setschedparam(&tattr, &sparam);
  /* [V377] 4096, not CONFIG_ES8311_WORKER_STACKSIZE (2048).
   *
   * v374/v375/v376 all died with
   *   xtensa_user_panic: User Exception: EXCCAUSE=001c task: es8311
   * and a stack dump showing 1824 of 2016 bytes used -- 192 bytes of
   * headroom -- at the point the transmitter starts while a capture is
   * already streaming.  Both es8311 workers (capture and playback) get the
   * same size, so this costs 2 x 2048 bytes of DRAM; see the matching
   * I2S_DMABUF_SLOTS change for where that memory comes from. */

  /* [V637] 4096 -> 8192.  The v636 PTT run died exactly the way v374-v376
   * did -- xtensa_user_panic: User Exception: EXCCAUSE=001c task: es8311 --
   * while a capture xfer was being armed, and the dump shows only 192 bytes
   * of the 4096-byte stack left (sp 0x3c1eeed0, top 0x3c1eef90).  The chain
   * left under the worker is
   *   es8311_workerthread -> es8311_processbegin -> I2S_RECEIVE
   *     -> esp32s3_i2s_receive -> i2s_dmabuf_alloc + per-buffer syslog()
   * and every syslog() drags vsnprintf in underneath it.  Pthread stacks are
   * carved out of the application heap (PSRAM), so the extra 4096 bytes cost
   * the internal DMA heap nothing -- which is the pool v636 just freed up. */
  pthread_attr_setstacksize(&tattr, 16384);  /* [V887] was 8192: the v887 crash is a register-window spill faulting at 0xfffffff1, i.e. a destroyed stack context, and v637 already measured only 192 B left at 4096 */

  /* [V751] Publish a fresh generation and clear the exit flag *before*
   * the thread can run.  A worker that is late (abandoned by a bounded
   * wait) compares its own generation against priv->worker_gen and then
   * leaves the message queue and the codec alone.
   */

  priv->worker_gen++;

  /* [V889] Clean single-variable experiment: IDENTICAL stack size to the
   * v887 baseline (16384), only the LOCATION changes (heap/PSRAM -> static
   * internal DRAM .bss).  See the long [V889] comment above g_audioops.
   * priv->worker_gen was just bumped, so it now holds this worker's
   * generation - the same value the worker snapshots as my_gen.
   */

  {
    int v889_ret;

    if (!g_v889_stack_taken)
      {
        g_v889_stack_taken = true;
        g_v889_stack_owner   = (void *)priv;                /* [V891B] */
        g_v889_stack_gen     = (uint32_t)priv->worker_gen;  /* [V891B] */

        /* pthread_attr_setstack() sets address and size in one call
         * (pthread_attr_setstackaddr() alone would leave the size at the
         * 4096 default and overflow the buffer).
         */

        v889_ret = pthread_attr_setstack(&tattr, g_v889_es8311_stack,
                                         ES8311_V889_STACK_SIZE);
        V1024S(LOG_ERR, "[V889]] setstack ret=%d slot=%u at=%p size=%u\n",
               v889_ret, 0u, g_v889_es8311_stack,
               (unsigned int)ES8311_V889_STACK_SIZE);
      }
    else
      {
        v889_ret = pthread_attr_setstacksize(&tattr, 16384);
        V1024S(LOG_ERR, "[V889]] slot busy, falling back to PSRAM heap stack "
               "(16384) ret=%d\n", v889_ret);
      }
  }

  priv->worker_done  = 0;
  priv->worker_stuck = 0;

  audinfo("Starting worker thread\n");
  ret = pthread_create(&priv->threadid, &tattr, es8311_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auderr("pthread_create failed: %d\n", ret);
    }
  else
    {
      pthread_setname_np(priv->threadid, "es8311");

      /* [V751] Detach at birth.  pthread_join() must never be called on
       * this thread -- see es8311_wait_worker_exit().
       */

      pthread_detach(priv->threadid);
      audinfo("Created worker thread\n");
    }

  return ret;
}

/****************************************************************************
 * Name: es8311_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8311_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int es8311_stop(FAR struct audio_lowerhalf_s *dev)
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;
  struct audio_msg_s term_msg;
  FAR void *value;
  int ret = 0;
  int msgret;
  uint8_t dac_iface;
  uint8_t adc_iface;

  audinfo("ES8311 Stop\n");

  if (priv->audio_mode == ES_MODULE_ADC ||
      priv->audio_mode == ES_MODULE_ADC_DAC)
    {
      adc_iface = es8311_readreg(priv, ES8311_SDPOUT_REG0A) | 0x40;
      ret |= es8311_writereg(priv, ES8311_SDPOUT_REG0A, adc_iface);
    }

  if (priv->audio_mode == ES_MODULE_DAC ||
      priv->audio_mode == ES_MODULE_ADC_DAC)
    {
      dac_iface = es8311_readreg(priv, ES8311_SDPIN_REG09) | 0x40;
      ret |= es8311_writereg(priv, ES8311_SDPIN_REG09,  dac_iface);
    }

  ret |= es8311_writereg(priv, ES8311_DAC_REG32,    0x00);
  ret |= es8311_writereg(priv, ES8311_ADC_REG17,    0x00);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG0E, 0xff);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG12, 0x02);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG14, 0x00);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG0D, 0xfa);
  ret |= es8311_writereg(priv, ES8311_ADC_REG15,    0x00);
  ret |= es8311_writereg(priv, ES8311_DAC_REG37,    0x08);
  ret |= es8311_writereg(priv, ES8311_GP_REG45,     0x01);

  /* Send a message to stop all audio streaming */

  term_msg.msg_id = AUDIO_MSG_STOP;
  term_msg.u.data = 0;

  /* [V649-C] Probe: how often is stop entered, and with what state?
   * The v648 panic happened in exactly this mq_send, on
   * recording_thread, right after "capture read returned -32".
   */
  {
    static int s_v649_stop_calls;

    
  }

  /* [V649-B] The worker owns priv->mq: it closes the queue and unlinks
   * the name just before it exits (see the tail of
   * es8311_workerthread).  Sending to it after that dereferences a
   * NULL f_inode and panics with EXCCAUSE=001c / vaddr=0x1c.  A second
   * stop on an already-stopped stream must therefore be harmless, and
   * pthread_join(NULL tid) must not be called either.
   */

  if (priv->threadid != 0)
    {
      /* [V751] Ask for termination directly as well as through the
       * queue: the worker re-reads this flag on every poll, even
       * when the STOP message cannot be delivered at all.
       */

      priv->terminating = true;

      if (priv->mq.f_inode != NULL)
        {
          struct timespec abstime;

          clock_gettime(CLOCK_REALTIME, &abstime);
          abstime.tv_nsec += 200 * 1000 * 1000;      /* +200 ms */
          if (abstime.tv_nsec >= 1000000000)
            {
              abstime.tv_sec  += 1;
              abstime.tv_nsec -= 1000000000;
            }

          /* [V751] Bounded send: the old blocking file_mq_send()
           * could itself hang forever when the queue was full and
           * the worker was not draining it.
           */

          msgret = file_mq_timedsend(&priv->mq,
                                     (FAR const char *)&term_msg,
                                     sizeof(term_msg),
                                     CONFIG_ES8311_MSG_PRIO,
                                     &abstime);
          if (msgret < 0)
            {
              V1024S(LOG_ERR, "[V751]] es8311_stop: STOP msg not delivered: %d\n",
                     msgret);
            }
        }
      else
        {
          /* [V751] The worker already closed its queue; it is either
           * gone or about to notice priv->terminating.
           */

          V1024S(LOG_ERR, "[V751]] es8311_stop: worker mq already closed, "
                 "relying on the terminating flag\n");
        }

      /* [V751] Wait with a hard bound instead of joining: the worker is
       * detached and must never be joined.
       */

      (void)value;
      es8311_wait_worker_exit(priv, 2000);
    }

  es8311_dump_registers(&priv->dev, "After stop");

  if (ret < 0)
    {
      auderr("Failed to stop operation.\n");
    }
  else
    {
      audinfo("Operation has been stopped.\n");
    }

  return ret;
}
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

/****************************************************************************
 * Name: es8311_pause
 *
 * Description: Pauses the playback.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8311_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int es8311_pause(FAR struct audio_lowerhalf_s *dev)
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;
  int ret = OK;

  audinfo("ES8311 Pause\n");

  if (priv->running && !priv->paused)
    {
      priv->paused = true;
#ifndef CONFIG_AUDIO_EXCLUDE_MUTE
      ret = es8311_setmute(priv, priv->audio_mode, true);
#endif
    }

  if (ret < 0)
    {
      auderr("Failed to pause operation.\n");
    }
  else
    {
      audinfo("Operation has been paused.\n");
    }

  return ret;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: es8311_resume
 *
 * Description: Resumes the playback.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8311_resume(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session)
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int es8311_resume(FAR struct audio_lowerhalf_s *dev)
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;
  int ret = OK;

  audinfo("ES8311 Resume\n");

  if (priv->running && priv->paused)
    {
      priv->paused = false;
#ifndef CONFIG_AUDIO_EXCLUDE_MUTE
      ret = es8311_setmute(priv, priv->audio_mode, false);
#endif /* CONFIG_AUDIO_EXCLUDE_MUTE */
      es8311_processbegin(priv);
    }

  if (ret < 0)
    {
      auderr("Failed to resume operation.\n");
    }
  else
    {
      audinfo("Operation has been resumed.\n");
    }

  return ret;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: es8311_enqueuebuffer
 *
 * Description:
 *   Enqueue an Audio Pipeline Buffer for processing.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   apb - A reference to the audio pipeline buffer.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8311_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb)
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;
  struct audio_msg_s term_msg;
  int ret;

  audinfo("Enqueueing: apb=%p curbyte=%d nbytes=%d flags=%04x\n",
          apb, apb->curbyte, apb->nbytes, apb->flags);

  ret = nxmutex_lock(&priv->pendlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Take a reference on the new audio buffer */

  apb_reference(apb);

  /* Add the new buffer to the tail of pending audio buffers */

  dq_addlast(&apb->dq_entry, &priv->pendq);
  nxmutex_unlock(&priv->pendlock);

  /* Send a message to the worker thread indicating that a new buffer has
   * been enqueued. If mq is NULL, then the playing has not yet started.
   * In that case we are just "priming the pump" and we don't need to send
   * any message.
   */

  ret = OK;
  if (priv->mq.f_inode != NULL)
    {
      term_msg.msg_id  = AUDIO_MSG_ENQUEUE;
      term_msg.u.data = 0;

      ret = file_mq_send(&priv->mq, (FAR const char *)&term_msg,
                         sizeof(term_msg), CONFIG_ES8311_MSG_PRIO);
      if (ret < 0)
        {
          auderr("file_mq_send failed: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: es8311_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   apb - A reference to the audio pipeline buffer.
 *
 * Returned Value:
 *   Returns OK.
 *
 ****************************************************************************/

static int es8311_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                               FAR struct ap_buffer_s *apb)
{
  audinfo("Cancelled apb=%p\n", apb);
  return OK;
}

/****************************************************************************
 * Name: es8311_ioctl
 *
 * Description: Perform a device IOCTL.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   cmd - The IOCTL command.
 *   arg - The argument of the IOCTL command.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8311_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                        unsigned long arg)
{
  int ret = OK;
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  FAR struct ap_buffer_info_s *bufinfo;
#endif /* CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS */

  /* Deal with ioctls passed from the upper-half driver */

  switch (cmd)
    {
      /* Report our preferred buffer size and quantity */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
      case AUDIOIOC_GETBUFFERINFO:
        {
          audinfo("AUDIOIOC_GETBUFFERINFO:\n");
          bufinfo              = (FAR struct ap_buffer_info_s *) arg;
          bufinfo->buffer_size = CONFIG_ES8311_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_ES8311_NUM_BUFFERS;
        }
        break;
#endif /* CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS */

      default:
        ret = -ENOTTY;
        audinfo("Unhandled ioctl: %d\n", cmd);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: es8311_reserve
 *
 * Description: Reserves a session (the only one we have).
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8311_reserve(FAR struct audio_lowerhalf_s *dev,
                          FAR void **session)
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int es8311_reserve(FAR struct audio_lowerhalf_s *dev)
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *) dev;
  int ret = OK;

  audinfo("ES8311 Reserve\n");

  /* Borrow the APBQ semaphore for thread sync */

  ret = nxmutex_lock(&priv->pendlock);
  if (ret < 0)
    {
      return ret;
    }

  /* [V755] *** Take the TX queues back before starting a session. ***
   *
   * es8311 has no way to reclaim containers once a playback has been
   * abandoned: the worker is left detached, priv->inflight is reset to
   * zero just below without anything ever handing the buffers back, and
   * the I2S layer keeps them in priv->tx.act / priv->tx.pend for ever.
   * Measured steady state: txa=1 txp=3, i.e. all four containers of
   * CONFIG_ESP32S3_I2S_MAXINFLIGHT, which starves every later capture
   * (i2s_buf_allocate() returns NULL -> "capture EIO" -> the UI hangs
   * on "Start listening").
   *
   * reserve() runs before any enqueue for this session, so everything
   * still sitting in the TX queues at this point is debris from the
   * previous one -- never data the caller has just handed us.  Whoever
   * submitted the buffers is responsible for taking them back, and this
   * is the only point where that can still be done safely.
   *
   * esp32s3_i2s_tx_flush() returns without touching a register when all
   * three TX queues are already empty, so the healthy path is a no-op. */

  esp32s3_i2s_tx_flush();

  if (priv->reserved)
    {
      ret = -EBUSY;
    }
  else
    {
      /* Initialize the session context */

#ifdef CONFIG_AUDIO_MULTI_SESSION
     *session           = NULL;
#endif /* CONFIG_AUDIO_MULTI_SESSION */
      priv->inflight    = 0;
      priv->running     = false;
      priv->paused      = false;
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
      priv->terminating = false;
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */
      priv->reserved    = true;
    }

  nxmutex_unlock(&priv->pendlock);
  return ret;
}

/****************************************************************************
 * Name: es8311_release
 *
 * Description: Releases the session (the only one we have).
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8311_release(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session)
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int es8311_release(FAR struct audio_lowerhalf_s *dev)
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;
  FAR void *value;
  int ret;

  audinfo("ES8311 Release\n");

  /* [V751] Detached worker: never join it.  Ask it to terminate and
   * wait with a hard bound so release cannot hang either.
   */

  (void)value;

  if (priv->threadid != 0)
    {
      priv->terminating = true;
      es8311_wait_worker_exit(priv, 2000);
    }

  /* Borrow the APBQ semaphore for thread sync */

  ret = nxmutex_lock(&priv->pendlock);

  /* Really we should free any queued buffers here */

  priv->reserved = false;
  nxmutex_unlock(&priv->pendlock);

  return ret;
}

/****************************************************************************
 * Name: es8311_audio_output
 *
 * Description:
 *   Initialize and configure the ES8311 device as an audio output device.
 *   This will be mostly used when adding input support.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   None. No failures are detected.
 *
 ****************************************************************************/

/* [V660] Dump the codec's register file to /dev/kmsg.
 *
 * es8311_dump_registers() already exists, but it prints through audinfo()
 * and CONFIG_AUDIO_INFO is not set on this build, so the whole call compiles
 * away -- which is why we have never actually seen what the codec's ADC,
 * PGA, mute and clock registers contain.  Rate-limited: two dumps are enough
 * (the first start of each direction).
 */

static void es8311_v660_dump(FAR struct es8311_dev_s *priv, const char *tag)
{
  char line[32];
  int i;
  int j;
  int off;

  for (i = 0; i < 0x20; i += 8)
    {
      off = 0;
      for (j = 0; j < 8; j++)
        {
          int v = es8311_readreg(priv, (uint8_t)(i + j));
          off += snprintf(line + off, sizeof(line) - (size_t)off, "%02x ",
                          (unsigned)(v & 0xff));
        }

      syslog(LOG_INFO, "[%s] [V660] reg%02x: %s\n", tag, i, line);
    }
}

static void es8311_audio_output(FAR struct es8311_dev_s *priv)
{
  audinfo("ES8311 set to output mode\n");

  priv->audio_mode = ES_MODULE_DAC;
}

/****************************************************************************
 * Name: es8311_audio_input
 *
 * Description:
 *   Initialize and configure the ES8311 device as an audio input device.
 *   This will be mostly used when adding input support.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   None. No failures are detected.
 *
 ****************************************************************************/

static void es8311_audio_input(FAR struct es8311_dev_s *priv)
{
  audinfo("ES8311 set to input mode\n");

  priv->audio_mode = ES_MODULE_ADC;
}

/****************************************************************************
 * Name: es8311_workerthread
 *
 *  This is the thread that feeds data to the chip and keeps the audio
 *  stream going.
 *
 * Input Parameters:
 *   pvarg - The thread arguments.
 *
 * Returned Value:
 *   Returns NULL.
 *
 ****************************************************************************/

/* [V643] PROBE: direct ROM console, does not go through /dev/kmsg */

static void *es8311_workerthread(pthread_addr_t pvarg)
{
  FAR struct es8311_dev_s *priv = (struct es8311_dev_s *) pvarg;
  struct audio_msg_s msg;
  FAR struct ap_buffer_s *apb;
  irqstate_t wflags;
  int my_gen;

  /* [V751c] volatile: a plain local let gcc's value-range propagation
   * prove the drain-bound branch dead and delete it from the object file.
   */

  volatile int term_timeouts;
  int msglen;
  unsigned int prio;

  /* [V643] PROBE */
  

  audinfo("ES8311 worker thread starting\n");

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  priv->terminating = false;
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

  /* [V751] Snapshot the generation we were created under and arm the
   * termination watchdog.
   */

  my_gen        = priv->worker_gen;
  term_timeouts = 0;

  priv->running = true;

  /* Loop as long as we are supposed to be running and as long as we have
   * buffers in-flight.
   */

  while (priv->running || priv->inflight > 0)
    {
      /* [V751] Superseded: a newer worker has been created for this
       * device (our owner gave up on us after a bounded wait).  Leave
       * immediately and without touching any shared state.
       */

      if (my_gen != priv->worker_gen)
        {
          break;
        }

      /* Check if we have been asked to terminate. We have to check if we
       * still have buffers in-flight. If we do, then we can't stop until
       * birds come back to roost.
       */

      if (priv->terminating && priv->inflight <= 0)
        {
          /* We are IDLE. Break out of the loop and exit. */

          break;
        }

      /* [V754] There is deliberately NO "force priv->inflight = 0" bound
       * here any more.
       *
       * v753 added one (a 1200 ms wall clock) and it panicked on the very
       * first hit: EXCCAUSE=001d StoreProhibited, vaddr=0x6e, inside
       * audio_capture_abort() reached from button_cmd_worker ->
       * button_ptt_release_commit -> voice_channel_stop.
       *
       * priv->inflight is the "how many apb's does the DMA still owe us"
       * account.  Zeroing it does not abandon the wait, it falsifies the
       * account: the worker then runs its full teardown (drain pendq +
       * apb_free + es8311_returnbuffers + file_mq_close/unlink +
       * AUDIO_CALLBACK_COMPLETE) while those apb's still belong to the
       * I2S DMA.  When the DMA completion finally arrives,
       * es8311_processdone() decrements a counter that is already 0
       * (DEBUGASSERT is compiled out here) and frees/re-enqueues buffers
       * that were already released -- heap corruption, and the next
       * audio_capture_abort() walks a dead structure.
       *
       * The safe fallback is the one already in place: this worker is
       * detached, so es8311_stop() abandons it after its bounded wait
       * (worker_stuck + threadid = 0) and the worker then notices via
       * worker_gen / worker_stuck and leaves *without touching any shared
       * state*.  Measured on v752: 22 abandons in 8 minutes, zero panics.
       *
       * term_timeouts is still counted below, for observation only.
       */

      /* Check if we can process more audio buffers */

      es8311_processbegin(priv);

      /* Wait for messages from our message queue.
       *
       * [V751] The wait is bounded so that a terminating worker
       * re-evaluates its exit conditions even when no completion ever
       * arrives.
       */

      {
        struct timespec abstime;

        clock_gettime(CLOCK_REALTIME, &abstime);
        abstime.tv_nsec += 200 * 1000 * 1000;           /* +200 ms */
        if (abstime.tv_nsec >= 1000000000)
          {
            abstime.tv_sec  += 1;
            abstime.tv_nsec -= 1000000000;
          }

        msglen = file_mq_timedreceive(&priv->mq, (FAR char *)&msg,
                                      sizeof(msg), &prio, &abstime);
      }

      /* Handle the case when we return with no message */

      if (msglen < sizeof(struct audio_msg_s))
        {
          if (msglen == -ETIMEDOUT)
            {
              /* Nothing arrived.  Count every timeout unconditionally:
               * the v751 build gated this on priv->terminating and gcc
               * then proved the counter unreachable and deleted the
               * whole drain-bound block.
               */

              term_timeouts++;
            }
          else
            {
              auderr("Message too small: %d\n", msglen);
            }

          continue;
        }

      /* [V751] A real message arrived -- the drain is alive again. */

      term_timeouts = 0;

      /* Process the message */

      switch (msg.msg_id)
        {
          /* The ISR has requested more data. We will catch this case at
           * the top of the loop.
           */

          case AUDIO_MSG_DATA_REQUEST:
            audinfo("AUDIO_MSG_DATA_REQUEST\n");
            break;

          /* Stop the playback */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
          case AUDIO_MSG_STOP:

            /* Indicate that we are terminating */

            audinfo("AUDIO_MSG_STOP: Terminating\n");
            priv->terminating = true;
            break;
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

          /* We have a new buffer to process. We will catch this case at
           * the top of the loop.
           */

          case AUDIO_MSG_ENQUEUE:
            audinfo("AUDIO_MSG_ENQUEUE\n");
            break;

          /* We will wake up from the I2S callback with this message */

          case AUDIO_MSG_COMPLETE:
            audinfo("AUDIO_MSG_COMPLETE\n");
            es8311_returnbuffers(priv);
            break;

          default:
            auderr("Ignoring message ID %d\n", msg.msg_id);
            break;
        }
    }

  /* [V751] A worker that has been superseded or abandoned by a bounded
   * wait must not touch the shared state any more: a newer worker may
   * already own priv->mq / priv->mqname and the codec.
   */

  if (my_gen != priv->worker_gen || priv->worker_stuck)
    {
      V1024S(LOG_ERR, "[V751]] es8311 worker gen %d exits late (gen now %d, "
             "stuck=%d) -- skipping mq/hw teardown\n",
             my_gen, priv->worker_gen, priv->worker_stuck);
      /* [V889] Same release as the normal exit path: this worker leaves
       * early, but it still owns the slot it was started on.
       */

      /* [V891B] Both halves of the ownership pair must match: priv so
       * the other es8311 instance cannot free this slot, gen so an
       * abandoned worker of this same instance cannot free a slot that
       * a newer worker has already taken.
       */

      if (g_v889_stack_taken &&
          g_v889_stack_owner == (void *)priv &&
          g_v889_stack_gen == (uint32_t)my_gen)
        {
          g_v889_stack_taken = false;
          V1024S(LOG_ERR, "[V889]] late worker gen %d released stack\n",
                 my_gen);
        }

      return NULL;
    }

  /* Reset the ES8311 hardware */

  es8311_reset(priv);

  /* Return any pending buffers in our pending queue */

  nxmutex_lock(&priv->pendlock);
  while ((apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq)) != NULL)
    {
      /* Release our reference to the buffer */

      apb_free(apb);

      /* Send the buffer back up to the previous level. */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
    }

  nxmutex_unlock(&priv->pendlock);

  /* Return any pending buffers in our done queue */

  es8311_returnbuffers(priv);

  /* Close the message queue */

  file_mq_close(&priv->mq);
  file_mq_unlink(priv->mqname);

  /* Send an AUDIO_MSG_COMPLETE message to the client */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */

  audinfo("ES8311 worker thread finishing\n");

  /* [V751] Publish the exit flag before returning: our owner waits on
   * this flag, never on pthread_join().
   */

  priv->worker_done = 1;

  /* [V889] Release the pinned internal-DRAM stack.  The generation guard
   * keeps an abandoned worker from clearing a slot that a newer worker has
   * already taken.  Placed after worker_done so it is the last thing this
   * thread does before returning.
   */

  /* [V891B] Same (priv, gen) pair test as the early-exit path above. */

  if (g_v889_stack_taken &&
      g_v889_stack_owner == (void *)priv &&
      g_v889_stack_gen == (uint32_t)my_gen)
    {
      g_v889_stack_taken = false;
      V1024S(LOG_ERR, "[V889]] worker gen %d released pinned stack\n",
             my_gen);
    }

  return NULL;
}

/****************************************************************************
 * Name: es8311_reset
 *
 * Description:
 *   Reset and re-initialize the ES8311.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void es8311_reset(FAR struct es8311_dev_s *priv)
{
  uint8_t regconfig;

  /* Put audio back to its initial configuration */

  audinfo("ES8311 reset triggered.\n");
  priv->samprate   = ES8311_DEFAULT_SAMPRATE;
  priv->bpsamp     = ES8311_DEFAULT_BPSAMP;

  /* Software reset. This puts all ES8311 registers back in their
   * default state.
   */

  es8311_writereg(priv, ES8311_CLK_MANAGER_REG01, 0x30);
  es8311_writereg(priv, ES8311_CLK_MANAGER_REG02, 0x00);
  es8311_writereg(priv, ES8311_CLK_MANAGER_REG03, 0x10);
  es8311_writereg(priv, ES8311_ADC_REG16,         0x24);
  es8311_writereg(priv, ES8311_CLK_MANAGER_REG04, 0x10);
  es8311_writereg(priv, ES8311_CLK_MANAGER_REG05, 0x00);
  es8311_writereg(priv, ES8311_SYSTEM_REG0B,      0x00);
  es8311_writereg(priv, ES8311_SYSTEM_REG0C,      0x00);
  es8311_writereg(priv, ES8311_SYSTEM_REG10,      0x1f);
  es8311_writereg(priv, ES8311_SYSTEM_REG11,      0x7f);
  es8311_writereg(priv, ES8311_RESET_REG00,       0x80);

  audinfo("ES8311 in Slave mode.\n");
  regconfig = es8311_readreg(priv, ES8311_RESET_REG00);
  regconfig &= 0xbf;
  es8311_writereg(priv, ES8311_RESET_REG00, regconfig);
  es8311_writereg(priv, ES8311_CLK_MANAGER_REG01, 0x3f);

  if (es8311_get_mclk_src() == ES8311_MCLK_FROM_MCLK_PIN)
    {
      regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG01);
      regconfig &= 0x7f;
      es8311_writereg(priv, ES8311_CLK_MANAGER_REG01, regconfig);
    }
  else
    {
      regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG01);
      regconfig |= 0x80;
      es8311_writereg(priv, ES8311_CLK_MANAGER_REG01, regconfig);
    }

  es8311_setsamplerate(priv);
  es8311_setbitspersample(priv);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG01);
  regconfig &= ~(0x40);
  es8311_writereg(priv, ES8311_CLK_MANAGER_REG01, regconfig);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG06);
  regconfig &= ~(0x20);
  es8311_writereg(priv, ES8311_CLK_MANAGER_REG06, regconfig);
  es8311_writereg(priv, ES8311_SYSTEM_REG13, 0x10);
  es8311_writereg(priv, ES8311_ADC_REG1B, 0x0a);
  es8311_writereg(priv, ES8311_ADC_REG1C, 0x6a);
  es8311_setvolume(priv, ES_MODULE_ADC, CONFIG_ES8311_INPUT_INITVOLUME);
  es8311_setvolume(priv, ES_MODULE_DAC, CONFIG_ES8311_OUTPUT_INITVOLUME);

  es8311_dump_registers(&priv->dev, "After reset");
  audinfo("ES8311 reset complete.\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* [V690] The single ES8311 on this board, so the application can reach it. */
static struct es8311_dev_s *g_es8311_priv;

/* [V690] Output volume control for the application.
 *
 * The xiaozhi board drives its volume keys through esp_codec_dev's
 * set_out_vol() -> ES8311 DAC_REG32.  The NuttX port has no equivalent path:
 * there is no AUDIOIOC_SETVOLUME (nuttx/audio/audio.h stops at
 * _AUDIOIOC(25)), AUDIO_FU_VOLUME only travels with an AUDIOIOC_CONFIGURE
 * that the es8311 lower half applies as
 * es8311_setvolume(priv, priv->audio_mode, volume) -- and audio_mode is
 * ES_MODULE_ADC while recording, so that ioctl would move the ADC digital
 * gain instead of the speaker.  The playback device is also opened per
 * session, so a button thread has no fd to talk to.
 *
 * CONFIG_BUILD_FLAT=y, so the ai_agent application links directly against
 * this symbol and can set the DAC volume at any time, including while idle.
 * volume is the AUDIO_FU_VOLUME scale (0..1000), 0 = mute.
 */

/* [V811] Mute or unmute the DAC without touching the rest of the codec.
 *
 * The speaker carries the right words and a hiss underneath, and everything on
 * the digital side checks out: RX registers 0x2f3de38f on all 146 captures,
 * RX data OR 0x0000ffff (the driver's own classifier calls that good), and the
 * TX stream dumped by v810 is an ordinary signed waveform.  I2C reports no
 * errors and v248's five register fixes are all in place.
 *
 * None of that separates a hiss generated inside the DAC from one generated
 * after it -- the amplifier, the wiring, the supply.  At the speaker the two are
 * indistinguishable, and everything measured so far lives upstream of the split.
 *
 * Reg 0x31 bit 6 is the DAC mute, so this function is the whole experiment: with
 * the stream still running and only the DAC muted, a hiss that persists cannot
 * have come from anything upstream.
 *
 * Deliberately minimal: one register, no state machine, no side effects, so the
 * comparison against an unmuted run is like for like. */
int es8311_ext_setmute(int mute);

int es8311_ext_setmute(int mute)
{
  FAR struct es8311_dev_s *priv = g_es8311_priv;

  if (priv == NULL)
    {
      V1024S(LOG_ERR, "[V811]] setmute(%d) dropped: codec not up yet\n",
             mute);
      return -ENODEV;
    }

  es8311_writereg(priv, ES8311_DAC_REG31, mute ? 0x40 : 0x00);

  V1024S(LOG_ERR, "[V811]] DAC mute=%d -> reg31=0x%02x (readback 0x%02x)\n",
         mute ? 1 : 0,
         (unsigned)(mute ? 0x40 : 0x00),
         (unsigned)(es8311_readreg(priv, ES8311_DAC_REG31) & 0xff));
  return OK;
}

void es8311_ext_setvolume(uint16_t volume)
{
  FAR struct es8311_dev_s *priv = g_es8311_priv;

  if (priv == NULL)
    {
      V1024S(LOG_ERR, "[V690]] volume %.3f dropped: codec not up yet\n",
             (double)volume);
      return;
    }

  if (volume > 1000)
    {
      volume = 1000;
    }

  es8311_setvolume(priv, ES_MODULE_DAC, volume);

  V1024S(LOG_INFO, "[V690]] volume -> %u (%s)\n",
         (unsigned)volume, volume == 0 ? "muted" : "on");
}

/****************************************************************************
 * Name: es8311_initialize
 *
 * Description:
 *   Initialize the ES8311 device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance.
 *   i2s     - An I2S driver instance.
 *   lower   - Persistent board configuration data.
 *
 * Returned Value:
 *   A new lower half audio interface for the ES8311 device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *
  es8311_initialize(FAR struct i2c_master_s *i2c,
                    FAR struct i2s_dev_s *i2s,
                    FAR const struct es8311_lower_s *lower)
{
  FAR struct es8311_dev_s *priv;

  audinfo("Initializing ES8311\n");

  /* Sanity check */

  DEBUGASSERT(i2c && i2s && lower);

  /* Allocate a ES8311 device structure */

  priv = kmm_zalloc(sizeof(struct es8311_dev_s));
  g_es8311_priv = priv;   /* [V690] single codec instance on this board */

  if (priv)
    {
      priv->dev.ops    = &g_audioops;
      priv->lower      = lower;
      priv->i2c        = i2c;
      priv->i2s        = i2s;

      nxmutex_init(&priv->pendlock);
      dq_init(&priv->pendq);
      dq_init(&priv->doneq);

      audinfo("ES8311: address=%02x frequency=%" PRId32 "\n",
              lower->address, lower->frequency);

      /* Reset and reconfigure the ES8311 hardware */

      es8311_dump_registers(&priv->dev, "Before reset");

      es8311_audio_output(priv);
      es8311_reset(priv);
      audinfo("ES8311 initialized.\n");
      return &priv->dev;
    }

  return NULL;
}
