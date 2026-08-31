/****************************************************************************
 * drivers/video/sc2336.c
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

/* Driver for the SmartSens SC2336 2MP MIPI CSI-2 image sensor.
 *
 * The sensor outputs RAW8 Bayer (BGGR) data on a 2-lane MIPI CSI-2 link.
 * On targets such as the ESP32-P4 the CSI receiver converts the RAW
 * stream to RGB565 inline, so this driver advertises RGB565 to the video
 * framework; the V4L2 layer never sees the RAW Bayer data.
 *
 * The camera module is self-clocked (24 MHz oscillator on the module) and
 * permanently powered; there are no XCLK, reset or power-down GPIOs.
 * Control is via SCCB (I2C compatible) at the fixed 7-bit address 0x30.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/param.h>
#include <sys/videoio.h>

#include <nuttx/clock.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/sc2336.h>

#include "sc2336_tables.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SC2336_I2C_ADDR          0x30    /* 7-bit SCCB slave address */
#define SC2336_I2C_FREQ          100000  /* SCCB frequency [Hz] */

/* Chip ID: registers 0x3107(H)/0x3108(L) */

#define SC2336_CHIP_ID           0xcb3a

/* The I2C bus is shared with a GT911 touch controller which has a known
 * first-transfer NACK quirk, so probe the chip ID a few times before
 * concluding that the sensor is absent.
 */

#define SC2336_PROBE_RETRIES     3
#define SC2336_PROBE_DELAY_USEC  (10 * USEC_PER_MSEC)

/* Settle time after software reset (register 0x0103) */

#define SC2336_RESET_DELAY_USEC  (5 * USEC_PER_MSEC)

/* Register 0x3221: horizontal mirror is bits [2:1], vertical flip is
 * bits [6:5].
 */

#define SC2336_MIRROR_MASK       0x06
#define SC2336_VFLIP_MASK        0x60

#define SC2336_FRAME_FPS         30

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sc2336_mode_s
{
  uint16_t width;                     /* Frame width [pixel] */
  uint16_t height;                    /* Frame height [pixel] */
  uint16_t fps;                       /* Frame rate [frame/sec] */
  uint8_t  data_lanes;                /* Number of CSI-2 data lanes */
  uint16_t lane_rate_mbps;            /* CSI-2 link rate per lane [Mbps] */
  FAR const sc2336_reginfo_t *regs;   /* Mode register table */
};

struct sc2336_dev_s
{
  struct imgsensor_s sensor;          /* Image sensor interface */
  mutex_t lock;                       /* Serializes I2C access and state */
  FAR struct i2c_master_s *i2c;       /* I2C master instance */
  struct i2c_config_s i2c_cfg;        /* I2C address/frequency */

  /* Currently loaded mode, NULL if none */

  FAR const struct sc2336_mode_s *cur_mode;

  bool streaming;                     /* Sensor is streaming */
  bool hflip;                         /* Horizontal mirror enabled */
  bool vflip;                         /* Vertical flip enabled */
};

typedef struct sc2336_dev_s sc2336_dev_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sc2336_i2c_write8(FAR sc2336_dev_t *priv, uint16_t regaddr,
                             uint8_t value);
static int sc2336_i2c_read8(FAR sc2336_dev_t *priv, uint16_t regaddr,
                            FAR uint8_t *value);
static int sc2336_write_reglist(FAR sc2336_dev_t *priv,
                                FAR const sc2336_reginfo_t *list);
static int sc2336_read_chipid(FAR sc2336_dev_t *priv,
                              FAR uint16_t *chipid);
static FAR const struct sc2336_mode_s *
sc2336_find_mode(uint16_t width, uint16_t height);
static int sc2336_set_stream(FAR sc2336_dev_t *priv, bool on);
static int sc2336_apply_flip(FAR sc2336_dev_t *priv);
static int sc2336_load_mode(FAR sc2336_dev_t *priv,
                            FAR const struct sc2336_mode_s *mode);
static int sc2336_check_frame_setting(imgsensor_stream_type_t type,
                                      uint8_t nr_datafmts,
                                      FAR imgsensor_format_t *datafmts,
                                      FAR imgsensor_interval_t *interval,
                              FAR const struct sc2336_mode_s **mode);

/* Image sensor operations */

static bool sc2336_is_available(FAR struct imgsensor_s *sensor);
static int sc2336_init(FAR struct imgsensor_s *sensor);
static int sc2336_uninit(FAR struct imgsensor_s *sensor);
static FAR const char *
sc2336_get_driver_name(FAR struct imgsensor_s *sensor);
static int sc2336_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval);
static int sc2336_start_capture(FAR struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval);
static int sc2336_stop_capture(FAR struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type);
static int sc2336_get_frame_interval(FAR struct imgsensor_s *sensor,
                                     imgsensor_stream_type_t type,
                                     FAR imgsensor_interval_t *interval);
static int sc2336_get_supported_value(FAR struct imgsensor_s *sensor,
                                      uint32_t id,
                                    FAR imgsensor_supported_value_t *value);
static int sc2336_get_value(FAR struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            FAR imgsensor_value_t *value);
static int sc2336_set_value(FAR struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Supported sensor modes.  Entry 0 is the default mode. */

static const struct sc2336_mode_s g_sc2336_modes[] =
{
  {
    .width          = 1024,
    .height         = 600,
    .fps            = SC2336_FRAME_FPS,
    .data_lanes     = 2,
    .lane_rate_mbps = 288,
    .regs           = g_sc2336_1024x600_raw8_30fps,
  },
  {
    .width          = 1280,
    .height         = 720,
    .fps            = SC2336_FRAME_FPS,
    .data_lanes     = 2,
    .lane_rate_mbps = 336,
    .regs           = g_sc2336_1280x720_raw8_30fps,
  },
};

/* Format/size/interval enumerations advertised to the V4L2 layer.  The
 * first entries define the initial frame setting.
 */

static const struct v4l2_fmtdesc g_sc2336_fmtdescs[] =
{
  {
    .index       = 0,
    .type        = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .flags       = 0,
    .description = "RGB565 (CSI inline conversion)",
    .pixelformat = V4L2_PIX_FMT_RGB565,
  },
};

static const struct v4l2_frmsizeenum g_sc2336_frmsizes[] =
{
  {
    .index        = 0,
    .buf_type     = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .pixel_format = V4L2_PIX_FMT_RGB565,
    .type         = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete     =
    {
      .width      = 1024,
      .height     = 600,
    },
  },
  {
    .index        = 1,
    .buf_type     = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .pixel_format = V4L2_PIX_FMT_RGB565,
    .type         = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete     =
    {
      .width      = 1280,
      .height     = 720,
    },
  },
};

static const struct v4l2_frmivalenum g_sc2336_frmintervals[] =
{
  {
    .index        = 0,
    .buf_type     = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .pixel_format = V4L2_PIX_FMT_RGB565,
    .width        = 1024,
    .height       = 600,
    .type         = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete     =
    {
      .numerator   = 1,
      .denominator = SC2336_FRAME_FPS,
    },
  },
  {
    .index        = 1,
    .buf_type     = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .pixel_format = V4L2_PIX_FMT_RGB565,
    .width        = 1280,
    .height       = 720,
    .type         = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete     =
    {
      .numerator   = 1,
      .denominator = SC2336_FRAME_FPS,
    },
  },
};

static const struct imgsensor_ops_s g_sc2336_ops =
{
  .is_available           = sc2336_is_available,
  .init                   = sc2336_init,
  .uninit                 = sc2336_uninit,
  .get_driver_name        = sc2336_get_driver_name,
  .validate_frame_setting = sc2336_validate_frame_setting,
  .start_capture          = sc2336_start_capture,
  .stop_capture           = sc2336_stop_capture,
  .get_frame_interval     = sc2336_get_frame_interval,
  .get_supported_value    = sc2336_get_supported_value,
  .get_value              = sc2336_get_value,
  .set_value              = sc2336_set_value,
};

static sc2336_dev_t g_sc2336_private =
{
  .sensor           =
  {
    .ops              = &g_sc2336_ops,
    .fmtdescs_num     = nitems(g_sc2336_fmtdescs),
    .fmtdescs         = g_sc2336_fmtdescs,
    .frmsizes_num     = nitems(g_sc2336_frmsizes),
    .frmsizes         = g_sc2336_frmsizes,
    .frmintervals_num = nitems(g_sc2336_frmintervals),
    .frmintervals     = g_sc2336_frmintervals,
  },
  .lock             = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sc2336_i2c_write8
 *
 * Description:
 *   Write an 8-bit value to a 16-bit sensor register.  The register
 *   address is transferred big-endian, followed by the value, in a single
 *   I2C write transaction.
 *
 ****************************************************************************/

static int sc2336_i2c_write8(FAR sc2336_dev_t *priv, uint16_t regaddr,
                             uint8_t value)
{
  uint8_t buf[3];
  uint8_t readback;
  int tries;
  int ret;

  buf[0] = (uint8_t)(regaddr >> 8);
  buf[1] = (uint8_t)(regaddr & 0xff);
  buf[2] = value;

  for (tries = 0; tries < 3; tries++)
    {
      ret = i2c_write(priv->i2c, &priv->i2c_cfg, buf, sizeof(buf));
      if (ret >= 0)
        {
          return ret;
        }

      /* The controller can report a timeout for a write the sensor did
       * receive and apply.  Read the register back: if it already holds
       * the requested value the write succeeded on the wire and failing
       * the whole operation (and tearing down a live capture pipeline)
       * over it would be wrong.  Self-clearing registers never match and
       * simply fall through to the retry.
       */

      if (i2c_writeread(priv->i2c, &priv->i2c_cfg, buf, 2,
                        &readback, 1) >= 0 && readback == value)
        {
          vwarn("i2c_write reg=0x%04x timed out but took effect\n",
                regaddr);
          return OK;
        }
    }

  verr("i2c_write reg=0x%04x failed: %d\n", regaddr, ret);
  return ret;
}

/****************************************************************************
 * Name: sc2336_i2c_read8
 *
 * Description:
 *   Read an 8-bit value from a 16-bit sensor register using a write of
 *   the big-endian register address followed by a repeated-start read.
 *
 ****************************************************************************/

static int sc2336_i2c_read8(FAR sc2336_dev_t *priv, uint16_t regaddr,
                            FAR uint8_t *value)
{
  uint8_t addr[2];
  int ret;

  addr[0] = (uint8_t)(regaddr >> 8);
  addr[1] = (uint8_t)(regaddr & 0xff);

  ret = i2c_writeread(priv->i2c, &priv->i2c_cfg, addr, sizeof(addr),
                      value, 1);
  if (ret < 0)
    {
      verr("i2c_writeread reg=0x%04x failed: %d\n", regaddr, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sc2336_write_reglist
 *
 * Description:
 *   Write a register table terminated by SC2336_REG_END.  Entries with
 *   the address SC2336_REG_DELAY request a delay whose duration in
 *   milliseconds is given by the value field.  A settle delay is inserted
 *   after any write to the software reset register since the tables do
 *   not carry an explicit delay entry after it.
 *
 ****************************************************************************/

static int sc2336_write_reglist(FAR sc2336_dev_t *priv,
                                FAR const sc2336_reginfo_t *list)
{
  int ret;
  int i;

  for (i = 0; list[i].reg != SC2336_REG_END; i++)
    {
      if (list[i].reg == SC2336_REG_DELAY)
        {
          nxsig_usleep(list[i].val * USEC_PER_MSEC);
          continue;
        }

      ret = sc2336_i2c_write8(priv, list[i].reg, list[i].val);
      if (ret < 0)
        {
          verr("Register table entry %d (0x%04x) failed: %d\n",
               i, list[i].reg, ret);
          return ret;
        }

      if (list[i].reg == SC2336_REG_SW_RESET)
        {
          nxsig_usleep(SC2336_RESET_DELAY_USEC);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sc2336_read_chipid
 *
 * Description:
 *   Read the 16-bit sensor ID from registers 0x3107/0x3108.
 *
 ****************************************************************************/

static int sc2336_read_chipid(FAR sc2336_dev_t *priv, FAR uint16_t *chipid)
{
  uint8_t id_h;
  uint8_t id_l;
  int ret;

  ret = sc2336_i2c_read8(priv, SC2336_REG_SENSOR_ID_H, &id_h);
  if (ret < 0)
    {
      return ret;
    }

  ret = sc2336_i2c_read8(priv, SC2336_REG_SENSOR_ID_L, &id_l);
  if (ret < 0)
    {
      return ret;
    }

  *chipid = (uint16_t)((id_h << 8) | id_l);
  return OK;
}

/****************************************************************************
 * Name: sc2336_find_mode
 *
 * Description:
 *   Find the sensor mode with the requested frame size.
 *
 ****************************************************************************/

static FAR const struct sc2336_mode_s *
sc2336_find_mode(uint16_t width, uint16_t height)
{
  size_t i;

  for (i = 0; i < nitems(g_sc2336_modes); i++)
    {
      if (g_sc2336_modes[i].width == width &&
          g_sc2336_modes[i].height == height)
        {
          return &g_sc2336_modes[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: sc2336_set_stream
 *
 * Description:
 *   Start or stop the sensor output stream.
 *
 ****************************************************************************/

static int sc2336_set_stream(FAR sc2336_dev_t *priv, bool on)
{
  int ret;

  ret = sc2336_i2c_write8(priv, SC2336_REG_SLEEP_MODE, on ? 0x01 : 0x00);
  if (ret >= 0)
    {
      priv->streaming = on;
    }

  return ret;
}

/****************************************************************************
 * Name: sc2336_apply_flip
 *
 * Description:
 *   Apply the cached mirror/flip settings to the flip register with a
 *   read-modify-write sequence.
 *
 ****************************************************************************/

static int sc2336_apply_flip(FAR sc2336_dev_t *priv)
{
  uint8_t regval;
  int ret;

  ret = sc2336_i2c_read8(priv, SC2336_REG_FLIP_MIRROR, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~(SC2336_MIRROR_MASK | SC2336_VFLIP_MASK);
  if (priv->hflip)
    {
      regval |= SC2336_MIRROR_MASK;
    }

  if (priv->vflip)
    {
      regval |= SC2336_VFLIP_MASK;
    }

  return sc2336_i2c_write8(priv, SC2336_REG_FLIP_MIRROR, regval);
}

/****************************************************************************
 * Name: sc2336_load_mode
 *
 * Description:
 *   Load the register table of the given mode.  Each table begins with a
 *   software reset and leaves the sensor with the stream stopped, so the
 *   cached flip settings are re-applied afterwards.
 *
 ****************************************************************************/

static int sc2336_load_mode(FAR sc2336_dev_t *priv,
                            FAR const struct sc2336_mode_s *mode)
{
  int ret;

  ret = sc2336_write_reglist(priv, mode->regs);
  if (ret < 0)
    {
      priv->cur_mode = NULL;
      return ret;
    }

  ret = sc2336_apply_flip(priv);
  if (ret < 0)
    {
      priv->cur_mode = NULL;
      return ret;
    }

  /* Stretch VTS to halve the frame rate (30 -> 15 fps) and, more
   * importantly, widen vertical blanking to ~46 ms.  The capture DMA
   * is re-armed by a 10 ms poll; with the default 13 ms blanking the
   * re-arm regularly lands inside the next frame's active data, which
   * costs frames to the bridge FIFO discard/resync path.  With this
   * blanking every re-arm fits, so every frame is captured and the
   * DMA blocks stay locked to frame boundaries.
   */

  ret = sc2336_i2c_write8(priv, 0x320e, 0x07);
  if (ret >= 0)
    {
      ret = sc2336_i2c_write8(priv, 0x320f, 0xd0);
    }

  if (ret < 0)
    {
      priv->cur_mode = NULL;
      return ret;
    }

  priv->cur_mode  = mode;
  priv->streaming = false;
  return OK;
}

/****************************************************************************
 * Name: sc2336_check_frame_setting
 *
 * Description:
 *   Verify that the requested frame setting matches a supported sensor
 *   mode.  On success the matching mode is returned via the mode
 *   argument.
 *
 ****************************************************************************/

static int sc2336_check_frame_setting(imgsensor_stream_type_t type,
                                      uint8_t nr_datafmts,
                                      FAR imgsensor_format_t *datafmts,
                                      FAR imgsensor_interval_t *interval,
                               FAR const struct sc2336_mode_s **mode)
{
  FAR const struct sc2336_mode_s *found;

  if (type != IMGSENSOR_STREAM_TYPE_VIDEO)
    {
      return -ENOTSUP;
    }

  if (nr_datafmts == 0 || datafmts == NULL)
    {
      return -EINVAL;
    }

  if (nr_datafmts > 1)
    {
      /* Sub-image (thumbnail) streams are not supported */

      return -ENOTSUP;
    }

  if (datafmts[IMGSENSOR_FMT_MAIN].pixelformat != IMGSENSOR_PIX_FMT_RGB565)
    {
      return -ENOTSUP;
    }

  found = sc2336_find_mode(datafmts[IMGSENSOR_FMT_MAIN].width,
                           datafmts[IMGSENSOR_FMT_MAIN].height);
  if (found == NULL)
    {
      return -ENOTSUP;
    }

  if (interval != NULL)
    {
      if (interval->numerator == 0 || interval->denominator == 0)
        {
          return -EINVAL;
        }

      if (interval->denominator != found->fps * interval->numerator)
        {
          return -ENOTSUP;
        }
    }

  if (mode != NULL)
    {
      *mode = found;
    }

  return OK;
}

/****************************************************************************
 * Name: sc2336_is_available
 *
 * Description:
 *   Probe the sensor by reading and checking its chip ID.  The probe is
 *   retried a few times because the shared I2C bus is known to NACK the
 *   very first transfer occasionally.
 *
 ****************************************************************************/

static bool sc2336_is_available(FAR struct imgsensor_s *sensor)
{
  FAR sc2336_dev_t *priv = (FAR sc2336_dev_t *)sensor;
  uint16_t chipid = 0;
  int ret = -ENODEV;
  int i;

  if (priv->i2c == NULL)
    {
      return false;
    }

  nxmutex_lock(&priv->lock);

  for (i = 0; i < SC2336_PROBE_RETRIES; i++)
    {
      ret = sc2336_read_chipid(priv, &chipid);
      if (ret >= 0)
        {
          break;
        }

      nxsig_usleep(SC2336_PROBE_DELAY_USEC);
    }

  nxmutex_unlock(&priv->lock);

  if (ret < 0)
    {
      verr("SC2336 not accessible: %d\n", ret);
      return false;
    }

  if (chipid != SC2336_CHIP_ID)
    {
      verr("Unexpected chip ID 0x%04x (expected 0x%04x)\n",
           chipid, SC2336_CHIP_ID);
      return false;
    }

  vinfo("SC2336 detected, chip ID 0x%04x\n", chipid);
  return true;
}

/****************************************************************************
 * Name: sc2336_init
 *
 * Description:
 *   Initialize the sensor: software reset and load the default mode
 *   register table.  The stream is left stopped; it is started by
 *   start_capture().
 *
 ****************************************************************************/

static int sc2336_init(FAR struct imgsensor_s *sensor)
{
  FAR sc2336_dev_t *priv = (FAR sc2336_dev_t *)sensor;
  int ret;

  if (priv->i2c == NULL)
    {
      return -ENODEV;
    }

  nxmutex_lock(&priv->lock);

  /* The mode table begins with a software reset entry; a settle delay is
   * inserted after it by sc2336_write_reglist().  Retry once: the shared
   * I2C bus is known to NACK the very first transfer occasionally, and
   * reloading the table restarts from the software reset, so a retry is
   * always safe.
   */

  ret = sc2336_load_mode(priv, &g_sc2336_modes[0]);
  if (ret < 0)
    {
      ret = sc2336_load_mode(priv, &g_sc2336_modes[0]);
    }

  nxmutex_unlock(&priv->lock);

  if (ret < 0)
    {
      verr("Failed to initialize SC2336: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sc2336_uninit
 *
 * Description:
 *   Stop the sensor output stream.
 *
 ****************************************************************************/

static int sc2336_uninit(FAR struct imgsensor_s *sensor)
{
  FAR sc2336_dev_t *priv = (FAR sc2336_dev_t *)sensor;
  int ret;

  if (priv->i2c == NULL)
    {
      return -ENODEV;
    }

  nxmutex_lock(&priv->lock);
  ret = sc2336_set_stream(priv, false);
  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: sc2336_get_driver_name
 ****************************************************************************/

static FAR const char *
sc2336_get_driver_name(FAR struct imgsensor_s *sensor)
{
  UNUSED(sensor);
  return "SC2336";
}

/****************************************************************************
 * Name: sc2336_validate_frame_setting
 ****************************************************************************/

static int sc2336_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval)
{
  UNUSED(sensor);

  return sc2336_check_frame_setting(type, nr_datafmts, datafmts,
                                    interval, NULL);
}

/****************************************************************************
 * Name: sc2336_start_capture
 *
 * Description:
 *   Verify the requested frame setting, load the matching mode register
 *   table if it is not the currently loaded one, and start streaming.
 *
 ****************************************************************************/

static int sc2336_start_capture(FAR struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval)
{
  FAR sc2336_dev_t *priv = (FAR sc2336_dev_t *)sensor;
  FAR const struct sc2336_mode_s *mode;
  int ret;

  ret = sc2336_check_frame_setting(type, nr_datafmts, datafmts,
                                   interval, &mode);
  if (ret < 0)
    {
      return ret;
    }

  nxmutex_lock(&priv->lock);

  if (priv->cur_mode != mode)
    {
      ret = sc2336_load_mode(priv, mode);
      if (ret < 0)
        {
          nxmutex_unlock(&priv->lock);
          return ret;
        }
    }

  ret = sc2336_set_stream(priv, true);
  nxmutex_unlock(&priv->lock);

  if (ret < 0)
    {
      verr("Failed to start capture: %d\n", ret);
      return ret;
    }

  vinfo("Streaming %ux%u@%u started\n",
        mode->width, mode->height, mode->fps);

  return OK;
}

/****************************************************************************
 * Name: sc2336_stop_capture
 ****************************************************************************/

static int sc2336_stop_capture(FAR struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type)
{
  FAR sc2336_dev_t *priv = (FAR sc2336_dev_t *)sensor;
  int ret;

  if (type != IMGSENSOR_STREAM_TYPE_VIDEO)
    {
      return -ENOTSUP;
    }

  nxmutex_lock(&priv->lock);
  ret = sc2336_set_stream(priv, false);
  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: sc2336_get_frame_interval
 ****************************************************************************/

static int sc2336_get_frame_interval(FAR struct imgsensor_s *sensor,
                                     imgsensor_stream_type_t type,
                                     FAR imgsensor_interval_t *interval)
{
  UNUSED(sensor);

  /* Note: the type argument is not checked because the V4L2 layer passes
   * the raw v4l2_buf_type here (unlike start/stop_capture, it is not
   * converted to imgsensor_stream_type_t).  All supported modes run at a
   * fixed 30 fps in any case.
   */

  UNUSED(type);

  if (interval == NULL)
    {
      return -EINVAL;
    }

  interval->numerator   = 1;
  interval->denominator = SC2336_FRAME_FPS;
  return OK;
}

/****************************************************************************
 * Name: sc2336_get_supported_value
 *
 * Description:
 *   Report the supported range of a camera parameter.  Only the
 *   mirror/flip controls are supported by this driver.
 *
 ****************************************************************************/

static int sc2336_get_supported_value(FAR struct imgsensor_s *sensor,
                                      uint32_t id,
                                    FAR imgsensor_supported_value_t *value)
{
  UNUSED(sensor);

  if (value == NULL)
    {
      return -EINVAL;
    }

  switch (id)
    {
      case IMGSENSOR_ID_HFLIP_VIDEO:
      case IMGSENSOR_ID_VFLIP_VIDEO:
      case IMGSENSOR_ID_HFLIP_STILL:
      case IMGSENSOR_ID_VFLIP_STILL:
        value->type                    = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        value->u.range.minimum         = 0;
        value->u.range.maximum         = 1;
        value->u.range.step            = 1;
        value->u.range.default_value   = 0;
        return OK;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: sc2336_get_value
 ****************************************************************************/

static int sc2336_get_value(FAR struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            FAR imgsensor_value_t *value)
{
  FAR sc2336_dev_t *priv = (FAR sc2336_dev_t *)sensor;

  UNUSED(size);

  if (value == NULL)
    {
      return -EINVAL;
    }

  switch (id)
    {
      case IMGSENSOR_ID_HFLIP_VIDEO:
      case IMGSENSOR_ID_HFLIP_STILL:
        value->value32 = priv->hflip;
        return OK;

      case IMGSENSOR_ID_VFLIP_VIDEO:
      case IMGSENSOR_ID_VFLIP_STILL:
        value->value32 = priv->vflip;
        return OK;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: sc2336_set_value
 ****************************************************************************/

static int sc2336_set_value(FAR struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t value)
{
  FAR sc2336_dev_t *priv = (FAR sc2336_dev_t *)sensor;
  bool enable = (value.value32 != 0);
  int ret;

  UNUSED(size);

  switch (id)
    {
      case IMGSENSOR_ID_HFLIP_VIDEO:
      case IMGSENSOR_ID_HFLIP_STILL:
        nxmutex_lock(&priv->lock);
        priv->hflip = enable;
        ret = sc2336_apply_flip(priv);
        nxmutex_unlock(&priv->lock);
        return ret;

      case IMGSENSOR_ID_VFLIP_VIDEO:
      case IMGSENSOR_ID_VFLIP_STILL:
        nxmutex_lock(&priv->lock);
        priv->vflip = enable;
        ret = sc2336_apply_flip(priv);
        nxmutex_unlock(&priv->lock);
        return ret;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sc2336_initialize
 *
 * Description:
 *   Initialize the SC2336 image sensor driver instance.
 *
 * Input Parameters:
 *   i2c - An I2C master instance for the bus the sensor is wired to.
 *
 * Returned Value:
 *   A non-NULL image sensor instance on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct imgsensor_s *sc2336_initialize(FAR struct i2c_master_s *i2c)
{
  FAR sc2336_dev_t *priv = &g_sc2336_private;

  if (i2c == NULL)
    {
      return NULL;
    }

  priv->i2c               = i2c;
  priv->i2c_cfg.frequency = SC2336_I2C_FREQ;
  priv->i2c_cfg.address   = SC2336_I2C_ADDR;
  priv->i2c_cfg.addrlen   = 7;
  priv->cur_mode          = NULL;
  priv->streaming         = false;

  return &priv->sensor;
}

/****************************************************************************
 * Name: sc2336_get_mode_info
 *
 * Description:
 *   Look up the MIPI CSI-2 link parameters of the sensor mode with the
 *   given frame size.
 *
 * Input Parameters:
 *   width          - Frame width in pixels.
 *   height         - Frame height in pixels.
 *   data_lanes     - The number of CSI-2 data lanes is returned here.
 *                    May be NULL.
 *   lane_rate_mbps - The per-lane link rate in Mbps is returned here.
 *                    May be NULL.
 *
 * Returned Value:
 *   Zero (OK) on success; -EINVAL if no mode matches the frame size.
 *
 ****************************************************************************/

int sc2336_get_mode_info(uint16_t width, uint16_t height,
                         FAR uint8_t *data_lanes,
                         FAR uint16_t *lane_rate_mbps)
{
  FAR const struct sc2336_mode_s *mode;

  mode = sc2336_find_mode(width, height);
  if (mode == NULL)
    {
      return -EINVAL;
    }

  if (data_lanes != NULL)
    {
      *data_lanes = mode->data_lanes;
    }

  if (lane_rate_mbps != NULL)
    {
      *lane_rate_mbps = mode->lane_rate_mbps;
    }

  return OK;
}

/****************************************************************************
 * 3A support: exposure and gain hooks for the ESP32-P4 CSI pipeline
 *
 * The ISP-side auto-exposure / auto-white-balance loop (esp_mipi_csi.c)
 * drives the sensor through these entry points, referenced as weak
 * symbols so neither side hard-depends on the other.
 ****************************************************************************/

/* Total gain (x1000) ladder and matching register triplets, from the
 * official esp_cam_sensor sc2336 driver (analog-gain-priority table):
 * {dgain_fine (0x3e07), dgain_coarse (0x3e06), analog gain (0x3e09)}.
 */

static const uint32_t g_sc2336_gain_x1000[] =
{
  1000, 1031, 1063, 1094, 1125, 1156, 1188, 1219,
  1250, 1281, 1313, 1344, 1375, 1406, 1438, 1469,
  1500, 1531, 1563, 1594, 1625, 1656, 1688, 1719,
  1750, 1781, 1813, 1844, 1875, 1906, 1938, 1969,
  2000, 2062, 2126, 2188, 2250, 2312, 2376, 2438,
  2500, 2562, 2626, 2688, 2750, 2812, 2876, 2938,
  3000, 3062, 3126, 3188, 3250, 3312, 3376, 3438,
  3500, 3562, 3626, 3688, 3750, 3812, 3876, 3938,
  4000, 4124, 4252, 4376, 4500, 4624, 4752, 4876,
  5000, 5124, 5252, 5376, 5500, 5624, 5752, 5876,
  6000, 6124, 6252, 6376, 6500, 6624, 6752, 6876,
  7000, 7124, 7252, 7376, 7500, 7624, 7752, 7876,
  8000, 8248, 8504, 8752, 9000, 9248, 9504, 9752,
  10000, 10248, 10504, 10752, 11000, 11248, 11504, 11752,
  12000, 12248, 12504, 12752, 13000, 13248, 13504, 13752,
  14000, 14248, 14504, 14752, 15000, 15248, 15504, 15752,
  16000, 16496, 17008, 17504, 18000, 18496, 19008, 19504,
  20000, 20496, 21008, 21504, 22000, 22496, 23008, 23504,
  24000, 24496, 25008, 25504, 26000, 26496, 27008, 27504,
  28000, 28496, 29008, 29504, 30000, 30496, 31008, 31504,
  32000, 32992, 34016, 35008, 36000, 36992, 38016, 39008,
  40000, 40992, 42016, 43008, 44000, 44992, 46016, 47008,
  48000, 48992, 50016, 51008, 52000, 52992, 54016, 55008,
  56000, 56992, 58016, 59008, 60000, 60992, 62016, 63008,
};

static const uint8_t g_sc2336_gain_regs[][3] =
{
  {0x80, 0x00, 0x00}, {0x84, 0x00, 0x00}, {0x88, 0x00, 0x00}, {0x8c, 0x00, 0x00},
  {0x90, 0x00, 0x00}, {0x94, 0x00, 0x00}, {0x98, 0x00, 0x00}, {0x9c, 0x00, 0x00},
  {0xa0, 0x00, 0x00}, {0xa4, 0x00, 0x00}, {0xa8, 0x00, 0x00}, {0xac, 0x00, 0x00},
  {0xb0, 0x00, 0x00}, {0xb4, 0x00, 0x00}, {0xb8, 0x00, 0x00}, {0xbc, 0x00, 0x00},
  {0xc0, 0x00, 0x00}, {0xc4, 0x00, 0x00}, {0xc8, 0x00, 0x00}, {0xcc, 0x00, 0x00},
  {0xd0, 0x00, 0x00}, {0xd4, 0x00, 0x00}, {0xd8, 0x00, 0x00}, {0xdc, 0x00, 0x00},
  {0xe0, 0x00, 0x00}, {0xe4, 0x00, 0x00}, {0xe8, 0x00, 0x00}, {0xec, 0x00, 0x00},
  {0xf0, 0x00, 0x00}, {0xf4, 0x00, 0x00}, {0xf8, 0x00, 0x00}, {0xfc, 0x00, 0x00},
  {0x80, 0x00, 0x08}, {0x84, 0x00, 0x08}, {0x88, 0x00, 0x08}, {0x8c, 0x00, 0x08},
  {0x90, 0x00, 0x08}, {0x94, 0x00, 0x08}, {0x98, 0x00, 0x08}, {0x9c, 0x00, 0x08},
  {0xa0, 0x00, 0x08}, {0xa4, 0x00, 0x08}, {0xa8, 0x00, 0x08}, {0xac, 0x00, 0x08},
  {0xb0, 0x00, 0x08}, {0xb4, 0x00, 0x08}, {0xb8, 0x00, 0x08}, {0xbc, 0x00, 0x08},
  {0xc0, 0x00, 0x08}, {0xc4, 0x00, 0x08}, {0xc8, 0x00, 0x08}, {0xcc, 0x00, 0x08},
  {0xd0, 0x00, 0x08}, {0xd4, 0x00, 0x08}, {0xd8, 0x00, 0x08}, {0xdc, 0x00, 0x08},
  {0xe0, 0x00, 0x08}, {0xe4, 0x00, 0x08}, {0xe8, 0x00, 0x08}, {0xec, 0x00, 0x08},
  {0xf0, 0x00, 0x08}, {0xf4, 0x00, 0x08}, {0xf8, 0x00, 0x08}, {0xfc, 0x00, 0x08},
  {0x80, 0x00, 0x09}, {0x84, 0x00, 0x09}, {0x88, 0x00, 0x09}, {0x8c, 0x00, 0x09},
  {0x90, 0x00, 0x09}, {0x94, 0x00, 0x09}, {0x98, 0x00, 0x09}, {0x9c, 0x00, 0x09},
  {0xa0, 0x00, 0x09}, {0xa4, 0x00, 0x09}, {0xa8, 0x00, 0x09}, {0xac, 0x00, 0x09},
  {0xb0, 0x00, 0x09}, {0xb4, 0x00, 0x09}, {0xb8, 0x00, 0x09}, {0xbc, 0x00, 0x09},
  {0xc0, 0x00, 0x09}, {0xc4, 0x00, 0x09}, {0xc8, 0x00, 0x09}, {0xcc, 0x00, 0x09},
  {0xd0, 0x00, 0x09}, {0xd4, 0x00, 0x09}, {0xd8, 0x00, 0x09}, {0xdc, 0x00, 0x09},
  {0xe0, 0x00, 0x09}, {0xe4, 0x00, 0x09}, {0xe8, 0x00, 0x09}, {0xec, 0x00, 0x09},
  {0xf0, 0x00, 0x09}, {0xf4, 0x00, 0x09}, {0xf8, 0x00, 0x09}, {0xfc, 0x00, 0x09},
  {0x80, 0x00, 0x0b}, {0x84, 0x00, 0x0b}, {0x88, 0x00, 0x0b}, {0x8c, 0x00, 0x0b},
  {0x90, 0x00, 0x0b}, {0x94, 0x00, 0x0b}, {0x98, 0x00, 0x0b}, {0x9c, 0x00, 0x0b},
  {0xa0, 0x00, 0x0b}, {0xa4, 0x00, 0x0b}, {0xa8, 0x00, 0x0b}, {0xac, 0x00, 0x0b},
  {0xb0, 0x00, 0x0b}, {0xb4, 0x00, 0x0b}, {0xb8, 0x00, 0x0b}, {0xbc, 0x00, 0x0b},
  {0xc0, 0x00, 0x0b}, {0xc4, 0x00, 0x0b}, {0xc8, 0x00, 0x0b}, {0xcc, 0x00, 0x0b},
  {0xd0, 0x00, 0x0b}, {0xd4, 0x00, 0x0b}, {0xd8, 0x00, 0x0b}, {0xdc, 0x00, 0x0b},
  {0xe0, 0x00, 0x0b}, {0xe4, 0x00, 0x0b}, {0xe8, 0x00, 0x0b}, {0xec, 0x00, 0x0b},
  {0xf0, 0x00, 0x0b}, {0xf4, 0x00, 0x0b}, {0xf8, 0x00, 0x0b}, {0xfc, 0x00, 0x0b},
  {0x80, 0x00, 0x0f}, {0x84, 0x00, 0x0f}, {0x88, 0x00, 0x0f}, {0x8c, 0x00, 0x0f},
  {0x90, 0x00, 0x0f}, {0x94, 0x00, 0x0f}, {0x98, 0x00, 0x0f}, {0x9c, 0x00, 0x0f},
  {0xa0, 0x00, 0x0f}, {0xa4, 0x00, 0x0f}, {0xa8, 0x00, 0x0f}, {0xac, 0x00, 0x0f},
  {0xb0, 0x00, 0x0f}, {0xb4, 0x00, 0x0f}, {0xb8, 0x00, 0x0f}, {0xbc, 0x00, 0x0f},
  {0xc0, 0x00, 0x0f}, {0xc4, 0x00, 0x0f}, {0xc8, 0x00, 0x0f}, {0xcc, 0x00, 0x0f},
  {0xd0, 0x00, 0x0f}, {0xd4, 0x00, 0x0f}, {0xd8, 0x00, 0x0f}, {0xdc, 0x00, 0x0f},
  {0xe0, 0x00, 0x0f}, {0xe4, 0x00, 0x0f}, {0xe8, 0x00, 0x0f}, {0xec, 0x00, 0x0f},
  {0xf0, 0x00, 0x0f}, {0xf4, 0x00, 0x0f}, {0xf8, 0x00, 0x0f}, {0xfc, 0x00, 0x0f},
  {0x80, 0x00, 0x1f}, {0x84, 0x00, 0x1f}, {0x88, 0x00, 0x1f}, {0x8c, 0x00, 0x1f},
  {0x90, 0x00, 0x1f}, {0x94, 0x00, 0x1f}, {0x98, 0x00, 0x1f}, {0x9c, 0x00, 0x1f},
  {0xa0, 0x00, 0x1f}, {0xa4, 0x00, 0x1f}, {0xa8, 0x00, 0x1f}, {0xac, 0x00, 0x1f},
  {0xb0, 0x00, 0x1f}, {0xb4, 0x00, 0x1f}, {0xb8, 0x00, 0x1f}, {0xbc, 0x00, 0x1f},
  {0xc0, 0x00, 0x1f}, {0xc4, 0x00, 0x1f}, {0xc8, 0x00, 0x1f}, {0xcc, 0x00, 0x1f},
  {0xd0, 0x00, 0x1f}, {0xd4, 0x00, 0x1f}, {0xd8, 0x00, 0x1f}, {0xdc, 0x00, 0x1f},
  {0xe0, 0x00, 0x1f}, {0xe4, 0x00, 0x1f}, {0xe8, 0x00, 0x1f}, {0xec, 0x00, 0x1f},
  {0xf0, 0x00, 0x1f}, {0xf4, 0x00, 0x1f}, {0xf8, 0x00, 0x1f}, {0xfc, 0x00, 0x1f},
};


#define SC2336_3A_GAIN_COUNT  (sizeof(g_sc2336_gain_x1000) / \
                               sizeof(g_sc2336_gain_x1000[0]))

/* VTS is stretched to 2000 in sc2336_load_mode; max exposure = VTS - 6 */

#define SC2336_3A_EXP_MIN     8
#define SC2336_3A_EXP_MAX     (2000 - 6)

int sc2336_3a_gain_count(void)
{
  return SC2336_3A_GAIN_COUNT;
}

uint32_t sc2336_3a_gain_x1000(int idx)
{
  if (idx < 0 || idx >= (int)SC2336_3A_GAIN_COUNT)
    {
      return 0;
    }

  return g_sc2336_gain_x1000[idx];
}

/****************************************************************************
 * Name: sc2336_3a_set_exposure
 *
 * Description:
 *   Set the exposure time in sensor lines.  The three shutter registers
 *   hold the value left-shifted by four (the low nibble of 0x3e02 is a
 *   fractional part, kept at zero).
 *
 ****************************************************************************/

int sc2336_3a_set_exposure(uint32_t lines)
{
  FAR sc2336_dev_t *priv = &g_sc2336_private;
  int ret;

  if (priv->i2c == NULL || !priv->streaming)
    {
      return -ENODEV;
    }

  if (lines < SC2336_3A_EXP_MIN)
    {
      lines = SC2336_3A_EXP_MIN;
    }

  if (lines > SC2336_3A_EXP_MAX)
    {
      lines = SC2336_3A_EXP_MAX;
    }

  nxmutex_lock(&priv->lock);
  ret = sc2336_i2c_write8(priv, 0x3e00, (lines >> 12) & 0x0f);
  if (ret >= 0)
    {
      ret = sc2336_i2c_write8(priv, 0x3e01, (lines >> 4) & 0xff);
    }

  if (ret >= 0)
    {
      ret = sc2336_i2c_write8(priv, 0x3e02, (lines & 0x0f) << 4);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: sc2336_3a_set_gain_index
 *
 * Description:
 *   Apply one entry of the total-gain ladder (digital fine, digital
 *   coarse, analog).  Returns the applied (clamped) index.
 *
 ****************************************************************************/

int sc2336_3a_set_gain_index(int idx)
{
  FAR sc2336_dev_t *priv = &g_sc2336_private;
  int ret;

  if (priv->i2c == NULL || !priv->streaming)
    {
      return -ENODEV;
    }

  if (idx < 0)
    {
      idx = 0;
    }

  if (idx >= (int)SC2336_3A_GAIN_COUNT)
    {
      idx = SC2336_3A_GAIN_COUNT - 1;
    }

  nxmutex_lock(&priv->lock);
  ret = sc2336_i2c_write8(priv, 0x3e07, g_sc2336_gain_regs[idx][0]);
  if (ret >= 0)
    {
      ret = sc2336_i2c_write8(priv, 0x3e06, g_sc2336_gain_regs[idx][1]);
    }

  if (ret >= 0)
    {
      ret = sc2336_i2c_write8(priv, 0x3e09, g_sc2336_gain_regs[idx][2]);
    }

  nxmutex_unlock(&priv->lock);
  return ret < 0 ? ret : idx;
}
