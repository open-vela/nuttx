/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-korvo-2/src/esp32s3_board_camera.c
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

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <sys/param.h>
#include <sys/videoio.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/ov3660.h>
#include <nuttx/video/v4l2_cap.h>

#include "esp32s3_camera.h"
#include "esp32s3_i2c.h"
#include "esp32s3-korvo-2.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OV3660_WIDTH   160
#define OV3660_HEIGHT  120

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct korvo2_camera_s
{
  struct imgsensor_s sensor;
  FAR struct i2c_master_s *i2c;
  bool initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool korvo2_camera_is_available(FAR struct imgsensor_s *sensor);
static int korvo2_camera_init(FAR struct imgsensor_s *sensor);
static int korvo2_camera_uninit(FAR struct imgsensor_s *sensor);
static FAR const char *korvo2_camera_get_driver_name(
  FAR struct imgsensor_s *sensor);
static int korvo2_camera_validate(FAR struct imgsensor_s *sensor,
                                  imgsensor_stream_type_t type,
                                  uint8_t nr_datafmts,
                                  FAR imgsensor_format_t *datafmts,
                                  FAR imgsensor_interval_t *interval);
static int korvo2_camera_start(FAR struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type,
                               uint8_t nr_datafmts,
                               FAR imgsensor_format_t *datafmts,
                               FAR imgsensor_interval_t *interval);
static int korvo2_camera_stop(FAR struct imgsensor_s *sensor,
                              imgsensor_stream_type_t type);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imgsensor_ops_s g_korvo2_camera_ops =
{
  .is_available           = korvo2_camera_is_available,
  .init                   = korvo2_camera_init,
  .uninit                 = korvo2_camera_uninit,
  .get_driver_name        = korvo2_camera_get_driver_name,
  .validate_frame_setting = korvo2_camera_validate,
  .start_capture          = korvo2_camera_start,
  .stop_capture           = korvo2_camera_stop,
};

static const struct v4l2_fmtdesc g_korvo2_camera_formats[] =
{
  {
    .pixelformat = V4L2_PIX_FMT_RGB565,
    .description = "RGB565",
  },
};

static const struct v4l2_frmsizeenum g_korvo2_camera_sizes[] =
{
  {
    .pixel_format = V4L2_PIX_FMT_RGB565,
    .type = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete =
      {
        .width = OV3660_WIDTH,
        .height = OV3660_HEIGHT,
      },
  },
};

static const struct v4l2_frmivalenum g_korvo2_camera_intervals[] =
{
  {
    .pixel_format = V4L2_PIX_FMT_RGB565,
    .width = OV3660_WIDTH,
    .height = OV3660_HEIGHT,
    .type = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete =
      {
        .numerator = 1,
        .denominator = 15,
      },
  },
};

static struct korvo2_camera_s g_korvo2_camera =
{
  .sensor =
    {
      .ops = &g_korvo2_camera_ops,
      .fmtdescs_num = nitems(g_korvo2_camera_formats),
      .fmtdescs = g_korvo2_camera_formats,
      .frmsizes_num = nitems(g_korvo2_camera_sizes),
      .frmsizes = g_korvo2_camera_sizes,
      .frmintervals_num = nitems(g_korvo2_camera_intervals),
      .frmintervals = g_korvo2_camera_intervals,
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: korvo2_camera_sensor_initialize
 *
 * Description:
 *   Probe and configure the OV3660 sensor through I2C.  Only performed
 *   once; subsequent calls are idempotent.
 *
 * Input Parameters:
 *   priv - Pointer to the board camera state
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int korvo2_camera_sensor_initialize(FAR struct korvo2_camera_s *priv)
{
  int ret;

  if (priv->initialized)
    {
      return OK;
    }

  ret = ov3660_initialize(priv->i2c);
  if (ret < 0)
    {
      return ret;
    }

  ret = ov3660_set_mode(priv->i2c, OV3660_MODE_RGB565_QQVGA);
  if (ret == OK)
    {
      priv->initialized = true;
    }

  return ret;
}

/****************************************************************************
 * Name: korvo2_camera_is_available
 *
 * Description:
 *   Return whether the OV3660 sensor can be detected on the I2C bus.
 *   Triggers sensor initialization on first call.
 *
 * Input Parameters:
 *   sensor - Pointer to the imgsensor instance
 *
 * Returned Value:
 *   True if the sensor is available; false otherwise.
 *
 ****************************************************************************/

static bool korvo2_camera_is_available(FAR struct imgsensor_s *sensor)
{
  FAR struct korvo2_camera_s *priv = (FAR struct korvo2_camera_s *)sensor;

  return korvo2_camera_sensor_initialize(priv) == OK;
}

/****************************************************************************
 * Name: korvo2_camera_init
 *
 * Description:
 *   Initialize the OV3660 sensor.  Called by the imgsensor framework.
 *
 * Input Parameters:
 *   sensor - Pointer to the imgsensor instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int korvo2_camera_init(FAR struct imgsensor_s *sensor)
{
  FAR struct korvo2_camera_s *priv = (FAR struct korvo2_camera_s *)sensor;

  return korvo2_camera_sensor_initialize(priv);
}

/****************************************************************************
 * Name: korvo2_camera_uninit
 *
 * Description:
 *   De-initialize the OV3660 sensor.  Currently a no-op.
 *
 * Input Parameters:
 *   sensor - Pointer to the imgsensor instance
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

static int korvo2_camera_uninit(FAR struct imgsensor_s *sensor)
{
  return OK;
}

/****************************************************************************
 * Name: korvo2_camera_get_driver_name
 *
 * Description:
 *   Return the sensor driver name string.
 *
 * Input Parameters:
 *   sensor - Pointer to the imgsensor instance (unused)
 *
 * Returned Value:
 *   Pointer to the string "OV3660".
 *
 ****************************************************************************/

static FAR const char *korvo2_camera_get_driver_name(
  FAR struct imgsensor_s *sensor)
{
  return "OV3660";
}

/****************************************************************************
 * Name: korvo2_camera_validate
 *
 * Description:
 *   Validate the requested stream format.  Only QQVGA RGB565 at 160x120
 *   is supported.
 *
 * Input Parameters:
 *   sensor      - Pointer to the imgsensor instance
 *   type        - Stream type (capture / etc.)
 *   nr_datafmts - Number of format descriptors
 *   datafmts    - Format descriptors to validate
 *   interval    - Frame interval (unused)
 *
 * Returned Value:
 *   Zero (OK) if the format is supported; -ENOTSUP otherwise.
 *
 ****************************************************************************/

static int korvo2_camera_validate(FAR struct imgsensor_s *sensor,
                                  imgsensor_stream_type_t type,
                                  uint8_t nr_datafmts,
                                  FAR imgsensor_format_t *datafmts,
                                  FAR imgsensor_interval_t *interval)
{
  if (nr_datafmts != 1 || datafmts == NULL ||
      datafmts[IMGSENSOR_FMT_MAIN].pixelformat !=
        IMGSENSOR_PIX_FMT_RGB565 ||
      datafmts[IMGSENSOR_FMT_MAIN].width != OV3660_WIDTH ||
      datafmts[IMGSENSOR_FMT_MAIN].height != OV3660_HEIGHT)
    {
      return -ENOTSUP;
    }

  return OK;
}

/****************************************************************************
 * Name: korvo2_camera_start
 *
 * Description:
 *   Start the camera stream.  Validates the format then sets the sensor
 *   to RGB565 QQVGA mode.
 *
 * Input Parameters:
 *   sensor      - Pointer to the imgsensor instance
 *   type        - Stream type
 *   nr_datafmts - Number of format descriptors
 *   datafmts    - Format descriptors for the stream
 *   interval    - Frame interval (unused)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int korvo2_camera_start(FAR struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type,
                               uint8_t nr_datafmts,
                               FAR imgsensor_format_t *datafmts,
                               FAR imgsensor_interval_t *interval)
{
  FAR struct korvo2_camera_s *priv = (FAR struct korvo2_camera_s *)sensor;
  int ret;

  ret = korvo2_camera_validate(sensor, type, nr_datafmts, datafmts,
                               interval);
  if (ret < 0)
    {
      return ret;
    }

  return ov3660_set_mode(priv->i2c, OV3660_MODE_RGB565_QQVGA);
}

/****************************************************************************
 * Name: korvo2_camera_stop
 *
 * Description:
 *   Stop the camera stream.  The LCD_CAM receiver stop is handled by the
 *   ESP32-S3 image-data lower half.  Currently a no-op at the sensor
 *   level.
 *
 * Input Parameters:
 *   sensor - Pointer to the imgsensor instance
 *   type   - Stream type
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

static int korvo2_camera_stop(FAR struct imgsensor_s *sensor,
                              imgsensor_stream_type_t type)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_camera_initialize(void)
{
  FAR struct korvo2_camera_s *priv = &g_korvo2_camera;
  int ret;

  ret = esp32s3_camera_initialize();
  if (ret < 0)
    {
      return ret;
    }

  priv->i2c = esp32s3_i2cbus_initialize(ESP32S3_I2C0);
  if (priv->i2c == NULL)
    {
      return -ENODEV;
    }

  ret = imgsensor_register(&priv->sensor);
  if (ret < 0)
    {
      return ret;
    }

  return capture_initialize("/dev/video0");
}
