/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_camera.c
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
 * This file implements a V4L2 capture driver for ESP32-P4 MIPI-CSI camera.
 * It uses the NuttX imgdata/imgsensor framework to register /dev/video0.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <semaphore.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <sys/videoio.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/imgsensor.h>

#ifdef CONFIG_VIDEO_STREAM
#  include <nuttx/video/v4l2_cap.h>
#endif

#include "esp_mipi_csi.h"
#include "esp_cam_sensor.h"
#include "esp_camera.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP_CAM_HRES       1024
#define ESP_CAM_VRES       600
#define ESP_CAM_BPP        16    /* RGB565 = 16 bits per pixel */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_cam_data_s
{
  struct imgdata_s       data;
  bool                   streaming;
  imgdata_capture_t      capture_cb;
  void                  *capture_arg;
  sem_t                  frame_sem;
};

struct esp_cam_sensor_s
{
  struct imgsensor_s     sensor;
  bool                   streaming;
};

/****************************************************************************
 * Private Function Prototypes - imgdata_ops
 ****************************************************************************/

static int esp_imgdata_init(FAR struct imgdata_s *data);
static int esp_imgdata_uninit(FAR struct imgdata_s *data);
static int esp_imgdata_set_buf(FAR struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               FAR imgdata_format_t *datafmts,
                               uint8_t *addr, uint32_t size);
static int esp_imgdata_validate_frame(FAR struct imgdata_s *data,
                                      uint8_t nr_datafmts,
                                      FAR imgdata_format_t *datafmts,
                                      FAR imgdata_interval_t *interval);
static int esp_imgdata_start_capture(FAR struct imgdata_s *data,
                                     uint8_t nr_datafmts,
                                     FAR imgdata_format_t *datafmts,
                                     FAR imgdata_interval_t *interval,
                                     FAR imgdata_capture_t callback,
                                     FAR void *arg);
static int esp_imgdata_stop_capture(FAR struct imgdata_s *data);

/****************************************************************************
 * Private Function Prototypes - imgsensor_ops
 ****************************************************************************/

static bool esp_imgsensor_is_available(FAR struct imgsensor_s *sensor);
static int  esp_imgsensor_init(FAR struct imgsensor_s *sensor);
static int  esp_imgsensor_uninit(FAR struct imgsensor_s *sensor);
static const char *esp_imgsensor_get_driver_name(
                    FAR struct imgsensor_s *sensor);
static int  esp_imgsensor_validate_frame(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval);
static int  esp_imgsensor_start_capture(FAR struct imgsensor_s *sensor,
                                        imgsensor_stream_type_t type,
                                        uint8_t nr_datafmts,
                                        FAR imgsensor_format_t *datafmts,
                                        FAR imgsensor_interval_t *interval);
static int  esp_imgsensor_stop_capture(FAR struct imgsensor_s *sensor,
                                       imgsensor_stream_type_t type);
static int  esp_imgsensor_get_supported_value(
                    FAR struct imgsensor_s *sensor,
                    uint32_t id,
                    FAR imgsensor_supported_value_t *value);
static int  esp_imgsensor_get_value(FAR struct imgsensor_s *sensor,
                                    uint32_t id, uint32_t size,
                                    FAR imgsensor_value_t *value);
static int  esp_imgsensor_set_value(FAR struct imgsensor_s *sensor,
                                    uint32_t id, uint32_t size,
                                    imgsensor_value_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imgdata_ops_s g_esp_imgdata_ops =
{
  .init                   = esp_imgdata_init,
  .uninit                 = esp_imgdata_uninit,
  .set_buf                = esp_imgdata_set_buf,
  .validate_frame_setting = esp_imgdata_validate_frame,
  .start_capture          = esp_imgdata_start_capture,
  .stop_capture           = esp_imgdata_stop_capture,
  .alloc                  = NULL,
  .free                   = NULL,
};

static const struct imgsensor_ops_s g_esp_imgsensor_ops =
{
  .is_available           = esp_imgsensor_is_available,
  .init                   = esp_imgsensor_init,
  .uninit                 = esp_imgsensor_uninit,
  .get_driver_name        = esp_imgsensor_get_driver_name,
  .validate_frame_setting = esp_imgsensor_validate_frame,
  .start_capture          = esp_imgsensor_start_capture,
  .stop_capture           = esp_imgsensor_stop_capture,
  .get_frame_interval     = NULL,
  .get_supported_value    = esp_imgsensor_get_supported_value,
  .get_value              = esp_imgsensor_get_value,
  .set_value              = esp_imgsensor_set_value,
};

/* Format descriptor for V4L2 enumeration */

static const struct v4l2_fmtdesc g_esp_fmtdescs[] =
{
  {
    .index       = 0,
    .type        = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .flags       = 0,
    .description = "RGB565",
    .pixelformat = V4L2_PIX_FMT_RGB565,
  },
};

/* Frame size enumeration */

static const struct v4l2_frmsizeenum g_esp_frmsizes[] =
{
  {
    .index        = 0,
    .pixel_format = V4L2_PIX_FMT_RGB565,
    .type         = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete     =
    {
      .width  = ESP_CAM_HRES,
      .height = ESP_CAM_VRES,
    },
  },
};

/* Frame interval enumeration: 30fps */

static const struct v4l2_frmivalenum g_esp_frmintervals[] =
{
  {
    .index        = 0,
    .pixel_format = V4L2_PIX_FMT_RGB565,
    .width        = ESP_CAM_HRES,
    .height       = ESP_CAM_VRES,
    .type         = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete     =
    {
      .numerator   = 1,
      .denominator = 30,
    },
  },
};

/* Driver instances */

static struct esp_cam_data_s g_esp_cam_data =
{
  .data =
  {
    .ops = &g_esp_imgdata_ops,
  },
};

static struct esp_cam_sensor_s g_esp_cam_sensor =
{
  .sensor =
  {
    .ops            = &g_esp_imgsensor_ops,
    .fmtdescs_num   = 1,
    .fmtdescs       = g_esp_fmtdescs,
    .frmsizes_num   = 1,
    .frmsizes       = g_esp_frmsizes,
    .frmintervals_num = 1,
    .frmintervals   = g_esp_frmintervals,
  },
};

/****************************************************************************
 * Private Functions - imgdata_ops
 ****************************************************************************/

static int esp_imgdata_init(FAR struct imgdata_s *data)
{
  syslog(LOG_INFO, "ESP Camera: imgdata init\n");
  nxsem_init(&g_esp_cam_data.frame_sem, 0, 0);
  return OK;
}

static int esp_imgdata_uninit(FAR struct imgdata_s *data)
{
  syslog(LOG_INFO, "ESP Camera: imgdata uninit\n");
  nxsem_destroy(&g_esp_cam_data.frame_sem);
  return OK;
}

static int esp_imgdata_set_buf(FAR struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               FAR imgdata_format_t *datafmts,
                               uint8_t *addr, uint32_t size)
{
  /* Buffer management is handled internally by the CSI driver.
   * This callback is informational only.
   */

  syslog(LOG_INFO, "ESP Camera: set_buf addr=%p size=%lu\n",
         addr, (unsigned long)size);
  return OK;
}

static int esp_imgdata_validate_frame(FAR struct imgdata_s *data,
                                      uint8_t nr_datafmts,
                                      FAR imgdata_format_t *datafmts,
                                      FAR imgdata_interval_t *interval)
{
  if (nr_datafmts < 1 || datafmts == NULL)
    {
      return -EINVAL;
    }

  /* We only support RGB565 at 1024x600 */

  if (datafmts[0].width != ESP_CAM_HRES ||
      datafmts[0].height != ESP_CAM_VRES ||
      datafmts[0].pixelformat != IMGDATA_PIX_FMT_RGB565)
    {
      return -EINVAL;
    }

  return OK;
}

static int esp_imgdata_start_capture(FAR struct imgdata_s *data,
                                     uint8_t nr_datafmts,
                                     FAR imgdata_format_t *datafmts,
                                     FAR imgdata_interval_t *interval,
                                     FAR imgdata_capture_t callback,
                                     FAR void *arg)
{
  int ret;

  syslog(LOG_INFO, "ESP Camera: start_capture\n");

  g_esp_cam_data.capture_cb = callback;
  g_esp_cam_data.capture_arg = arg;

  /* Start the CSI controller */

  ret = esp_csi_start();
  if (ret < 0)
    {
      return ret;
    }

  g_esp_cam_data.streaming = true;

  /* TODO: In full implementation, DMA ISR would call capture_cb
   * when each frame completes. For now, the framework is in place.
   */

  return OK;
}

static int esp_imgdata_stop_capture(FAR struct imgdata_s *data)
{
  syslog(LOG_INFO, "ESP Camera: stop_capture\n");

  g_esp_cam_data.streaming = false;
  g_esp_cam_data.capture_cb = NULL;
  g_esp_cam_data.capture_arg = NULL;

  return esp_csi_stop();
}

/****************************************************************************
 * Private Functions - imgsensor_ops
 ****************************************************************************/

static bool esp_imgsensor_is_available(FAR struct imgsensor_s *sensor)
{
  return true;
}

static int esp_imgsensor_init(FAR struct imgsensor_s *sensor)
{
  syslog(LOG_INFO, "ESP Camera: imgsensor init\n");

  /* Initialize the OV5647 sensor */

  return esp_cam_sensor_init();
}

static int esp_imgsensor_uninit(FAR struct imgsensor_s *sensor)
{
  syslog(LOG_INFO, "ESP Camera: imgsensor uninit\n");
  return esp_cam_sensor_stop();
}

static const char *esp_imgsensor_get_driver_name(
                    FAR struct imgsensor_s *sensor)
{
  return "ESP32P4-OV5647";
}

static int esp_imgsensor_validate_frame(FAR struct imgsensor_s *sensor,
                                        imgsensor_stream_type_t type,
                                        uint8_t nr_datafmts,
                                        FAR imgsensor_format_t *datafmts,
                                        FAR imgsensor_interval_t *interval)
{
  if (nr_datafmts < 1 || datafmts == NULL)
    {
      return -EINVAL;
    }

  if (datafmts[0].width != ESP_CAM_HRES ||
      datafmts[0].height != ESP_CAM_VRES)
    {
      return -EINVAL;
    }

  return OK;
}

static int esp_imgsensor_start_capture(FAR struct imgsensor_s *sensor,
                                       imgsensor_stream_type_t type,
                                       uint8_t nr_datafmts,
                                       FAR imgsensor_format_t *datafmts,
                                       FAR imgsensor_interval_t *interval)
{
  syslog(LOG_INFO, "ESP Camera: sensor start_capture\n");

  g_esp_cam_sensor.streaming = true;
  return esp_cam_sensor_start();
}

static int esp_imgsensor_stop_capture(FAR struct imgsensor_s *sensor,
                                      imgsensor_stream_type_t type)
{
  syslog(LOG_INFO, "ESP Camera: sensor stop_capture\n");

  g_esp_cam_sensor.streaming = false;
  return esp_cam_sensor_stop();
}

static int esp_imgsensor_get_supported_value(
                    FAR struct imgsensor_s *sensor,
                    uint32_t id,
                    FAR imgsensor_supported_value_t *value)
{
  /* No configurable parameters supported yet */

  return -ENOTTY;
}

static int esp_imgsensor_get_value(FAR struct imgsensor_s *sensor,
                                   uint32_t id, uint32_t size,
                                   FAR imgsensor_value_t *value)
{
  return -ENOTTY;
}

static int esp_imgsensor_set_value(FAR struct imgsensor_s *sensor,
                                   uint32_t id, uint32_t size,
                                   imgsensor_value_t value)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_camera_initialize
 *
 * Description:
 *   Initialize the camera subsystem and register /dev/video0.
 *   Called from board bringup.
 ****************************************************************************/

int esp_camera_initialize(void)
{
  int ret;

  syslog(LOG_INFO, "ESP Camera: Initializing camera subsystem\n");

  /* Step 1: Initialize the MIPI-CSI controller */

  ret = esp_csi_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ESP Camera: CSI init failed: %d\n", ret);
      return ret;
    }

  /* Step 2: Register with V4L2 capture framework (if available) */

#ifdef CONFIG_VIDEO_STREAM
  {
    FAR struct imgsensor_s *sensor_ptr;
    sensor_ptr = &g_esp_cam_sensor.sensor;

    ret = capture_register(ESP_CAMERA_DEVPATH,
                           &g_esp_cam_data.data,
                           &sensor_ptr,
                           1);
    if (ret < 0)
      {
        syslog(LOG_ERR, "ESP Camera: capture_register failed: %d\n", ret);
        return ret;
      }
  }
#else
  syslog(LOG_WARNING,
         "ESP Camera: CONFIG_VIDEO_STREAM not enabled, "
         "V4L2 device not registered\n");
#endif

  syslog(LOG_INFO, "ESP Camera: %s initialized successfully\n",
         ESP_CAMERA_DEVPATH);
  return OK;
}
