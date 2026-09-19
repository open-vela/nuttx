/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-box/src/esp32s3_board_touchsceen_gt911.c
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

#include <stdio.h>
#include <syslog.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>
#include <nuttx/input/touchscreen.h>

#include "esp32s3_i2c.h"
#include "esp32s3_gpio.h"
#include "hardware/esp32s3_gpio_sigmap.h"

#include "esp32s3-box.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GT911 maximum report frame size */

#define GT911_BUFFER_SIZE     41
#define GT911_TOUCHPOINTS     5

/* GT911 board configuration */

#define GT911_ADDR            TOUCHSCEEN_ADDR
#define GT911_CLOCK           TOUCHSCEEN_CLOCK

#define GT911_PATH            CONFIG_ESP32S3_BOARD_TOUCHSCREEN_PATH
#define GT911_WORK_DELAY      CONFIG_ESP32S3_BOARD_TOUCHSCREEN_SAMPLE_DELAYS
#define GT911_SAMPLE_CACHES   CONFIG_ESP32S3_BOARD_TOUCHSCREEN_SAMPLE_CACHES

/* GT911 registers address */

#define GT911_READ_XY_REG     0x814e
#define GT911_READ_DATA_REG   0x814f
#define GT911_CONFIG_REG      0x8047
#define GT911_PRODUCT_ID_REG  0x8140

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of one GT911 driver instance */

struct gt911_dev_s
{
  struct touch_lowerhalf_s  touch_lower;    /* Touchsrceen lowerhalf */

  bool                has_report;           /* Mark if report event */

  struct i2c_master_s *i2c;                 /* I2C master port */
  struct work_s       work;                 /* Read sample data work */
  spinlock_t          lock;                 /* Device specific lock. */

  uint8_t buffer[GT911_BUFFER_SIZE];        /* Read buffer */

  /* Diag counters (read-only debug; flat build app calls
   * gt911_diag_snapshot)
   */

  uint32_t            diag_polls;           /* worker polls */
  uint32_t            diag_i2c_err;         /* I2C transfer failures */
  uint32_t            diag_down;            /* TOUCH_DOWN samples sent */
  uint32_t            diag_up;              /* TOUCH_UP samples sent */
  uint8_t             diag_last_status;     /* last 0x814E status byte */
  uint16_t            diag_last_x;          /* last mapped x */
  uint16_t            diag_last_y;          /* last mapped y */
};

/* This structure describes the frame of touchpoint */

begin_packed_struct struct gt911_touchpoint_s
{
  uint8_t id;                               /* Not used */
  uint16_t x;                               /* Touch X-axis */
  uint16_t y;                               /* Touch Y-axis */
  uint16_t pressure;                        /* Touch pressure */
  uint8_t  reserved;                        /* Not used */
} end_packed_struct;

/* This structure describes the frame of touchpoint */

begin_packed_struct struct gt911_data_s
{
  uint8_t touchpoints     : 4;              /* Touch point number */
  uint8_t has_key         : 1;              /* 1: key is inpressed */
  uint8_t proximity_valid : 1;              /* Not used */
  uint8_t large_detected  : 1;              /* 1: large-area touch */
  uint8_t buffer_status   : 1;              /* 1: input data is valid */

  struct gt911_touchpoint_s touchpoint[0];
} end_packed_struct;

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct gt911_dev_s g_gt911_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gt911_write_reg
 *
 * Description:
 *   Read GT911 continuous registers value.
 *
 * Input Parameters:
 *   dev    - GT911 object pointer
 *   reg    - Register start address
 *   buf    - Register value buffer
 *   buflen - Register value buffer length
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gt911_read_reg(struct gt911_dev_s *dev,
                          uint16_t reg,
                          uint8_t *buf,
                          int buflen)
{
  int ret;

  /* Send the Register Address, MSB first */

  uint8_t regbuf[2] =
  {
    reg >> 8,   /* First Byte: MSB */
    reg & 0xff  /* Second Byte: LSB */
  };

  /* Compose the I2C Messages */

  struct i2c_msg_s msgv[2] =
  {
    {
      /* Send the I2C Register Address */

      .frequency = GT911_CLOCK,
      .addr      = GT911_ADDR,
      .flags     = 0,
      .buffer    = regbuf,
      .length    = sizeof(regbuf)
    },
    {
      /* Receive the I2C Register Values */

      .frequency = GT911_CLOCK,
      .addr      = GT911_ADDR,
      .flags     = I2C_M_READ,
      .buffer    = buf,
      .length    = buflen
    }
  };

  const int msgv_len = sizeof(msgv) / sizeof(msgv[0]);

  iinfo("reg=0x%x, buflen=%d\n", reg, buflen);
  DEBUGASSERT(dev && dev->i2c && buf);

  /* Execute the I2C Transfer */

  ret = I2C_TRANSFER(dev->i2c, msgv, msgv_len);
  if (ret < 0)
    {
      ierr("I2C Read failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_DEBUG_INPUT_INFO
  iinfodumpbuffer("gt911_read_reg", buf, buflen);
#endif /* CONFIG_DEBUG_INPUT_INFO */

  return 0;
}

/****************************************************************************
 * Name: gt911_write_reg
 *
 * Description:
 *   Write GT911 register value.
 *
 * Input Parameters:
 *   dev - GT911 object pointer
 *   reg - Register address
 *   val - Register value
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gt911_write_reg(struct gt911_dev_s *dev,
                           uint16_t reg,
                           uint8_t val)
{
  int ret;

  /* Send the Register Address, MSB first */

  uint8_t regbuf[3] =
  {
    reg >> 8,   /* First Byte: MSB */
    reg & 0xff, /* Second Byte: LSB */
    val,
  };

  /* Compose the I2C Messages */

  struct i2c_msg_s msgv[1] =
  {
    {
      /* Send the I2C Register Address */

      .frequency = GT911_CLOCK,
      .addr      = GT911_ADDR,
      .flags     = 0,
      .buffer    = regbuf,
      .length    = sizeof(regbuf)
    }
  };

  const int msgv_len = sizeof(msgv) / sizeof(msgv[0]);

  iinfo("reg=0x%x, val=%d\n", reg, val);
  DEBUGASSERT(dev && dev->i2c);

  /* Execute the I2C Transfer */

  ret = I2C_TRANSFER(dev->i2c, msgv, msgv_len);
  if (ret < 0)
    {
      ierr("I2C Write failed: %d\n", ret);
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: gt911_touch_event
 *
 * Description:
 *   Process touch event. Read touchpoint data and send to touch event.
 *
 * Input Parameters:
 *   dev - GT911 object pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gt911_touch_event(struct gt911_dev_s *dev)
{
  struct gt911_data_s *data = (struct gt911_data_s *)dev->buffer;
  struct gt911_touchpoint_s *tp = data->touchpoint;

  /* Key mechanism (NuttX touchscreen upper layer): touch_event() writes to
   * queue based on sample->npoints (circbuf element size =
   * SIZEOF_TOUCH_SAMPLE_S(maxpoint)), LVGL-side read() expects to read full
   * in one go SIZEOF_TOUCH_SAMPLE_S(maxpoint) bytes. If npoints is less than
   * maxpoint, queue elements are not fully written, LVGL determines
   * 'incomplete read'. and ignores all touches! Therefore, sample must be
   * constructed with a maxpoint-sized buffer, npoints =
   * GT911_TOUCHPOINTS(=maxpoint), valid points filled in point[0], Remaining
   * points flags=0 (invalid, LVGL skips).
   */

  uint8_t sample_buf[SIZEOF_TOUCH_SAMPLE_S(GT911_TOUCHPOINTS)];
  struct touch_sample_s *sample =
    (struct touch_sample_s *)sample_buf;
  struct touch_point_s *point = sample->point;

  memset(sample_buf, 0, sizeof(sample_buf));
  sample->npoints = GT911_TOUCHPOINTS;

  /* Coordinate direction adaptation: GT911 sensor native 320×480 (portrait)
   * coordinates, Panel is 480×320 landscape (LANDSCAPE+MV=1). Swap X/Y and
   * invert Y, mapping to LVGL's 480×320 coordinate system. If orientation
   * still incorrect, adjust swap/flip combination here (4 options):
   * Landscape mapping = (x,y) → (319-y, x) or (y, x) = (319-y, 479-x) or
   * (y, 479-x) Currently using the (x,y)->(y, 319-x) variant, fine-tuned per
   * real device.
   */

  point->x         = tp->y;
  point->y         = 319 - tp->x;
  point->pressure  = tp->pressure;
  point->flags     = TOUCH_POS_VALID | TOUCH_PRESSURE_VALID;

  dev->diag_last_status = data->buffer_status;
  dev->diag_last_x      = point->x;
  dev->diag_last_y      = point->y;

  if (data->buffer_status)
    {
      point->flags |= TOUCH_DOWN;
      dev->has_report = true;
      dev->diag_down++;

      /* ⚠️ 2026-08-21: Removed [Touch] down debug printf—prints for
       * every touch event will flood the USB-Serial-JTAG console
       * (intertwined with audio debug output into garbled text, and large
       * amounts UART output may freeze the console). Touch events themselves
       * queue normally, no functional impact.
       */
    }
  else
    {
      point->flags |= TOUCH_UP;
      dev->has_report = false;
      dev->diag_up++;
    }

  touch_event(dev->touch_lower.priv, sample);
}

/****************************************************************************
 * Name: gt911_event
 *
 * Description:
 *   Process GT911 event.
 *
 * Input Parameters:
 *   dev - GT911 object pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gt911_event(struct gt911_dev_s *dev)
{
  struct gt911_data_s *data = (struct gt911_data_s *)dev->buffer;

  if (!data->has_key)
    {
      gt911_touch_event(dev);
    }
  else
    {
      ierr("ERROR: event is invalid\n");
    }
}

/****************************************************************************
 * Name: gt911_worker
 *
 * Description:
 *   Process GT911 work, read GT911 report frame and process it.
 *
 * Input Parameters:
 *   arg - GT911 object pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gt911_worker(void *arg)
{
  int ret;
  struct gt911_dev_s *dev = (struct gt911_dev_s *)arg;
  struct gt911_data_s *data = (struct gt911_data_s *)dev->buffer;
  clock_t delay = GT911_WORK_DELAY;
  bool touched = false;

  dev->diag_polls++;

  ret = gt911_read_reg(dev, GT911_READ_XY_REG, dev->buffer, 1);
  if (ret != 0)
    {
      ierr("ERROR: I2C_TRANSFER() failed: %d\n", ret);
      dev->diag_i2c_err++;
      goto exit;
    }

  if (data->buffer_status &&
      (data->touchpoints > 0) &&
      (data->touchpoints < GT911_TOUCHPOINTS))
    {
      ret = gt911_read_reg(dev, GT911_READ_DATA_REG,
                           &dev->buffer[1], data->touchpoints * 8);
      if (ret != 0)
        {
          ierr("ERROR: I2C_TRANSFER() failed: %d\n", ret);
          dev->diag_i2c_err++;
          goto exit;
        }

      touched = true;
    }
  else if (dev->has_report)
    {
      touched = true;
    }

  ret = gt911_write_reg(dev, GT911_READ_XY_REG, 0);
  if (ret != 0)
    {
      ierr("ERROR: I2C_TRANSFER() failed: %d\n", ret);
      dev->diag_i2c_err++;
      goto exit;
    }

  if (touched)
    {
      gt911_event(dev);
      delay = 1;
    }

exit:
  ret = work_queue(LPWORK, &dev->work, gt911_worker, dev, delay);
  if (ret != 0)
    {
      ierr("ERROR: work_queue() failed: %d\n", ret);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gt911_diag_snapshot
 *
 * Description:
 *   Debug helper (flat build): snapshot GT911 driver counters for the UI
 *   app to correlate producer/consumer liveness.  Array layout:
 *     [0]=polls [1]=i2c_err [2]=down [3]=up [4]=last_status
 *     [5]=last_x [6]=last_y [7]=has_report
 *
 ****************************************************************************/

int gt911_diag_snapshot(uint32_t out[8])
{
  struct gt911_dev_s *dev = &g_gt911_dev;

  if (out == NULL)
    {
      return -EINVAL;
    }

  out[0] = dev->diag_polls;
  out[1] = dev->diag_i2c_err;
  out[2] = dev->diag_down;
  out[3] = dev->diag_up;
  out[4] = dev->diag_last_status;
  out[5] = dev->diag_last_x;
  out[6] = dev->diag_last_y;
  out[7] = dev->has_report ? 1u : 0u;
  return 0;
}

/****************************************************************************
 * Name: board_touchscreen_initialize
 *
 * Description:
 *   Initialize touchpad.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_touchscreen_initialize(void)
{
  int ret;
  struct gt911_dev_s *dev = &g_gt911_dev;

  /* Step 1: Reset+INT sequence to set I2C address
   * According to GT911 datasheet:
   * 1. RST output LOW
   * 2. INT output LOW (hold >100μs)
   * 3. RST output HIGH (release reset)
   * 4. Wait >5ms
   * 5. INT switch to floating input (external pull-up makes it HIGH)
   */

  /* Configure RST (GPIO48) and INT (GPIO3) as outputs */

  esp32s3_configgpio(48, OUTPUT);  /* RST */
  esp32s3_configgpio(3, OUTPUT);   /* INT */

  /* RST = LOW, INT = LOW */

  esp32s3_gpiowrite(48, false);   /* RST = LOW */
  esp32s3_gpiowrite(3, false);    /* INT = LOW */
  up_udelay(200);                 /* Wait >100μs */

  /* RST = HIGH (release reset) */

  esp32s3_gpiowrite(48, true);    /* RST = HIGH */
  up_mdelay(10);                  /* Wait >5ms */

  /* INT switch to floating input (external pull-up) */

  esp32s3_configgpio(3, INPUT);   /* INT = floating input */
  up_mdelay(50);                  /* Wait for GT911 to initialize */

  /* Step 2: Initialize I2C bus */

  dev->i2c = esp32s3_i2cbus_initialize(TOUCHSCEEN_I2C);
  if (!dev->i2c)
    {
      printf("[Touch] ERROR: Failed to initialize I2C port %d\n",
             TOUCHSCEEN_I2C);
      return -ENODEV;
    }

  /* Step 3: Probe GT911 by reading product ID */

  uint8_t pid[4];
  ret = gt911_read_reg(dev, GT911_PRODUCT_ID_REG, pid, 4);
  if (ret < 0)
    {
      printf("[Touch] ERROR: Failed to read product ID: %d\n", ret);
      return ret;
    }

  printf("[Touch] GT911 detected: PID='%c%c%c%c'\n",
         pid[0], pid[1], pid[2], pid[3]);

  /* Step 4: Register touch device Must set maxpoint! TSIOC_GETMAXPOINTS
   * returns lower->maxpoint, If unset, LVGL receives 0 and determines
   * "unsupported maxpoint", refusing to create indev.
   */

  dev->touch_lower.maxpoint = GT911_TOUCHPOINTS;

  ret = touch_register(&dev->touch_lower, GT911_PATH,
                       GT911_SAMPLE_CACHES);
  if (ret < 0)
    {
      printf("[Touch] ERROR: touch_register() failed: %d\n", ret);
      return ret;
    }

  printf("[Touch] registered at %s\n", GT911_PATH);

  /* Step 5: Start worker queue */

  ret = work_queue(LPWORK, &dev->work, gt911_worker,
                   dev, GT911_WORK_DELAY);
  if (ret != 0)
    {
      printf("[Touch] ERROR: work_queue() failed: %d\n", ret);
      return ret;
    }

  return 0;
}
