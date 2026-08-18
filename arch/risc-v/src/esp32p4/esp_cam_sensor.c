/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_cam_sensor.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>

#include "esp_cam_sensor.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Forward declaration for esp_i2cbus_initialize */

struct i2c_master_s *esp_i2cbus_initialize(int port);
int esp_i2cbus_uninitialize(struct i2c_master_s *dev);

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Register-value pair for sensor initialization sequence */

struct sc2336_regval_s
{
  uint16_t reg;
  uint8_t  val;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct i2c_master_s *g_i2c_dev;
static bool g_sensor_initialized;

/* SC2336 initialization sequence for 1024x600 @ 30fps RAW8, MIPI 2-lane,
 * 24MHz input, 288Mbps.  Ported from ESP-IDF esp_cam_sensor component
 * (init_reglist_MIPI_2lane_1024x600_raw8_30fps).  Terminated by
 * SC2336_REG_END (0xffff).
 */

static const struct sc2336_regval_s g_sc2336_init_regs[] =
{
  {0x0103, 0x01},
  {SC2336_REG_SLEEP_MODE, 0x00},
  {0x36e9, 0x80},
  {0x37f9, 0x80},
  {0x301f, 0xc7},
  {0x3031, 0x08},
  {0x3037, 0x00},
  {0x3106, 0x05},
  {0x3200, 0x01},
  {0x3201, 0xb4},
  {0x3202, 0x00},
  {0x3203, 0xf0},
  {0x3204, 0x05},
  {0x3205, 0xd3},
  {0x3206, 0x03},
  {0x3207, 0x4f},
  {0x3208, 0x04},
  {0x3209, 0x00},
  {0x320a, 0x02},
  {0x320b, 0x58},
  {0x320c, 0x09},
  {0x320d, 0x60},
  {0x320e, 0x03},
  {0x320f, 0xe8},
  {0x3210, 0x00},
  {0x3211, 0x10},
  {0x3212, 0x00},
  {0x3213, 0x04},
  {0x3248, 0x04},
  {0x3249, 0x0b},
  {0x3253, 0x08},
  {0x3301, 0x09},
  {0x3302, 0xff},
  {0x3303, 0x10},
  {0x3306, 0x60},
  {0x3307, 0x02},
  {0x330a, 0x01},
  {0x330b, 0x10},
  {0x330c, 0x16},
  {0x330d, 0xff},
  {0x3318, 0x02},
  {0x3321, 0x0a},
  {0x3327, 0x0e},
  {0x332b, 0x12},
  {0x3333, 0x10},
  {0x3334, 0x40},
  {0x335e, 0x06},
  {0x335f, 0x0a},
  {0x3364, 0x1f},
  {0x337c, 0x02},
  {0x337d, 0x0e},
  {0x3390, 0x09},
  {0x3391, 0x0f},
  {0x3392, 0x1f},
  {0x3393, 0x20},
  {0x3394, 0x20},
  {0x3395, 0xff},
  {0x33a2, 0x04},
  {0x33b1, 0x80},
  {0x33b2, 0x68},
  {0x33b3, 0x42},
  {0x33f9, 0x78},
  {0x33fb, 0xd8},
  {0x33fc, 0x0f},
  {0x33fd, 0x1f},
  {0x349f, 0x03},
  {0x34a6, 0x0f},
  {0x34a7, 0x1f},
  {0x34a8, 0x42},
  {0x34a9, 0x06},
  {0x34aa, 0x01},
  {0x34ab, 0x28},
  {0x34ac, 0x01},
  {0x34ad, 0x90},
  {0x3630, 0xf4},
  {0x3633, 0x22},
  {0x3639, 0xf4},
  {0x363c, 0x47},
  {0x3670, 0x09},
  {0x3674, 0xf4},
  {0x3675, 0xfb},
  {0x3676, 0xed},
  {0x367c, 0x09},
  {0x367d, 0x0f},
  {0x3690, 0x22},
  {0x3691, 0x22},
  {0x3692, 0x22},
  {0x3698, 0x89},
  {0x3699, 0x96},
  {0x369a, 0xd0},
  {0x369b, 0xd0},
  {0x369c, 0x09},
  {0x369d, 0x0f},
  {0x36a2, 0x09},
  {0x36a3, 0x0f},
  {0x36a4, 0x1f},
  {0x36d0, 0x01},
  {0x36ea, 0x08},
  {0x36eb, 0x0a},
  {0x36ec, 0x1a},
  {0x36ed, 0x18},
  {0x3722, 0xe1},
  {0x3724, 0x41},
  {0x3725, 0xc1},
  {0x3728, 0x20},
  {0x37fa, 0x08},
  {0x37fb, 0x32},
  {0x37fc, 0x11},
  {0x37fd, 0x37},
  {0x3900, 0x0d},
  {0x3905, 0x98},
  {0x391b, 0x81},
  {0x391c, 0x10},
  {0x3933, 0x81},
  {0x3934, 0xc5},
  {0x3940, 0x68},
  {0x3941, 0x00},
  {0x3942, 0x01},
  {0x3943, 0xc6},
  {0x3952, 0x02},
  {0x3953, 0x0f},
  {0x3e01, 0x37},
  {0x3e02, 0xe0},
  {0x3e08, 0x1f},
  {0x3e1b, 0x14},
  {0x4509, 0x38},
  {0x4819, 0x05},
  {0x481b, 0x03},
  {0x481d, 0x0a},
  {0x481f, 0x02},
  {0x4821, 0x08},
  {0x4823, 0x03},
  {0x4825, 0x02},
  {0x4827, 0x03},
  {0x4829, 0x04},
  {0x5799, 0x06},
  {0x5ae0, 0xfe},
  {0x5ae1, 0x40},
  {0x5ae2, 0x30},
  {0x5ae3, 0x28},
  {0x5ae4, 0x20},
  {0x5ae5, 0x30},
  {0x5ae6, 0x28},
  {0x5ae7, 0x20},
  {0x5ae8, 0x3c},
  {0x5ae9, 0x30},
  {0x5aea, 0x28},
  {0x5aeb, 0x3c},
  {0x5aec, 0x30},
  {0x5aed, 0x28},
  {0x5aee, 0xfe},
  {0x5aef, 0x40},
  {0x5af4, 0x30},
  {0x5af5, 0x28},
  {0x5af6, 0x20},
  {0x5af7, 0x30},
  {0x5af8, 0x28},
  {0x5af9, 0x20},
  {0x5afa, 0x3c},
  {0x5afb, 0x30},
  {0x5afc, 0x28},
  {0x5afd, 0x3c},
  {0x5afe, 0x30},
  {0x5aff, 0x28},
  {0x36e9, 0x53},
  {0x37f9, 0x53},
  {SC2336_REG_END, 0x00},
};

#define SC2336_INIT_REGS_COUNT \
    (sizeof(g_sc2336_init_regs) / sizeof(g_sc2336_init_regs[0]))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sc2336_i2c_write_reg
 *
 * Description:
 *   Write a single byte to an SC2336 register (16-bit address).
 ****************************************************************************/

static int sc2336_i2c_write_reg(uint16_t reg, uint8_t val)
{
  struct i2c_msg_s msg;
  uint8_t buf[3];
  int ret;

  /* 方案：拆分为两个 I2C transaction，中间加 STOP */
  /* Transaction 1: 写寄存器地址 (2 bytes) */
  buf[0] = (reg >> 8) & 0xff;
  buf[1] = reg & 0xff;

  msg.frequency = SC2336_I2C_FREQ;
  msg.addr      = SC2336_I2C_ADDR;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 2;

  ret = I2C_TRANSFER(g_i2c_dev, &msg, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "SC2336: I2C write-addr failed reg=0x%04x ret=%d\n", reg, ret);
      return ret;
    }

  /* Transaction 2: 写数据 (1 byte) */
  buf[0] = val;

  msg.frequency = SC2336_I2C_FREQ;
  msg.addr      = SC2336_I2C_ADDR;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 1;

  ret = I2C_TRANSFER(g_i2c_dev, &msg, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "SC2336: I2C write-data failed reg=0x%04x ret=%d\n", reg, ret);
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: sc2336_i2c_read_reg
 *
 * Description:
 *   Read a single byte from an SC2336 register (16-bit address).
 ****************************************************************************/

static int sc2336_i2c_read_reg(uint16_t reg, uint8_t *val)
{
  struct i2c_msg_s msgs[2];
  uint8_t addr_buf[2];
  int ret;

  /* First message: write register address */

  addr_buf[0] = (reg >> 8) & 0xff;
  addr_buf[1] = reg & 0xff;

  msgs[0].frequency = SC2336_I2C_FREQ;
  msgs[0].addr      = SC2336_I2C_ADDR;
  msgs[0].flags     = 0;         /* Write */
  msgs[0].buffer    = addr_buf;
  msgs[0].length    = 2;

  /* Second message: read data (with repeated start) */

  msgs[1].frequency = SC2336_I2C_FREQ;
  msgs[1].addr      = SC2336_I2C_ADDR;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = val;
  msgs[1].length    = 1;

  /* Try combined write+read (repeated start) first */

  ret = I2C_TRANSFER(g_i2c_dev, msgs, 2);
  if (ret >= 0)
    {
      return ret;
    }

  /* If combined fails, try separate transactions (some drivers need this) */

  ret = I2C_TRANSFER(g_i2c_dev, &msgs[0], 1);  /* Write addr */
  if (ret < 0)
    {
      syslog(LOG_ERR, "SC2336: I2C write-addr failed reg=0x%04x ret=%d\n",
             reg, ret);
      return ret;
    }

  ret = I2C_TRANSFER(g_i2c_dev, &msgs[1], 1);  /* Read data */
  if (ret < 0)
    {
      syslog(LOG_ERR, "SC2336: I2C read failed reg=0x%04x ret=%d\n",
             reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sc2336_check_chip_id
 *
 * Description:
 *   Read and verify the SC2336 chip ID.
 ****************************************************************************/

static int sc2336_check_chip_id(void)
{
  uint8_t id_h;
  uint8_t id_l;
  uint16_t chip_id;
  int ret;

  ret = sc2336_i2c_read_reg(SC2336_CHIP_ID_H_REG, &id_h);
  if (ret < 0)
    {
      return ret;
    }

  ret = sc2336_i2c_read_reg(SC2336_CHIP_ID_L_REG, &id_l);
  if (ret < 0)
    {
      return ret;
    }

  chip_id = ((uint16_t)id_h << 8) | id_l;

  syslog(LOG_INFO, "SC2336: Chip ID = 0x%04x (expected 0x%04x)\n",
         chip_id, SC2336_CHIP_ID);

  if (chip_id != SC2336_CHIP_ID)
    {
      syslog(LOG_ERR, "SC2336: Chip ID mismatch!\n");
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: sc2336_write_init_sequence
 *
 * Description:
 *   Write the initialization register sequence to the sensor.
 *   The table is terminated by SC2336_REG_END; SC2336_REG_DELAY marks
 *   a delay (value = milliseconds).
 ****************************************************************************/

static int sc2336_write_init_sequence(void)
{
  int ret;
  unsigned int i;
  unsigned int count = 0;

  for (i = 0; i < SC2336_INIT_REGS_COUNT; i++)
    {
      uint16_t reg = g_sc2336_init_regs[i].reg;
      uint8_t  val = g_sc2336_init_regs[i].val;

      if (reg == SC2336_REG_END)
        {
          break;
        }

      if (reg == SC2336_REG_DELAY)
        {
          usleep((useconds_t)val * 1000);
          continue;
        }

      if (reg == SC2336_REG_SW_RESET)
        {
          /* Writing 0x01 to 0x0103 triggers an immediate internal reset.
           * The sensor resets so quickly that it NACKs the data byte and
           * stays unresponsive on the I2C bus for a variable period while
           * it re-initializes. The reset command still takes effect, so we
           * tolerate the NACK and then poll the sensor (chip-ID read) until
           * it acknowledges again before continuing with the sequence.
           */

          int w;

          sc2336_i2c_write_reg(reg, val);   /* NACK expected; reset applied */

          for (w = 0; w < 50; w++)          /* up to ~500ms */
            {
              uint8_t id_h;

              usleep(10000);                /* 10ms per poll */
              if (sc2336_i2c_read_reg(SC2336_CHIP_ID_H_REG, &id_h) >= 0)
                {
                  break;
                }
            }

          syslog(LOG_INFO, "SC2336: sensor ready %d ms after reset\n",
                 (w + 1) * 10);
          count++;
          continue;
        }

      ret = sc2336_i2c_write_reg(reg, val);
      if (ret < 0)
        {
          syslog(LOG_ERR, "SC2336: Init sequence failed at index %d "
                 "(reg=0x%04x)\n", i, reg);
          return ret;
        }

      count++;
    }

  syslog(LOG_INFO, "SC2336: Init sequence written (%u registers)\n",
         count);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_cam_sensor_init
 ****************************************************************************/

int esp_cam_sensor_init(void)
{
  int ret;
  int retry;

  if (g_sensor_initialized)
    {
      return OK;
    }

  syslog(LOG_INFO, "SC2336: Initializing camera sensor\n");

  /* Get the I2C bus instance.
   * The I2C bus should already be initialized in board bringup.
   * We reference port 0 (which uses GPIO7=SDA, GPIO8=SCL).
   */

#ifdef CONFIG_I2C_DRIVER
  g_i2c_dev = esp_i2cbus_initialize(SC2336_I2C_PORT);
  if (g_i2c_dev == NULL)
    {
      syslog(LOG_ERR, "SC2336: Failed to get I2C bus %d\n", SC2336_I2C_PORT);
      return -ENODEV;
    }
#else
  syslog(LOG_WARNING, "SC2336: I2C not enabled, sensor init skipped\n");
  g_sensor_initialized = true;
  return OK;
#endif

  /* Verify chip ID with retries (sensor may need time after power up) */

  syslog(LOG_INFO, "SC2336: Checking chip ID...\n");

  for (retry = 0; retry < 3; retry++)
    {
      ret = sc2336_check_chip_id();
      if (ret >= 0)
        {
          break;
        }

      syslog(LOG_WARNING, "SC2336: Chip ID check failed (attempt %d/3), "
             "retrying in 100ms...\n", retry + 1);
      usleep(100000);  /* 100ms delay between retries */
    }

  if (ret < 0)
    {
      syslog(LOG_ERR, "SC2336: Chip ID verification failed after 3 "
             "attempts. Ensure camera module is connected to the "
             "MIPI CSI connector.\n");
      return ret;
    }

  /* Write initialization register sequence */

  ret = sc2336_write_init_sequence();
  if (ret < 0)
    {
      return ret;
    }

  g_sensor_initialized = true;
  syslog(LOG_INFO, "SC2336: Sensor initialized (%dx%d @ %dfps RAW8)\n",
         SC2336_WIDTH, SC2336_HEIGHT, SC2336_FPS);
  return OK;
}

/****************************************************************************
 * Name: esp_cam_sensor_start
 ****************************************************************************/

int esp_cam_sensor_start(void)
{
  if (!g_sensor_initialized)
    {
      return -EINVAL;
    }

  /* Start streaming: exit sleep mode (SC2336_REG_SLEEP_MODE = 1) */

  sc2336_i2c_write_reg(SC2336_REG_SLEEP_MODE, 0x01);

  syslog(LOG_INFO, "SC2336: Streaming started\n");
  return OK;
}

/****************************************************************************
 * Name: esp_cam_sensor_stop
 ****************************************************************************/

int esp_cam_sensor_stop(void)
{
  if (!g_sensor_initialized)
    {
      return -EINVAL;
    }

  /* Stop streaming: enter sleep mode (SC2336_REG_SLEEP_MODE = 0) */

  sc2336_i2c_write_reg(SC2336_REG_SLEEP_MODE, 0x00);

  syslog(LOG_INFO, "SC2336: Streaming stopped\n");
  return OK;
}
