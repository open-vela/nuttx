/****************************************************************************
 * drivers/sensors/aht2x_uorb.c
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
#include <string.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/sensors/aht2x.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/signal.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_AHT2X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_AHT2X_I2C_FREQUENCY
#  define CONFIG_AHT2X_I2C_FREQUENCY 400000
#endif

#ifndef CONFIG_AHT2X_CACHE_TIME_MS
#  define CONFIG_AHT2X_CACHE_TIME_MS 250
#endif

#define AHT2X_CMD_INITIALIZE    0xbe
#define AHT2X_CMD_TRIGGER       0xac
#define AHT2X_CMD_SOFT_RESET    0xba

#define AHT2X_STATUS_BUSY       (1 << 7)
#define AHT2X_STATUS_CALIBRATED (1 << 3)

#define AHT2X_POWERON_DELAY_US  100000
#define AHT2X_RESET_DELAY_US    20000
#define AHT2X_INIT_DELAY_US     10000
#define AHT2X_MEASURE_DELAY_US  80000
#define AHT2X_BUSY_DELAY_US     10000
#define AHT2X_BUSY_RETRIES      10

#define AHT2X_SENSOR_HUMIDITY   0
#define AHT2X_SENSOR_TEMPERATURE 1
#define AHT2X_SENSOR_COUNT      2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct aht2x_dev_s;

struct aht2x_sensor_s
{
  struct sensor_lowerhalf_s lower;
  FAR struct aht2x_dev_s   *dev;
  size_t                    event_size;
};

struct aht2x_sample_s
{
  uint64_t timestamp;
  float temperature;
  float humidity;
};

struct aht2x_dev_s
{
  struct aht2x_sensor_s sensor[AHT2X_SENSOR_COUNT];
  FAR struct i2c_master_s *i2c;
  mutex_t lock;
  struct aht2x_sample_s sample;
  uint32_t frequency;
  uint8_t addr;
  bool cache_valid;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int aht2x_fetch(FAR struct sensor_lowerhalf_s *lower,
                       FAR struct file *filep, FAR char *buffer,
                       size_t buflen);
static int aht2x_control(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_aht2x_ops =
{
  .fetch   = aht2x_fetch,
  .control = aht2x_control,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int aht2x_write(FAR struct aht2x_dev_s *priv,
                       FAR const uint8_t *buffer, size_t buflen)
{
  struct i2c_msg_s msg;
  int ret;

  msg.frequency = priv->frequency;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)buffer;
  msg.length    = buflen;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return ret < 0 ? ret : OK;
}

static int aht2x_read(FAR struct aht2x_dev_s *priv,
                      FAR uint8_t *buffer, size_t buflen)
{
  struct i2c_msg_s msg;
  int ret;

  msg.frequency = priv->frequency;
  msg.addr      = priv->addr;
  msg.flags     = I2C_M_READ;
  msg.buffer    = buffer;
  msg.length    = buflen;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return ret < 0 ? ret : OK;
}

static uint8_t aht2x_crc8(FAR const uint8_t *buffer, size_t buflen)
{
  uint8_t crc = 0xff;
  size_t i;
  int bit;

  for (i = 0; i < buflen; i++)
    {
      crc ^= buffer[i];
      for (bit = 0; bit < 8; bit++)
        {
          crc = (crc & 0x80) != 0 ? (crc << 1) ^ 0x31 : crc << 1;
        }
    }

  return crc;
}

static int aht2x_get_status(FAR struct aht2x_dev_s *priv,
                            FAR uint8_t *status)
{
  /* AHT20/AHT21 return the current status as the first byte of a plain
   * read.  This is also the sequence used by the module reference library.
   */

  return aht2x_read(priv, status, 1);
}

static int aht2x_initialize(FAR struct aht2x_dev_s *priv)
{
  uint8_t initialize[] =
  {
    AHT2X_CMD_INITIALIZE, 0x08, 0x00
  };

  uint8_t status;
  int ret;

  nxsig_usleep(AHT2X_POWERON_DELAY_US);

  ret = aht2x_get_status(priv, &status);
  if (ret < 0)
    {
      return ret;
    }

  if ((status & AHT2X_STATUS_CALIBRATED) == 0)
    {
      ret = aht2x_write(priv, initialize, sizeof(initialize));
      if (ret < 0)
        {
          return ret;
        }

      nxsig_usleep(AHT2X_INIT_DELAY_US);

      ret = aht2x_get_status(priv, &status);
      if (ret < 0)
        {
          return ret;
        }

      if ((status & AHT2X_STATUS_CALIBRATED) == 0)
        {
          snerr("AHT2x calibration-enable bit did not set\n");
          return -EIO;
        }
    }

  return OK;
}

static int aht2x_reset(FAR struct aht2x_dev_s *priv)
{
  uint8_t command = AHT2X_CMD_SOFT_RESET;
  int ret;

  ret = aht2x_write(priv, &command, 1);
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(AHT2X_RESET_DELAY_US);
  priv->cache_valid = false;
  return aht2x_initialize(priv);
}

static int aht2x_measure(FAR struct aht2x_dev_s *priv)
{
  uint8_t command[] =
  {
    AHT2X_CMD_TRIGGER, 0x33, 0x00
  };

  uint8_t data[7];
  uint32_t raw_humidity;
  uint32_t raw_temperature;
  uint64_t now;
  int retries;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  now = sensor_get_timestamp();
  if (priv->cache_valid &&
      now - priv->sample.timestamp <
      (uint64_t)CONFIG_AHT2X_CACHE_TIME_MS * 1000)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  ret = aht2x_write(priv, command, sizeof(command));
  if (ret < 0)
    {
      goto out;
    }

  nxsig_usleep(AHT2X_MEASURE_DELAY_US);

  for (retries = 0; retries < AHT2X_BUSY_RETRIES; retries++)
    {
      ret = aht2x_read(priv, data, sizeof(data));
      if (ret < 0)
        {
          goto out;
        }

      if ((data[0] & AHT2X_STATUS_BUSY) == 0)
        {
          break;
        }

      nxsig_usleep(AHT2X_BUSY_DELAY_US);
    }

  if ((data[0] & AHT2X_STATUS_BUSY) != 0)
    {
      ret = -ETIMEDOUT;
      goto out;
    }

  if (aht2x_crc8(data, 6) != data[6])
    {
      snerr("AHT2x measurement CRC mismatch\n");
      ret = -EBADMSG;
      goto out;
    }

  raw_humidity = ((uint32_t)data[1] << 12) |
                 ((uint32_t)data[2] << 4) |
                 ((uint32_t)data[3] >> 4);
  raw_temperature = ((uint32_t)(data[3] & 0x0f) << 16) |
                    ((uint32_t)data[4] << 8) |
                    data[5];

  priv->sample.timestamp = sensor_get_timestamp();
  priv->sample.humidity = raw_humidity * (100.0f / 1048576.0f);
  priv->sample.temperature =
    raw_temperature * (200.0f / 1048576.0f) - 50.0f;

  if (priv->sample.humidity < 0.0f)
    {
      priv->sample.humidity = 0.0f;
    }
  else if (priv->sample.humidity > 100.0f)
    {
      priv->sample.humidity = 100.0f;
    }

  priv->cache_valid = true;
  ret = OK;

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

static int aht2x_fetch(FAR struct sensor_lowerhalf_s *lower,
                       FAR struct file *filep, FAR char *buffer,
                       size_t buflen)
{
  FAR struct aht2x_sensor_s *sensor =
    container_of(lower, FAR struct aht2x_sensor_s, lower);
  FAR struct aht2x_dev_s *priv = sensor->dev;
  int ret;

  if (buflen != sensor->event_size)
    {
      return -EINVAL;
    }

  ret = aht2x_measure(priv);
  if (ret < 0)
    {
      return ret;
    }

  if (lower->type == SENSOR_TYPE_AMBIENT_TEMPERATURE)
    {
      struct sensor_temp event;

      event.timestamp   = priv->sample.timestamp;
      event.temperature = priv->sample.temperature;
      memcpy(buffer, &event, sizeof(event));
    }
  else if (lower->type == SENSOR_TYPE_RELATIVE_HUMIDITY)
    {
      struct sensor_humi event;

      event.timestamp = priv->sample.timestamp;
      event.humidity  = priv->sample.humidity;
      memcpy(buffer, &event, sizeof(event));
    }
  else
    {
      return -EINVAL;
    }

  return buflen;
}

static int aht2x_control(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, int cmd,
                         unsigned long arg)
{
  FAR struct aht2x_sensor_s *sensor =
    container_of(lower, FAR struct aht2x_sensor_s, lower);
  FAR struct aht2x_dev_s *priv = sensor->dev;
  int ret;

  switch (cmd)
    {
      case SNIOC_RESET:
        ret = nxmutex_lock(&priv->lock);
        if (ret < 0)
          {
            return ret;
          }

        ret = aht2x_reset(priv);
        nxmutex_unlock(&priv->lock);
        return ret;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int aht2x_register(int devno, FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct aht2x_dev_s *priv;
  FAR struct aht2x_sensor_s *sensor;
  int ret;

  if (i2c == NULL || addr != AHT2X_I2C_ADDR)
    {
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c       = i2c;
  priv->addr      = addr;
  priv->frequency = CONFIG_AHT2X_I2C_FREQUENCY;
  nxmutex_init(&priv->lock);

  ret = aht2x_initialize(priv);
  if (ret < 0)
    {
      snerr("AHT2x probe failed: %d\n", ret);
      goto err_free;
    }

  sensor = &priv->sensor[AHT2X_SENSOR_TEMPERATURE];
  sensor->dev             = priv;
  sensor->event_size      = sizeof(struct sensor_temp);
  sensor->lower.ops       = &g_aht2x_ops;
  sensor->lower.type      = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->lower.nbuffer   = 1;

  ret = sensor_register(&sensor->lower, devno);
  if (ret < 0)
    {
      goto err_free;
    }

  sensor = &priv->sensor[AHT2X_SENSOR_HUMIDITY];
  sensor->dev             = priv;
  sensor->event_size      = sizeof(struct sensor_humi);
  sensor->lower.ops       = &g_aht2x_ops;
  sensor->lower.type      = SENSOR_TYPE_RELATIVE_HUMIDITY;
  sensor->lower.nbuffer   = 1;

  ret = sensor_register(&sensor->lower, devno);
  if (ret < 0)
    {
      sensor_unregister(
        &priv->sensor[AHT2X_SENSOR_TEMPERATURE].lower, devno);
      goto err_free;
    }

  sninfo("AHT20/AHT21 registered as temperature/humidity instance %d\n",
         devno);
  return OK;

err_free:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_AHT2X */
