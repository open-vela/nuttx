/****************************************************************************
 * drivers/sensors/ens160_uorb.c
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
#include <stdio.h>
#include <string.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/sensors/ens160.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/signal.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_ENS160)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ENS160_I2C_FREQUENCY
#  define CONFIG_ENS160_I2C_FREQUENCY 400000
#endif

#ifndef CONFIG_ENS160_CACHE_TIME_MS
#  define CONFIG_ENS160_CACHE_TIME_MS 100
#endif

#define ENS160_REG_PART_ID       0x00
#define ENS160_REG_OPMODE        0x10
#define ENS160_REG_CONFIG        0x11
#define ENS160_REG_TEMP_IN       0x13
#define ENS160_REG_DATA_STATUS   0x20

#define ENS160_OPMODE_SLEEP      0x00
#define ENS160_OPMODE_IDLE       0x01
#define ENS160_OPMODE_STANDARD   0x02
#define ENS160_OPMODE_RESET      0xf0

#define ENS160_STATUS_NEWDAT     (1 << 1)
#define ENS160_STATUS_VALIDITY_SHIFT 2
#define ENS160_STATUS_VALIDITY_MASK  (3 << ENS160_STATUS_VALIDITY_SHIFT)
#define ENS160_STATUS_ERROR      (1 << 6)

#define ENS160_BOOT_DELAY_US     10000
#define ENS160_RESET_DELAY_US    100000
#define ENS160_MODE_DELAY_US     20000

#define ENS160_SENSOR_CO2        0
#define ENS160_SENSOR_TVOC       1
#define ENS160_SENSOR_IAQ        2
#define ENS160_SENSOR_COUNT      3

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ens160_dev_s;

struct ens160_sensor_s
{
  struct sensor_lowerhalf_s lower;
  FAR struct ens160_dev_s  *dev;
  size_t                    event_size;
  uint8_t                   kind;
};

struct ens160_dev_s
{
  struct ens160_sensor_s sensor[ENS160_SENSOR_COUNT];
  FAR struct i2c_master_s *i2c;
  mutex_t lock;
  struct ens160_iaq_s sample;
  uint32_t frequency;
  uint8_t addr;
  bool cache_valid;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ens160_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static int ens160_set_calibvalue(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 unsigned long arg);
static int ens160_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_ens160_ops =
{
  .fetch          = ens160_fetch,
  .set_calibvalue = ens160_set_calibvalue,
  .control        = ens160_control,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ens160_read_regs(FAR struct ens160_dev_s *priv, uint8_t reg,
                            FAR uint8_t *buffer, size_t buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* ENS160 datasheet section 14.1.3 permits a STOP after writing the
   * register pointer, followed by a separate read transfer.  ENS160 retains
   * the pointer across the STOP condition.
   */

  msg.frequency = priv->frequency;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = &reg;
  msg.length    = 1;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      return ret;
    }

  msg.flags  = I2C_M_READ;
  msg.buffer = buffer;
  msg.length = buflen;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return ret < 0 ? ret : OK;
}

static int ens160_write_regs(FAR struct ens160_dev_s *priv, uint8_t reg,
                             FAR const uint8_t *buffer, size_t buflen)
{
  struct i2c_msg_s msg;
  uint8_t transfer[5];
  int ret;

  if (buflen > sizeof(transfer) - 1)
    {
      return -E2BIG;
    }

  transfer[0] = reg;
  memcpy(&transfer[1], buffer, buflen);

  msg.frequency = priv->frequency;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = transfer;
  msg.length    = buflen + 1;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return ret < 0 ? ret : OK;
}

static int ens160_write_reg(FAR struct ens160_dev_s *priv, uint8_t reg,
                            uint8_t value)
{
  return ens160_write_regs(priv, reg, &value, 1);
}

static int ens160_read_part_id(FAR struct ens160_dev_s *priv,
                               FAR uint16_t *part_id)
{
  uint8_t id[2];
  int ret;

  ret = ens160_read_regs(priv, ENS160_REG_PART_ID, id, sizeof(id));
  if (ret >= 0)
    {
      *part_id = (uint16_t)id[0] | (uint16_t)id[1] << 8;
    }

  return ret;
}

static int ens160_set_mode(FAR struct ens160_dev_s *priv, uint8_t mode,
                           useconds_t delay)
{
  int ret;

  ret = ens160_write_reg(priv, ENS160_REG_OPMODE, mode);
  if (ret >= 0 && delay > 0)
    {
      nxsig_usleep(delay);
    }

  return ret;
}

static int ens160_initialize(FAR struct ens160_dev_s *priv)
{
  uint16_t part_id;
  int ret;

  nxsig_usleep(ENS160_BOOT_DELAY_US);

  ret = ens160_read_part_id(priv, &part_id);
  if (ret < 0)
    {
      return ret;
    }

  if (part_id != ENS160_PART_ID)
    {
      snerr("ENS160 unexpected part ID: 0x%04x\n", part_id);
      return -ENODEV;
    }

  ret = ens160_set_mode(priv, ENS160_OPMODE_RESET,
                        ENS160_RESET_DELAY_US);
  if (ret < 0)
    {
      return ret;
    }

  /* Keep all interrupt sources disabled by default. */

  ret = ens160_write_reg(priv, ENS160_REG_CONFIG, 0);
  if (ret < 0)
    {
      return ret;
    }

  ret = ens160_set_mode(priv, ENS160_OPMODE_STANDARD,
                        ENS160_MODE_DELAY_US);
  if (ret >= 0)
    {
      priv->cache_valid = false;
    }

  return ret;
}

static int ens160_read_sample(FAR struct ens160_dev_s *priv)
{
  uint8_t data[6];
  uint8_t status;
  uint64_t now;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  now = sensor_get_timestamp();
  if (priv->cache_valid &&
      now - priv->sample.timestamp <
      (uint64_t)CONFIG_ENS160_CACHE_TIME_MS * 1000)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  ret = ens160_read_regs(priv, ENS160_REG_DATA_STATUS, data, sizeof(data));
  if (ret < 0)
    {
      goto out;
    }

  status = data[0];
  if ((status & ENS160_STATUS_ERROR) != 0)
    {
      snerr("ENS160 reports an operating-mode error: 0x%02x\n", status);
      ret = -EIO;
      goto out;
    }

  priv->sample.timestamp  = sensor_get_timestamp();
  priv->sample.aqi        = data[1];
  priv->sample.tvoc_ppb   = (uint16_t)data[2] |
                            (uint16_t)data[3] << 8;
  priv->sample.eco2_ppm   = (uint16_t)data[4] |
                            (uint16_t)data[5] << 8;
  priv->sample.validity   =
    (status & ENS160_STATUS_VALIDITY_MASK) >>
    ENS160_STATUS_VALIDITY_SHIFT;
  priv->sample.data_ready =
    (status & ENS160_STATUS_NEWDAT) != 0;
  priv->cache_valid = true;
  ret = OK;

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

static int ens160_set_environment(FAR struct ens160_dev_s *priv,
                                  FAR const struct ens160_env_s *env)
{
  uint16_t raw_temperature;
  uint16_t raw_humidity;
  uint8_t data[4];

  if (env == NULL ||
      env->temperature < -40.0f || env->temperature > 85.0f ||
      env->humidity < 0.0f || env->humidity > 100.0f)
    {
      return -ERANGE;
    }

  raw_temperature = (uint16_t)((env->temperature + 273.15f) * 64.0f +
                               0.5f);
  raw_humidity = (uint16_t)(env->humidity * 512.0f + 0.5f);

  data[0] = raw_temperature & 0xff;
  data[1] = raw_temperature >> 8;
  data[2] = raw_humidity & 0xff;
  data[3] = raw_humidity >> 8;

  return ens160_write_regs(priv, ENS160_REG_TEMP_IN, data, sizeof(data));
}

static int ens160_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct ens160_sensor_s *sensor =
    container_of(lower, FAR struct ens160_sensor_s, lower);
  FAR struct ens160_dev_s *priv = sensor->dev;
  int ret;

  if (buflen != sensor->event_size)
    {
      return -EINVAL;
    }

  ret = ens160_read_sample(priv);
  if (ret < 0)
    {
      return ret;
    }

  switch (sensor->kind)
    {
      case ENS160_SENSOR_CO2:
        {
          struct sensor_co2 event;

          event.timestamp = priv->sample.timestamp;
          event.co2       = priv->sample.eco2_ppm;
          memcpy(buffer, &event, sizeof(event));
        }
        break;

      case ENS160_SENSOR_TVOC:
        {
          struct sensor_tvoc event;

          event.timestamp = priv->sample.timestamp;

          /* Sensor Framework TVOC is expressed in ppm; ENS160 reports ppb. */

          event.tvoc = priv->sample.tvoc_ppb / 1000.0f;
          memcpy(buffer, &event, sizeof(event));
        }
        break;

      case ENS160_SENSOR_IAQ:
        memcpy(buffer, &priv->sample, sizeof(priv->sample));
        break;

      default:
        return -EINVAL;
    }

  return buflen;
}

static int ens160_set_calibvalue(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 unsigned long arg)
{
  FAR struct ens160_sensor_s *sensor =
    container_of(lower, FAR struct ens160_sensor_s, lower);
  FAR struct ens160_dev_s *priv = sensor->dev;
  FAR const struct ens160_env_s *env =
    (FAR const struct ens160_env_s *)(uintptr_t)arg;
  int ret;

  if (env == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = ens160_set_environment(priv, env);
  nxmutex_unlock(&priv->lock);
  return ret;
}

static int ens160_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, int cmd,
                          unsigned long arg)
{
  FAR struct ens160_sensor_s *sensor =
    container_of(lower, FAR struct ens160_sensor_s, lower);
  FAR struct ens160_dev_s *priv = sensor->dev;
  int ret;

  switch (cmd)
    {
      case SNIOC_RESET:
        ret = nxmutex_lock(&priv->lock);
        if (ret < 0)
          {
            return ret;
          }

        ret = ens160_initialize(priv);
        nxmutex_unlock(&priv->lock);
        return ret;

      case SNIOC_READID:
        if (arg == 0)
          {
            return -EINVAL;
          }

        ret = nxmutex_lock(&priv->lock);
        if (ret < 0)
          {
            return ret;
          }

        ret = ens160_read_part_id(
          priv, (FAR uint16_t *)(uintptr_t)arg);
        nxmutex_unlock(&priv->lock);
        return ret;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ens160_register(int devno, FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct ens160_dev_s *priv;
  FAR struct ens160_sensor_s *sensor;
  char path[32];
  int ret;

  if (i2c == NULL ||
      (addr != ENS160_I2C_ADDR_LOW && addr != ENS160_I2C_ADDR_HIGH))
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
  priv->frequency = CONFIG_ENS160_I2C_FREQUENCY;
  nxmutex_init(&priv->lock);

  ret = ens160_initialize(priv);
  if (ret < 0)
    {
      snerr("ENS160 probe failed at 0x%02x: %d\n", addr, ret);
      goto err_free;
    }

  sensor = &priv->sensor[ENS160_SENSOR_CO2];
  sensor->dev           = priv;
  sensor->kind          = ENS160_SENSOR_CO2;
  sensor->event_size    = sizeof(struct sensor_co2);
  sensor->lower.ops     = &g_ens160_ops;
  sensor->lower.type    = SENSOR_TYPE_CO2;
  sensor->lower.nbuffer = 1;

  ret = sensor_register(&sensor->lower, devno);
  if (ret < 0)
    {
      goto err_free;
    }

  sensor = &priv->sensor[ENS160_SENSOR_TVOC];
  sensor->dev           = priv;
  sensor->kind          = ENS160_SENSOR_TVOC;
  sensor->event_size    = sizeof(struct sensor_tvoc);
  sensor->lower.ops     = &g_ens160_ops;
  sensor->lower.type    = SENSOR_TYPE_TVOC;
  sensor->lower.nbuffer = 1;

  ret = sensor_register(&sensor->lower, devno);
  if (ret < 0)
    {
      goto err_co2;
    }

  sensor = &priv->sensor[ENS160_SENSOR_IAQ];
  sensor->dev           = priv;
  sensor->kind          = ENS160_SENSOR_IAQ;
  sensor->event_size    = sizeof(struct ens160_iaq_s);
  sensor->lower.ops     = &g_ens160_ops;
  sensor->lower.type    = SENSOR_TYPE_CUSTOM;
  sensor->lower.nbuffer = 1;

  snprintf(path, sizeof(path), "/dev/uorb/sensor_ens160%d", devno);
  ret = sensor_custom_register(&sensor->lower, path,
                               sizeof(struct ens160_iaq_s));
  if (ret < 0)
    {
      goto err_tvoc;
    }

  sninfo("ENS160 registered at 0x%02x as instance %d\n", addr, devno);
  return OK;

err_tvoc:
  sensor_unregister(&priv->sensor[ENS160_SENSOR_TVOC].lower, devno);
err_co2:
  sensor_unregister(&priv->sensor[ENS160_SENSOR_CO2].lower, devno);
err_free:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_ENS160 */
