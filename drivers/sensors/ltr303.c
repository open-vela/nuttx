/****************************************************************************
 * drivers/sensors/ltr303.c
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

/* Register logic ported from the SiFli-SDK
 * customer/peripherals/sensor/LTR303/LTR303.c to a NuttX character driver.
 * The chip is the LiteOn LTR-303ALS-01 ambient light sensor at I2C address
 * 0x29, with two 16-bit light channels (CH0 = visible + IR full spectrum,
 * CH1 = IR).  Note: the original SDK implementation of SetIntegrationTime /
 * SetMeasurementRate mistakenly wrote ALS_CTRL; this port corrects it to
 * write the MEAS_RATE register.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ltr303.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LTR303)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LTR303_I2C_FREQUENCY
#  define CONFIG_LTR303_I2C_FREQUENCY 400000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ltr303_dev_s
{
  FAR struct i2c_master_s *i2c;   /* I2C interface */
  uint8_t addr;                   /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     ltr303_readreg(FAR struct ltr303_dev_s *priv,
                              uint8_t regaddr, FAR uint8_t *buf,
                              uint8_t len);
static int     ltr303_writereg(FAR struct ltr303_dev_s *priv,
                               uint8_t regaddr, uint8_t regval);
static int     ltr303_modifyreg(FAR struct ltr303_dev_s *priv,
                                uint8_t regaddr, uint8_t clearbits,
                                uint8_t setbits);
static int     ltr303_config(FAR struct ltr303_dev_s *priv);
static int     ltr303_read_data(FAR struct ltr303_dev_s *priv,
                                FAR struct ltr303_data_s *data);

/* Character device methods */

static ssize_t ltr303_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static int     ltr303_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ltr303_fops =
{
  NULL,               /* open */
  NULL,               /* close */
  ltr303_read,        /* read */
  NULL,               /* write */
  NULL,               /* seek */
  ltr303_ioctl,       /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ltr303_readreg
 *
 * Description:
 *   Read len bytes starting from the specified register.
 *
 ****************************************************************************/

static int ltr303_readreg(FAR struct ltr303_dev_s *priv,
                          uint8_t regaddr, FAR uint8_t *buf, uint8_t len)
{
  struct i2c_config_s config;
  int ret;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(buf != NULL);

  config.frequency = CONFIG_LTR303_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  ret = i2c_write(priv->i2c, &config, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: ltr303 i2c_write failed: %d\n", ret);
      return ret;
    }

  ret = i2c_read(priv->i2c, &config, buf, len);
  if (ret < 0)
    {
      snerr("ERROR: ltr303 i2c_read failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ltr303_writereg
 *
 * Description:
 *   Write a single byte to a register.
 *
 ****************************************************************************/

static int ltr303_writereg(FAR struct ltr303_dev_s *priv,
                           uint8_t regaddr, uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

  DEBUGASSERT(priv != NULL);

  buffer[0] = regaddr;
  buffer[1] = regval;

  config.frequency = CONFIG_LTR303_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("ERROR: ltr303 i2c_write(reg) failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ltr303_modifyreg
 *
 * Description:
 *   Read-modify-write a register (clear clearbits, set setbits).
 *
 ****************************************************************************/

static int ltr303_modifyreg(FAR struct ltr303_dev_s *priv,
                            uint8_t regaddr, uint8_t clearbits,
                            uint8_t setbits)
{
  uint8_t regval;
  int ret;

  ret = ltr303_readreg(priv, regaddr, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~clearbits;
  regval |= setbits;

  return ltr303_writereg(priv, regaddr, regval);
}

/****************************************************************************
 * Name: ltr303_config
 *
 * Description:
 *   Power-on configuration: verify ID (soft check, warn only), then
 *   enable ALS + gain 1x + integration time 100 ms + rate 50 ms.
 *
 ****************************************************************************/

static int ltr303_config(FAR struct ltr303_dev_s *priv)
{
  uint8_t part_id = 0;
  uint8_t manu_id = 0;
  int ret;

  /* Read the part/manufacturer ID (soft check: only warn on mismatch
   * so the real values can be observed on the first board bring-up)
   */

  ret = ltr303_readreg(priv, LTR303_PART_ID_REG, &part_id, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = ltr303_readreg(priv, LTR303_MANU_ID_REG, &manu_id, 1);
  if (ret < 0)
    {
      return ret;
    }

  sninfo("LTR303 part id = 0x%02x (expect 0x%02x), manu id = 0x%02x\n",
         part_id, LTR303_PART_ID_VALUE, manu_id);

  if (part_id != LTR303_PART_ID_VALUE)
    {
      snwarn("WARN: LTR303 unexpected part id 0x%02x\n", part_id);
    }

  /* Enable ALS + gain 1x (clear the gain bits, set the mode bit) */

  ret = ltr303_writereg(priv, LTR303_ALS_CTRL, LTR303_ALS_CTRL_MODE);
  if (ret < 0)
    {
      return ret;
    }

  /* Integration time 100 ms (bits[5:3]=0) + rate 50 ms (bits[2:0]=0) */

  return ltr303_writereg(priv, LTR303_MEAS_RATE, 0x00);
}

/****************************************************************************
 * Name: ltr303_read_data
 *
 * Description:
 *   Burst-read 4 bytes: CH1 low/high + CH0 low/high.  CH0 = full
 *   spectrum (visible + IR), CH1 = IR.
 *
 ****************************************************************************/

static int ltr303_read_data(FAR struct ltr303_dev_s *priv,
                            FAR struct ltr303_data_s *data)
{
  uint8_t buf[4];
  int ret;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(data != NULL);

  ret = ltr303_readreg(priv, LTR303_CH1DATA, buf, 4);
  if (ret < 0)
    {
      return ret;
    }

  data->ch1 = ((uint16_t)buf[1] << 8) | buf[0];   /* CH1 = infrared (IR) */
  data->ch0 = ((uint16_t)buf[3] << 8) | buf[2];   /* CH0 = visible + IR */

  return OK;
}

/****************************************************************************
 * Name: ltr303_read
 *
 * Description:
 *   Character device read(): read both light channels once and return
 *   the data structure.
 *
 ****************************************************************************/

static ssize_t ltr303_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ltr303_dev_s *priv = inode->i_private;
  struct ltr303_data_s data;
  int ret;

  DEBUGASSERT(priv != NULL);

  if (buflen < sizeof(data))
    {
      return -EINVAL;
    }

  ret = ltr303_read_data(priv, &data);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buffer, &data, sizeof(data));
  return sizeof(data);
}

/****************************************************************************
 * Name: ltr303_ioctl
 ****************************************************************************/

static int ltr303_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ltr303_dev_s *priv = inode->i_private;
  int ret;

  DEBUGASSERT(priv != NULL);

  switch (cmd)
    {
    case SNIOC_LTR303READ:
      ret = ltr303_read_data(priv, (FAR struct ltr303_data_s *)arg);
      break;

    default:
      snerr("ERROR: Unrecognized cmd: %d arg: %lu\n", cmd, arg);
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ltr303_register
 ****************************************************************************/

int ltr303_register(FAR const char *devpath,
                    FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct ltr303_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(i2c != NULL);

  priv = kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate ltr303 instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  ret = ltr303_config(priv);
  if (ret < 0)
    {
      snerr("ERROR: ltr303_config failed: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  ret = register_driver(devpath, &g_ltr303_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: register_driver failed: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  sninfo("LTR303 registered as %s\n", devpath);
  return OK;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_LTR303 */
