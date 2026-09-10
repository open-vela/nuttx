/****************************************************************************
 * drivers/sensors/mmc5603.c
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
 * customer/peripherals/sensor/MMC56x3/mmc56x3.c to a NuttX character
 * driver.  The chip is the MEMSIC MMC5603NJ 3-axis geomagnetic sensor at
 * I2C address 0x30, outputting 20-bit magnetic field data
 * (0.0625 mG/LSB, full scale about +/-30 G).
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
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/mmc5603.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MMC5603)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_MMC5603_I2C_FREQUENCY
#  define CONFIG_MMC5603_I2C_FREQUENCY 400000
#endif

#define MMC5603_DATA_LEN        9   /* X/Y/Z 16-bit + 3 high nibbles */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mmc5603_dev_s
{
  FAR struct i2c_master_s *i2c;   /* I2C interface */
  uint8_t addr;                   /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     mmc5603_readreg(FAR struct mmc5603_dev_s *priv,
                               uint8_t regaddr, FAR uint8_t *buf,
                               uint8_t len);
static int     mmc5603_writereg(FAR struct mmc5603_dev_s *priv,
                                uint8_t regaddr, uint8_t regval);
static int     mmc5603_read_id(FAR struct mmc5603_dev_s *priv,
                               FAR uint8_t *id);
static int     mmc5603_reset(FAR struct mmc5603_dev_s *priv);
static int     mmc5603_set_continuous(FAR struct mmc5603_dev_s *priv,
                                      bool continuous);
static int     mmc5603_set_datarate(FAR struct mmc5603_dev_s *priv,
                                    uint16_t rate);
static int     mmc5603_config(FAR struct mmc5603_dev_s *priv);
static int     mmc5603_read_data(FAR struct mmc5603_dev_s *priv,
                                 FAR struct mmc5603_data_s *data);

/* Character device methods */

static ssize_t mmc5603_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static int     mmc5603_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_mmc5603_fops =
{
  NULL,               /* open */
  NULL,               /* close */
  mmc5603_read,       /* read */
  NULL,               /* write */
  NULL,               /* seek */
  mmc5603_ioctl,      /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmc5603_readreg
 *
 * Description:
 *   Read len bytes starting from the specified register (write the
 *   register address, then RESTART and read).
 *
 ****************************************************************************/

static int mmc5603_readreg(FAR struct mmc5603_dev_s *priv,
                           uint8_t regaddr, FAR uint8_t *buf, uint8_t len)
{
  struct i2c_config_s config;
  int ret;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(buf != NULL);

  config.frequency = CONFIG_MMC5603_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address first */

  ret = i2c_write(priv->i2c, &config, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: mmc5603 i2c_write failed: %d\n", ret);
      return ret;
    }

  /* RESTART and then burst-read len bytes */

  ret = i2c_read(priv->i2c, &config, buf, len);
  if (ret < 0)
    {
      snerr("ERROR: mmc5603 i2c_read failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: mmc5603_writereg
 *
 * Description:
 *   Write a single byte to a register (register address + data, no
 *   RESTART).
 *
 ****************************************************************************/

static int mmc5603_writereg(FAR struct mmc5603_dev_s *priv,
                            uint8_t regaddr, uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

  DEBUGASSERT(priv != NULL);

  buffer[0] = regaddr;
  buffer[1] = regval;

  config.frequency = CONFIG_MMC5603_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("ERROR: mmc5603 i2c_write(reg) failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: mmc5603_read_id
 ****************************************************************************/

static int mmc5603_read_id(FAR struct mmc5603_dev_s *priv, FAR uint8_t *id)
{
  return mmc5603_readreg(priv, MMC5603_PRODUCT_ID, id, 1);
}

/****************************************************************************
 * Name: mmc5603_reset
 *
 * Description:
 *   Perform a software reset following the SDK sequence:
 *   CTRL1=0x80 reset -> 20 ms -> CTRL0=0x08 (TM_M) -> 1 ms ->
 *   CTRL0=0x10 (TM_T) -> 1 ms.
 *
 ****************************************************************************/

static int mmc5603_reset(FAR struct mmc5603_dev_s *priv)
{
  int ret;

  ret = mmc5603_writereg(priv, MMC5603_CTRL1_REG, MMC5603_CTRL1_SW_RST);
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(20000);  /* 20 ms reset delay */

  ret = mmc5603_writereg(priv, MMC5603_CTRL0_REG, MMC5603_CTRL0_CMD_SET);
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(1000);

  ret = mmc5603_writereg(priv, MMC5603_CTRL0_REG, MMC5603_CTRL0_CMD_RESET);
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(1000);

  return OK;
}

/****************************************************************************
 * Name: mmc5603_set_continuous
 *
 * Description:
 *   After setting CTRL0.SET, write the CM_FREQ bit of CTRL2 to switch
 *   between continuous and single measurement mode.
 *
 ****************************************************************************/

static int mmc5603_set_continuous(FAR struct mmc5603_dev_s *priv,
                                  bool continuous)
{
  uint8_t regval;
  int ret;

  ret = mmc5603_writereg(priv, MMC5603_CTRL0_REG,
                         MMC5603_CTRL0_CMM_FREQ |
                         MMC5603_CTRL0_AUTO_SR);
  if (ret < 0)
    {
      return ret;
    }

  ret = mmc5603_readreg(priv, MMC5603_CTRL2_REG, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  if (continuous)
    {
      regval |= MMC5603_CTRL2_CMM_EN;
    }
  else
    {
      regval &= ~MMC5603_CTRL2_CMM_EN;
    }

  return mmc5603_writereg(priv, MMC5603_CTRL2_REG, regval);
}

/****************************************************************************
 * Name: mmc5603_set_datarate
 *
 * Description:
 *   Set the output data rate: rate=1000 uses the special setting
 *   (ODR=255 + CTRL2.ODR_1000), otherwise write the ODR register
 *   directly (0-255).
 *
 ****************************************************************************/

static int mmc5603_set_datarate(FAR struct mmc5603_dev_s *priv,
                                uint16_t rate)
{
  uint8_t regval;
  int ret;

  if (rate > 255)
    {
      rate = 1000;
    }

  if (rate == 1000)
    {
      ret = mmc5603_writereg(priv, MMC5603_ODR_REG, 255);
      if (ret < 0)
        {
          return ret;
        }

      ret = mmc5603_readreg(priv, MMC5603_CTRL2_REG, &regval, 1);
      if (ret < 0)
        {
          return ret;
        }

      regval |= MMC5603_CTRL2_ODR_1000;
    }
  else
    {
      ret = mmc5603_writereg(priv, MMC5603_ODR_REG, (uint8_t)rate);
      if (ret < 0)
        {
          return ret;
        }

      ret = mmc5603_readreg(priv, MMC5603_CTRL2_REG, &regval, 1);
      if (ret < 0)
        {
          return ret;
        }

      regval &= ~MMC5603_CTRL2_ODR_1000;
    }

  return mmc5603_writereg(priv, MMC5603_CTRL2_REG, regval);
}

/****************************************************************************
 * Name: mmc5603_config
 *
 * Description:
 *   Power-on configuration: verify product ID -> reset -> 100 Hz data
 *   rate -> continuous measurement mode.
 *
 ****************************************************************************/

static int mmc5603_config(FAR struct mmc5603_dev_s *priv)
{
  uint8_t id = 0;
  int ret;

  /* Read and verify the product ID */

  ret = mmc5603_read_id(priv, &id);
  if (ret < 0)
    {
      return ret;
    }

  sninfo("MMC5603 product id = 0x%02x (expect 0x%02x)\n", id,
         MMC5603_CHIP_ID);
  if (id != MMC5603_CHIP_ID)
    {
      snerr("ERROR: MMC5603 wrong product id 0x%02x\n", id);
      return -ENODEV;
    }

  /* Reset + configure */

  ret = mmc5603_reset(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = mmc5603_set_datarate(priv, 100);   /* 100 Hz */
  if (ret < 0)
    {
      return ret;
    }

  return mmc5603_set_continuous(priv, true);
}

/****************************************************************************
 * Name: mmc5603_read_data
 *
 * Description:
 *   Burst-read 9 bytes and decode the three 20-bit signed magnetic field
 *   values (center offset 2^19 already subtracted).
 *   20-bit assembly: X = buf[0]<<12 | buf[1]<<4 | buf[6]>>4.
 *
 ****************************************************************************/

static int mmc5603_read_data(FAR struct mmc5603_dev_s *priv,
                             FAR struct mmc5603_data_s *data)
{
  uint8_t buf[MMC5603_DATA_LEN];
  int32_t x;
  int32_t y;
  int32_t z;
  int ret;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(data != NULL);

  ret = mmc5603_readreg(priv, MMC5603_OUT_X_L, buf, MMC5603_DATA_LEN);
  if (ret < 0)
    {
      return ret;
    }

  x = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) |
      ((int32_t)buf[6] >> 4);
  y = ((int32_t)buf[2] << 12) | ((int32_t)buf[3] << 4) |
      ((int32_t)buf[7] >> 4);
  z = ((int32_t)buf[4] << 12) | ((int32_t)buf[5] << 4) |
      ((int32_t)buf[8] >> 4);

  /* Subtract the center offset (2^19) to get a 20-bit signed value */

  data->x = x - MMC5603_CENTER_OFFSET;
  data->y = y - MMC5603_CENTER_OFFSET;
  data->z = z - MMC5603_CENTER_OFFSET;

  return OK;
}

/****************************************************************************
 * Name: mmc5603_read
 *
 * Description:
 *   Character device read(): read the 3-axis magnetic field once and
 *   return the data structure.
 *
 ****************************************************************************/

static ssize_t mmc5603_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mmc5603_dev_s *priv = inode->i_private;
  struct mmc5603_data_s data;
  int ret;

  DEBUGASSERT(priv != NULL);

  if (buflen < sizeof(data))
    {
      return -EINVAL;
    }

  ret = mmc5603_read_data(priv, &data);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buffer, &data, sizeof(data));
  return sizeof(data);
}

/****************************************************************************
 * Name: mmc5603_ioctl
 ****************************************************************************/

static int mmc5603_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mmc5603_dev_s *priv = inode->i_private;
  int ret;

  DEBUGASSERT(priv != NULL);

  switch (cmd)
    {
    case SNIOC_MMC5603READ:
      ret = mmc5603_read_data(priv, (FAR struct mmc5603_data_s *)arg);
      break;

    case SNIOC_READID:
      {
        FAR uint8_t *id = (FAR uint8_t *)arg;
        ret = mmc5603_read_id(priv, id);
      }
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
 * Name: mmc5603_register
 ****************************************************************************/

int mmc5603_register(FAR const char *devpath,
                     FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct mmc5603_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(i2c != NULL);

  priv = kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate mmc5603 instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  /* Power-on configuration (includes product ID verification; on
   * failure free the memory and return the error)
   */

  ret = mmc5603_config(priv);
  if (ret < 0)
    {
      snerr("ERROR: mmc5603_config failed: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  ret = register_driver(devpath, &g_mmc5603_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: register_driver failed: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  sninfo("MMC5603 registered as %s\n", devpath);
  return OK;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_MMC5603 */
