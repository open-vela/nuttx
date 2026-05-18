/****************************************************************************
 * drivers/sensors/qma7981.c
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

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/qma7981.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_QMA7981)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* QMA7981 register map (datasheet rev 0.96, table 7) */

#define QMA7981_REG_CHIP_ID         0x00
#define QMA7981_REG_DX_LSB          0x01
#define QMA7981_REG_FSR             0x0f
#define QMA7981_REG_BW              0x10
#define QMA7981_REG_PM              0x11
#define QMA7981_REG_SR              0x36

/* Soft-reset trigger sequence (write 0xb6 then 0x00 to REG_SR) */

#define QMA7981_SOFT_RESET_TRIGGER  0xb6
#define QMA7981_SOFT_RESET_RELEASE  0x00

/* PM register: 0x80 = mode_active, 100 kHz MCK */

#define QMA7981_PM_ACTIVE_100KHZ    0x80

/* Full-scale range: ±8 g (1 LSB ≈ 0.98 mg) */

#define QMA7981_FSR_8G              0x04

/* Bandwidth / output data rate: 100 kHz / 1935 ≈ 51.7 Hz */

#define QMA7981_BW_52HZ             0xe2

/* Valid CHIP_ID range observed across silicon revisions e0..e9.
 * Datasheet does not publish a fixed value; we accept this range to
 * tolerate v1 (0xe0..0xe7) and v2 (0xe8..0xe9) parts.
 */

#define QMA7981_CHIP_ID_MIN         0xe0
#define QMA7981_CHIP_ID_MAX         0xe9

#define QMA7981_I2C_FREQUENCY       400000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct qma7981_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t                  addr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  qma7981_read_reg(FAR struct qma7981_dev_s *priv,
                             uint8_t reg, FAR uint8_t *buf, size_t len);
static int  qma7981_write_reg(FAR struct qma7981_dev_s *priv,
                              uint8_t reg, uint8_t val);
static int  qma7981_chip_init(FAR struct qma7981_dev_s *priv);

static int     qma7981_open(FAR struct file *filep);
static int     qma7981_close(FAR struct file *filep);
static ssize_t qma7981_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t qma7981_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_qma7981_fops =
{
  qma7981_open,   /* open  */
  qma7981_close,  /* close */
  qma7981_read,   /* read  */
  qma7981_write,  /* write */
  NULL,           /* seek  */
  NULL,           /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int qma7981_read_reg(FAR struct qma7981_dev_s *priv,
                            uint8_t reg, FAR uint8_t *buf, size_t len)
{
  struct i2c_msg_s msgs[2];
  int ret;

  msgs[0].frequency = QMA7981_I2C_FREQUENCY;
  msgs[0].addr      = priv->addr;
  msgs[0].flags     = 0;
  msgs[0].buffer    = &reg;
  msgs[0].length    = 1;

  msgs[1].frequency = QMA7981_I2C_FREQUENCY;
  msgs[1].addr      = priv->addr;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = buf;
  msgs[1].length    = len;

  ret = I2C_TRANSFER(priv->i2c, msgs, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER (read reg 0x%02x) failed: %d\n", reg, ret);
    }

  return ret;
}

static int qma7981_write_reg(FAR struct qma7981_dev_s *priv,
                             uint8_t reg, uint8_t val)
{
  struct i2c_msg_s msg;
  uint8_t buf[2];
  int ret;

  buf[0] = reg;
  buf[1] = val;

  msg.frequency = QMA7981_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 2;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER (write reg 0x%02x) failed: %d\n", reg, ret);
    }

  return ret;
}

static int qma7981_chip_init(FAR struct qma7981_dev_s *priv)
{
  uint8_t chip_id = 0;
  int ret;

  /* Read CHIP_ID to confirm the device responds */

  ret = qma7981_read_reg(priv, QMA7981_REG_CHIP_ID, &chip_id, 1);
  if (ret < 0)
    {
      snerr("ERROR: failed to read QMA7981 CHIP_ID\n");
      return ret;
    }

  if (chip_id < QMA7981_CHIP_ID_MIN || chip_id > QMA7981_CHIP_ID_MAX)
    {
      snerr("ERROR: unexpected QMA7981 CHIP_ID 0x%02x (expected 0xe0..0xe9)\n",
            chip_id);
      return -ENODEV;
    }

  sninfo("QMA7981 detected, CHIP_ID=0x%02x\n", chip_id);

  /* Soft-reset: write trigger then release. */

  qma7981_write_reg(priv, QMA7981_REG_SR, QMA7981_SOFT_RESET_TRIGGER);
  up_udelay(100);
  qma7981_write_reg(priv, QMA7981_REG_SR, QMA7981_SOFT_RESET_RELEASE);

  /* Configure: active mode, ±8 g range, ~52 Hz ODR */

  qma7981_write_reg(priv, QMA7981_REG_PM,  QMA7981_PM_ACTIVE_100KHZ);
  qma7981_write_reg(priv, QMA7981_REG_FSR, QMA7981_FSR_8G);
  qma7981_write_reg(priv, QMA7981_REG_BW,  QMA7981_BW_52HZ);

  return OK;
}

/****************************************************************************
 * Character driver methods
 ****************************************************************************/

static int qma7981_open(FAR struct file *filep)
{
  return OK;
}

static int qma7981_close(FAR struct file *filep)
{
  return OK;
}

static ssize_t qma7981_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct qma7981_dev_s *priv = inode->i_private;
  struct qma7981_data_s sample;
  uint8_t raw[6];
  int ret;

  if (buflen < sizeof(sample))
    {
      return -EINVAL;
    }

  /* Burst-read DX_LSB..DZ_MSB (6 bytes).  Each axis is a 14-bit value left-
   * aligned in a little-endian 16-bit field; we shift right by 2 bits to
   * recover the signed sample.
   */

  ret = qma7981_read_reg(priv, QMA7981_REG_DX_LSB, raw, sizeof(raw));
  if (ret < 0)
    {
      return ret;
    }

  sample.x = ((int16_t)((raw[1] << 8) | raw[0])) >> 2;
  sample.y = ((int16_t)((raw[3] << 8) | raw[2])) >> 2;
  sample.z = ((int16_t)((raw[5] << 8) | raw[4])) >> 2;

  memcpy(buffer, &sample, sizeof(sample));
  return sizeof(sample);
}

static ssize_t qma7981_write(FAR struct file *filep,
                             FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int qma7981_register(FAR const char *devpath,
                     FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct qma7981_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL && i2c != NULL);

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: failed to allocate QMA7981 device\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  ret = qma7981_chip_init(priv);
  if (ret < 0)
    {
      snwarn("WARNING: QMA7981 not detected on bus (errno %d).  "
             "Driver will register but read() will fail with EIO.  "
             "Check that the EYE board has the accelerometer "
             "populated and that I2C0 (SDA=GPIO4, SCL=GPIO5) is "
             "wired correctly.\n", ret);
    }

  ret = register_driver(devpath, &g_qma7981_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: register_driver(%s) failed: %d\n", devpath, ret);
      kmm_free(priv);
      return ret;
    }

  sninfo("QMA7981 registered at %s (addr 0x%02x)\n", devpath, addr);
  return OK;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_QMA7981 */
