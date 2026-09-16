/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_thermal.c
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_thermal.h"

#if defined(CONFIG_LS2K0300_THERMAL)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LS2K_MIN_TEMP     -40
#define LS2K_MAX_TEMP     125
#define THSENS_OUT_OFFSET 100

struct ls2k0300_thermal_dev_s
{
  uintptr_t base;
  int id;
  bool opened;
};

static uint32_t thermal_getreg(FAR struct ls2k0300_thermal_dev_s *priv,
                               uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/* Set the temperature alarm thresholds
 *
 * low : temperature in degree
 * high: temperature in degree
 */

static int ls2k_tsensor_set(struct ls2k0300_thermal_dev_s *priv,
                            int low, int high, bool enable)
{
  uint32_t reg_ctrl = 0;
  int reg_off;
  uint8_t lo_en = 8;
  uint8_t hi_en = 8;

  /* thsens_int_ctrl register have 4 sets of setup bits each using 2 Bytes */

  reg_off = priv->id * 2;

  if (low > high)
    {
      return -EINVAL;
    }

  low = low < LS2K_MIN_TEMP ? LS2K_MIN_TEMP : low;
  high = high > LS2K_MAX_TEMP ? LS2K_MAX_TEMP : high;

  low += THSENS_OUT_OFFSET;
  high += THSENS_OUT_OFFSET;

  reg_ctrl |= low;
  reg_ctrl |= enable ? (1 << lo_en) : 0;
  putreg16(reg_ctrl, priv->base + LS2K0300_TSENSOR_CTRL_LO + reg_off);

  reg_ctrl = 0;
  reg_ctrl |= high;
  reg_ctrl |= enable ? (1 << hi_en) : 0;
  putreg16(reg_ctrl, priv->base + LS2K0300_TSENSOR_CTRL_HI + reg_off);

  return 0;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ls2k0300_thermal_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ls2k0300_thermal_dev_s *priv = inode->i_private;

  if (priv->opened)
    {
      return -EBUSY;
    }

  priv->opened = true;
  return OK;
}

static int ls2k0300_thermal_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ls2k0300_thermal_dev_s *priv = inode->i_private;

  priv->opened = false;
  return OK;
}

static ssize_t ls2k0300_thermal_read(FAR struct file *filep,
                                    FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ls2k0300_thermal_dev_s *priv = inode->i_private;
  uint32_t reg_val;
  int32_t temp;

  if (buflen < sizeof(int32_t))
    {
      return -EINVAL;
    }

  reg_val = thermal_getreg(priv, LS2K0300_TSENSOR_OUT);
  temp = ((reg_val & 0x7ff) * 569) - 394700;

  memcpy(buffer, &temp, sizeof(int32_t));
  return sizeof(int32_t);
}

static int ls2k0300_thermal_ioctl(FAR struct file *filep, int cmd,
                                 unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ls2k0300_thermal_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      case 0x1001:
        {
          int32_t *temp = (int32_t *)(uintptr_t)arg;
          uint32_t reg_val;

          reg_val = thermal_getreg(priv, LS2K0300_TSENSOR_OUT);
          *temp = ((reg_val & 0x7ff) * 569) - 394700;
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static const struct file_operations g_thermal_fops =
{
  ls2k0300_thermal_open,
  ls2k0300_thermal_close,
  ls2k0300_thermal_read,
  NULL,
  NULL,
  ls2k0300_thermal_ioctl,
  NULL
};

int ls2k0300_thermal_initialize(void)
{
  struct ls2k0300_thermal_dev_s *priv;
  int ret;

  priv = kmm_malloc(sizeof(struct ls2k0300_thermal_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->base = LS2K0300_TSENSOR_BASE;
  priv->id = 0;
  priv->opened = false;

  putreg8(0xff, priv->base + LS2K0300_TSENSOR_STATUS);

  /* Enable thermal sensor */

  putreg32(0x0000ff03, priv->base + LS2K0300_TSENSOR_CFG);

  ls2k_tsensor_set(priv, 5, 120, false);

  ret = register_driver("/dev/thermal0", &g_thermal_fops, 0666, priv);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  return OK;
}

#endif
