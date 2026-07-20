/****************************************************************************
 * drivers/video/ov3660.c
 *
 * SPDX-License-Identifier: Apache-2.0 AND MIT
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
 * The OV3660 register initialization sequences are adapted from the
 * Espressif esp32-camera project (commit 202df95), which carries the
 * following MIT license:
 *
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/ov3660.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_OV3660_I2CADDR
#  define CONFIG_OV3660_I2CADDR 0x3c
#endif

#ifndef CONFIG_OV3660_FREQUENCY
#  define CONFIG_OV3660_FREQUENCY 100000
#endif

#define OV3660_REG_CHIPIDH  0x300a
#define OV3660_REG_CHIPIDL  0x300b
#define OV3660_CHIPID       0x3660

#define REG_DLY             0xffff
#define REGLIST_TAIL        0x0000

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The OV3660 register settings are derived from the OpenMV OV3660 driver as
 * carried by Espressif's esp32-camera project at commit
 * 202df95d7b1dc72e9303ad78f47b8dc9f339e6a1.
 */

static const uint16_t g_ov3660_default_regs[][2] =
{
  {0x3008, 0x82},
  {REG_DLY, 10},
  {0x3103, 0x13},
  {0x3008, 0x42},
  {0x3017, 0xff},
  {0x3018, 0xff},
  {0x302c, 0xc3},
  {0x4740, 0x21},
  {0x3611, 0x01},
  {0x3612, 0x2d},
  {0x3032, 0x00},
  {0x3614, 0x80},
  {0x3618, 0x00},
  {0x3619, 0x75},
  {0x3622, 0x80},
  {0x3623, 0x00},
  {0x3624, 0x03},
  {0x3630, 0x52},
  {0x3632, 0x07},
  {0x3633, 0xd2},
  {0x3704, 0x80},
  {0x3708, 0x66},
  {0x3709, 0x12},
  {0x370b, 0x12},
  {0x3717, 0x00},
  {0x371b, 0x60},
  {0x371c, 0x00},
  {0x3901, 0x13},
  {0x3600, 0x08},
  {0x3620, 0x43},
  {0x3702, 0x20},
  {0x3739, 0x48},
  {0x3730, 0x20},
  {0x370c, 0x0c},
  {0x3a18, 0x00},
  {0x3a19, 0xf8},
  {0x3000, 0x10},
  {0x3004, 0xef},
  {0x6700, 0x05},
  {0x6701, 0x19},
  {0x6702, 0xfd},
  {0x6703, 0xd1},
  {0x6704, 0xff},
  {0x6705, 0xff},
  {0x3c01, 0x80},
  {0x3c00, 0x04},
  {0x3a08, 0x00},
  {0x3a09, 0x62},
  {0x3a0e, 0x08},
  {0x3a0a, 0x00},
  {0x3a0b, 0x52},
  {0x3a0d, 0x09},
  {0x3a00, 0x3a},
  {0x3a14, 0x09},
  {0x3a15, 0x30},
  {0x3a02, 0x09},
  {0x3a03, 0x30},
  {0x440e, 0x08},
  {0x4520, 0x0b},
  {0x460b, 0x37},
  {0x4713, 0x02},
  {0x471c, 0xd0},
  {0x5086, 0x00},
  {0x5002, 0x00},
  {0x501f, 0x00},
  {0x3008, 0x02},
  {0x5180, 0xff},
  {0x5181, 0xf2},
  {0x5182, 0x00},
  {0x5183, 0x14},
  {0x5184, 0x25},
  {0x5185, 0x24},
  {0x5186, 0x16},
  {0x5187, 0x16},
  {0x5188, 0x16},
  {0x5189, 0x68},
  {0x518a, 0x60},
  {0x518b, 0xe0},
  {0x518c, 0xb2},
  {0x518d, 0x42},
  {0x518e, 0x35},
  {0x518f, 0x56},
  {0x5190, 0x56},
  {0x5191, 0xf8},
  {0x5192, 0x04},
  {0x5193, 0x70},
  {0x5194, 0xf0},
  {0x5195, 0xf0},
  {0x5196, 0x03},
  {0x5197, 0x01},
  {0x5198, 0x04},
  {0x5199, 0x12},
  {0x519a, 0x04},
  {0x519b, 0x00},
  {0x519c, 0x06},
  {0x519d, 0x82},
  {0x519e, 0x38},
  {0x5381, 0x1d},
  {0x5382, 0x60},
  {0x5383, 0x03},
  {0x5384, 0x0c},
  {0x5385, 0x78},
  {0x5386, 0x84},
  {0x5387, 0x7d},
  {0x5388, 0x6b},
  {0x5389, 0x12},
  {0x538a, 0x01},
  {0x538b, 0x98},
  {0x5480, 0x01},
  {0x5000, 0xa7},
  {0x5800, 0x0c},
  {0x5801, 0x09},
  {0x5802, 0x0c},
  {0x5803, 0x0c},
  {0x5804, 0x0d},
  {0x5805, 0x17},
  {0x5806, 0x06},
  {0x5807, 0x05},
  {0x5808, 0x04},
  {0x5809, 0x06},
  {0x580a, 0x09},
  {0x580b, 0x0e},
  {0x580c, 0x05},
  {0x580d, 0x01},
  {0x580e, 0x01},
  {0x580f, 0x01},
  {0x5810, 0x05},
  {0x5811, 0x0d},
  {0x5812, 0x05},
  {0x5813, 0x01},
  {0x5814, 0x01},
  {0x5815, 0x01},
  {0x5816, 0x05},
  {0x5817, 0x0d},
  {0x5818, 0x08},
  {0x5819, 0x06},
  {0x581a, 0x05},
  {0x581b, 0x07},
  {0x581c, 0x0b},
  {0x581d, 0x0d},
  {0x581e, 0x12},
  {0x581f, 0x0d},
  {0x5820, 0x0e},
  {0x5821, 0x10},
  {0x5822, 0x10},
  {0x5823, 0x1e},
  {0x5824, 0x53},
  {0x5825, 0x15},
  {0x5826, 0x05},
  {0x5827, 0x14},
  {0x5828, 0x54},
  {0x5829, 0x25},
  {0x582a, 0x33},
  {0x582b, 0x33},
  {0x582c, 0x34},
  {0x582d, 0x16},
  {0x582e, 0x24},
  {0x582f, 0x41},
  {0x5830, 0x50},
  {0x5831, 0x42},
  {0x5832, 0x15},
  {0x5833, 0x25},
  {0x5834, 0x34},
  {0x5835, 0x33},
  {0x5836, 0x24},
  {0x5837, 0x26},
  {0x5838, 0x54},
  {0x5839, 0x25},
  {0x583a, 0x15},
  {0x583b, 0x25},
  {0x583c, 0x53},
  {0x583d, 0xcf},
  {0x3a0f, 0x30},
  {0x3a10, 0x28},
  {0x3a1b, 0x30},
  {0x3a1e, 0x28},
  {0x3a11, 0x60},
  {0x3a1f, 0x14},
  {0x5302, 0x28},
  {0x5303, 0x20},
  {0x5306, 0x1c},
  {0x5307, 0x28},
  {0x4002, 0xc5},
  {0x4003, 0x81},
  {0x4005, 0x12},
  {0x5688, 0x11},
  {0x5689, 0x11},
  {0x568a, 0x11},
  {0x568b, 0x11},
  {0x568c, 0x11},
  {0x568d, 0x11},
  {0x568e, 0x11},
  {0x568f, 0x11},
  {0x5580, 0x06},
  {0x5588, 0x00},
  {0x5583, 0x40},
  {0x5584, 0x2c},
  {0x5001, 0x83},
  {REGLIST_TAIL, 0x00}
};

static const uint16_t g_ov3660_qvga_jpeg_regs[][2] =
{
  {0x3a0f, 0x3b},
  {0x3a10, 0x32},
  {0x3a1b, 0x3b},
  {0x3a1e, 0x32},
  {0x3a11, 0x76},
  {0x3a1f, 0x19},
  {0x501f, 0x00},
  {0x4300, 0x30},
  {0x3002, 0x00},
  {0x3006, 0xff},
  {0x471c, 0x50},
  {0x3800, 0x00},
  {0x3801, 0x00},
  {0x3802, 0x00},
  {0x3803, 0x00},
  {0x3804, 0x08},
  {0x3805, 0x1f},
  {0x3806, 0x06},
  {0x3807, 0x0b},
  {0x3808, 0x01},
  {0x3809, 0x40},
  {0x380a, 0x00},
  {0x380b, 0xf0},
  {0x380c, 0x08},
  {0x380d, 0xfc},
  {0x380e, 0x03},
  {0x380f, 0x0f},
  {0x3810, 0x00},
  {0x3811, 0x08},
  {0x3812, 0x00},
  {0x3813, 0x02},
  {0x3814, 0x31},
  {0x3815, 0x31},
  {0x3820, 0x01},
  {0x3821, 0x21},
  {0x4514, 0xaa},
  {0x4520, 0x0b},
  {0x5001, 0xa3},
  {0x303a, 0x00},
  {0x303b, 0x1e},
  {0x303c, 0x11},
  {0x303d, 0x30},
  {0x3824, 0x0a},
  {0x460c, 0x22},
  {REGLIST_TAIL, 0x00}
};

static const uint16_t g_ov3660_rgb565_qqvga_regs[][2] =
{
  {0x501f, 0x01},
  {0x4300, 0x61},
  {0x3800, 0x00},
  {0x3801, 0x00},
  {0x3802, 0x00},
  {0x3803, 0x00},
  {0x3804, 0x08},
  {0x3805, 0x1f},
  {0x3806, 0x06},
  {0x3807, 0x0b},
  {0x3808, 0x00},
  {0x3809, 0xa0},
  {0x380a, 0x00},
  {0x380b, 0x78},
  {0x380c, 0x08},
  {0x380d, 0xfc},
  {0x380e, 0x03},
  {0x380f, 0x0f},
  {0x3810, 0x00},
  {0x3811, 0x08},
  {0x3812, 0x00},
  {0x3813, 0x02},
  {0x3814, 0x31},
  {0x3815, 0x31},
  {0x3820, 0x01},
  {0x3821, 0x01},
  {0x4514, 0xaa},
  {0x4520, 0x0b},
  {0x5001, 0xa3},
  {0x303a, 0x00},
  {0x303b, 0x08},
  {0x303c, 0x11},
  {0x303d, 0x00},
  {0x3824, 0x08},
  {0x460c, 0x22},
  {REGLIST_TAIL, 0x00}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov3660_write
 *
 * Description:
 *   Write a single 8-bit value to an OV3660 register over I2C/SCCB.
 *
 * Input Parameters:
 *   i2c - I2C master device
 *   reg - 16-bit register address
 *   val - Value to write
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ov3660_write(FAR struct i2c_master_s *i2c,
                        uint16_t reg, uint8_t val)
{
  struct i2c_config_s config;
  uint8_t buf[3];
  int ret;

  buf[0] = (uint8_t)(reg >> 8);
  buf[1] = (uint8_t)(reg & 0xff);
  buf[2] = val;

  config.frequency = CONFIG_OV3660_FREQUENCY;
  config.address   = CONFIG_OV3660_I2CADDR;
  config.addrlen   = 7;

  ret = i2c_write(i2c, &config, buf, sizeof(buf));
  if (ret < 0)
    {
      gerr("ERROR: register 0x%04x write failed: %d\n", reg, ret);
    }

#ifdef CONFIG_OV3660_REGDEBUG
  else
    {
      ginfo("0x%04x <- 0x%02x\n", reg, val);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: ov3660_read
 *
 * Description:
 *   Read a single 8-bit value from an OV3660 register over I2C/SCCB.
 *
 * Input Parameters:
 *   i2c - I2C master device
 *   reg - 16-bit register address
 *
 * Returned Value:
 *   The register value on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ov3660_read(FAR struct i2c_master_s *i2c, uint16_t reg)
{
  struct i2c_config_s config;
  uint8_t addr[2];
  uint8_t val;
  int ret;

  addr[0] = (uint8_t)(reg >> 8);
  addr[1] = (uint8_t)(reg & 0xff);

  config.frequency = CONFIG_OV3660_FREQUENCY;
  config.address   = CONFIG_OV3660_I2CADDR;
  config.addrlen   = 7;

  ret = i2c_writeread(i2c, &config, addr, sizeof(addr), &val, 1);
  if (ret < 0)
    {
      gerr("ERROR: register 0x%04x read failed: %d\n", reg, ret);
      return ret;
    }

#ifdef CONFIG_OV3660_REGDEBUG
  ginfo("0x%04x -> 0x%02x\n", reg, val);
#endif

  return val;
}

/****************************************************************************
 * Name: ov3660_write_regs
 *
 * Description:
 *   Write a list of register-value pairs (or insert delays) to the OV3660.
 *   The list is terminated by REGLIST_TAIL. DELAY entries use REG_DLY as
 *   the register address and the delay duration in milliseconds as value.
 *
 * Input Parameters:
 *   i2c  - I2C master device
 *   regs - Pointer to a REGLIST_TAIL-terminated register table
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ov3660_write_regs(FAR struct i2c_master_s *i2c,
                             FAR const uint16_t (*regs)[2])
{
  int i;
  int ret;

  for (i = 0; regs[i][0] != REGLIST_TAIL; i++)
    {
      if (regs[i][0] == REG_DLY)
        {
          up_mdelay(regs[i][1]);
          continue;
        }

      ret = ov3660_write(i2c, regs[i][0], (uint8_t)regs[i][1]);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov3660_set_mode
 *
 * Description:
 *   Switch OV3660 output format and frame geometry.
 *   The caller is responsible for serializing access to the sensor.
 *
 * Input Parameters:
 *   i2c  - I2C/SCCB bus used by the sensor
 *   mode - Requested mode (see ov3660_mode_e)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ov3660_set_mode(FAR struct i2c_master_s *i2c, enum ov3660_mode_e mode)
{
  int ret;

  if (i2c == NULL)
    {
      return -EINVAL;
    }

  switch (mode)
    {
      case OV3660_MODE_JPEG_QVGA:
        ret = ov3660_write_regs(i2c, g_ov3660_qvga_jpeg_regs);
        break;

      case OV3660_MODE_RGB565_QQVGA:
        ret = ov3660_write_regs(i2c, g_ov3660_rgb565_qqvga_regs);
        break;

      default:
        return -EINVAL;
    }

  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(100);
  return OK;
}

/****************************************************************************
 * Name: ov3660_initialize
 *
 * Description:
 *   Probe and initialize OV3660 with default settings and JPEG QVGA mode.
 *   The board must power the sensor, release PWDN and reset, and provide a
 *   stable 20 MHz XCLK before calling this function.
 *
 * Input Parameters:
 *   i2c - I2C/SCCB bus used by the sensor
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ov3660_initialize(FAR struct i2c_master_s *i2c)
{
  int idh;
  int idl;
  int ret;

  if (i2c == NULL)
    {
      return -EINVAL;
    }

  idh = ov3660_read(i2c, OV3660_REG_CHIPIDH);
  if (idh < 0)
    {
      return idh;
    }

  idl = ov3660_read(i2c, OV3660_REG_CHIPIDL);
  if (idl < 0)
    {
      return idl;
    }

  if (((idh << 8) | idl) != OV3660_CHIPID)
    {
      gerr("ERROR: unsupported chip ID 0x%02x%02x\n", idh, idl);
      return -ENODEV;
    }

  ginfo("OV3660 detected, chip ID 0x%02x%02x\n", idh, idl);

  ret = ov3660_write(i2c, 0x3008, 0x82);
  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(100);

  ret = ov3660_write_regs(i2c, g_ov3660_default_regs);
  if (ret < 0)
    {
      return ret;
    }

  return ov3660_set_mode(i2c, OV3660_MODE_JPEG_QVGA);
}
