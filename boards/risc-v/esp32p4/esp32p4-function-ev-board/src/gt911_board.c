/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/gt911_board.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>

#include "espressif/esp_i2c.h"
#include "gt911_board.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C bus and address.  GT911 usually answers at 0x5D (7-bit); some
 * modules strap it to 0x14, so probe the primary and fall back.
 */

#define GT911_I2C_BUS        0
#define GT911_ADDR_PRIMARY   0x5d
#define GT911_ADDR_FALLBACK  0x14
#define GT911_FREQUENCY      400000

/* GT911 registers.  The 16-bit register address is written MSB first. */

#define GT911_REG_PRODUCT_ID 0x8140  /* 4-byte ASCII product id ("911") */
#define GT911_REG_STATUS     0x814e  /* bit7=buffer ready, bits0-3=points */
#define GT911_REG_POINT1     0x814f  /* first point, 8 bytes per point */

#define GT911_STATUS_READY   0x80
#define GT911_STATUS_POINTS  0x0f

#define GT911_MAX_POINTS     5
#define GT911_POINT_SIZE     8

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct i2c_master_s *g_gt911_i2c;
static uint8_t g_gt911_addr = GT911_ADDR_PRIMARY;
static bool g_gt911_ready;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gt911_i2c_read
 *
 * Description:
 *   Read bytes from a GT911 register over I2C.  The 16-bit register
 *   address is sent MSB first, matching the NuttX gt9xx driver.
 *
 ****************************************************************************/

static int gt911_i2c_read(uint8_t addr, uint16_t reg, uint8_t *buf,
                          size_t buflen)
{
  uint8_t regbuf[2] =
    {
      reg >> 8,    /* First byte: MSB */
      reg & 0xff   /* Second byte: LSB */
    };

  struct i2c_msg_s msgv[2] =
    {
      {
        .frequency = GT911_FREQUENCY,
        .addr      = addr,
        .flags     = 0,
        .buffer    = regbuf,
        .length    = sizeof(regbuf)
      },
      {
        .frequency = GT911_FREQUENCY,
        .addr      = addr,
        .flags     = I2C_M_READ,
        .buffer    = buf,
        .length    = buflen
      }
    };

  return I2C_TRANSFER(g_gt911_i2c, msgv, 2);
}

/****************************************************************************
 * Name: gt911_i2c_write
 *
 * Description:
 *   Write a single byte to a GT911 register over I2C.
 *
 ****************************************************************************/

static int gt911_i2c_write(uint8_t addr, uint16_t reg, uint8_t val)
{
  uint8_t regbuf[2] =
    {
      reg >> 8,    /* First byte: MSB */
      reg & 0xff   /* Second byte: LSB */
    };

  uint8_t buf[1] =
    {
      val          /* Register value */
    };

  struct i2c_msg_s msgv[2] =
    {
      {
        .frequency = GT911_FREQUENCY,
        .addr      = addr,
        .flags     = 0,
        .buffer    = regbuf,
        .length    = sizeof(regbuf)
      },
      {
        .frequency = GT911_FREQUENCY,
        .addr      = addr,
        .flags     = I2C_M_NOSTART,
        .buffer    = buf,
        .length    = sizeof(buf)
      }
    };

  return I2C_TRANSFER(g_gt911_i2c, msgv, 2);
}

/****************************************************************************
 * Name: gt911_probe
 *
 * Description:
 *   Read the product-id register to confirm a GT911 is present at addr.
 *
 ****************************************************************************/

static int gt911_probe(uint8_t addr)
{
  uint8_t id[4];
  int ret;

  ret = gt911_i2c_read(addr, GT911_REG_PRODUCT_ID, id, sizeof(id));
  if (ret < 0)
    {
      return ret;
    }

  iinfo("GT911 probe addr=0x%02x id=%02x %02x %02x %02x\n",
        addr, id[0], id[1], id[2], id[3]);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gt911_board_init
 ****************************************************************************/

int gt911_board_init(void)
{
  g_gt911_i2c = esp_i2cbus_initialize(GT911_I2C_BUS);
  if (g_gt911_i2c == NULL)
    {
      ierr("ERROR: gt911: failed to get I2C%d bus\n", GT911_I2C_BUS);
      return -ENODEV;
    }

  /* Probe 0x5D first, then fall back to 0x14. */

  if (gt911_probe(GT911_ADDR_PRIMARY) < 0)
    {
      if (gt911_probe(GT911_ADDR_FALLBACK) < 0)
        {
          ierr("ERROR: gt911: no device at 0x5d or 0x14\n");
          return -ENODEV;
        }

      g_gt911_addr = GT911_ADDR_FALLBACK;
    }

  g_gt911_ready = true;
  iinfo("GT911 ready on I2C%d addr=0x%02x\n", GT911_I2C_BUS, g_gt911_addr);

  return OK;
}

/****************************************************************************
 * Name: gt911_board_poll
 ****************************************************************************/

int gt911_board_poll(struct gt911_touch_s *touches, int max_points)
{
  uint8_t data[GT911_MAX_POINTS * GT911_POINT_SIZE];
  uint8_t status;
  int points;
  int ret;
  int i;

  if (!g_gt911_ready)
    {
      return -EAGAIN;
    }

  if (max_points > GT911_MAX_POINTS)
    {
      max_points = GT911_MAX_POINTS;
    }

  /* Read the status register (0x814E). */

  ret = gt911_i2c_read(g_gt911_addr, GT911_REG_STATUS, &status, 1);
  if (ret < 0)
    {
      ierr("ERROR: gt911: status read failed: %d\n", ret);
      return ret;
    }

  /* No new touch data: the caller should retain its previous state. */

  if ((status & GT911_STATUS_READY) == 0)
    {
      return -EAGAIN;
    }

  points = status & GT911_STATUS_POINTS;
  if (points > max_points)
    {
      points = max_points;
    }

  /* Read and decode each point (8 bytes: track_id, x_lo, x_hi, y_lo,
   * y_hi, size_lo, size_hi, reserved).
   */

  if (points > 0)
    {
      ret = gt911_i2c_read(g_gt911_addr, GT911_REG_POINT1, data,
                           points * GT911_POINT_SIZE);
      if (ret < 0)
        {
          ierr("ERROR: gt911: point read failed: %d\n", ret);

          /* Still clear the status so the controller does not stall. */

          (void)gt911_i2c_write(g_gt911_addr, GT911_REG_STATUS, 0);
          return ret;
        }

      for (i = 0; i < points; i++)
        {
          uint8_t *p = &data[i * GT911_POINT_SIZE];

          touches[i].x       = (uint16_t)(p[1] | (p[2] << 8));
          touches[i].y       = (uint16_t)(p[3] | (p[4] << 8));
          touches[i].pressed = (p[0] != 0);
        }
    }

  /* Clear the status register so the controller can report new data. */

  ret = gt911_i2c_write(g_gt911_addr, GT911_REG_STATUS, 0);
  if (ret < 0)
    {
      ierr("ERROR: gt911: status clear failed: %d\n", ret);
      return ret;
    }

  return points;
}
