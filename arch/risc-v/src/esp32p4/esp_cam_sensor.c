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

/* OV5647 Key Registers */

#define OV5647_SW_RESET_REG    0x0103   /* Software reset */
#define OV5647_MIPI_CTRL00     0x4800   /* MIPI control */
#define OV5647_STREAM_ON_REG   0x4202   /* Stream control: 0=on, 1=off */
#define OV5647_IO_PAD_OUTPUT   0x300d   /* IO pad output enable */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Register-value pair for sensor initialization sequence */

struct ov5647_regval_s
{
  uint16_t reg;
  uint8_t  val;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct i2c_master_s *g_i2c_dev;
static bool g_sensor_initialized;

/* Minimal OV5647 initialization sequence for 1024x600 @ 30fps RAW8.
 * This is a subset of the full init sequence. A complete sequence
 * would include PLL setup, timing, analog settings, etc.
 * For initial bring-up, we focus on: reset, ID check, basic config.
 */

static const struct ov5647_regval_s g_ov5647_init_regs[] =
{
  /* Software reset */

  {0x0103, 0x01},

  /* Delay needed after reset - handled in code */

  /* System clock and PLL configuration for 24MHz input, 2-lane MIPI */

  {0x0100, 0x00},   /* Standby mode */
  {0x3034, 0x08},   /* PLL: 8-bit mode */
  {0x3035, 0x21},   /* PLL: system clock divider */
  {0x3036, 0x46},   /* PLL: multiplier */
  {0x303c, 0x11},   /* PLL: MIPI divider */
  {0x3106, 0xf5},   /* SRB control */

  /* Timing control for 1024x600 */

  {0x3820, 0x41},   /* Timing: V flip + V binning */
  {0x3821, 0x07},   /* Timing: H mirror + H binning */
  {0x3808, 0x04},   /* H output size high: 1024 = 0x0400 */
  {0x3809, 0x00},   /* H output size low */
  {0x380a, 0x02},   /* V output size high: 600 = 0x0258 */
  {0x380b, 0x58},   /* V output size low */

  /* H/V window offset */

  {0x3800, 0x00},   /* H crop start high */
  {0x3801, 0x00},   /* H crop start low */
  {0x3802, 0x00},   /* V crop start high */
  {0x3803, 0x00},   /* V crop start low */
  {0x3804, 0x0a},   /* H crop end high: 2623 */
  {0x3805, 0x3f},   /* H crop end low */
  {0x3806, 0x07},   /* V crop end high: 1955 */
  {0x3807, 0xa3},   /* V crop end low */

  /* Timing: total H/V size */

  {0x380c, 0x07},   /* Total H size high */
  {0x380d, 0x68},   /* Total H size low: 1896 */
  {0x380e, 0x03},   /* Total V size high */
  {0x380f, 0xd8},   /* Total V size low: 984 */

  /* Analog and digital settings */

  {0x3612, 0x59},
  {0x3618, 0x00},
  {0x3614, 0x28},
  {0x3630, 0x36},
  {0x3632, 0x44},
  {0x3631, 0x22},

  /* MIPI configuration */

  {0x3a02, 0x03},   /* AEC: max exposure H */
  {0x3a03, 0xd8},   /* AEC: max exposure L */
  {0x3a08, 0x01},   /* AEC: B50 step */
  {0x3a09, 0x27},
  {0x3a0a, 0x00},   /* AEC: B60 step */
  {0x3a0b, 0xf6},
  {0x3a0e, 0x03},   /* AEC: B50 max */
  {0x3a0d, 0x04},   /* AEC: B60 max */
  {0x3a14, 0x03},   /* AEC: max exposure 50Hz H */
  {0x3a15, 0xd8},   /* AEC: max exposure 50Hz L */

  /* ISP configuration */

  {0x5001, 0x01},   /* ISP: enable AWB */
  {0x5000, 0x06},   /* ISP: enable LENC + BPC */

  /* MIPI 2-lane configuration */

  {0x4800, 0x04},   /* MIPI: non-continuous clock */
  {0x4837, 0x18},   /* MIPI: global timing (pclk period) */

  /* Format: RAW8 */

  {0x3034, 0x08},   /* 8-bit output */
  {0x3035, 0x21},

  /* End marker */
};

#define OV5647_INIT_REGS_COUNT \
    (sizeof(g_ov5647_init_regs) / sizeof(g_ov5647_init_regs[0]))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov5647_i2c_write_reg
 *
 * Description:
 *   Write a single byte to an OV5647 register (16-bit address).
 ****************************************************************************/

static int ov5647_i2c_write_reg(uint16_t reg, uint8_t val)
{
  struct i2c_msg_s msg;
  uint8_t buf[3];
  int ret;

  buf[0] = (reg >> 8) & 0xff;   /* Register address high byte */
  buf[1] = reg & 0xff;          /* Register address low byte */
  buf[2] = val;                  /* Data byte */

  msg.frequency = OV5647_I2C_FREQ;
  msg.addr      = OV5647_I2C_ADDR;
  msg.flags     = 0;            /* Write */
  msg.buffer    = buf;
  msg.length    = 3;

  ret = I2C_TRANSFER(g_i2c_dev, &msg, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "OV5647: I2C write failed reg=0x%04x ret=%d\n",
             reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ov5647_i2c_read_reg
 *
 * Description:
 *   Read a single byte from an OV5647 register (16-bit address).
 ****************************************************************************/

static int ov5647_i2c_read_reg(uint16_t reg, uint8_t *val)
{
  struct i2c_msg_s msgs[2];
  uint8_t addr_buf[2];
  int ret;

  /* First message: write register address */

  addr_buf[0] = (reg >> 8) & 0xff;
  addr_buf[1] = reg & 0xff;

  msgs[0].frequency = OV5647_I2C_FREQ;
  msgs[0].addr      = OV5647_I2C_ADDR;
  msgs[0].flags     = 0;         /* Write */
  msgs[0].buffer    = addr_buf;
  msgs[0].length    = 2;

  /* Second message: read data (with repeated start) */

  msgs[1].frequency = OV5647_I2C_FREQ;
  msgs[1].addr      = OV5647_I2C_ADDR;
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
      syslog(LOG_ERR, "OV5647: I2C write-addr failed reg=0x%04x ret=%d\n",
             reg, ret);
      return ret;
    }

  ret = I2C_TRANSFER(g_i2c_dev, &msgs[1], 1);  /* Read data */
  if (ret < 0)
    {
      syslog(LOG_ERR, "OV5647: I2C read failed reg=0x%04x ret=%d\n",
             reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ov5647_check_chip_id
 *
 * Description:
 *   Read and verify the OV5647 chip ID.
 ****************************************************************************/

static int ov5647_check_chip_id(void)
{
  uint8_t id_h;
  uint8_t id_l;
  uint16_t chip_id;
  int ret;

  ret = ov5647_i2c_read_reg(OV5647_CHIP_ID_H_REG, &id_h);
  if (ret < 0)
    {
      return ret;
    }

  ret = ov5647_i2c_read_reg(OV5647_CHIP_ID_L_REG, &id_l);
  if (ret < 0)
    {
      return ret;
    }

  chip_id = ((uint16_t)id_h << 8) | id_l;

  syslog(LOG_INFO, "OV5647: Chip ID = 0x%04x (expected 0x%04x)\n",
         chip_id, OV5647_CHIP_ID);

  if (chip_id != OV5647_CHIP_ID)
    {
      syslog(LOG_ERR, "OV5647: Chip ID mismatch!\n");
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: ov5647_write_init_sequence
 *
 * Description:
 *   Write the initialization register sequence to the sensor.
 ****************************************************************************/

static int ov5647_write_init_sequence(void)
{
  int ret;
  unsigned int i;

  for (i = 0; i < OV5647_INIT_REGS_COUNT; i++)
    {
      ret = ov5647_i2c_write_reg(g_ov5647_init_regs[i].reg,
                                 g_ov5647_init_regs[i].val);
      if (ret < 0)
        {
          syslog(LOG_ERR, "OV5647: Init sequence failed at index %d\n", i);
          return ret;
        }

      /* Small delay between writes for stability */

      if (g_ov5647_init_regs[i].reg == 0x0103)
        {
          /* After software reset, wait 10ms */

          usleep(10000);
        }
    }

  syslog(LOG_INFO, "OV5647: Init sequence written (%d registers)\n",
         (int)OV5647_INIT_REGS_COUNT);
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

  syslog(LOG_INFO, "OV5647: Initializing camera sensor\n");

  /* Get the I2C bus instance.
   * The I2C bus should already be initialized in board bringup.
   * We reference port 0 (which uses GPIO7=SDA, GPIO8=SCL).
   */

#ifdef CONFIG_I2C_DRIVER
  g_i2c_dev = esp_i2cbus_initialize(OV5647_I2C_PORT);
  if (g_i2c_dev == NULL)
    {
      syslog(LOG_ERR, "OV5647: Failed to get I2C bus %d\n", OV5647_I2C_PORT);
      return -ENODEV;
    }
#else
  syslog(LOG_WARNING, "OV5647: I2C not enabled, sensor init skipped\n");
  g_sensor_initialized = true;
  return OK;
#endif

  /* Scan I2C bus for any responsive device (diagnostic) */

  syslog(LOG_INFO, "OV5647: Scanning I2C bus for devices...\n");
  {
    struct i2c_msg_s msg;
    uint8_t dummy = 0;
    int found = 0;

    for (uint8_t addr = 0x08; addr < 0x78; addr++)
      {
        msg.frequency = 100000;
        msg.addr      = addr;
        msg.flags     = I2C_M_READ;
        msg.buffer    = &dummy;
        msg.length    = 1;

        ret = I2C_TRANSFER(g_i2c_dev, &msg, 1);
        if (ret >= 0)
          {
            syslog(LOG_INFO, "OV5647: I2C device at 0x%02x\n", addr);
            found++;
          }
      }

    if (found > 10)
      {
        /* Too many devices = SDA stuck low (bus fault).
         * Attempt I2C bus recovery by sending clock pulses on SCL.
         * This makes any slave holding SDA low release it.
         */

        syslog(LOG_WARNING,
               "OV5647: %d addresses responded (SDA stuck low?). "
               "Attempting bus recovery...\n", found);

        /* Manually toggle SCL GPIO to recover bus.
         * Configure GPIO8 (SCL) as output, pulse it 9+ times.
         */

        volatile uint32_t *gpio_out_w1ts = (volatile uint32_t *)0x500E0008;
        volatile uint32_t *gpio_out_w1tc = (volatile uint32_t *)0x500E000C;
        volatile uint32_t *gpio_enable_w1ts = (volatile uint32_t *)0x500E0024;
        volatile uint32_t *io_mux_gpio8 = (volatile uint32_t *)(0x500E1000 + 0x04 + 8 * 4);

        /* Save current IO MUX config */

        uint32_t saved_mux = *io_mux_gpio8;

        /* Set GPIO8 as simple GPIO output */

        *io_mux_gpio8 = (1 << 12);  /* func_sel = 1 (GPIO) */
        *gpio_enable_w1ts = (1 << 8);

        /* Send 9 clock pulses */

        for (int i = 0; i < 9; i++)
          {
            *gpio_out_w1tc = (1 << 8);  /* SCL low */
            for (volatile int d = 0; d < 4000; d++) { }
            *gpio_out_w1ts = (1 << 8);  /* SCL high */
            for (volatile int d = 0; d < 4000; d++) { }
          }

        /* Generate STOP condition: SDA low->high while SCL is high */

        /* First pull SDA low */

        volatile uint32_t *io_mux_gpio7 = (volatile uint32_t *)(0x500E1000 + 0x04 + 7 * 4);
        uint32_t saved_mux7 = *io_mux_gpio7;
        *io_mux_gpio7 = (1 << 12);
        *gpio_enable_w1ts = (1 << 7);
        *gpio_out_w1tc = (1 << 7);   /* SDA low */
        for (volatile int d = 0; d < 4000; d++) { }
        *gpio_out_w1ts = (1 << 7);   /* SDA high (STOP) */
        for (volatile int d = 0; d < 4000; d++) { }

        /* Restore IO MUX config for I2C function */

        *io_mux_gpio8 = saved_mux;
        *io_mux_gpio7 = saved_mux7;

        syslog(LOG_INFO, "OV5647: Bus recovery complete, "
               "re-initializing I2C...\n");

        /* Re-initialize I2C to restore proper state */

        usleep(10000);  /* 10ms settle */
      }
    else if (found == 0)
      {
        syslog(LOG_WARNING,
               "OV5647: No I2C devices found! Check camera connection.\n");
      }
    else
      {
        syslog(LOG_INFO, "OV5647: Found %d device(s) on I2C bus\n", found);
      }
  }

  /* Verify chip ID with retries (camera may need time after power up) */

  /* First, reset I2C controller to clear any stuck state from prior ops.
   * The I2C FSM may be stuck if a previous transfer was interrupted.
   */

  {
    /* I2C0 base = 0x500C4000 */
    /* HP_SYS_CLKRST_HP_RST_EN0 at 0x500E60C0: bit22 = I2C0 reset */

    volatile uint32_t *rst_en0 = (volatile uint32_t *)0x500E60C0;
    uint32_t val = *rst_en0;
    *rst_en0 = val | (1 << 22);   /* Assert I2C0 reset */
    for (volatile int d = 0; d < 1000; d++) { }
    *rst_en0 = val & ~(1 << 22);  /* Deassert I2C0 reset */
    for (volatile int d = 0; d < 1000; d++) { }

    syslog(LOG_INFO, "OV5647: I2C0 controller reset done\n");

    /* Re-acquire I2C bus after reset */

    g_i2c_dev = esp_i2cbus_initialize(OV5647_I2C_PORT);
    if (g_i2c_dev == NULL)
      {
        syslog(LOG_ERR, "OV5647: Failed to re-get I2C bus after reset\n");
        return -ENODEV;
      }
  }

  for (retry = 0; retry < 3; retry++)
    {
      ret = ov5647_check_chip_id();
      if (ret >= 0)
        {
          break;
        }

      syslog(LOG_WARNING, "OV5647: Chip ID check failed (attempt %d/3), "
             "retrying in 100ms...\n", retry + 1);
      usleep(100000);  /* 100ms delay between retries */
    }

  if (ret < 0)
    {
      syslog(LOG_ERR, "OV5647: Chip ID verification failed after 3 attempts\n");
      syslog(LOG_ERR, "OV5647: Ensure camera module is connected to "
             "MIPI CSI connector\n");
      return ret;
    }

  /* Write initialization register sequence */

  ret = ov5647_write_init_sequence();
  if (ret < 0)
    {
      return ret;
    }

  g_sensor_initialized = true;
  syslog(LOG_INFO, "OV5647: Sensor initialized (%dx%d @ %dfps RAW8)\n",
         OV5647_WIDTH, OV5647_HEIGHT, OV5647_FPS);
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

  /* Start streaming: write 0x00 to stream control register */

  ov5647_i2c_write_reg(OV5647_STREAM_ON_REG, 0x00);

  /* Set MIPI pad output enable */

  ov5647_i2c_write_reg(OV5647_IO_PAD_OUTPUT, 0x00);

  /* Exit standby */

  ov5647_i2c_write_reg(0x0100, 0x01);

  syslog(LOG_INFO, "OV5647: Streaming started\n");
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

  /* Stop streaming: write 0x01 to stream control register */

  ov5647_i2c_write_reg(OV5647_STREAM_ON_REG, 0x01);

  /* Enter standby */

  ov5647_i2c_write_reg(0x0100, 0x00);

  syslog(LOG_INFO, "OV5647: Streaming stopped\n");
  return OK;
}
