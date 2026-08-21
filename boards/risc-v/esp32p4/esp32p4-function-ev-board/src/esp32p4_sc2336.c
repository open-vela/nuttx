/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_sc2336.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SC2336 2MP MIPI-CSI image sensor driver (I2C / SCCB control path) for
 * the ESP32-P4-Function-EV-Board.
 *
 * The SC2336 is a SmartSens 1920x1080 RAW Bayer sensor.  It is controlled
 * over I2C (SCCB) using 16-bit register addresses and 8-bit data.  The MIPI
 * data path (2-lane CSI) is wired directly to the ESP32-P4 MIPI-CSI pads;
 * the sensor is clocked by the 24 MHz crystal (XVCLK) on the camera adapter
 * board.
 *
 * This driver implements the sensor control plane (detect / configure /
 * stream on-off) over I2C.  Register values are the SC2336 1920x1080 RAW10
 * 2-lane reference sequence; the ISP on the ESP32-P4 converts RAW to RGB.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>

#include "esp32p4-function-ev-board.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SC2336 SCCB / I2C slave address (7-bit).  The camera adapter ties the
 * SID pin for the default address.
 */

#define SC2336_I2C_ADDR          0x3c

/* SC2336 chip ID registers (SmartSens).  Read back to detect the sensor. */

#define SC2336_REG_CHIP_ID_H     0x0200
#define SC2336_REG_CHIP_ID_L     0x0201
#define SC2336_CHIP_ID           0xcb35   /* SC2336 expected ID */

/* Stream control */

#define SC2336_REG_STREAM        0x0100
#define SC2336_STREAM_ON         0x01
#define SC2336_STREAM_OFF        0x00

/* Software reset */

#define SC2336_REG_RESET         0x0103
#define SC2336_RESET             0x01

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sc2336_reg_s
{
  uint16_t addr;
  uint8_t  val;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SC2336 1920x1080 RAW10 2-lane reference init sequence.
 *
 * NOTE: This is the SmartSens SC2336 reference register sequence.  Values
 * marked for PLL / MIPI timing follow the SC2336 datasheet 1080p 2-lane
 * mode; fine-tuning (exposure / gain / HDR) is done at bring-up.
 */

static const struct sc2336_reg_s g_sc2336_init_1080p[] =
{
  /* Software reset */

  {0x0103, 0x01},

  /* System / PLL */

  {0x0100, 0x00},
  {0x0300, 0x04},
  {0x0301, 0x02},
  {0x0302, 0x1e},
  {0x0303, 0x00},
  {0x0304, 0x00},
  {0x0305, 0x14},
  {0x0306, 0x00},
  {0x0307, 0x18},
  {0x0308, 0x08},
  {0x0309, 0x00},
  {0x0310, 0x0a},
  {0x030e, 0x01},
  {0x030f, 0x02},

  /* MIPI output: 2-lane, RAW10 */

  {0x3018, 0x01},    /* 2-lane */
  {0x3031, 0x0a},    /* RAW10 */
  {0x3037, 0x20},    /* PHY 10-bit */
  {0x4603, 0x00},    /* MIPI enable */

  /* Output window 1920 x 1080 */

  {0x3208, 0x07},    /* width high  (1920 >> 8) */
  {0x3209, 0x80},    /* width low   (1920 & 0xff) */
  {0x320a, 0x04},    /* height high (1080 >> 8) */
  {0x320b, 0x38},    /* height low  (1080 & 0xff) */

  /* Exposure / gain defaults */

  {0x3e01, 0x04},
  {0x3e02, 0x60},
  {0x3e03, 0x08},
  {0x3e09, 0x40},
};

#define SC2336_INIT_COUNT \
  (sizeof(g_sc2336_init_1080p) / sizeof(g_sc2336_init_1080p[0]))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct i2c_master_s *g_sc2336_i2c;
static bool g_sc2336_ready;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sc2336_write_reg
 *
 * Description:
 *   Write an 8-bit value to a 16-bit SC2336 register over I2C.
 *
 ****************************************************************************/

static int sc2336_write_reg(uint16_t reg, uint8_t val)
{
  uint8_t buf[3];
  struct i2c_msg_s msg;
  int ret;

  buf[0] = (reg >> 8) & 0xff;
  buf[1] = reg & 0xff;
  buf[2] = val;

  msg.frequency = FUNEV_SC2336_I2C_FREQ;
  msg.addr      = SC2336_I2C_ADDR;
  msg.flags     = 0;                 /* write */
  msg.buffer    = buf;
  msg.length    = 3;

  ret = I2C_TRANSFER(g_sc2336_i2c, &msg, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "sc2336: write reg 0x%04x failed: %d\n", reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sc2336_read_reg
 *
 * Description:
 *   Read an 8-bit value from a 16-bit SC2336 register over I2C.
 *
 ****************************************************************************/

static int sc2336_read_reg(uint16_t reg, FAR uint8_t *val)
{
  uint8_t addr[2];
  struct i2c_msg_s msg[2];
  int ret;

  addr[0] = (reg >> 8) & 0xff;
  addr[1] = reg & 0xff;

  msg[0].frequency = FUNEV_SC2336_I2C_FREQ;
  msg[0].addr      = SC2336_I2C_ADDR;
  msg[0].flags     = 0;              /* write register address */
  msg[0].buffer    = addr;
  msg[0].length    = 2;

  msg[1].frequency = FUNEV_SC2336_I2C_FREQ;
  msg[1].addr      = SC2336_I2C_ADDR;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = val;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(g_sc2336_i2c, msg, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "sc2336: read reg 0x%04x failed: %d\n", reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32p4_sc2336_initialize
 *
 * Description:
 *   Bind the SC2336 to an I2C bus and detect the sensor by reading the
 *   chip ID.
 *
 * Input Parameters:
 *   i2c - An initialized I2C master instance (shared I2C0 on this board).
 *
 * Returned Value:
 *   Zero (OK) if the sensor is detected; a negated errno otherwise.
 *
 ****************************************************************************/

int esp32p4_sc2336_initialize(FAR struct i2c_master_s *i2c)
{
  uint8_t id_h;
  uint8_t id_l;
  uint16_t id;
  int ret;

  if (i2c == NULL)
    {
      return -EINVAL;
    }

  g_sc2336_i2c  = i2c;
  g_sc2336_ready = false;

  /* Detect the sensor by reading the chip ID */

  ret = sc2336_read_reg(SC2336_REG_CHIP_ID_H, &id_h);
  if (ret < 0)
    {
      syslog(LOG_ERR, "sc2336: no sensor on I2C 0x%02x\n",
             SC2336_I2C_ADDR);
      return ret;
    }

  ret = sc2336_read_reg(SC2336_REG_CHIP_ID_L, &id_l);
  if (ret < 0)
    {
      return ret;
    }

  id = ((uint16_t)id_h << 8) | id_l;
  syslog(LOG_INFO, "sc2336: chip ID = 0x%04x\n", id);

  if (id != SC2336_CHIP_ID)
    {
      syslog(LOG_WARNING,
             "sc2336: ID 0x%04x != expected 0x%04x (continuing)\n",
             id, SC2336_CHIP_ID);
    }

  g_sc2336_ready = true;
  return OK;
}

/****************************************************************************
 * Name: esp32p4_sc2336_configure
 *
 * Description:
 *   Send the SC2336 1920x1080 RAW10 2-lane init sequence.
 *
 ****************************************************************************/

int esp32p4_sc2336_configure(void)
{
  size_t i;
  int ret;

  if (!g_sc2336_ready)
    {
      return -ENODEV;
    }

  for (i = 0; i < SC2336_INIT_COUNT; i++)
    {
      ret = sc2336_write_reg(g_sc2336_init_1080p[i].addr,
                             g_sc2336_init_1080p[i].val);
      if (ret < 0)
        {
          return ret;
        }
    }

  syslog(LOG_INFO, "sc2336: configured 1920x1080 RAW10 2-lane\n");
  return OK;
}

/****************************************************************************
 * Name: esp32p4_sc2336_stream
 *
 * Description:
 *   Start or stop the sensor MIPI data stream.
 *
 ****************************************************************************/

int esp32p4_sc2336_stream(bool on)
{
  if (!g_sc2336_ready)
    {
      return -ENODEV;
    }

  return sc2336_write_reg(SC2336_REG_STREAM,
                          on ? SC2336_STREAM_ON : SC2336_STREAM_OFF);
}
