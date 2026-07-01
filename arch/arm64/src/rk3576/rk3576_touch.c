/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_touch.c
 *
 * I2C Touch screen driver (GT911 compatible)
 * Supports up to 5-point multi-touch
 ***************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "rk3576_touch.h"
#include "rk3576_i2c.h"

#ifdef CONFIG_RK3576_TOUCH

#define TOUCH_I2C_BUS     0
#define TOUCH_I2C_ADDR    0x38
#define TOUCH_REG_XH      0x03
#define TOUCH_REG_P1_XH   0x03
#define TOUCH_REG_P1_XL   0x04
#define TOUCH_REG_P1_YH   0x05
#define TOUCH_REG_P1_YL   0x06
#define TOUCH_REG_P1_WEIGHT 0x07
#define TOUCH_REG_P1_MISC  0x08
#define TOUCH_REG_TRACKID  0x08
#define TOUCH_REG_CONFIG   0x80
#define TOUCH_REG_COMMAND  0xff

#define TOUCH_MAX_POINTS   5
#define TOUCH_READ_LEN     34  /* Header + 5 points * 6 bytes + checksum */

struct rk3576_touch_s
{
  int max_points;
  int bus;
  uint8_t addr;
};

static struct rk3576_touch_s g_touch;

void rk3576_touch_init(void)
{
  g_touch.bus = TOUCH_I2C_BUS;
  g_touch.addr = TOUCH_I2C_ADDR;
  g_touch.max_points = TOUCH_MAX_POINTS;

  rk3576_i2c_init(TOUCH_I2C_BUS);
  rk3576_i2c_set_speed(TOUCH_I2C_BUS, RK3576_I2C_SPEED_FAST);
  rk3576_i2c_set_address(TOUCH_I2C_BUS, TOUCH_I2C_ADDR);

  uinfo("Touch: initialized (I2C%d, addr 0x%02x, max %d points)\n",
        TOUCH_I2C_BUS, TOUCH_I2C_ADDR, TOUCH_MAX_POINTS);
}

int rk3576_touch_read(struct rk3576_touch_point_s *point)
{
  uint8_t buf[TOUCH_READ_LEN];
  int ret;

  if (point == NULL)
    {
      return -EINVAL;
    }

  memset(point, 0, sizeof(*point));

  /* Read touch status and first point */

  ret = rk3576_i2c_read_reg(TOUCH_I2C_BUS, TOUCH_REG_P1_XH, buf, 6);
  if (ret < 0)
    {
      return ret;
    }

  /* Parse touch event from high byte of YH register */

  uint8_t event = (buf[2] >> 6) & 0x03;
  uint8_t touch_num = buf[4] & 0x0f;

  if (touch_num > 0 && event != 0)
    {
      point->touched = true;
      point->id = buf[5] & 0x0f;
      point->x = ((uint16_t)(buf[0] & 0x0f) << 8) | buf[1];
      point->y = ((uint16_t)(buf[2] & 0x0f) << 8) | buf[3];
      point->pressure = (buf[4] >> 0) & 0xff;
    }

  return OK;
}

int rk3576_touch_read_multi(struct rk3576_touch_point_s *points, int max_points)
{
  uint8_t buf[TOUCH_READ_LEN];
  int ret;
  int count;

  if (points == NULL || max_points <= 0)
    {
      return -EINVAL;
    }

  memset(points, 0, sizeof(*points) * max_points);

  /* Read status byte first */

  ret = rk3576_i2c_read_reg(TOUCH_I2C_BUS, TOUCH_REG_XH, buf, 2);
  if (ret < 0)
    {
      return ret;
    }

  count = buf[1] & 0x0f;
  if (count > max_points)
    {
      count = max_points;
    }

  if (count == 0)
    {
      return 0;
    }

  /* Read all touch point data */

  int bytes = 2 + count * 8;
  ret = rk3576_i2c_read_reg(TOUCH_I2C_BUS, TOUCH_REG_P1_XH, buf, bytes);
  if (ret < 0)
    {
      return ret;
    }

  for (int i = 0; i < count; i++)
    {
      int off = i * 8;
      uint8_t event = (buf[off + 2] >> 6) & 0x03;

      if (event != 0)
        {
          points[i].touched = true;
          points[i].id = buf[off + 5] & 0x0f;
          points[i].x = ((uint16_t)(buf[off + 0] & 0x0f) << 8) | buf[off + 1];
          points[i].y = ((uint16_t)(buf[off + 2] & 0x0f) << 8) | buf[off + 3];
          points[i].pressure = (buf[off + 4] >> 0) & 0xff;
        }
    }

  return count;
}

int rk3576_touch_get_max_points(void)
{
  return g_touch.max_points;
}

#endif /* CONFIG_RK3576_TOUCH */
