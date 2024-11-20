/* Copyright (C) 2024 Xiaomi Corporation
 * display.c
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stdlib.h>

#include <sys/ioctl.h>

#include <nuttx/video/fb.h>

#include "brightness.h"
#include <uv.h>

#include "display.h"
#include "private.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct display_brightness_s
{
    int fd;                /* file descriptor to the frame buffer device */
    uv_loop_t *loop;       /* libuv loop */
    int target;            /* target brightness */
    int current;           /* current brightness */
    float ramp;            /* ramp rate per timer callback */
    int steps;             /* How may steps have been made. */
    int start_brightness;  /* start brightness of ramp */
    uv_timer_t ramp_timer; /* Timer to smoothly change brightness */
    brightness_update_cb_t *cb;
    void *user_data;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int write_brightness(struct display_brightness_s *display,
                            int brightness)
{
  if (display->current == brightness)
    {
      return OK;
    }

    info("Set brightness to %d\n", brightness);

    display->current = brightness;
  if (display->cb)
    {
        display->cb(BRIGHTNESS_MONITOR_LEVEL,
                    brightness, display->user_data);
    }

    ioctl(display->fd, FBIOSET_POWER, brightness);
  return 0;
}

static int read_brightness(struct display_brightness_s *display,
                           int *brightness)
{
    *brightness = display->current; /* Use cached value as default */
    ioctl(display->fd, FBIOGET_POWER, brightness);
  return 0;
}

static void timer_close_cb(uv_handle_t *handle)
{
    struct display_brightness_s *display = handle->data;
    free(display);
}

static void ramp_timer_cb(uv_timer_t *handle)
{
    struct display_brightness_s *display = handle->data;
    int current;
    int ret;

    display->steps++;

    current = display->start_brightness +
              (int)(display->steps * display->ramp);

  if ((display->ramp > 0 && current >= display->target) ||
        (display->ramp < 0 && current <= display->target))
    {
        current = display->target;
        uv_timer_stop(handle);
    }

    ret = write_brightness(display, current);
  if (ret < 0)
    {
        err("Failed to write brightness, %d\n", ret);
        return;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct display_brightness_s *display_brightness_open_device(
                                                        const char *devpath,
                                                        uv_loop_t *loop)
{
    struct display_brightness_s *display;
    int fd;
    int brightness;

    display = zalloc(sizeof(struct display_brightness_s));
  if (!display)
    {
        err("Failed to allocate memory\n");
      return NULL;
    }

    fd = open(devpath, O_RDWR | O_CLOEXEC);
  if (fd < 0)
    {
        free(display);
        err("Failed to open %s, %d\n", devpath, errno);
      return NULL;
    }

    display->loop = loop;
    display->fd = fd;

    read_brightness(display, &brightness);
    display->current = brightness;
    uv_timer_init(loop, &display->ramp_timer);
    display->ramp_timer.data = display;

  return display;
}

int display_brightness_set(struct display_brightness_s *display,
                           int brightness,
                           int ramp)
{
    int set = brightness;
    uv_timer_stop(&display->ramp_timer);

  if (ramp == BRIGHTNESS_RAMP_SPEED_DEFAULT)
    {
#ifdef CONFIG_BRIGHTNESS_RAMP_SPEED_DEFAULT
        ramp = CONFIG_BRIGHTNESS_RAMP_SPEED_DEFAULT;
#else
        ramp = DISPLAY_BRIGHTNESS_RAMP_SPEED_DEFAULT;
#endif
    }
  else if (ramp == BRIGHTNESS_RAMP_SPEED_OFF)
    {
        ramp = 0;
    }

  /* Check special brightness level */

  if (brightness == BRIGHTNESS_LEVEL_OFF)
    {
        brightness = 0;
    }
  else if (brightness == BRIGHTNESS_LEVEL_FULL)
    {
        brightness = 255;
    }

  /* Limit the brightness value */

  else if (brightness > BACKLIGHT_LEVEL_MAX)
    {
        brightness = BACKLIGHT_LEVEL_MAX;
    }
  else if (brightness < BACKLIGHT_LEVEL_MIN)
    {
        brightness = BACKLIGHT_LEVEL_MIN;
    }

    display->target = brightness;

    syslog(LOG_INFO, "Set brightness to %d(clamp: %d), ramp %d\n", set,
           brightness, ramp);

  if (ramp == 0)
    {
        display->ramp = 0;
      return write_brightness(display, brightness);
    }
    else
    {
        uv_update_time(display->loop);
        display->ramp = ramp / 1000.f * DISPLAY_BRIGHTNESS_RAMP_TIMER_PERIOD;
      if (brightness < display->current)
        {
            display->ramp = -display->ramp;
        }

        display->steps = 0;
        display->start_brightness = display->current;
        uv_timer_start(&display->ramp_timer, ramp_timer_cb,
                       DISPLAY_BRIGHTNESS_RAMP_TIMER_PERIOD,
                       DISPLAY_BRIGHTNESS_RAMP_TIMER_PERIOD);
    }

  return 0;
}

int display_brightness_get(struct display_brightness_s *display,
                           int *brightness)
{
    *brightness = display->current;
  return 0;
}

void display_brightness_close_device(struct display_brightness_s *display)
{
  if (display == NULL)
    {
        return;
    }

    uv_timer_stop(&display->ramp_timer);
    uv_close((uv_handle_t *)&display->ramp_timer, timer_close_cb);
    close(display->fd);
}

int display_brightness_set_update_cb(struct display_brightness_s *display,
                                     brightness_update_cb_t *cb,
                                     void *user_data)
{
  if (display == NULL)
      return -EINVAL;

    display->cb = cb;
    display->user_data = user_data;
  return 0;
}
