/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_lvgl.c
 *
 * LVGL integration for RK3576
 *
 * Provides display driver, touch input driver, and task handler
 * for the LVGL graphics library.
 *
 * Requires: CONFIG_GRAPHICS_LVGL=y (LVGL submodule must be downloaded)
 *
 ***************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>

#include "arm64_internal.h"
#include "rk3576_fb.h"
#include "rk3576_touch.h"

#if defined(CONFIG_RK3576_LVGL) && defined(CONFIG_GRAPHICS_LVGL)

#include "lvgl.h"
#include "rk3576_lvgl.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

#ifndef CONFIG_RK3576_FB_WIDTH
#  define CONFIG_RK3576_FB_WIDTH   1024
#endif

#ifndef CONFIG_RK3576_FB_HEIGHT
#  define CONFIG_RK3576_FB_HEIGHT  600
#endif

#ifndef CONFIG_RK3576_FB_DEPTH
#  define CONFIG_RK3576_FB_DEPTH   32
#endif

/***************************************************************************
 * Private Types
 ***************************************************************************/

struct rk3576_lvgl_s
{
  bool initialized;
};

/***************************************************************************
 * Private Data
 ***************************************************************************/

static struct rk3576_lvgl_s g_lvgl;

/***************************************************************************
 * LVGL Display Driver
 ***************************************************************************/

static void rk3576_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area,
                                  uint8_t *color_p)
{
  uint8_t *fbmem = rk3576_fb_getmem();
  int w = area->x2 - area->x1 + 1;
  int bpp = CONFIG_RK3576_FB_DEPTH / 8;
  int stride = CONFIG_RK3576_FB_WIDTH * bpp;

  if (fbmem == NULL)
    {
      lv_display_flush_ready(disp);
      return;
    }

  for (int y = area->y1; y <= area->y2; y++)
    {
      uint8_t *dst = fbmem + y * stride + area->x1 * bpp;
      memcpy(dst, color_p, w * bpp);
      color_p += w * bpp;
    }

  lv_display_flush_ready(disp);
}

/***************************************************************************
 * LVGL Touch Input Driver
 ***************************************************************************/

static void rk3576_lvgl_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
  struct rk3576_touch_point_s point;
  int ret;

  ret = rk3576_touch_read(&point);
  if (ret < 0 || !point.touched)
    {
      data->state = LV_INDEV_STATE_RELEASED;
    }
  else
    {
      data->state = LV_INDEV_STATE_PRESSED;
      data->point.x = point.x;
      data->point.y = point.y;
    }

  UNUSED(indev);
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

int rk3576_lvgl_init(void)
{
  lv_display_t *disp;
  lv_indev_t *indev;
  uint8_t *fbmem;
  int buf_size;

  if (g_lvgl.initialized)
    {
      return OK;
    }

  fbmem = rk3576_fb_getmem();
  if (fbmem == NULL)
    {
      _err("LVGL: Framebuffer not available\n");
      return -ENODEV;
    }

  lv_init();

  buf_size = CONFIG_RK3576_FB_WIDTH * (CONFIG_RK3576_FB_DEPTH / 8) *
             (CONFIG_RK3576_FB_HEIGHT / 2);

  disp = lv_display_create(CONFIG_RK3576_FB_WIDTH,
                            CONFIG_RK3576_FB_HEIGHT);
  if (disp == NULL)
    {
      _err("LVGL: Failed to create display\n");
      return -ENOMEM;
    }

  lv_display_set_flush_cb(disp, rk3576_lvgl_flush_cb);
  lv_display_set_buffers(disp, fbmem, NULL, buf_size,
                          LV_DISPLAY_RENDER_MODE_PARTIAL);

  indev = lv_indev_create();
  if (indev == NULL)
    {
      _err("LVGL: Failed to create indev\n");
      return -ENOMEM;
    }

  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, rk3576_lvgl_read_cb);

  g_lvgl.initialized = true;

  ginfo("LVGL: initialized (%dx%d, %dbpp)\n",
        CONFIG_RK3576_FB_WIDTH, CONFIG_RK3576_FB_HEIGHT,
        CONFIG_RK3576_FB_DEPTH);

  return OK;
}

void rk3576_lvgl_task(void)
{
  if (g_lvgl.initialized)
    {
      lv_timer_handler();
    }
}

#else

int rk3576_lvgl_init(void)
{
  return -ENODEV;
}

void rk3576_lvgl_task(void)
{
}

#endif /* CONFIG_RK3576_LVGL && CONFIG_GRAPHICS_LVGL */
